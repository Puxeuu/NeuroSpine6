"""
Subprocés Python que gestiona el port sèrie dels motors SitoDrive.

Aquest fitxer no s'executa mai directament pel viewer. El viewer arrenca
aquest subprocés a través de _DaemonBackend (sitodrive.py) i li parla via
JSON per stdin/stdout.


Protocol de comunicació amb el procés pare
    pare  -> daemon  per stdin   (JSON de comandes)
    daemon -> pare   per stdout  (JSON d'events)
    daemon -> pare   per stderr  (missatges de diagnòstic)

Comandes acceptades (una per línia):

    {"cmd": "select_mode", "mode": "POSITION_MODE", "fb1": 10, "fb2": 10, "fb3": 20}
    {"cmd": "write_profile_position", "current": [...], "velocity": [...], "positions": [...]}
    {"cmd": "write_position", ...}
    {"cmd": "write_currents", "currents": [...]}
    {"cmd": "write_velocities", "current": [...], "velocity": [...]}
    {"cmd": "write_mit", ..., "kp": [...], "ki": [...]}
    {"cmd": "reset"}
    {"cmd": "stop"}
    {"cmd": "ping"}

Events emesos:

    {"event": "ready",  "motors": [1, 2, ...]}
    {"event": "status", "t": 1234.5, "motors": [{"id":1, "pos":..., ...}, ...]}
    {"event": "ack",    "for": "<cmd>"}
    {"event": "error",  "msg": "..."}
    {"event": "closed"}

Executable per debug directe:

    python -m src.hardware._daemon COM3 921600 1 2 3 4 5 6
    python -m src.hardware._daemon tcp://192.168.1.140:5000 921600 1 2 3 4 5 6

Si PORT comença per "tcp://", en lloc d'obrir un port sèrie local es connecta
al pont TCP del Ventuno Q (ventuno/motor_daemon.cpp). El baud s'ignora (el fixa
el daemon del Ventuno). La resta del daemon no sap la diferència: _TcpLink
imita l'API de serial.Serial i retorna les mateixes trames de 13 bytes.
"""

from __future__ import annotations

import json
import os
import select
import socket
import struct
import sys
import threading
import time
import traceback
from pathlib import Path


# ==============================================================================
# FALLBACK TCP: _TcpLink prova primer l'adreça del port de l'app (p. ex.
# tcp://10.42.0.2:5000, l'Ethernet directe) i, si no respon, aquestes adreces
# en ordre. Afegeix-n'hi més si cal.
# Compte: la suma de timeouts ha de quedar per sota de READY_TIMEOUT_S (8 s) de
# _DaemonBackend (sitodrive.py), o l'app donarà timeout abans del fallback.
# ==============================================================================

TCP_FALLBACK_HOSTS = [
    "tcp://192.168.1.140:5000",                                                 # WiFi del Ventuno
]
TCP_PRIMARY_TIMEOUT_S = 1.5                                                     # Adreça del port de l'app
TCP_FALLBACK_TIMEOUT_S = 3.0                                                    # Cada adreça de TCP_FALLBACK_HOSTS
TCP_LINK_NAMES = {                                                              # Només per al log
    "10.42.0.2": "Ethernet",
    "192.168.1.140": "WiFi",
}


# ==============================================================================
# Quan aquest fitxer s'executa com a subprocés (python -m src.hardware._daemon)
# cal assegurar que pot importar src.hardware.sitodrive.
# QUÈ FA EL SERIAL?
#   port = serial.Serial("COM3", 921600)    # obre COM3 a 921600 bps
#   port.write(b"\x55\x05...")              # envia bytes al dispositiu
#   data = port.read(13)                    # llegeix 13 bytes del dispositiu
#   port.close()                            # tanca
# ==============================================================================

_THIS_FILE = Path(__file__).resolve()
_SRC_DIR = _THIS_FILE.parent.parent.parent
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))                                           # Afegeix l'arrel al sys.path perquè Python pugui fer import src.hardware.sitodrive.

try:
    import serial                                                               # Intenta importar pyserial (la llibreria que parla amb el port sèrie).
except ImportError:
    sys.stdout.write('{"event": "error", "msg": "pyserial no instal·lat."}\n')
    sys.stdout.flush()
    sys.exit(1)


#Importa les constants i funcions encoder del protocol:
from src.hardware.sitodrive import (HEADER, PACKET_SIZE, Mode, FeedbackType, encode_reset, encode_mode_selection, encode_profile_position, encode_current, encode_profile_velocity, encode_mit_position, encode_mit_pid,)





def _write_msg(d: dict) -> None:                                                # Converteix el dict a JSON i l'envia per stdout (que el parent llegirà via _read_stdout).
    try:
        sys.stdout.write(json.dumps(d) + "\n")
        sys.stdout.flush()
    except (BrokenPipeError, OSError):
        os._exit(0)


def _log(msg: str) -> None:                                                     # Missatge de diagnòstic per stderr (NO per stdout) amb el prefix [motor-daemon].
    try:
        sys.stderr.write(f"[motor-daemon] {msg}\n")
        sys.stderr.flush()
    except Exception:
        pass




# ==============================================================================
# _TcpLink: substitut de serial.Serial que parla amb el pont TCP del Ventuno.
#
# Protocol TCP (12 bytes fixos, mateix que ventuno/motor_daemon.cpp):
#   [0xA5, tipus, motor_id, cmd, d0..d7]
#   tipus 0x01 CAN_TX (Mac->Ventuno), 0x02 CAN_RX (feedback), 0x03 PING, 0x04 ERROR
#
# Trama sèrie (13 bytes) <-> trama TCP (12 bytes): es treu "88 05 06" i es posa
# "A5 tipus" al davant. Les 10 últimes (id, cmd, 8 dades) són idèntiques.
# ==============================================================================

def _parse_tcp_url(url: str) -> tuple:                                          # "tcp://10.42.0.2:5000" -> ("10.42.0.2", 5000)
    hostport = url[len("tcp://"):] if url.startswith("tcp://") else url
    host, _, port = hostport.rpartition(":")
    if not host or not port:
        raise ValueError(f"URL TCP incorrecta: {url!r} (format tcp://IP:PORT)")
    return host, int(port)


class _TcpLink:

    MAGIC = 0xA5
    T_CAN_TX = 0x01
    T_CAN_RX = 0x02
    T_PING = 0x03
    T_ERROR = 0x04
    FRAME = 12
    PING_PERIOD_S = 0.2                                                         # El daemon del Ventuno talla si no rep res en 1 s
    RETRY_PERIOD_S = 1.0

    def __init__(self, url: str, timeout: float = 0.02, write_timeout: float = 0.5) -> None:
        primary = _parse_tcp_url(url)
        self._candidates = [(primary, TCP_PRIMARY_TIMEOUT_S)]                   # [(adreça, timeout)], en ordre de preferència
        for fb in TCP_FALLBACK_HOSTS:
            addr = _parse_tcp_url(fb)
            if all(addr != a for a, _ in self._candidates):                     # Sense duplicats (p. ex. si el port de l'app ja és el WiFi)
                self._candidates.append((addr, TCP_FALLBACK_TIMEOUT_S))
        self.timeout = timeout                                                  # Temps màxim que read() espera dades (com el timeout de pyserial)
        self.write_timeout = write_timeout
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()                                           # Protegeix _sock i les escriptures (cmd-reader i fil de ping)
        self._rx = bytearray()                                                  # Bytes TCP rebuts encara no convertits
        self._next_retry = 0.0
        self._closed = threading.Event()
        self._connect()                                                         # Si falla, l'excepció arriba a main() -> event "error" -> el pare ho veu
        self._ping_thread = threading.Thread(target=self._ping_loop, daemon=True, name="tcp-ping")
        self._ping_thread.start()

    def _connect(self) -> None:                                                 # Prova les adreces en ordre; la primera que respon guanya
        failed = []
        for i, (addr, conn_timeout) in enumerate(self._candidates):
            try:
                sock = socket.create_connection(addr, timeout=conn_timeout)
            except OSError as e:                                                # Host unreachable, refused o timeout
                failed.append(f"{addr[0]}:{addr[1]} ({e})")
                continue
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)          # Sense Nagle: cada trama surt de seguida
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            sock.settimeout(self.write_timeout)                                 # Afecta sendall(); el recv va amb select()
            with self._lock:
                self._sock = sock
                self._rx.clear()
            name = TCP_LINK_NAMES.get(addr[0], addr[0])
            if i == 0:
                _log(f"[tcp] connectat per {name} ({addr[0]}:{addr[1]})")
            else:
                _log(f"[tcp] fallback a {name} ({addr[0]}:{addr[1]}); no han respost: {'; '.join(failed)}")
            return
        raise OSError("cap adreça TCP respon: " + "; ".join(failed))

    def _drop(self) -> None:                                                    # Marca la connexió com a perduda; read() la reintentarà
        with self._lock:
            sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        self._next_retry = time.monotonic() + self.RETRY_PERIOD_S

    @property
    def in_waiting(self) -> int:                                                # Sempre 0: el reader crida read(1) i read() retorna tot el que hi hagi
        return 0

    def read(self, size: int = 1) -> bytes:
        """Espera fins a self.timeout i retorna les trames de feedback rebudes en
        format sèrie (13 bytes cadascuna). Pot retornar més de `size` bytes."""
        sock = self._sock
        if sock is None:                                                        # Desconnectat: reintent cada RETRY_PERIOD_S
            if self._closed.is_set() or time.monotonic() < self._next_retry:
                time.sleep(self.timeout)
                return b""
            try:
                self._connect()                                                 # Torna a provar en ordre: l'Ethernet primer
                _log("reconnectat")
            except OSError as e:
                self._next_retry = time.monotonic() + self.RETRY_PERIOD_S
                _log(f"reconnexió fallida: {e}")
            return b""

        try:
            ready, _, _ = select.select([sock], [], [], self.timeout)
            if not ready:
                return b""
            data = sock.recv(4096)
        except (OSError, ValueError) as e:
            self._drop()
            raise serial.SerialException(f"TCP: {e}")
        if not data:
            self._drop()
            raise serial.SerialException("el Ventuno ha tancat la connexió TCP")

        rx = self._rx
        rx.extend(data)
        out = bytearray()
        i = 0
        n = len(rx)
        while i + self.FRAME <= n:
            if rx[i] != self.MAGIC:                                             # Desincronitzat: busquem el següent 0xA5
                i += 1
                continue
            kind = rx[i + 1]
            if kind == self.T_CAN_RX:
                out += bytes([HEADER, 0x05, 0x06]) + rx[i + 2:i + 12]
            elif kind == self.T_ERROR:
                _log(f"error del Ventuno, codi 0x{rx[i + 4]:02X} (1=sèrie caigut, 2=error escriptura sèrie)")
            i += self.FRAME                                                     # T_PING (pong): no cal fer res
        del rx[:i]
        return bytes(out)

    def write(self, data: bytes) -> int:
        """Rep una o més trames sèrie de 13 bytes i les envia com a CAN_TX."""
        data = bytes(data)
        if len(data) % PACKET_SIZE:
            raise ValueError(f"cal un múltiple de {PACKET_SIZE} bytes, n'he rebut {len(data)}")
        frames = bytearray()
        for i in range(0, len(data), PACKET_SIZE):
            if data[i] != HEADER or data[i + 1] != 0x05 or data[i + 2] != 0x06:
                raise ValueError(f"trama sense capçalera 88 05 06: {data[i:i + PACKET_SIZE].hex(' ')}")
            frames += bytes([self.MAGIC, self.T_CAN_TX]) + data[i + 3:i + PACKET_SIZE]
        self._send(bytes(frames))
        return len(data)

    def _send(self, payload: bytes) -> None:
        with self._lock:
            sock = self._sock
            if sock is None:
                raise serial.SerialException("TCP desconnectat del Ventuno")
            try:
                sock.sendall(payload)
            except socket.timeout as e:
                raise serial.SerialTimeoutException(f"TCP: {e}")
            except OSError as e:
                self._sock = None
                self._next_retry = time.monotonic() + self.RETRY_PERIOD_S
                try:
                    sock.close()
                except OSError:
                    pass
                raise serial.SerialException(f"TCP: {e}")

    def _ping_loop(self) -> None:
        ping = bytes([self.MAGIC, self.T_PING]) + bytes(10)
        while not self._closed.wait(self.PING_PERIOD_S):
            try:
                self._send(ping)
            except Exception:
                pass                                                            # El reader ja gestiona la reconnexió

    def flush(self) -> None:                                                    # sendall() ja ha lliurat les dades al kernel
        pass

    def close(self) -> None:
        self._closed.set()
        self._drop()




def main() -> None:
    args = sys.argv[1:]                                                         # Si hem executat python -m src.hardware._daemon COM3 921600 1 2 3 4 5 6, args = ["COM3", "921600", "1", "2", "3", "4", "5", "6"].   Si falta algun o són incorrectes, envia JSON d'error al parent i surt.
    if len(args) < 3:
        _write_msg({"event": "error", "msg": "ús: -m src.hardware._daemon PORT BAUD ID1 [ID2 ...]"})
        sys.exit(2)

    port = args[0]                                                              # assignem port
    try:
        baud = int(args[1])
    except ValueError:
        _write_msg({"event": "error", "msg": f"baud incorrecte: {args[1]}"})
        sys.exit(2)
    try:
        motor_ids = [int(x, 0) for x in args[2:]]                               # assignem motor_ids
    except ValueError:
        _write_msg({"event": "error", "msg": f"IDs incorrectes: {args[2:]}"})
        sys.exit(2)
    _log(f"obrint {port} @ {baud} per motors {motor_ids}")                      # Print perquè vull comprovar que la info és correcte. 



    try:                                                                        # INTENTEM OBRIR EL PORT SÈRIE amb timeout 20ms (o el pont TCP del Ventuno)
        if port.startswith("tcp://"):
            ser = _TcpLink(port, timeout=0.02, write_timeout=0.5)
        else:
            ser = serial.Serial(port=port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=0.02, write_timeout=0.5, rtscts=False, xonxoff=False,)
    except Exception as e:
        _write_msg({"event": "error", "msg": f"No s'ha pogut obrir {port} @ {baud}: {e}"})
        sys.exit(1)
    _log("port obert")                                                          # _log va a stderr  i  _write_msg va a stdout


    statuses = {mid: {"id": mid, "pos": 0, "motor_pos": 0, "curr": 0, "vel": 0, "pwm": 0, "volt": 0, "mos_temp": 0, "mt_temp": 0, "g431_temp": 0, "warn": 0, "err": 0, "last_b1": 0.0, "last_b2": 0.0, "last_b3": 0.0,} for mid in motor_ids}
    status_lock = threading.Lock()                                              # Diccionari amb l'estat de cada motor (una entrada per motor, 6 total). Protegit pel lock perquè diversos threads el toquen.
    stop_flag = threading.Event()                                               # Event que serveix com a semàfor global. Quan algú fa stop_flag.set(), tots els bucles del daemon ho veuen i surten.
    counters = {"bytes_read": 0, "packets_parsed": 0, "bytes_skipped": 0, "write_errors": 0}        # Comptadors per diagnòstic


    def _update_status(motor_id: int, fb_type: int, payload: bytes) -> None:                        # Funció interna. Rep un paquet de feedback d'un motor i actualitza el seu estat.
        if motor_id not in statuses:                                                                # Si el motor_id no és dels que gestionem, ignora silenciosament. Defensiu: potser arriba soroll amb un id invàlid.
            return
        now = time.monotonic()
        with status_lock:                                                                           # Adquireix el lock. Mentre som aquí dins, cap altre thread toca statuses.
            s = statuses[motor_id]
            try:
                if fb_type == int(FeedbackType.VOLT_PWM_CUR_VEL):    # 0xB1
                    v, p, c, w = struct.unpack('>hhhh', payload)
                    s["volt"] = v
                    s["pwm"] = p
                    s["curr"] = c
                    s["vel"] = w
                    s["last_b1"] = now
                elif fb_type == int(FeedbackType.POSITIONS):         # 0xB2
                    mp, ap = struct.unpack('>ii', payload)
                    s["motor_pos"] = mp
                    s["pos"] = ap
                    s["last_b2"] = now
                elif fb_type == int(FeedbackType.TEMP_FAULTS):       # 0xB3
                    s["mos_temp"] = payload[0]
                    s["mt_temp"] = payload[1]
                    s["g431_temp"] = payload[2]
                    s["warn"] = payload[3]
                    s["err"] = payload[4]
                    s["last_b3"] = now
            except struct.error:
                pass



    # --------------------------------------------------------------------------
    # THREAD DE LECTURA DEL PORT SÈRIE: Bucle continu que llegeix bytes, extreu paquets de 13 bytes.
    # --------------------------------------------------------------------------
    
    def reader_thread() -> None:
        buf = bytearray()
        while not stop_flag.is_set():                                                               # Bucle infinit fins que algú demani parar.
            try:
                wait = ser.in_waiting                                                               # quants bytes hi ha disponibles al buffer del sistema sense bloquejar.
                if wait > 0:
                    chunk = ser.read(wait)                                                          # Si n'hi ha, els llegeix tots d'una.
                else:
                    chunk = ser.read(1)                                                             # Si no, llegeix 1 byte (bloquejant fins timeout de 20 ms) per no fer busy-wait.
                if chunk:
                    counters["bytes_read"] += len(chunk)                                            # Si ha llegit alguna cosa, actualitza comptador i afegeix al buffer.
                    buf.extend(chunk)

                i = 0
                n = len(buf)
                while i + PACKET_SIZE <= n:
                    if buf[i] == HEADER and buf[i + 1] == 0x05 and buf[i + 2] == 0x06:              # Bucle intern que parseja paquets complets del buffer. PACKET_SIZE = 13. Mentre hi hagi almenys 13 bytes des de la posició i.
                        motor_id = buf[i + 3]                                                       # Extreu els camps del paquet:
                        fb_type = buf[i + 4]
                        payload = bytes(buf[i + 5:i + 13])
                        _update_status(motor_id, fb_type, payload)                                  # Processa el paquet, incrementa comptador, salta els 13 bytes.
                        counters["packets_parsed"] += 1
                        i += PACKET_SIZE
                    else:
                        counters["bytes_skipped"] += 1                                              # Si no és un inici de paquet vàlid, salta 1 byte i segueix buscant. Segurament és soroll.
                        i += 1

                if i > 0:                                                                           # Neteja els bytes ja processats del buffer.
                    del buf[:i]
                if len(buf) > 4096:                                                                 # Protecció: si el buffer creix massa (>4KB), el retalla deixant només els últims 52 bytes.
                    del buf[:-PACKET_SIZE * 4]

            except serial.SerialException as e:                                                     # si peta la lectura (cable desconnectat, etc.), log + petit sleep i continua. No peta tot el programa per això. 
                _log(f"error sèrie: {e}")
                time.sleep(0.1)
            except Exception as e:
                _log(f"reader: {e}")
                time.sleep(0.05)


    # --------------------------------------------------------------------------
    # EXECUTOR DE COMANDES: Rep una comanda JSON i escriu els paquets corresponents al port sèrie.
    # --------------------------------------------------------------------------
    def execute(cmd: dict) -> bool:
        c = cmd.get("cmd")                                                                          # Agafa el camp cmd del dict per saber què fer.
        try:
            if c == "select_mode":
                mode = getattr(Mode, cmd["mode"])                                                   # getattr(Mode, "CURRENT_MODE") equival a Mode.CURRENT_MODE. Útil quan el nom ve com a string del JSON.
                fb1 = int(cmd.get("fb1", 10))                                                       # Si no envia el feedback, ell automàticament posa 10ms
                fb2 = int(cmd.get("fb2", 10))
                fb3 = int(cmd.get("fb3", 20))
                for mid in motor_ids:
                    ser.write(encode_mode_selection(mid, mode, fb1, fb2, fb3))  

            elif c == "write_profile_position":
                cur = cmd["current"]; vel = cmd["velocity"]; pos = cmd["positions"]
                if len(cur) == 1: cur = cur * len(motor_ids)                                        # Si rep un sol valor, el replica per a tots els motors. Ex: cur = [100] → cur = [100, 100, 100, 100, 100, 100]. 
                if len(vel) == 1: vel = vel * len(motor_ids)
                if len(pos) == 1: pos = pos * len(motor_ids)
                for mid, ci, vi, pi in zip(motor_ids, cur, vel, pos):
                    ser.write(encode_profile_position(
                        mid, int(ci), int(vi), int(pi), Mode.PROFILE_POSITION_MODE))

            elif c == "write_position":
                cur = cmd["current"]; vel = cmd["velocity"]; pos = cmd["positions"]
                if len(cur) == 1: cur = cur * len(motor_ids)
                if len(vel) == 1: vel = vel * len(motor_ids)
                if len(pos) == 1: pos = pos * len(motor_ids)
                for mid, ci, vi, pi in zip(motor_ids, cur, vel, pos):
                    ser.write(encode_profile_position(mid, int(ci), int(vi), int(pi), Mode.POSITION_MODE))

            elif c == "write_currents":
                currents = cmd["currents"]
                if len(currents) == 1: currents = currents * len(motor_ids)
                for mid, ci in zip(motor_ids, currents):
                    ser.write(encode_current(mid, int(ci)))

            elif c == "write_velocities":
                cur = cmd["current"]; vel = cmd["velocity"]
                if len(cur) == 1: cur = cur * len(motor_ids)
                if len(vel) == 1: vel = vel * len(motor_ids)
                for mid, ci, vi in zip(motor_ids, cur, vel):
                    ser.write(encode_profile_velocity(mid, int(ci), int(vi)))

            elif c == "write_mit":
                cur = cmd["current"]; vel = cmd["velocity"]; pos = cmd["positions"]
                kp  = cmd["kp"];      ki  = cmd["ki"]
                if len(cur) == 1: cur = cur * len(motor_ids)
                if len(vel) == 1: vel = vel * len(motor_ids)
                if len(pos) == 1: pos = pos * len(motor_ids)
                if len(kp)  == 1: kp  = kp  * len(motor_ids)
                if len(ki)  == 1: ki  = ki  * len(motor_ids)
                for mid, ci, vi, pi, kpi, kii in zip(motor_ids, cur, vel, pos, kp, ki):
                    ser.write(encode_mit_position(mid, int(ci), int(vi), int(pi)))
                    ser.write(encode_mit_pid(mid, float(kpi), float(kii)))

            elif c == "reset":
                for mid in motor_ids:
                    ser.write(encode_reset(mid))

            elif c == "stop":
                stop_flag.set()
                _write_msg({"event": "ack", "for": "stop"})
                return False                                                                        # Aturar el daemon. Activa stop_flag (que tots els bucles comproven) i retorna False per dir al cmd_reader_thread que surti.

            elif c == "raw_bytes":                                                                  # Envia bytes arbitraris. & 0xFF garanteix que cada byte val 0-255. Útil per calibracions, per exemple. 
                raw = bytes(int(b) & 0xFF for b in cmd["bytes"])
                ser.write(raw)

            elif c == "ping":                                                                       # Comanda que no fa res però el parent la pot fer servir per comprovar que el daemon respon.
                pass

            else:
                _write_msg({"event": "error", "msg": f"cmd desconeguda: {c}"})
                return True

            ser.flush()                                                                             # Flush del port per garantir que els bytes s'envien ja.
            _write_msg({"event": "ack", "for": c})
            return True

        except serial.SerialTimeoutException as e:                                                  # Si l'escriptura al port falla (timeout o altre error), reporta i continua. No peta tot per un error puntual.
            counters["write_errors"] += 1
            _write_msg({"event": "error", "msg": f"write timeout a {c!r}: {e}"})
            return True
        except Exception as e:
            counters["write_errors"] += 1
            _write_msg({"event": "error", "msg": f"cmd {c!r}: {e}"})
            return True


    # --------------------------------------------------------------------------
    # THREAD LECTOR D'STDIN: Rep les comandes JSON del pare i les porta a execute().
    # --------------------------------------------------------------------------
    def cmd_reader_thread() -> None:                                                                # Itera línia a línia d'stdin. Bloquejant: espera fins que arribi una línia del parent.
        try:
            for line in sys.stdin:
                if stop_flag.is_set():                                                              # Si estem tancant, surt. 
                    break
                line = line.strip()                                                                 # Si la línia és buida, salta.
                if not line:
                    continue
                try:
                    cmd = json.loads(line)                                                          # Parseja JSON. Si falla, reporta error
                except Exception as e:
                    _write_msg({"event": "error", "msg": f"JSON incorrecte: {e}: {line[:120]!r}"})
                    continue
                if not execute(cmd):                                                                # Executa la comanda. Si retorna False (stop), surt del bucle.
                    break
        except Exception as e:                                                                      # Si tot el bucle peta, informa el parent. Finally sempre activa stop_flag perquè la resta del daemon també es tanqui.
            _write_msg({"event": "error", "msg": f"cmd reader crashed: {e}"})
        finally:
            stop_flag.set()


    # --------------------------------------------------------------------------
    # ARRENCADA DELS THREADS
    # --------------------------------------------------------------------------
    reader = threading.Thread(target=reader_thread, daemon=True, name="serial-reader")
    reader.start()

    cmd_thread = threading.Thread(target=cmd_reader_thread, daemon=True, name="cmd-reader")
    cmd_thread.start()

    _write_msg({"event": "ready", "motors": motor_ids, "port": port, "baud": baud})                 # Avisa al parent que estem preparats. Això desbloqueja el _ready.wait(8s) del constructor del _DaemonBackend.
    _log("ready")


    # --------------------------------------------------------------------------
    # BUCLE PRINCIPAL - Emissor d'estat a 25 Hz + diagnòstic cada 5s
    # --------------------------------------------------------------------------
    period = 1.0 / 25.0                                                                             # Freqüència 25 Hz (40 ms entre tics). Pròxim diagnòstic d'aquí 5 segons.
    next_diag = time.monotonic() + 5.0

    while not stop_flag.is_set():                                                                   # Bucle principal fins que algú demani parar.
        with status_lock:
            payload = [dict(statuses[mid]) for mid in motor_ids]                                    # Pren el lock i fa una còpia de l'estat de tots els motors.
        _write_msg({"event": "status", "t": time.monotonic(), "motors": payload})                   # Envia JSON amb l'estat al parent. Això arriba a _read_stdout 25 vegades per segon.

        now = time.monotonic()
        if now >= next_diag:
            _log(f"diag: bytes_read={counters['bytes_read']} "                                      # Cada 5 segons imprimeix comptadors de diagnòstic a stderr. Útil per veure si el daemon treballa bé.
                 f"packets_parsed={counters['packets_parsed']} "
                 f"bytes_skipped={counters['bytes_skipped']} "
                 f"write_errors={counters['write_errors']}")
            next_diag = now + 5.0

        time.sleep(period)                                                                          # Dorm 40 ms. Evitem saturar CPU.


    # --------------------------------------------------------------------------
    # TANCAMENT
    # --------------------------------------------------------------------------
    try:
        ser.close()                                                                                 # Tanca sí o sí el daemon
    except Exception:
        pass
    _write_msg({"event": "closed"})                                                                 # Avisa al parent que estem tancats. El parent ho rep al _read_stdout i surt del seu bucle.




# -------------------- MAIN --------------------------------

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        _write_msg({"event": "closed"})
    except Exception as e:
        _write_msg({"event": "error", "msg": f"fatal: {e}", "trace": traceback.format_exc()})
        sys.exit(1)