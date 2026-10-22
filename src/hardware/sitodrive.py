"""
Driver base dels motors SitoDrive.

Aquest fitxer és la capa més baixa del hardware.

    1. Simulador (port=None)
       Un simulador pur en Python que corre dins del propi procés.
       Ideal per tests i desenvolupament sense hardware.

    2. Hardware real (port="COM3", "/dev/ttyUSB0", ...)
       Arrenca un subprocés Python separat (_daemon.py) que fa d'intermediari
       amb el port sèrie. El procés pare (viewer) mai importa el backend
       C++ directament: només parla amb el daemon via JSON.

    3. Hardware remot (port="tcp://192.168.1.140:5000")
       Igual que el 2, però el daemon es connecta per TCP al Ventuno Q
       (ventuno/motor_daemon.cpp), que és qui té el convertidor TA_CAN.
       La interfície de MotorBus és idèntica; el baudrate s'ignora.

Per què fem servir un subprocés en comptes d'importar el backend C++?
Perquè algunes combinacions Windows + Python 3.14 + PyQt5 + sitodrive_cpp
provoquen crash silenciosos si tot es carrega al mateix procés. Aïllant
el driver en un subprocés separat evitem aquest problema.
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


# ==============================================================================
# CONSTANTS DEL PROTOCOL: Cal que coincideixin amb el firmware dels motors (motor_control.h de sitodrive).
# ==============================================================================

HEADER = 0x88
PREFIX = bytes([0x05, 0x06])
PACKET_SIZE = 13
DATA_SIZE = 8


# ==============================================================================
# MODES DE CONTROL: definits un per un a google DOCS meu
# ==============================================================================

class Mode(IntEnum):
    RESET                 = 0x00
    MODE_SELECTION        = 0x01
    ASYNC_MODE            = 0x03
    OPEN_LOOP_MODE        = 0x04
    CURRENT_MODE          = 0x05
    PROFILE_VELOCITY_MODE = 0x06
    PROFILE_POSITION_MODE = 0x07
    POSITION_MODE         = 0x08
    MIT_MODE              = 0x09
    BRAKE_CONTROL         = 0x20
    OPENLOOP_ACC_DEC_SET  = 0x30
    VELOCITY_ACC_DEC_SET  = 0x31
    CURRENT_PID_SET       = 0x42
    VELOCITY_PID_SET      = 0x43
    POSITION_PID_SET      = 0x44
    MIT_PID_SET           = 0x45


class FeedbackType(IntEnum):
    RESET_REPLY      = 0xB0
    VOLT_PWM_CUR_VEL = 0xB1
    POSITIONS        = 0xB2
    TEMP_FAULTS      = 0xB3


# ==============================================================================
# MOTORSTATUS: Estructura amb tota la informació que un motor ens pot retornar.
# ==============================================================================

@dataclass
class MotorStatus:
    motor_id: int = 0 
    voltage_mV: int = 0
    pwm: int = 0
    current_mA: int = 0
    velocity: int = 0
    motor_position: int = 0
    actuator_position: int = 0
    mos_temp: int = 0
    mt_temp: int = 0
    g431_temp: int = 0
    warning_type: int = 0
    error_type: int = 0
    timestamp: float = 0.0


# ==============================================================================
# BUILDERS DE PAQUETS
# Format:  [HEADER, 0x05, 0x06, motor_id, cmd, data(8 bytes)]  = 13 bytes
# ==============================================================================

def _build_packet(motor_id: int, command: int, data: bytes) -> bytes:
    if len(data) != DATA_SIZE:
        raise ValueError(f"data ha de tenir {DATA_SIZE} bytes, en té {len(data)}")
    return bytes([HEADER]) + PREFIX + bytes([motor_id, command]) + data         # Ho fem per no haver de posar sempre el header 0x88 i el PREFIX...


def encode_reset(motor_id: int) -> bytes:
    return _build_packet(motor_id, Mode.RESET, b'\x55' * DATA_SIZE)             # Posem directament el mode reset que es el 0x00


def encode_mode_selection(motor_id: int, mode: Mode, fb_cycle1: int = 10, fb_cycle2: int = 0, fb_cycle3: int = 0) -> bytes:
    data = bytes([int(mode), fb_cycle1, fb_cycle2, fb_cycle3, 0x55, 0x55, 0x55, 0x55])
    return _build_packet(motor_id, Mode.MODE_SELECTION, data)


def encode_open_loop(motor_id: int, pwm: int) -> bytes:
    return _build_packet(motor_id, Mode.OPEN_LOOP_MODE, struct.pack('>h', pwm) + b'\x55' * 6)


def encode_current(motor_id: int, current_mA: int) -> bytes:
    return _build_packet(motor_id, Mode.CURRENT_MODE, struct.pack('>h', current_mA) + b'\x55' * 6)


def encode_profile_velocity(motor_id: int, current_mA: int, velocity: int) -> bytes:
    return _build_packet(motor_id, Mode.PROFILE_VELOCITY_MODE, struct.pack('>hh', current_mA, velocity) + b'\x55' * 4)


def encode_profile_position(motor_id: int, current_mA: int, velocity: int, position: int, mode: Mode = Mode.PROFILE_POSITION_MODE) -> bytes:
    return _build_packet(motor_id, mode, struct.pack('>hhi', current_mA, velocity, position))


def encode_position(motor_id: int, current_mA: int, velocity: int, position: int) -> bytes:
    return encode_profile_position(motor_id, current_mA, velocity, position, Mode.POSITION_MODE)


def encode_mit_position(motor_id: int, current_mA: int, velocity: int, position: int) -> bytes:
    return encode_profile_position(motor_id, current_mA, velocity, position, Mode.MIT_MODE)


def encode_mit_pid(motor_id: int, kp: float, ki: float) -> bytes:
    return _build_packet(motor_id, Mode.MIT_PID_SET, struct.pack('>ff', kp, ki))


def encode_current_pid(motor_id: int, kp: float, ki: float) -> bytes:
    return _build_packet(motor_id, Mode.CURRENT_PID_SET, struct.pack('>ff', kp, ki))


def encode_velocity_pid(motor_id: int, kp: float, ki: float) -> bytes:
    return _build_packet(motor_id, Mode.VELOCITY_PID_SET, struct.pack('>ff', kp, ki))


def encode_position_pid(motor_id: int, kp: float, kd: float) -> bytes:
    return _build_packet(motor_id, Mode.POSITION_PID_SET, struct.pack('>ff', kp, kd))


def encode_brake(motor_id: int, pwm: int, time_ms: int) -> bytes:
    return _build_packet(motor_id, Mode.BRAKE_CONTROL, struct.pack('>hH', pwm, time_ms) + b'\x55' * 4)


def encode_openloop_acc_dec(motor_id: int, acc: int, dec: int) -> bytes:
    return _build_packet(motor_id, Mode.OPENLOOP_ACC_DEC_SET, struct.pack('>ii', acc, dec))


def encode_velocity_acc_dec(motor_id: int, acc: int, dec: int) -> bytes:
    return _build_packet(motor_id, Mode.VELOCITY_ACC_DEC_SET, struct.pack('>ii', acc, dec))




# ==============================================================================
# FEEDBACK
# ==============================================================================

def parse_feedback(buffer: bytes) -> Tuple[List[Tuple[int, int, bytes]], bytes]:

    out: List[Tuple[int, int, bytes]] = []
    i = 0
    n = len(buffer)
    while i + PACKET_SIZE <= n:
        if (buffer[i] == HEADER and buffer[i + 1] == 0x05 and buffer[i + 2] == 0x06):
            motor_id = buffer[i + 3]
            fb_type = buffer[i + 4]
            payload = buffer[i + 5:i + 13]
            out.append((motor_id, fb_type, bytes(payload)))
            i += PACKET_SIZE
        else:
            i += 1
    return out, bytes(buffer[i:])


def update_status_from_packet(status: MotorStatus, fb_type: int, payload: bytes) -> None:
    if fb_type == FeedbackType.VOLT_PWM_CUR_VEL:            # Seria el B1
        v, p, c, w = struct.unpack('>hhhh', payload)
        status.voltage_mV = v
        status.pwm = p
        status.current_mA = c
        status.velocity = w
    elif fb_type == FeedbackType.POSITIONS:                 # Seria el B2
        mp, ap = struct.unpack('>ii', payload)
        status.motor_position = mp
        status.actuator_position = ap
    elif fb_type == FeedbackType.TEMP_FAULTS:               # Seria el B3
        status.mos_temp = payload[0]                        # No cal fer unpack perquè ocupen 1 únic byte (de 0-255). Els altres ocupen més d'un byte i s'ha de fer unapck. 
        status.mt_temp = payload[1]
        status.g431_temp = payload[2]
        status.warning_type = payload[3]
        status.error_type = payload[4]
    status.timestamp = time.monotonic()


# ==============================================================================
# BACKEND SIMULADOR: Simulador per fer tests i desenvolupar sense connexió física.
# ==============================================================================

class _SimMotor:
    def __init__(self, motor_id: int) -> None:
        self.motor_id = motor_id
        self.actuator_position = 0
        self.motor_position = 0
        self.velocity = 0
        self.current_mA = 0


class _SimBackend:
    def __init__(self, motor_ids: Sequence[int]) -> None:
        self.motors: Dict[int, _SimMotor] = {
            mid: _SimMotor(mid) for mid in motor_ids
        }

    def feed_command(self, pkt: bytes) -> None:
        if len(pkt) != PACKET_SIZE or pkt[0] != HEADER:                         # Verifica que el paquet sigui vàlid.
            return
        motor_id = pkt[3]
        cmd = pkt[4]
        data = pkt[5:13]
        m = self.motors.get(motor_id)
        if m is None:
            return


        if cmd in (Mode.PROFILE_POSITION_MODE, Mode.POSITION_MODE, Mode.MIT_MODE):
            target = struct.unpack('>i', data[4:8])[0]
            m.actuator_position = target
            m.motor_position = target
        elif cmd == Mode.PROFILE_VELOCITY_MODE:
            m.velocity = struct.unpack('>h', data[2:4])[0]
        elif cmd == Mode.CURRENT_MODE:
            m.current_mA = struct.unpack('>h', data[0:2])[0]


    def synthesize_feedback(self, motor_id: int, fb_types: Sequence[int]) -> List[bytes]:
        out = []
        m = self.motors[motor_id]
        for ft in fb_types:
            if ft == FeedbackType.VOLT_PWM_CUR_VEL:                             # Genera paquets de feedback com si el motor real els hagués enviat.
                payload = struct.pack('>hhhh', 24000, 0, m.current_mA, m.velocity)
            elif ft == FeedbackType.POSITIONS:
                payload = struct.pack('>ii', m.motor_position, m.actuator_position)
            elif ft == FeedbackType.TEMP_FAULTS:
                payload = bytes([35, 30, 32, 0, 0, 0, 0, 0])
            else:
                continue
            out.append(bytes([HEADER, 0x05, 0x06, motor_id, ft]) + payload)
        return out


# ==============================================================================
# BACKEND HARDWARE: Arrenca un subprocés Python que parla amb el port sèrie via pyserial.
# La comunicació parent <-> daemon és amb JSON per stdin/stdout.
# ==============================================================================

class _DaemonBackend:

    READY_TIMEOUT_S = 8.0                                                       # temps màxim esperant que el daemon arrenqui

    def __init__(self, port: str, baudrate: int, motor_ids: Sequence[int]) -> None:
        self.motor_ids = list(motor_ids)                                        # Guarda els ids com a atribut.
        self._latest: Dict[int, dict] = {}                                      # Diccionari buit on anirem guardant l'últim estat conegut de cada motor. Clau = id del motor, valor = dict amb posició, velocitat, corrent, etc.
        self._lock = threading.Lock()                                           # Un lock (candau). Com que diversos fils poden tocar _latest alhora, el lock serveix per garantir que només un fil a la vegada el modifica.
        self._ready = threading.Event()                                         # Un Event (semàfor binari). Comença "no activat". Quan el daemon ens diu "estic llest", algú farà .set() i l'event passa a "activat". Permet que un fil esperi fins que passi.
        self._closed = threading.Event()                                        # Un altre Event, per indicar "estem tancant la connexió".
        self._error: Optional[str] = None                                       # Variable per guardar missatges d'error. Comença a None (sense error).
        self._proc: Optional[subprocess.Popen] = None                           # Variable per guardar el subprocés quan l'hàgim arrencat. De moment None.


        src_dir = Path(__file__).resolve().parent.parent.parent                 # Troba la carpeta arrel del projecte. On està neurospine.py
        env = os.environ.copy()                                                 # Copia totes les variables d'entorn actuals del sistema.
        existing = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = (str(src_dir) + os.pathsep + existing if existing else str(src_dir))                        # Afegeix la carpeta del projecte al principi del PYTHONPATH. Això és perquè quan el daemon arranqui, pugui fer import src.hardware....
        cmd = [sys.executable, "-u", "-m", "src.hardware._daemon", port, str(baudrate)] + [str(m) for m in motor_ids]   # Executem _daemon.py

        creationflags = 0
        if sys.platform == "win32":
            creationflags = 0x08000000                                          # Només al Windows, afegim el flag CREATE_NO_WINDOW perquè no apareixi una finestra negra de consola quan arranquem el subprocés. Netedat visual.

        try:
            self._proc = subprocess.Popen(                                      # Intentem arrencar el subprocés
                cmd,                                                            # la comanda a executar.
                stdin=subprocess.PIPE,                                          # podrem escriure al daemon (per enviar comandes).
                stdout=subprocess.PIPE,                                         # podrem llegir el que el daemon escriu (per rebre feedback).
                stderr=subprocess.PIPE,                                         # podrem llegir els errors del daemon.
                text=True,
                encoding="utf-8",
                bufsize=1,
                env=env,
                creationflags=creationflags,
            )
        except Exception as e:
            raise RuntimeError(f"No s'ha pogut arrencar el daemon: {e}") from e


        self._stdout_thread = threading.Thread(target=self._read_stdout, daemon=True)   # Crea un thread (fil d'execució paral·lel) que executarà la funció _read_stdout. daemon=True vol dir que es tancarà automàticament quan el procés principal acabi.
        self._stdout_thread.start()                                                     # engega el thread. Des d'aquest moment, _read_stdout corre en paral·lel llegint contínuament el que el daemon escriu per stdout.
        self._stderr_thread = threading.Thread(target=self._read_stderr, daemon=True)   # IGUAL PELS ERRORS
        self._stderr_thread.start()                                             


        if not self._ready.wait(self.READY_TIMEOUT_S):                          # Bloqueja el fil principal fins que algú faci _ready.set() o passin 8 segons.
            self.close(quiet=True)
            if self._error:
                raise RuntimeError(f"Daemon no ready: {self._error}")
            raise RuntimeError( "Daemon no ha respost dins del timeout. ")

        if self._error:
            self.close(quiet=True)
            raise RuntimeError(f"Error del daemon: {self._error}")





    def _read_stdout(self) -> None:                                             # LECTURA DEL STDOUT
        if self._proc is None or self._proc.stdout is None:                     # Protecció: si el procés no existeix, surt.
            return
        try:
            for line in self._proc.stdout:                                      # Bucle que llegeix línia a línia de la sortida del daemon. Si no hi ha res a llegir, es bloqueja esperant.
                if self._closed.is_set():                                       # Si algú ha marcat que tanquem, surt del bucle.
                    break
                line = line.strip()                                             # Treu salts de línia i espais. Si la línia és buida, salta.
                if not line:
                    continue
                try:
                    msg = json.loads(line)                                      # Intenta parsejar la línia com a JSON. Si no és JSON vàlid (bug, línia corrupta), salta silenciosament.
                except Exception:
                    continue

                ev = msg.get("event")                                           # Agafa el tipus d'event del dict. Pot ser "ready", "status", "error", "ack" o "closed".
                if ev == "ready":
                    self._ready.set()                                           # El daemon diu que està llest → activa l'event per desbloquejar el constructor que estava esperant.
                elif ev == "status":
                    with self._lock:
                        for m in msg.get("motors", []):                         # El daemon envia estat dels motors.
                            mid = m.get("id")
                            if mid is not None:
                                self._latest[mid] = m
                elif ev == "error":                                             # El daemon dona un error. Guarda el missatge si no n'hi havia cap. Desbloqueja el constructor si estava esperant (perquè pugui llançar l'excepció).
                    if self._error is None:
                        self._error = msg.get("msg", "unknown")
                    if not self._ready.is_set():
                        self._ready.set()
                elif ev == "ack":
                    pass
                elif ev == "closed":                                            # El daemon ha tancat. Sortim del bucle.
                    break
        finally:
            self._closed.set()                                                  # Marca com a tancat i desbloqueja qualsevol fil que estigui esperant.
            self._ready.set()


    def _read_stderr(self) -> None:                                             # LECTURA DEL STDERR
        if self._proc is None or self._proc.stderr is None:
            return
        try:
            for line in self._proc.stderr:
                if not line:
                    break
                sys.stderr.write(line)
                sys.stderr.flush()                                              # Reenvia el stderr del daemon tal qual (ja porta el prefix [motor-daemon]). Ex: [motor-daemon] obrint tcp://192.168.1.140:5000...
        except Exception:
            pass





    def send(self, cmd: dict) -> None:                                          # Envia una comanda JSON al daemon per stdin.
        if self._closed.is_set() or self._proc is None:                         # Si estem tancats, no enviem res.
            return
        try:
            line = json.dumps(cmd) + "\n"                                       # Converteix el dict a JSON, afegeix \n, i l'escriu al stdin del daemon.
            self._proc.stdin.write(line)
            self._proc.stdin.flush()
        except (BrokenPipeError, OSError) as e:                                 # Si el pipe està trencat (daemon mort), guarda error i marca tancat.
            self._error = f"error d'escriptura: {e}"
            self._closed.set()                                                  # executem set()

    def latest_status(self) -> Dict[int, dict]:                                 # Retorna una còpia de l'estat de tots els motors
        with self._lock:
            return dict(self._latest)

    def alive(self) -> bool:                                                    # Està viu?
        return (self._proc is not None
                and self._proc.poll() is None
                and not self._closed.is_set())

    def close(self, quiet: bool = False) -> None:                               # tura el daemon de manera controlada.
        if self._closed.is_set() and self._proc is None:                        # Si ja està tancat, no fer res.
            return
        try:
            self.send({"cmd": "stop"})                                          # Envia la comanda stop al daemon, que tanca el port/TCP i surt. El daemon no envia RESET (en tancar l'app, el RESET el fa abans RobotWorkspace.cleanup).
        except Exception:
            pass
        if self._proc is not None:
            try:
                self._proc.wait(timeout=2.0)                                    # Espera fins a 2 segons que el procés acabi netament. Si no acaba, el mata amb kill() (SIGKILL).
            except subprocess.TimeoutExpired:
                try:
                    self._proc.kill()
                except Exception:
                    pass
        self._closed.set()





# ==============================================================================
# MOTORBUS - Classe pública
# El motor_controller.py fa servir només aquesta classe.
# ==============================================================================

class MotorBus:

    def __init__(self, motor_ids: Sequence[int], port: Optional[str] = None, baudrate: int = 921600, timeout_s: float = 0.05) -> None:
        self.motor_ids = list(motor_ids)                                        # Guarda els ids, el port, un lock, i crea un MotorStatus buit per cada motor on anirem acumulant el feedback.
        self.port = port
        self.lock = threading.Lock()
        self.status: Dict[int, MotorStatus] = {mid: MotorStatus(motor_id=mid) for mid in self.motor_ids}

        if port is None:                                                        # Tria el backend: sense port = simulador.   Amb port = daemon real.
            self._sim: Optional[_SimBackend] = _SimBackend(self.motor_ids)      # El simulador emmagatzemarà les dades directament a feed_command i no passaran pels motors sitodrive.
            self._daemon: Optional[_DaemonBackend] = None
        else:
            self._sim = None
            self._daemon = _DaemonBackend(port, baudrate, self.motor_ids)




    def close(self) -> None:                                                    # Tanca el daemon si existia. El simulador no cal tancar-lo (no té I/O).
        if self._daemon is not None:
            self._daemon.close()

    def reset_all(self) -> None:                                                # Envia RESET a tots els motors (LED blau). Daemon: una sola comanda JSON. Simulador: un paquet encode per cada motor.
        with self.lock:
            if self._daemon is not None:
                self._daemon.send({"cmd": "reset"})
            else:
                for mid in self.motor_ids:
                    self._sim.feed_command(encode_reset(mid))



    def select_mode(self, mode: Mode, fb_cycle1: int = 10, fb_cycle2: int = 10, fb_cycle3: int = 0) -> None:
        with self.lock:                                                         # Mateix patró: envia JSON al daemon o un paquet per motor al simulador.
            if self._daemon is not None:
                self._daemon.send({
                    "cmd": "select_mode",
                    "mode": mode.name,
                    "fb1": fb_cycle1,
                    "fb2": fb_cycle2,
                    "fb3": fb_cycle3,
                })
            else:
                for mid in self.motor_ids:
                    self._sim.feed_command(encode_mode_selection(mid, mode, fb_cycle1, fb_cycle2, fb_cycle3))


    def write_positions(self, current_mA: Sequence[int], velocity: Sequence[int], positions: Sequence[int]) -> None:
        with self.lock:
            if self._daemon is not None:
                self._daemon.send({
                    "cmd": "write_profile_position",
                    "current":   [int(x) for x in current_mA],
                    "velocity":  [int(x) for x in velocity],
                    "positions": [int(x) for x in positions],
                })
            else:
                for mid, c, v, p in zip(self.motor_ids, current_mA, velocity, positions):
                    self._sim.feed_command(encode_profile_position(mid, int(c), int(v), int(p), Mode.PROFILE_POSITION_MODE))

    def write_positions_pure(self, current_mA: Sequence[int], velocity: Sequence[int], positions: Sequence[int]) -> None:
        with self.lock:
            if self._daemon is not None:
                self._daemon.send({
                    "cmd": "write_position",
                    "current":   [int(x) for x in current_mA],
                    "velocity":  [int(x) for x in velocity],
                    "positions": [int(x) for x in positions],
                })
            else:
                for mid, c, v, p in zip(self.motor_ids, current_mA, velocity, positions):
                    self._sim.feed_command(encode_profile_position(mid, int(c), int(v), int(p), Mode.POSITION_MODE))

    def write_mit(self, current_mA: Sequence[int], velocity: Sequence[int], positions: Sequence[int], kp: Sequence[float], ki: Sequence[float]) -> None:
        with self.lock:
            if self._daemon is not None:
                self._daemon.send({
                    "cmd": "write_mit",
                    "current":   [int(x) for x in current_mA],
                    "velocity":  [int(x) for x in velocity],
                    "positions": [int(x) for x in positions],
                    "kp":        [float(x) for x in kp],
                    "ki":        [float(x) for x in ki],
                })
            else:
                for mid, c, v, p in zip(self.motor_ids, current_mA, velocity, positions):
                    self._sim.feed_command(encode_mit_position(mid, int(c), int(v), int(p)))

    def write_currents(self, currents_mA: Sequence[int]) -> None:
        with self.lock:
            if self._daemon is not None:
                self._daemon.send({
                    "cmd": "write_currents",
                    "currents": [int(x) for x in currents_mA],
                })
            else:
                for mid, c in zip(self.motor_ids, currents_mA):
                    self._sim.feed_command(encode_current(mid, int(c)))

    def write_velocities(self, currents_mA: Sequence[int], velocities: Sequence[int]) -> None:
        with self.lock:
            if self._daemon is not None:
                self._daemon.send({
                    "cmd": "write_velocities",
                    "current":  [int(x) for x in currents_mA],
                    "velocity": [int(x) for x in velocities],
                })
            else:
                for mid, c, v in zip(self.motor_ids, currents_mA, velocities):
                    self._sim.feed_command(encode_profile_velocity(mid, int(c), int(v)))




    def poll_feedback(self) -> Dict[int, MotorStatus]:                          # Retorna el diccionari {motor_id: MotorStatus} amb l'estat actual. Això és el que crida el handguide 100 cops/segon per llegir velocitat, corrent, posició.
        if self._daemon is not None:
            latest = self._daemon.latest_status()
            for mid in self.motor_ids:
                d = latest.get(mid)
                if d is None:
                    continue
                s = self.status[mid]
                s.actuator_position = int(d.get("pos", 0))
                s.motor_position    = int(d.get("motor_pos", 0))
                s.current_mA        = int(d.get("curr", 0))
                s.velocity          = int(d.get("vel", 0))
                s.pwm               = int(d.get("pwm", 0))
                s.voltage_mV        = int(d.get("volt", 0))
                s.mos_temp          = int(d.get("mos_temp", 0))
                s.mt_temp           = int(d.get("mt_temp", 0))
                s.g431_temp         = int(d.get("g431_temp", 0))
                s.warning_type      = int(d.get("warn", 0))
                s.error_type        = int(d.get("err", 0))
                s.timestamp         = time.monotonic()
            return dict(self.status)


        with self.lock:
            chunks: List[bytes] = []
            for mid in self.motor_ids:
                chunks.extend(self._sim.synthesize_feedback(mid, [int(FeedbackType.VOLT_PWM_CUR_VEL), int(FeedbackType.POSITIONS), int(FeedbackType.TEMP_FAULTS)]))
            buf = b''.join(chunks)
            pkts, _ = parse_feedback(buf)
            for mid, ft, payload in pkts:
                if mid in self.status:
                    update_status_from_packet(self.status[mid], ft, payload)
        return dict(self.status)