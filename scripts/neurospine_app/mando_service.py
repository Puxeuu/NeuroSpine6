"""
Servei únic de lectura del mando Arduino, compartit per tots els workspaces.

MOTIVACIÓ:
    Abans, el viewer_cobot i el Visualitzador cadascun obria el seu propi
    port serie (COM3 i COM6 respectivament) i tenia el seu propi timer.
    Això funcionava perquè eren scripts independents.

    Ara que tots dos són workspaces dins la mateixa app, unifiquem la
    lectura en un únic servei: obre el port un cop, llegeix a 20 Hz, i
    emet un signal Qt que els subscriptors poden connectar per rebre les
    dades en temps real.

DADES QUE PUBLICA (via signal 'reading'):
    dict amb claus:
        pols_j1..j6    counts crus dels 6 encoders (int)
        rad_x, y, z    orientació XYZ del mando en radians
        raw_x, y, z    translació XYZ del mando en mm (bruts)
        timestamp      time.time() de la lectura

Els workspaces apliquen els seus propis offsets/conversions sobre aquestes
dades brutes (planning ho reprojecta sobre el TAC, robot ho passa al DLS).

FALLBACK MODE:
    Si el port no s'obre, el servei entra en mode simulació silenciosa:
    no emet cap dada. Els workspaces han de gestionar-ho i mostrar
    "MODE SIMULACIÓ" a la UI.
"""

import re                                                                       # importem el mòdul re
import time                                                                     # importem el mòdul time
import math                                                                     # importem el mòdul math

from qtpy import QtCore                                                         # importem QtCore des de qtpy

try:                                                                            # intentem el bloc següent
    import serial                                                               # importem el mòdul serial
    _HAS_SERIAL = True                                                          # assignem _HAS_SERIAL
except ImportError:                                                             # capturem excepció: ImportError
    _HAS_SERIAL = False                                                         # assignem _HAS_SERIAL


# Constants d'encoder (del mando actual, corresponen a viewer_cobot i Visualitzador)
PULSOS_PER_VOLTA = 1200.0                                                       # assignem PULSOS_PER_VOLTA
CONV_GRAUS = 360.0 / PULSOS_PER_VOLTA                                           # assignem CONV_GRAUS
CONV_MM    = (math.pi * 35.0) / PULSOS_PER_VOLTA                                # assignem CONV_MM


# ------------------------------------------------------------------------------
# FORMAT DEL MANDO NOU (COM5)
# ------------------------------------------------------------------------------
# L'Arduino envia (només quan hi ha canvi):
#     "E1:     123   E2:     -45   E3:       0   E4: ...   E6:      12"
# E1 = punta/capçal ... E6 = base (ordre de la cadena, de dalt a baix).
#
# Segons la cadena del URDF (mando6dof.urdf, base -> punta):
#     j1/j2 Z  ->  j3 X  ->  j4 Y  ->  j5 Rz  ->  j6 Ry  ->  j7 Rx
# per tant:  E6=Z  E5=X  E4=Y  E3=Rz  E2=Ry  E1=Rx
#
# La resta del codi (viewer_cobot, workspaces) treballa amb l'ordre antic
# de 'pols' (slot0=Ry, slot1=Z, slot2=Y, slot3=X, slot4=Rx, slot5=Rz).
# ORDRE_E_A_POLS[k] = índex de l'encoder E (0-based) que va al slot k.
# Si algun eix surt canviat, només cal tocar aquesta taula.
ORDRE_E_A_POLS = (1, 5, 3, 4, 0, 2)                                             # slot k ← E[ORDRE_E_A_POLS[k]] (E2,E6,E4,E5,E1,E3)

# Accepta "E1:    123" i també el sketch de test "E1: A1 B0 I=57 p=123  |"
_RE_ENCODER = re.compile(r"E([1-6])\s*:\s*(?:[^|]*?p=)?(-?\d+)")                # captura "E<n>: <valor>" o "E<n>: ... p=<valor>"


def parse_mando_line(line):                                                     # definim la funció parse_mando_line()
    """
    Parseja una línia del mando. Torna la llista de 6 'pols' en l'ordre
    que espera la resta del codi, o None si la línia no és vàlida.
    Accepta també el format antic ("X: n|Y: n|...") per compatibilitat.
    """
    if not line:                                                                # línia buida
        return None
    trobats = _RE_ENCODER.findall(line)                                         # [(n, valor), ...]
    if trobats:                                                                 # format nou E1..E6
        e = {int(n): int(v) for n, v in trobats}                                # dict n -> valor
        if len(e) < 6:                                                          # línia incompleta
            return None
        e_list = [e[i] for i in range(1, 7)]                                    # [E1..E6]
        return [e_list[i] for i in ORDRE_E_A_POLS]                              # reordenem a l'ordre antic
    if "|" in line:                                                             # format antic
        parts = [p for p in line.split("|") if ":" in p]                        # assignem parts
        if len(parts) < 6:
            return None
        try:
            return [int(p.split(":")[1].strip()) for p in parts[:6]]
        except ValueError:
            return None
    return None


class MandoService(QtCore.QObject):                                             # definim la classe MandoService
    """
    Servei singleton per a la lectura del mando 6-DOF.

    Ús:
        service = MandoService(port='COM5', baud=115200)
        service.reading.connect(my_callback)     # rep un dict per cada lectura
        service.start()
        ...
        service.stop()
    """

    # Signals que els workspaces poden escoltar
    reading         = QtCore.Signal(dict)       # nova lectura del mando
    connection_ok   = QtCore.Signal(bool)       # True quan connecta, False quan cau

    def __init__(self, port="COM5", baud=115200, poll_ms=50, parent=None):      # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.port = port                                                        # assignem self.port
        self.baud = baud                                                        # assignem self.baud
        self.poll_ms = poll_ms                                                  # assignem self.poll_ms

        self._serial = None                                                     # assignem self._serial
        self._connected = False                                                 # assignem self._connected
        self._buf = b""                                                         # bytes pendents (línia a mitges)
        self._timer = QtCore.QTimer(self)                                     # self._timer ← resultat de QTimer()
        self._timer.timeout.connect(self._poll)                                 # executem connect()

    # --------------------------------------------------------------------
    def start(self):                                                            # definim la funció start()
        """Obre el port i comença el polling. Torna True si connecta."""        # """Obre el port i comença el polling. Torna True si conne...
        if not _HAS_SERIAL:                                                     # si not _HAS_SERIAL
            print("[MandoService] pyserial no disponible — mode simulació")     # executem print()
            self.connection_ok.emit(False)                                      # executem emit()
            return False                                                        # retornem: False

        try:                                                                    # intentem el bloc següent
            self._serial = serial.Serial(self.port, self.baud, timeout=0)       # self._serial ← resultat de Serial()
            self._connected = True                                              # assignem self._connected
            self.connection_ok.emit(True)                                       # executem emit()
            self._timer.start(self.poll_ms)                                     # executem start()
            print(f"[MandoService] Connectat a {self.port} @ {self.baud}")
            return True                                                         # retornem: True
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[MandoService] No es pot obrir {self.port}: {e}")
            self._connected = False                                             # assignem self._connected
            self.connection_ok.emit(False)                                      # executem emit()
            return False                                                        # retornem: False

    def stop(self):                                                             # definim la funció stop()
        """Atura el polling i tanca el port."""                                 # """Atura el polling i tanca el port."""
        self._timer.stop()                                                      # executem stop()
        if self._serial:                                                        # si self._serial
            try:                                                                # intentem el bloc següent
                self._serial.close()                                            # executem close()
            except Exception:                                                   # capturem excepció: Exception
                pass                                                            # no fem res (placeholder)
        self._serial = None                                                     # assignem self._serial
        self._connected = False                                                 # assignem self._connected
        self._buf = b""                                                         # buidem el buffer

    def is_connected(self):                                                     # definim la funció is_connected()
        return self._connected                                                  # retornem: self._connected

    # --------------------------------------------------------------------
    def _poll(self):                                                            # definim la funció _poll()
        """Llegeix una línia del port serie si n'hi ha, i emet la dada."""      # """Llegeix una línia del port serie si n'hi ha, i emet la...
        if not self._serial or self._serial.in_waiting == 0:                    # si not self._serial or self._serial.in_w...
            return                                                              # retornem sense valor

        try:                                                                    # intentem el bloc següent
            self._buf += self._serial.read(self._serial.in_waiting)             # afegim el que hagi arribat
            if b"\n" not in self._buf:                                          # encara no hi ha cap línia sencera
                return                                                          # retornem sense valor
            *lines, self._buf = self._buf.split(b"\n")                          # l'últim tros pot ser una línia a mitges

            # Prenem la última línia vàlida (la més recent, descartem el backlog)
            pols = None                                                         # assignem pols
            for raw in reversed(lines):                                         # de la més nova a la més vella
                pols = parse_mando_line(raw.decode("utf-8", errors="ignore").strip())
                if pols is not None:
                    break
            if pols is None:                                                    # cap línia vàlida (ex: "Inicialitzant...")
                return                                                          # retornem sense valor

            # Conversions estàndard (mateixes que fan viewer_cobot i Visualitzador)
            rad_x = math.radians(pols[0] * CONV_GRAUS)                          # rad_x ← resultat de radians()
            rad_y = math.radians(-pols[1] * CONV_GRAUS)                         # rad_y ← resultat de radians()
            rad_z = math.radians(pols[2] * CONV_GRAUS)                          # rad_z ← resultat de radians()
            raw_x = pols[4] * CONV_MM                                           # assignem raw_x
            raw_y = pols[3] * CONV_MM                                           # assignem raw_y
            raw_z = pols[5] * CONV_MM                                           # assignem raw_z

            self.reading.emit({                                                 # executem emit()
                "pols":       pols,
                "rad_x":      rad_x, "rad_y": rad_y, "rad_z": rad_z,
                "raw_x":      raw_x, "raw_y": raw_y, "raw_z": raw_z,
                "timestamp":  time.time(),
            })
        except Exception as e:                                                  # capturem excepció: Exception as e
            # No fem soroll a cada error de lectura; només si es cau la connexió
            if self._connected:                                                 # si self._connected
                print(f"[MandoService] Error de lectura: {e}")
                self._connected = False                                         # assignem self._connected
                self.connection_ok.emit(False)                                  # executem emit()


# ==============================================================================
# SINGLETON GLOBAL
# ==============================================================================
#
# Els workspaces accedeixen al mando via get_mando_service(). La primera
# crida crea la instància; les següents retornen la mateixa. Això garanteix
# una sola connexió al port serie.

_INSTANCE = None                                                                # assignem _INSTANCE


def get_mando_service(port="COM5", baud=115200):                                # definim la funció get_mando_service()
    """Retorna la instància única del servei (crea-la si cal)."""               # """Retorna la instància única del servei (crea-la si cal)...
    global _INSTANCE                                                            # declarem global: _INSTANCE
    if _INSTANCE is None:                                                       # si _INSTANCE is None
        _INSTANCE = MandoService(port=port, baud=baud)                          # _INSTANCE ← resultat de MandoService()
    return _INSTANCE                                                            # retornem: _INSTANCE
