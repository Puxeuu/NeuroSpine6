import re
import time
import math

from qtpy import QtCore

try:
    import serial
    _HAS_SERIAL = True
except ImportError:
    _HAS_SERIAL = False


PULSOS_PER_VOLTA = 1200.0
CONV_GRAUS = 360.0 / PULSOS_PER_VOLTA
CONV_MM    = (math.pi * 35.0) / PULSOS_PER_VOLTA


ORDRE_E_A_POLS = (1, 5, 3, 4, 0, 2)

_RE_ENCODER = re.compile(r"E([1-6])\s*:\s*(?:[^|]*?p=)?(-?\d+)")


def parse_mando_line(line):
    if not line:
        return None
    trobats = _RE_ENCODER.findall(line)
    if trobats:
        e = {int(n): int(v) for n, v in trobats}
        if len(e) < 6:
            return None
        e_list = [e[i] for i in range(1, 7)]
        return [e_list[i] for i in ORDRE_E_A_POLS]
    if "|" in line:
        parts = [p for p in line.split("|") if ":" in p]
        if len(parts) < 6:
            return None
        try:
            return [int(p.split(":")[1].strip()) for p in parts[:6]]
        except ValueError:
            return None
    return None


class MandoService(QtCore.QObject):
    reading         = QtCore.Signal(dict)
    connection_ok   = QtCore.Signal(bool)

    def __init__(self, port="COM5", baud=115200, poll_ms=50, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.poll_ms = poll_ms

        self._serial = None
        self._connected = False
        self._buf = b""
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._poll)

    def start(self):
        if not _HAS_SERIAL:
            print("[MandoService] pyserial no disponible — mode simulació")
            self.connection_ok.emit(False)
            return False

        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0)
            self._connected = True
            self.connection_ok.emit(True)
            self._timer.start(self.poll_ms)
            print(f"[MandoService] Connectat a {self.port} @ {self.baud}")
            return True
        except Exception as e:
            print(f"[MandoService] No es pot obrir {self.port}: {e}")
            self._connected = False
            self.connection_ok.emit(False)
            return False

    def stop(self):
        self._timer.stop()
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self._connected = False
        self._buf = b""

    def is_connected(self):
        return self._connected

    def _poll(self):
        if not self._serial or self._serial.in_waiting == 0:
            return

        try:
            self._buf += self._serial.read(self._serial.in_waiting)
            if b"\n" not in self._buf:
                return
            *lines, self._buf = self._buf.split(b"\n")

            pols = None
            for raw in reversed(lines):
                pols = parse_mando_line(raw.decode("utf-8", errors="ignore").strip())
                if pols is not None:
                    break
            if pols is None:
                return

            rad_x = math.radians(pols[0] * CONV_GRAUS)
            rad_y = math.radians(-pols[1] * CONV_GRAUS)
            rad_z = math.radians(pols[2] * CONV_GRAUS)
            raw_x = pols[4] * CONV_MM
            raw_y = pols[3] * CONV_MM
            raw_z = pols[5] * CONV_MM

            self.reading.emit({
                "pols":       pols,
                "rad_x":      rad_x, "rad_y": rad_y, "rad_z": rad_z,
                "raw_x":      raw_x, "raw_y": raw_y, "raw_z": raw_z,
                "timestamp":  time.time(),
            })
        except Exception as e:
            if self._connected:
                print(f"[MandoService] Error de lectura: {e}")
                self._connected = False
                self.connection_ok.emit(False)


_INSTANCE = None


def get_mando_service(port="COM5", baud=115200):
    global _INSTANCE
    if _INSTANCE is None:
        _INSTANCE = MandoService(port=port, baud=baud)
    return _INSTANCE

