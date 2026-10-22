import time

import numpy as np
from qtpy import QtCore

from src.hardware.sitodrive import (
    Mode, encode_current, encode_mode_selection, encode_position, encode_reset,
)


J6_MOTOR_ID      = 6
TICK_HZ          = 20
SLEW_MA_PER_S    = 4000.0
VEL_STOPPED      = 50
STOPPED_READS    = 3
POS_STILL_COUNTS = 30
STOP_TIMEOUT_S   = 4.0
FRESH_TIMEOUT_S  = 0.3
RESET_SETTLE_S   = 0.3
WATCH_S          = 0.3
WATCH_MAX_COUNTS = 65536 // 16
NO_RESPONSE_S    = 1.5
PARK_CURRENT_MA  = 300
PARK_VELOCITY    = 1
ENC_PER_REV      = 65536


def _first(*vals):
    return next((v for v in vals if v is not None), None)


def _wrap(a):
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


def _j6_cfg(mc):
    for i, j in enumerate(mc.config.joints):
        if j.motor_id == J6_MOTOR_ID:
            return i, j
    raise RuntimeError("motor_config no té cap joint amb motor_id 6")


def _send_raw(bus, pkt):
    if bus._daemon is not None:
        bus._daemon.send({"cmd": "raw_bytes", "bytes": list(pkt)})
    else:
        bus._sim.feed_command(pkt)


def _daemon_entry(bus):
    return bus._daemon.latest_status().get(J6_MOTOR_ID) or {}


def latest_position(bus):
    if bus._daemon is None:
        st = bus.poll_feedback().get(J6_MOTOR_ID)
        return int(st.actuator_position) if st is not None else None
    d = _daemon_entry(bus)
    if float(d.get("last_b2", 0.0)) <= 0.0:
        return None
    return int(d.get("pos", 0))


def read_fresh_position(bus, timeout_s=FRESH_TIMEOUT_S):
    if bus._daemon is None:
        return latest_position(bus)
    t0 = float(_daemon_entry(bus).get("last_b2", 0.0))
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        time.sleep(0.01)
        d = _daemon_entry(bus)
        t = float(d.get("last_b2", 0.0))
        if t > t0 and t > 0.0:
            return int(d.get("pos", 0))
    return None


def fw_wrap(counts, enc=ENC_PER_REV):
    half = enc // 2
    return int(((int(counts) + half) % enc) - half)


_FW_WRAPS = None


def fw_frame(counts):
    return int(counts) if _FW_WRAPS is False else fw_wrap(counts)


def learn_frame(raw, measured):
    global _FW_WRAPS
    if raw is None or fw_wrap(raw) == int(raw):
        return
    half = ENC_PER_REV // 2
    if abs(measured - fw_wrap(raw)) < half:
        new = True
    elif abs(measured - raw) < half:
        new = False
    else:
        return
    if new != _FW_WRAPS:
        print(f"[J6] firmware: després d'un RESET el comptador "
              f"{'torna a una volta' if new else 'manté les voltes'}")
        _FW_WRAPS = new


def q_from_counts(cfg, counts):
    return (counts - cfg.zero_offset_counts) / cfg.encoder_counts_per_rev * 2.0 * np.pi / cfg.direction


def frame_fix(target, measured, enc=ENC_PER_REV):
    n = int(round((measured - target) / enc))
    return target + n * enc


def _exclude_j6(mc, on):
    excl = [m for m in (mc._auto_exclude_motor_ids or []) if m != J6_MOTOR_ID]
    mc._auto_exclude_motor_ids = excl + [J6_MOTOR_ID] if on else excl


def j6_to_position(viewer, pos, settle_from=None, watch=True, post_reset=False,
                   raw=None):
    mc = viewer.motor_controller
    bus = mc.bus
    idx, cfg = _j6_cfg(mc)

    if settle_from is None:
        _send_raw(bus, encode_current(J6_MOTOR_ID, 0))
        _send_raw(bus, encode_reset(J6_MOTOR_ID))
        settle_from = time.monotonic()
    wait = RESET_SETTLE_S - (time.monotonic() - settle_from)
    if wait > 0:
        time.sleep(wait)

    target = int(pos) if post_reset else fw_frame(pos)
    raw_pre = raw if post_reset else pos

    def mk(t, weak=False):
        if weak:
            return encode_position(J6_MOTOR_ID, PARK_CURRENT_MA, PARK_VELOCITY, int(t))
        return encode_position(J6_MOTOR_ID, int(cfg.default_current_mA),
                               int(cfg.default_velocity_counts), int(t))

    verify = watch and bus._daemon is not None
    pkt = mk(target, weak=verify)
    t_switch = float(_daemon_entry(bus).get("last_b2", 0.0)) if bus._daemon is not None else 0.0
    _send_raw(bus, pkt)
    _send_raw(bus, encode_mode_selection(J6_MOTOR_ID, Mode.POSITION_MODE,
                                         fb_cycle1=10, fb_cycle2=10, fb_cycle3=0))
    _send_raw(bus, pkt)
    _send_raw(bus, pkt)
    for _ in range(10):
        time.sleep(0.01)
        _send_raw(bus, pkt)

    if watch and bus._daemon is not None:
        worst, seen = 0, False
        t_b2 = t_switch
        t_end = time.monotonic() + WATCH_S
        while time.monotonic() < t_end:
            time.sleep(0.01)
            _send_raw(bus, pkt)
            d = _daemon_entry(bus)
            if float(d.get("last_b2", 0.0)) <= t_b2:
                continue
            t_b2 = float(d.get("last_b2", 0.0))
            p = int(d.get("pos", 0))
            if not seen:
                learn_frame(raw_pre, p)
                fixed = frame_fix(target, p)
                if fixed != target:
                    print(f"[J6] marc del comptador corregit: target {target} → {fixed} (mesurat {p})")
                    target = fixed
                pkt = mk(target)
                _send_raw(bus, pkt)
            seen = True
            worst = max(worst, abs(p - target))
            if worst > WATCH_MAX_COUNTS:
                _send_raw(bus, encode_reset(J6_MOTOR_ID))
                _exclude_j6(mc, True)
                print(f"[J6] SEGURETAT: s'ha mogut {worst} counts en passar a POSITION "
                      f"→ RESET (lliure). Passa scripts/diagnostic_j6.py")
                return False
        if seen:
            print(f"[J6] POSITION OK · desviació màxima {worst} counts")
        else:
            print("[J6] avís: cap feedback després de passar a POSITION (el motor no ha canviat de mode?)")

    q5 = q_from_counts(cfg, target)
    viewer.q[idx] = q5
    viewer._j6_ik_override = None
    _exclude_j6(mc, False)
    if hasattr(viewer, "sync_target_to_current"):
        viewer.sync_target_to_current()
    print(f"[J6] fixat en POSITION a {target} counts · q={q5:+.3f} rad ({np.degrees(q5):+.1f}°)")
    return True


def ensure_j6_position_hold(viewer):
    mc = getattr(viewer, "motor_controller", None)
    if mc is None or not mc.is_connected or mc.bus is None:
        return
    if J6_MOTOR_ID not in (mc._auto_exclude_motor_ids or []):
        return
    pos = _first(read_fresh_position(mc.bus), latest_position(mc.bus))
    if pos is None:
        print("[J6] fora del send_q i sense posició coneguda: el deixo com està")
        return
    print("[J6] era fora del send_q en canviar de mode → el torno a POSITION on és")
    j6_to_position(viewer, pos)


class J6Drill(QtCore.QObject):
    stateChanged = QtCore.Signal(str)

    OFF, ARMED, RUNNING, STOPPING = "off", "armed", "running", "stopping"

    def __init__(self, viewer, setpoint_fn=None, parent=None):
        super().__init__(parent)
        self.viewer = viewer
        self._setpoint_fn = setpoint_fn
        self.state = self.OFF
        self.saved_pos = None
        self._t_reset = 0.0
        self._setpoint = 0
        self._cmd = 0.0
        self._zero_reads = 0
        self._last_b1 = 0.0
        self._last_pos = None
        self._t_stop = 0.0
        self._t_start = 0.0
        self._start_pos = None
        self._warned_no_response = False
        self._park_raw = None
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)

    @property
    def spinning(self):
        return self.state in (self.RUNNING, self.STOPPING)

    def _bus(self):
        mc = getattr(self.viewer, "motor_controller", None)
        if mc is None or not mc.is_connected or mc.bus is None or mc.bus._daemon is None:
            return None, None
        return mc, mc.bus

    def _set_state(self, s):
        if s != self.state:
            self.state = s
            print(f"[J6] estat → {s}")
            self.stateChanged.emit(s)

    def arm(self):
        if self.state != self.OFF:
            return
        idx = _j6_cfg(self.viewer.motor_controller)[0]
        self.viewer._j6_ik_override = float(self.viewer.q[idx])
        mc, bus = self._bus()
        if bus is None:
            self.state = self.ARMED
            return
        _exclude_j6(mc, True)
        pos = _first(read_fresh_position(bus), latest_position(bus))
        if pos is not None:
            self.saved_pos = fw_frame(pos)
            weak = encode_position(J6_MOTOR_ID, PARK_CURRENT_MA, PARK_VELOCITY, int(pos))
            for _ in range(3):
                _send_raw(bus, weak)
                time.sleep(0.01)
        _send_raw(bus, encode_reset(J6_MOTOR_ID))
        self._t_reset = time.monotonic()
        self._set_state(self.ARMED)
        print(f"[J6] IK5 · J6 en RESET (blau), posició guardada {self.saved_pos}")

    def disarm(self):
        if self.spinning:
            self.finish_now()
        if self.state == self.OFF:
            return
        self._timer.stop()
        mc, bus = self._bus()
        if bus is None:
            self.viewer._j6_ik_override = None
            self._set_state(self.OFF)
            return
        pos = read_fresh_position(bus, timeout_s=0.15)
        if pos is None:
            pos = self.saved_pos if self.saved_pos is not None else latest_position(bus)
        if pos is None:
            print("[J6] ERROR: sense posició del J6; queda en RESET i fora del send_q")
            self.viewer._j6_ik_override = None
            self._set_state(self.OFF)
            return
        ok = j6_to_position(self.viewer, pos, settle_from=self._t_reset,
                            post_reset=True, raw=self._park_raw)
        if ok and hasattr(self.viewer, "set_j6_center"):
            idx = _j6_cfg(mc)[0]
            self.viewer.set_j6_center(self.viewer.q[idx])
        self._park_raw = None
        self.saved_pos = None
        self._set_state(self.OFF)

    def start(self, current_mA):
        if self.state == self.RUNNING:
            self._setpoint = int(current_mA)
            return True
        if self.state == self.STOPPING:
            self._setpoint = int(current_mA)
            self._set_state(self.RUNNING)
            return True
        if self.state != self.ARMED:
            print("[J6] Start ignorat: cal estar en IK5")
            return False
        mc, bus = self._bus()
        if bus is None:
            return False

        wait = RESET_SETTLE_S - (time.monotonic() - self._t_reset)
        if wait > 0:
            time.sleep(wait)

        _send_raw(bus, encode_mode_selection(J6_MOTOR_ID, Mode.CURRENT_MODE,
                                             fb_cycle1=10, fb_cycle2=10, fb_cycle3=0))
        time.sleep(0.05)
        self._cmd = 0.0
        self._setpoint = int(current_mA)
        self._step_current(self._setpoint, bus)
        self._t_start = time.monotonic()
        self._start_pos = self.saved_pos
        self._warned_no_response = False
        self._set_state(self.RUNNING)
        self._timer.start(int(1000 / TICK_HZ))
        print(f"[J6] START · rampa fins a {self._setpoint} mA")
        return True

    def set_current(self, current_mA):
        if self.state == self.RUNNING:
            self._setpoint = int(current_mA)

    def stop(self):
        if self.state != self.RUNNING:
            return
        self._zero_reads, self._last_pos = 0, None
        self._t_stop = time.monotonic()
        self._set_state(self.STOPPING)

    def finish_now(self, timeout_s=STOP_TIMEOUT_S):
        if not self.spinning:
            return
        self._timer.stop()
        mc, bus = self._bus()
        if bus is None:
            self._set_state(self.ARMED)
            return
        self._set_state(self.STOPPING)
        t0 = time.monotonic()
        self._zero_reads, self._last_pos = 0, None
        while time.monotonic() - t0 < timeout_s:
            self._step_current(0.0, bus)
            if self._cmd == 0.0 and self._motor_quiet(bus):
                break
            time.sleep(1.0 / TICK_HZ)
        self._park()

    def abort(self):
        self._timer.stop()
        if self.state == self.OFF:
            return
        mc, bus = self._bus()
        if bus is not None:
            try:
                _send_raw(bus, encode_current(J6_MOTOR_ID, 0))
                if self.state in (self.RUNNING, self.STOPPING):
                    self._zero_reads, self._last_pos = 0, None
                    t_end = time.monotonic() + 1.5
                    while time.monotonic() < t_end and not self._motor_quiet(bus):
                        _send_raw(bus, encode_current(J6_MOTOR_ID, 0))
                        time.sleep(0.05)
                    pos = _first(read_fresh_position(bus), latest_position(bus), self.saved_pos)
                else:
                    pos = _first(read_fresh_position(bus, timeout_s=0.1), self.saved_pos,
                                 latest_position(bus))
                _send_raw(bus, encode_reset(J6_MOTOR_ID))
                if pos is not None:
                    idx, cfg = _j6_cfg(mc)
                    self.viewer.q[idx] = q_from_counts(cfg, fw_frame(pos))
                _exclude_j6(mc, False)
            except Exception as e:
                print(f"[J6] error a l'aturada d'emergència: {e}")
        self.viewer._j6_ik_override = None
        self._cmd, self._setpoint, self.saved_pos = 0.0, 0, None
        self._set_state(self.OFF)

    def _park(self):
        self._timer.stop()
        mc, bus = self._bus()
        if bus is not None:
            pos = _first(read_fresh_position(bus), latest_position(bus))
            _send_raw(bus, encode_current(J6_MOTOR_ID, 0))
            _send_raw(bus, encode_reset(J6_MOTOR_ID))
            self._t_reset = time.monotonic()
            if pos is not None:
                target = fw_frame(pos)
                self.saved_pos = target
                self._park_raw = pos
                idx, cfg = _j6_cfg(mc)
                q5 = q_from_counts(cfg, target)
                self.viewer.q[idx] = q5
                self.viewer._j6_ik_override = q5
                if hasattr(self.viewer, "sync_target_to_current"):
                    self.viewer.sync_target_to_current()
                print(f"[J6] STOP · aturat a {pos} (després del RESET: {target}) · J6 en RESET (blau)")
            else:
                print("[J6] STOP · sense posició del J6 · J6 en RESET (blau)")
        self._cmd, self._setpoint = 0.0, 0
        self._set_state(self.ARMED)

    def _step_current(self, target, bus):
        step = SLEW_MA_PER_S / TICK_HZ
        delta = float(target) - self._cmd
        self._cmd += max(-step, min(step, delta))
        if target == 0 and abs(self._cmd) < 1.0:
            self._cmd = 0.0
        _send_raw(bus, encode_current(J6_MOTOR_ID, int(round(self._cmd))))

    def _motor_quiet(self, bus):
        d = _daemon_entry(bus)
        t = float(d.get("last_b2", 0.0))
        if t <= self._last_b1:
            return self._zero_reads >= STOPPED_READS
        self._last_b1 = t
        pos = int(d.get("pos", 0))
        still = (self._last_pos is not None and abs(pos - self._last_pos) < POS_STILL_COUNTS
                 and abs(int(d.get("vel", 0))) < VEL_STOPPED)
        self._last_pos = pos
        self._zero_reads = self._zero_reads + 1 if still else 0
        return self._zero_reads >= STOPPED_READS

    def _check_response(self, bus):
        if self._warned_no_response or abs(self._cmd) < 300:
            return
        if time.monotonic() - self._t_start < NO_RESPONSE_S:
            return
        pos = latest_position(bus)
        if pos is None or self._start_pos is None or abs(pos - self._start_pos) < 200:
            self._warned_no_response = True
            d = _daemon_entry(bus)
            print(f"[J6] AVÍS: {int(self._cmd)} mA aplicats durant {NO_RESPONSE_S:.1f} s i la "
                  f"broca no gira (pos={pos}, curr mesurat={d.get('curr')}, vel={d.get('vel')}). "
                  f"Passa el diagnòstic: python scripts/diagnostic_j6.py")

    def _update_display(self, bus):
        pos = latest_position(bus)
        if pos is None:
            return
        idx, cfg = _j6_cfg(self.viewer.motor_controller)
        q = (pos - cfg.zero_offset_counts) / cfg.encoder_counts_per_rev * 2.0 * np.pi / cfg.direction
        self.viewer.q[idx] = _wrap(q)

    def _tick(self):
        mc, bus = self._bus()
        if bus is None:
            print("[J6] motors desconnectats durant la broca")
            self._timer.stop()
            self._set_state(self.ARMED)
            return
        try:
            if self.state == self.RUNNING:
                if self._setpoint_fn is not None:
                    try:
                        self._setpoint = int(self._setpoint_fn())
                    except Exception:
                        pass
                self._step_current(self._setpoint, bus)
                self._check_response(bus)
            else:
                self._step_current(0.0, bus)
                quiet = self._cmd == 0.0 and self._motor_quiet(bus)
                if quiet or time.monotonic() - self._t_stop > STOP_TIMEOUT_S:
                    if not quiet:
                        print("[J6] no s'ha aturat del tot a temps: el guardo igualment")
                    self._park()
                    return
            self._update_display(bus)
        except Exception as e:
            print(f"[J6] error al tick: {e} → aturada")
            self._timer.stop()
            try:
                _send_raw(bus, encode_current(J6_MOTOR_ID, 0))
                _send_raw(bus, encode_reset(J6_MOTOR_ID))
                self._t_reset = time.monotonic()
            except Exception:
                pass
            self._set_state(self.ARMED)

