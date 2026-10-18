"""
Broca del J6 (mode IK5): tota la lògica del motor 6 en un sol lloc.

QUÈ HEM APRÈS DEL FIRMWARE SITODRIVE (i per què abans fallava):

    1. NOMÉS CANVIA DE MODE DES DE RESET, i després d'un RESET necessita
       temps (el hand-guide espera ≥0.2 s). Un mode_selection enviat 50 ms
       després del RESET s'ignora: el motor queda blau i la broca no gira.
       → Esperem RESET_SETTLE_S des de l'últim RESET abans de canviar de mode.

    2. RECORDA L'ÚLTIM TARGET DE POSICIÓ encara que passi per CURRENT o
       RESET, i en tornar a POSITION el reprèn. Si la broca ha fet N voltes,
       el J6 les DESFÀ ("es mou contínuament en canviar de mode").
       → En tornar a POSITION enviem el target just abans i just després del
         mode_selection. I en entrar a IK5 deixem guardats al firmware
         paràmetres fluixos (PARK_*): si en algun instant anés cap al target
         vell, seria tan lent i fluix que no es notaria.

    3. DESPRÉS D'UN RESET, EL COMPTADOR DE POSICIÓ TORNA A UNA SOLA VOLTA
       (vist al registre de l'usuari: 311034 → ~-16646 després de 5 voltes).
       Mateixa posició física, número diferent. Si li enviéssim 311034,
       per a ell serien 5 voltes més enllà.
       → fw_frame(): els targets després d'un RESET es calculen en aquest
         comptador d'una volta; l'offset de configuració NO es toca mai.
         Amb el primer feedback es comprova (learn_frame / frame_fix) i, si
         el firmware comptés diferent, es corregeix sol.

CICLE (la proposta de l'usuari):

      OFF ──arm()──▶ ARMED ──start()──▶ RUNNING ──stop()──▶ STOPPING
       ▲   (entrar     (J6 en RESET,      (CURRENT_MODE,        │ rampa a 0,
       │    a IK5)      blau, mort)        rampa de corrent)    │ motor quiet
       │                  ▲                                     │
       │                  └──────── _park(): guarda posició ◀───┘
       │                            (comptador post-RESET) i RESET (blau)
       └──disarm()── (sortir d'IK5): RESET assentat → POSITION amb
                     target = posició guardada → el J6 no es mou

    OFF      J6 en POSITION_MODE, dins del send_q (FK, IK6, MANDO...).
    ARMED    J6 en RESET (lliure), fora del send_q. L'IK5 fa servir un q[5]
             congelat (_j6_ik_override): J1-J5 no noten el J6.
    RUNNING  J6 en CURRENT_MODE. L'IK5 continua amb J1-J5 → XYZ/RPY van.
    STOPPING rampa a 0 mA fins que la posició és estable → _park() → ARMED.
"""

import time                                                                     # importem el mòdul time

import numpy as np                                                              # importem el mòdul numpy as np
from qtpy import QtCore                                                         # importem QtCore des de qtpy

from src.hardware.sitodrive import (                                            # importem els builders de paquets
    Mode, encode_current, encode_mode_selection, encode_position, encode_reset,
)


J6_MOTOR_ID      = 6                                                            # ID del motor de la broca
TICK_HZ          = 20                                                           # refresc de la comanda de corrent
SLEW_MA_PER_S    = 4000.0                                                       # rampa: 0 → 2000 mA en 0.5 s
VEL_STOPPED      = 50                                                           # |vel| per sota d'això = motor quiet
STOPPED_READS    = 3                                                            # lectures fresques seguides per confirmar-ho
POS_STILL_COUNTS = 30                                                           # i la posició no ha canviat més de ~0.16° entre lectures
STOP_TIMEOUT_S   = 4.0                                                          # si no s'atura, el parquem igualment
FRESH_TIMEOUT_S  = 0.3                                                          # temps màxim esperant un paquet de posició nou
RESET_SETTLE_S   = 0.3                                                          # temps mínim després d'un RESET abans de canviar de mode
WATCH_S          = 0.3                                                          # vigilem el J6 després de tornar-lo a POSITION
WATCH_MAX_COUNTS = 65536 // 16                                                  # si se'n va més de 1/16 de volta (22.5°) → RESET de seguretat
NO_RESPONSE_S    = 1.5                                                          # broca sense moure's amb corrent → avís
PARK_CURRENT_MA  = 300                                                          # paràmetres "fluixos" que el firmware guarda en entrar a IK5...
PARK_VELOCITY    = 1                                                            # ...i fa servir en el moviment fantasma cap al target vell
ENC_PER_REV      = 65536                                                        # counts per volta del J6 (sortida)


# ==============================================================================
# HELPERS DE BAIX NIVELL (sense estat)
# ==============================================================================

def _first(*vals):                                                              # definim la funció _first()
    """El primer valor que no sigui None (una posició pot valer 0)."""
    return next((v for v in vals if v is not None), None)                       # retornem: valor o None


def _wrap(a):                                                                   # definim la funció _wrap()
    """Angle a [-π, π)."""
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)                           # retornem: angle embolicat


def _j6_cfg(mc):                                                                # definim la funció _j6_cfg()
    """(índex a q, config del joint) del motor J6."""
    for i, j in enumerate(mc.config.joints):                                    # iterem: i, j in enumerate(...)
        if j.motor_id == J6_MOTOR_ID:                                           # si és el J6
            return i, j                                                         # retornem: i, j
    raise RuntimeError("motor_config no té cap joint amb motor_id 6")           # llencem excepció


def _send_raw(bus, pkt):                                                        # definim la funció _send_raw()
    """Envia un paquet a un sol motor (daemon real o simulador)."""
    if bus._daemon is not None:                                                 # bus amb daemon
        bus._daemon.send({"cmd": "raw_bytes", "bytes": list(pkt)})              # executem send()
    else:                                                                       # bus amb simulador
        bus._sim.feed_command(pkt)                                              # executem feed_command()


def _daemon_entry(bus):                                                         # definim la funció _daemon_entry()
    """Últim estat del J6 que ha publicat el daemon (dict) o {}."""
    return bus._daemon.latest_status().get(J6_MOTOR_ID) or {}                   # retornem: dict del J6


def latest_position(bus):                                                       # definim la funció latest_position()
    """Última posició coneguda del J6 (pot ser vella). None si mai n'hi ha hagut."""
    if bus._daemon is None:                                                     # simulador
        st = bus.poll_feedback().get(J6_MOTOR_ID)                               # st ← resultat de get()
        return int(st.actuator_position) if st is not None else None            # retornem: posició
    d = _daemon_entry(bus)                                                      # d ← resultat de _daemon_entry()
    if float(d.get("last_b2", 0.0)) <= 0.0:                                     # mai rebut cap paquet de posició
        return None                                                             # retornem: None
    return int(d.get("pos", 0))                                                 # retornem: posició


def read_fresh_position(bus, timeout_s=FRESH_TIMEOUT_S):                        # definim la funció read_fresh_position()
    """
    Espera un paquet de posició (B2) del J6 POSTERIOR a aquesta crida.
    Retorna None si no n'arriba cap (p. ex. motor en RESET sense feedback).
    No crida processEvents: el daemon i el seu lector viuen en altres fils.
    """
    if bus._daemon is None:                                                     # simulador: sempre fresc
        return latest_position(bus)                                             # retornem: posició
    t0 = float(_daemon_entry(bus).get("last_b2", 0.0))                          # instant de l'últim paquet conegut
    deadline = time.monotonic() + timeout_s                                     # deadline ← resultat de monotonic()
    while time.monotonic() < deadline:                                          # mentre no s'acabi el temps
        time.sleep(0.01)                                                        # executem sleep()
        d = _daemon_entry(bus)                                                  # d ← resultat de _daemon_entry()
        t = float(d.get("last_b2", 0.0))                                        # t ← resultat de float()
        if t > t0 and t > 0.0:                                                  # ha arribat un paquet NOU
            return int(d.get("pos", 0))                                         # retornem: posició fresca
    return None                                                                 # retornem: None


def fw_wrap(counts, enc=ENC_PER_REV):                                           # definim la funció fw_wrap()
    """
    On queda el comptador del J6 DESPRÉS D'UN RESET.

    El firmware, en fer RESET, reinicia la posició a l'encoder absolut d'una
    sola volta: el comptador de moltes voltes (p. ex. 311034 després de
    trepar 5 voltes) passa a [-enc/2, enc/2) → -16646. Mateixa posició
    física, número diferent. Si després li enviéssim el target 311034, per
    a ell serien 5 voltes més enllà. (El codi antic ja feia aquest càlcul.)
    """
    half = enc // 2                                                             # assignem half
    return int(((int(counts) + half) % enc) - half)                             # retornem: comptador d'una volta


# Com compta el firmware després d'un RESET. None = encara no ho hem vist
# (suposem que sí reinicia, que és el que fa el robot real); el primer cop que
# arriba feedback després d'un RESET ho comprovem i ho apuntem (learn_frame).
_FW_WRAPS = None                                                                # assignem _FW_WRAPS


def fw_frame(counts):                                                           # definim la funció fw_frame()
    """Counts d'abans del RESET → com els comptarà el firmware després."""
    return int(counts) if _FW_WRAPS is False else fw_wrap(counts)               # retornem: counts post-RESET


def learn_frame(raw, measured):                                                 # definim la funció learn_frame()
    """
    Amb la posició d'abans del RESET (raw) i la primera mesura d'després,
    apunta si el firmware reinicia el comptador. Només és informatiu quan
    raw és fora d'una volta (si no, els dos marcs coincideixen).
    """
    global _FW_WRAPS                                                            # global _FW_WRAPS
    if raw is None or fw_wrap(raw) == int(raw):                                 # no distingible
        return                                                                  # retornem sense valor
    half = ENC_PER_REV // 2                                                     # assignem half
    if abs(measured - fw_wrap(raw)) < half:                                     # reinicia a una volta
        new = True                                                              # assignem new
    elif abs(measured - raw) < half:                                            # manté les voltes
        new = False                                                             # assignem new
    else:                                                                       # no concloent
        return                                                                  # retornem sense valor
    if new != _FW_WRAPS:                                                        # si canvia
        print(f"[J6] firmware: després d'un RESET el comptador "
              f"{'torna a una volta' if new else 'manté les voltes'}")
        _FW_WRAPS = new                                                         # assignem _FW_WRAPS


def q_from_counts(cfg, counts):                                                # definim la funció q_from_counts()
    """q (rad) del J6 per a uns counts, amb l'offset de configuració (MAI es toca)."""
    return (counts - cfg.zero_offset_counts) / cfg.encoder_counts_per_rev * 2.0 * np.pi / cfg.direction


def frame_fix(target, measured, enc=ENC_PER_REV):                               # definim la funció frame_fix()
    """
    Si el target i la posició mesurada difereixen en (gairebé) voltes
    senceres, és que el firmware compta en un altre "marc": corregim el
    target amb aquestes voltes perquè sigui la posició física on és.
    """
    n = int(round((measured - target) / enc))                                   # voltes de diferència
    return target + n * enc                                                     # retornem: target corregit


def _exclude_j6(mc, on):                                                        # definim la funció _exclude_j6()
    """Treu (on=True) o torna (on=False) el J6 del send_q del bucle de 100 Hz."""
    excl = [m for m in (mc._auto_exclude_motor_ids or []) if m != J6_MOTOR_ID]  # excl ← resultat de list()
    mc._auto_exclude_motor_ids = excl + [J6_MOTOR_ID] if on else excl           # assignem mc._auto_exclude_motor_ids


def j6_to_position(viewer, pos, settle_from=None, watch=True, post_reset=False,  # definim la funció j6_to_position()
                   raw=None):
    """
    Passa el J6 de RESET a POSITION_MODE amb target = `pos` i el torna al
    send_q del viewer. NO l'ha de moure.

      1. Espera que hagin passat RESET_SETTLE_S des del RESET (settle_from);
         si no sabem quan va ser, fa un RESET ara i espera.
      2. target = pos ABANS del mode_selection (per si el firmware el guarda),
         mode_selection(POSITION), i target = pos ENGANXAT just darrere
         (sense pausa) i repetit 100 ms → el firmware no té temps d'anar cap
         a un target vell.
      3. Vigilant: si el J6 se'n va més de 1/16 de volta → RESET de seguretat.
      4. q[5] = pos (voltes senceres fora de l'offset), J6 al send_q,
         referència de l'IK = TCP actual.

    Retorna True si ha quedat fixat.
    """
    mc = viewer.motor_controller                                                # assignem mc
    bus = mc.bus                                                                # assignem bus
    idx, cfg = _j6_cfg(mc)                                                      # idx, cfg ← resultat de _j6_cfg()

    if settle_from is None:                                                     # estat desconegut: RESET ara
        _send_raw(bus, encode_current(J6_MOTOR_ID, 0))                          # executem _send_raw()
        _send_raw(bus, encode_reset(J6_MOTOR_ID))                               # executem _send_raw()
        settle_from = time.monotonic()                                          # settle_from ← resultat de monotonic()
    wait = RESET_SETTLE_S - (time.monotonic() - settle_from)                    # temps que falta
    if wait > 0:                                                                # si wait > 0
        time.sleep(wait)                                                        # executem sleep()

    # pos ja en el comptador post-RESET (post_reset=True) o d'abans del RESET
    target = int(pos) if post_reset else fw_frame(pos)                          # target ← com compta el firmware ara
    raw_pre = raw if post_reset else pos                                        # valor d'abans del RESET (per aprendre)

    def mk(t, weak=False):                                                      # mode + target en un sol paquet
        if weak:                                                                # força i velocitat mínimes
            return encode_position(J6_MOTOR_ID, PARK_CURRENT_MA, PARK_VELOCITY, int(t))
        return encode_position(J6_MOTOR_ID, int(cfg.default_current_mA),
                               int(cfg.default_velocity_counts), int(t))

    # Fins que el primer feedback confirmi que el target és al lloc bo (el
    # comptador del firmware quadra), l'enviem FLUIX: si no quadrés, el J6
    # no tindria força per moure's abans de la correcció.
    verify = watch and bus._daemon is not None                                  # podem verificar amb feedback
    pkt = mk(target, weak=verify)                                               # pkt ← resultat de mk()
    t_switch = float(_daemon_entry(bus).get("last_b2", 0.0)) if bus._daemon is not None else 0.0
    _send_raw(bus, pkt)                                                         # target abans del canvi de mode
    _send_raw(bus, encode_mode_selection(J6_MOTOR_ID, Mode.POSITION_MODE,
                                         fb_cycle1=10, fb_cycle2=10, fb_cycle3=0))
    _send_raw(bus, pkt)                                                         # i just darrere, sense pausa
    _send_raw(bus, pkt)                                                         # executem _send_raw()
    for _ in range(10):                                                         # 100 ms reforçant el target
        time.sleep(0.01)                                                        # executem sleep()
        _send_raw(bus, pkt)                                                     # executem _send_raw()

    if watch and bus._daemon is not None:                                       # vigilant (només motors reals)
        worst, seen = 0, False                                                  # assignem worst, seen
        t_b2 = t_switch                                                         # només feedback posterior al canvi
        t_end = time.monotonic() + WATCH_S                                      # t_end ← resultat de monotonic()
        while time.monotonic() < t_end:                                         # mentre vigilem
            time.sleep(0.01)                                                    # executem sleep()
            _send_raw(bus, pkt)                                                 # el target bo continua arribant: s'aplica
                                                                                # tan bon punt el firmware l'accepti
            d = _daemon_entry(bus)                                              # d ← resultat de _daemon_entry()
            if float(d.get("last_b2", 0.0)) <= t_b2:                            # sense feedback nou
                continue                                                        # passem a la següent iteració
            t_b2 = float(d.get("last_b2", 0.0))                                 # t_b2 ← resultat de float()
            p = int(d.get("pos", 0))                                            # p ← posició nova
            if not seen:                                                        # primer feedback: comprovem el "marc"
                learn_frame(raw_pre, p)                                         # executem learn_frame()
                fixed = frame_fix(target, p)                                    # fixed ← resultat de frame_fix()
                if fixed != target:                                             # el firmware compta diferent
                    print(f"[J6] marc del comptador corregit: target {target} → {fixed} (mesurat {p})")
                    target = fixed                                              # assignem target
                pkt = mk(target)                                                # verificat: ara amb la força normal
                _send_raw(bus, pkt)                                             # executem _send_raw()
            seen = True                                                         # assignem seen
            worst = max(worst, abs(p - target))                                 # worst ← resultat de max()
            if worst > WATCH_MAX_COUNTS:                                        # se n'està anant: el deixem lliure
                _send_raw(bus, encode_reset(J6_MOTOR_ID))                       # executem _send_raw()
                _exclude_j6(mc, True)                                           # que el bucle de 100 Hz no el toqui
                print(f"[J6] SEGURETAT: s'ha mogut {worst} counts en passar a POSITION "
                      f"→ RESET (lliure). Passa scripts/diagnostic_j6.py")
                return False                                                    # retornem: False
        if seen:                                                                # si seen
            print(f"[J6] POSITION OK · desviació màxima {worst} counts")
        else:                                                                   # en cas contrari
            print("[J6] avís: cap feedback després de passar a POSITION (el motor no ha canviat de mode?)")

    q5 = q_from_counts(cfg, target)                                             # q exacta del target (offset intacte)
    viewer.q[idx] = q5                                                          # el model sap on és el motor
    viewer._j6_ik_override = None                                               # l'IK torna a fer servir q[5]
    _exclude_j6(mc, False)                                                      # J6 torna al send_q (envia el mateix target)
    if hasattr(viewer, "sync_target_to_current"):                               # referència IK = TCP actual
        viewer.sync_target_to_current()                                         # executem sync_target_to_current()
    print(f"[J6] fixat en POSITION a {target} counts · q={q5:+.3f} rad ({np.degrees(q5):+.1f}°)")
    return True                                                                 # retornem: True


def ensure_j6_position_hold(viewer):                                            # definim la funció ensure_j6_position_hold()
    """
    Xarxa de seguretat per als modes que NO són IK5: si el J6 encara és fora
    del send_q (no hauria de passar), el torna a POSITION on és.
    """
    mc = getattr(viewer, "motor_controller", None)                              # mc ← resultat de getattr()
    if mc is None or not mc.is_connected or mc.bus is None:                     # si no hi ha motors
        return                                                                  # retornem sense valor
    if J6_MOTOR_ID not in (mc._auto_exclude_motor_ids or []):                   # ja és al send_q
        return                                                                  # retornem sense valor
    pos = _first(read_fresh_position(mc.bus), latest_position(mc.bus))                # pos ← posició del J6
    if pos is None:                                                             # sense cap posició: no el toquem
        print("[J6] fora del send_q i sense posició coneguda: el deixo com està")
        return                                                                  # retornem sense valor
    print("[J6] era fora del send_q en canviar de mode → el torno a POSITION on és")
    j6_to_position(viewer, pos)                                                 # executem j6_to_position()


# ==============================================================================
# CONTROLADOR DE LA BROCA
# ==============================================================================

class J6Drill(QtCore.QObject):                                                  # definim la classe J6Drill
    """Màquina d'estats de la broca (vegeu el docstring del mòdul)."""

    stateChanged = QtCore.Signal(str)                                           # "off" | "armed" | "running" | "stopping"

    OFF, ARMED, RUNNING, STOPPING = "off", "armed", "running", "stopping"       # estats

    def __init__(self, viewer, setpoint_fn=None, parent=None):                  # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.viewer = viewer                                                    # assignem self.viewer
        self._setpoint_fn = setpoint_fn                                         # font de veritat del corrent (slider)
        self.state = self.OFF                                                   # assignem self.state
        self.saved_pos = None                                                   # posició guardada del J6 (counts)
        self._t_reset = 0.0                                                     # instant de l'últim RESET del J6
        self._setpoint = 0                                                      # corrent objectiu (mA)
        self._cmd = 0.0                                                         # corrent enviat (amb rampa)
        self._zero_reads = 0                                                    # lectures seguides amb el motor quiet
        self._last_b1 = 0.0                                                     # per saber si una lectura és nova
        self._last_pos = None                                                   # posició de la lectura anterior
        self._t_stop = 0.0                                                      # inici de la parada
        self._t_start = 0.0                                                     # inici de la broca
        self._start_pos = None                                                  # posició a l'arrencada
        self._warned_no_response = False                                        # avís de "no respon" ja donat
        self._park_raw = None                                                   # posició d'abans del RESET del Stop
        self._timer = QtCore.QTimer(self)                                      # self._timer ← resultat de QTimer()
        self._timer.timeout.connect(self._tick)                                 # executem connect()

    # ---- consultes -----------------------------------------------------------

    @property
    def spinning(self):                                                         # definim la funció spinning()
        """True mentre el J6 és en CURRENT_MODE (girant o aturant-se)."""
        return self.state in (self.RUNNING, self.STOPPING)                      # retornem: bool

    def _bus(self):                                                             # definim la funció _bus()
        mc = getattr(self.viewer, "motor_controller", None)                     # mc ← resultat de getattr()
        if mc is None or not mc.is_connected or mc.bus is None or mc.bus._daemon is None:
            return None, None                                                   # retornem: None, None
        return mc, mc.bus                                                       # retornem: mc, mc.bus

    def _set_state(self, s):                                                    # definim la funció _set_state()
        if s != self.state:                                                     # si canvia
            self.state = s                                                      # assignem self.state
            print(f"[J6] estat → {s}")                                          # executem print()
            self.stateChanged.emit(s)                                           # executem emit()

    # ---- IK5: entrar / sortir ------------------------------------------------

    def arm(self):                                                              # definim la funció arm()
        """
        Entrar a IK5: guarda la posició del J6, el treu del send_q, l'IK el
        congela, i RESET → el J6 queda blau (lliure, "mort") fins a Start.
        """
        if self.state != self.OFF:                                              # ja armat
            return                                                              # retornem sense valor
        idx = _j6_cfg(self.viewer.motor_controller)[0]                          # índex del J6 a q
        self.viewer._j6_ik_override = float(self.viewer.q[idx])                 # l'IK5 no veu el J6
        mc, bus = self._bus()                                                   # mc, bus ← resultat de _bus()
        if bus is None:                                                         # sense motors: només la part de l'IK
            self.state = self.ARMED                                             # assignem self.state
            return                                                              # retornem sense valor
        _exclude_j6(mc, True)                                                   # ABANS del RESET: cap paquet de posició més
        pos = _first(read_fresh_position(bus), latest_position(bus))                  # pos ← posició actual
        if pos is not None:                                                     # si pos is not None
            self.saved_pos = fw_frame(pos)                                      # com la comptarà el firmware després del RESET
            # Últim target ABANS del RESET, amb velocitat i corrent mínims: el
            # firmware el guarda, i en sortir d'IK5 (quan passa a POSITION i
            # durant uns ms encara ignora el target nou) anirà cap a aquest
            # target vell tan lent i fluix que no es nota. Sense això, el J6
            # feia "endavant → reset → enrere" en passar a IK6/FK.
            weak = encode_position(J6_MOTOR_ID, PARK_CURRENT_MA, PARK_VELOCITY, int(pos))
            for _ in range(3):                                                  # iterem: 3 cops
                _send_raw(bus, weak)                                            # executem _send_raw()
                time.sleep(0.01)                                                # executem sleep()
        _send_raw(bus, encode_reset(J6_MOTOR_ID))                               # J6 blau
        self._t_reset = time.monotonic()                                        # self._t_reset ← resultat de monotonic()
        self._set_state(self.ARMED)                                             # executem _set_state()
        print(f"[J6] IK5 · J6 en RESET (blau), posició guardada {self.saved_pos}")

    def disarm(self):                                                           # definim la funció disarm()
        """
        Sortir d'IK5 (o abans d'un reset/reconnexió): si gira l'atura, i
        torna el J6 a POSITION amb target = posició guardada → no es mou.
        """
        if self.spinning:                                                       # si gira: parada completa
            self.finish_now()                                                   # executem finish_now()
        if self.state == self.OFF:                                              # ja fora
            return                                                              # retornem sense valor
        self._timer.stop()                                                      # executem stop()
        mc, bus = self._bus()                                                   # mc, bus ← resultat de _bus()
        if bus is None:                                                         # sense motors: només la part de l'IK
            self.viewer._j6_ik_override = None                                  # assignem self.viewer._j6_ik_override
            self._set_state(self.OFF)                                           # executem _set_state()
            return                                                              # retornem sense valor
        pos = read_fresh_position(bus, timeout_s=0.15)                          # si en RESET hi ha feedback, és la millor
        if pos is None:                                                         # si no, la que vam guardar
            pos = self.saved_pos if self.saved_pos is not None else latest_position(bus)
        if pos is None:                                                         # cap posició: no el podem fixar
            print("[J6] ERROR: sense posició del J6; queda en RESET i fora del send_q")
            self.viewer._j6_ik_override = None                                  # assignem self.viewer._j6_ik_override
            self._set_state(self.OFF)                                           # executem _set_state()
            return                                                              # retornem sense valor
        ok = j6_to_position(self.viewer, pos, settle_from=self._t_reset,        # pos ja és en el comptador post-RESET
                            post_reset=True, raw=self._park_raw)
        if ok and hasattr(self.viewer, "set_j6_center"):                        # slider del J6 centrat on ha quedat
            idx = _j6_cfg(mc)[0]                                                # idx ← índex del J6 a q
            self.viewer.set_j6_center(self.viewer.q[idx])                       # ±180° a cada banda
        self._park_raw = None                                                   # assignem self._park_raw
        self.saved_pos = None                                                   # assignem self.saved_pos
        self._set_state(self.OFF)                                               # executem _set_state()

    # ---- broca ---------------------------------------------------------------

    def start(self, current_mA):                                                # definim la funció start()
        """Engega (o reprèn) la broca. Retorna False si no es pot."""
        if self.state == self.RUNNING:                                          # ja gira: només canvia el corrent
            self._setpoint = int(current_mA)                                    # assignem self._setpoint
            return True                                                         # retornem: True
        if self.state == self.STOPPING:                                         # encara en CURRENT_MODE: reprenem
            self._setpoint = int(current_mA)                                    # assignem self._setpoint
            self._set_state(self.RUNNING)                                       # executem _set_state()
            return True                                                         # retornem: True
        if self.state != self.ARMED:                                            # només des d'IK5
            print("[J6] Start ignorat: cal estar en IK5")                       # executem print()
            return False                                                        # retornem: False
        mc, bus = self._bus()                                                   # mc, bus ← resultat de _bus()
        if bus is None:                                                         # si bus is None
            return False                                                        # retornem: False

        # El J6 és en RESET des d'arm()/park(). El firmware necessita temps
        # després del RESET per acceptar el canvi de mode.
        wait = RESET_SETTLE_S - (time.monotonic() - self._t_reset)              # temps que falta
        if wait > 0:                                                            # si wait > 0
            time.sleep(wait)                                                    # executem sleep()

        # El mateix que feia el codi vell que funcionava: mode_selection(CURRENT)
        # des de RESET, respir, i corrent.
        _send_raw(bus, encode_mode_selection(J6_MOTOR_ID, Mode.CURRENT_MODE,
                                             fb_cycle1=10, fb_cycle2=10, fb_cycle3=0))
        time.sleep(0.05)                                                        # marge perquè el firmware processi
        self._cmd = 0.0                                                         # assignem self._cmd
        self._setpoint = int(current_mA)                                        # assignem self._setpoint
        self._step_current(self._setpoint, bus)                                 # primer graó de la rampa ja
        self._t_start = time.monotonic()                                        # per detectar si no respon
        self._start_pos = self.saved_pos                                        # posició a l'arrencada
        self._warned_no_response = False                                        # assignem self._warned_no_response
        self._set_state(self.RUNNING)                                           # executem _set_state()
        self._timer.start(int(1000 / TICK_HZ))                                  # executem start()
        print(f"[J6] START · rampa fins a {self._setpoint} mA")
        return True                                                             # retornem: True

    def set_current(self, current_mA):                                          # definim la funció set_current()
        if self.state == self.RUNNING:                                          # si gira
            self._setpoint = int(current_mA)                                    # assignem self._setpoint

    def stop(self):                                                             # definim la funció stop()
        """Parada normal (no bloqueja): rampa a 0, espera que s'aturi, park()."""
        if self.state != self.RUNNING:                                          # si no gira
            return                                                              # retornem sense valor
        self._zero_reads, self._last_pos = 0, None                                # comptadors de 'quiet' a zero
        self._t_stop = time.monotonic()                                         # self._t_stop ← resultat de monotonic()
        self._set_state(self.STOPPING)                                          # executem _set_state()

    def finish_now(self, timeout_s=STOP_TIMEOUT_S):                             # definim la funció finish_now()
        """
        Parada completa BLOQUEJANT (canvis de mode): la mateixa seqüència que
        stop(), sense tornar al bucle de Qt. Acaba en ARMED (blau).
        """
        if not self.spinning:                                                   # res a fer
            return                                                              # retornem sense valor
        self._timer.stop()                                                      # executem stop()
        mc, bus = self._bus()                                                   # mc, bus ← resultat de _bus()
        if bus is None:                                                         # sense motors
            self._set_state(self.ARMED)                                         # executem _set_state()
            return                                                              # retornem sense valor
        self._set_state(self.STOPPING)                                          # executem _set_state()
        t0 = time.monotonic()                                                   # t0 ← resultat de monotonic()
        self._zero_reads, self._last_pos = 0, None                              # comptadors de 'quiet' a zero
        while time.monotonic() - t0 < timeout_s:                                # fins al timeout
            self._step_current(0.0, bus)                                        # rampa cap a 0 mA
            if self._cmd == 0.0 and self._motor_quiet(bus):                     # quiet de veritat
                break                                                           # sortim del bucle
            time.sleep(1.0 / TICK_HZ)                                           # executem sleep()
        self._park()                                                            # executem _park()

    def abort(self):                                                            # definim la funció abort()
        """
        STOP / E-STOP: 0 mA a l'instant (el motor deixa de fer força). Si
        girava, ens quedem en CURRENT a 0 mA (on SÍ hi ha feedback) fins que
        s'atura, per saber on queda de veritat; després RESET (el qui crida
        farà el RESET de tots). Deixem el J6 al send_q amb q[5] = on és,
        perquè en reconnectar (enable_position_control_at) rebi el target
        correcte i no desfaci voltes.
        """
        self._timer.stop()                                                      # executem stop()
        if self.state == self.OFF:                                              # res a fer
            return                                                              # retornem sense valor
        mc, bus = self._bus()                                                   # mc, bus ← resultat de _bus()
        if bus is not None:                                                     # si bus is not None
            try:                                                                # intentem el bloc següent
                _send_raw(bus, encode_current(J6_MOTOR_ID, 0))                  # força zero, ja
                if self.state in (self.RUNNING, self.STOPPING):                 # girava: esperem que s'aturi
                    self._zero_reads, self._last_pos = 0, None                    # comptadors de 'quiet' a zero
                    t_end = time.monotonic() + 1.5                              # t_end ← resultat de monotonic()
                    while time.monotonic() < t_end and not self._motor_quiet(bus):
                        _send_raw(bus, encode_current(J6_MOTOR_ID, 0))          # executem _send_raw()
                        time.sleep(0.05)                                        # executem sleep()
                    pos = _first(read_fresh_position(bus), latest_position(bus), self.saved_pos)
                else:                                                           # en RESET (armat): la guardada
                    pos = _first(read_fresh_position(bus, timeout_s=0.1), self.saved_pos,
                                 latest_position(bus))
                _send_raw(bus, encode_reset(J6_MOTOR_ID))                       # executem _send_raw()
                if pos is not None:                                             # si pos is not None
                    idx, cfg = _j6_cfg(mc)                                      # idx, cfg ← resultat de _j6_cfg()
                    self.viewer.q[idx] = q_from_counts(cfg, fw_frame(pos))      # q[5] = on és (comptador post-RESET)
                _exclude_j6(mc, False)                                          # en reconnectar rebrà el target bo
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[J6] error a l'aturada d'emergència: {e}")
        self.viewer._j6_ik_override = None                                      # assignem self.viewer._j6_ik_override
        self._cmd, self._setpoint, self.saved_pos = 0.0, 0, None                # assignem
        self._set_state(self.OFF)                                               # executem _set_state()

    # ---- intern --------------------------------------------------------------

    def _park(self):                                                            # definim la funció _park()
        """
        Motor quiet: guarda la posició, RESET (blau) → ARMED.
        Després del RESET el firmware compta en una sola volta, així que la
        posició es guarda en aquest comptador (fw_frame): p. ex. 311034 →
        -16646, el mateix lloc físic. En sortir d'IK5, disarm() hi tornarà.
        """
        self._timer.stop()                                                      # executem stop()
        mc, bus = self._bus()                                                   # mc, bus ← resultat de _bus()
        if bus is not None:                                                     # si bus is not None
            pos = _first(read_fresh_position(bus), latest_position(bus))        # en CURRENT hi ha feedback: fresca
            _send_raw(bus, encode_current(J6_MOTOR_ID, 0))                      # executem _send_raw()
            _send_raw(bus, encode_reset(J6_MOTOR_ID))                           # J6 blau, preparat per un altre Start
            self._t_reset = time.monotonic()                                    # self._t_reset ← resultat de monotonic()
            if pos is not None:                                                 # si pos is not None
                target = fw_frame(pos)                                          # com la comptarà el firmware
                self.saved_pos = target                                         # assignem self.saved_pos
                self._park_raw = pos                                            # d'abans del RESET (per aprendre)
                idx, cfg = _j6_cfg(mc)                                          # idx, cfg ← resultat de _j6_cfg()
                q5 = q_from_counts(cfg, target)                                 # offset de configuració intacte
                self.viewer.q[idx] = q5                                         # dibuix 3D = on s'ha aturat
                self.viewer._j6_ik_override = q5                                # i l'IK5 hi congela el J6
                if hasattr(self.viewer, "sync_target_to_current"):              # sense salts a l'IK
                    self.viewer.sync_target_to_current()                        # executem sync_target_to_current()
                print(f"[J6] STOP · aturat a {pos} (després del RESET: {target}) · J6 en RESET (blau)")
            else:                                                               # en cas contrari
                print("[J6] STOP · sense posició del J6 · J6 en RESET (blau)")  # executem print()
        self._cmd, self._setpoint = 0.0, 0                                      # assignem
        self._set_state(self.ARMED)                                             # executem _set_state()

    def _step_current(self, target, bus):                                       # definim la funció _step_current()
        """Mou el corrent enviat cap a `target` amb rampa i l'envia."""
        step = SLEW_MA_PER_S / TICK_HZ                                          # assignem step
        delta = float(target) - self._cmd                                       # assignem delta
        self._cmd += max(-step, min(step, delta))                               # sumem a self._cmd
        if target == 0 and abs(self._cmd) < 1.0:                                # si arribem a 0
            self._cmd = 0.0                                                     # assignem self._cmd
        _send_raw(bus, encode_current(J6_MOTOR_ID, int(round(self._cmd))))     # executem _send_raw()

    def _motor_quiet(self, bus):                                                # definim la funció _motor_quiet()
        """
        Compta lectures NOVES amb el motor quiet: velocitat baixa I la
        posició pràcticament igual que a la lectura anterior. La posició no
        depèn de les unitats de velocitat del firmware: si la guardéssim
        amb el motor encara rodant, en sortir d'IK5 hi tornaria uns graus.
        """
        d = _daemon_entry(bus)                                                  # d ← resultat de _daemon_entry()
        t = float(d.get("last_b2", 0.0))                                        # t ← resultat de float()
        if t <= self._last_b1:                                                  # lectura repetida: no compta
            return self._zero_reads >= STOPPED_READS                            # retornem: bool
        self._last_b1 = t                                                       # assignem self._last_b1
        pos = int(d.get("pos", 0))                                              # pos ← resultat de int()
        still = (self._last_pos is not None and abs(pos - self._last_pos) < POS_STILL_COUNTS
                 and abs(int(d.get("vel", 0))) < VEL_STOPPED)                   # still ← quiet de veritat
        self._last_pos = pos                                                    # assignem self._last_pos
        self._zero_reads = self._zero_reads + 1 if still else 0                 # assignem self._zero_reads
        return self._zero_reads >= STOPPED_READS                                # retornem: bool

    def _check_response(self, bus):                                             # definim la funció _check_response()
        """Avís (un cop) si amb corrent aplicat la broca no s'ha mogut gens."""
        if self._warned_no_response or abs(self._cmd) < 300:                    # ja avisat o corrent massa baix
            return                                                              # retornem sense valor
        if time.monotonic() - self._t_start < NO_RESPONSE_S:                    # encara aviat
            return                                                              # retornem sense valor
        pos = latest_position(bus)                                              # pos ← resultat de latest_position()
        if pos is None or self._start_pos is None or abs(pos - self._start_pos) < 200:
            self._warned_no_response = True                                     # assignem self._warned_no_response
            d = _daemon_entry(bus)                                              # d ← resultat de _daemon_entry()
            print(f"[J6] AVÍS: {int(self._cmd)} mA aplicats durant {NO_RESPONSE_S:.1f} s i la "
                  f"broca no gira (pos={pos}, curr mesurat={d.get('curr')}, vel={d.get('vel')}). "
                  f"Passa el diagnòstic: python scripts/diagnostic_j6.py")

    def _update_display(self, bus):                                             # definim la funció _update_display()
        """La broca 3D gira amb el motor real (només visual: l'IK no ho veu)."""
        pos = latest_position(bus)                                              # pos ← resultat de latest_position()
        if pos is None:                                                         # si pos is None
            return                                                              # retornem sense valor
        idx, cfg = _j6_cfg(self.viewer.motor_controller)                        # idx, cfg ← resultat de _j6_cfg()
        q = (pos - cfg.zero_offset_counts) / cfg.encoder_counts_per_rev * 2.0 * np.pi / cfg.direction
        self.viewer.q[idx] = _wrap(q)                                           # embolicat: no topa amb els límits

    def _tick(self):                                                            # definim la funció _tick()
        mc, bus = self._bus()                                                   # mc, bus ← resultat de _bus()
        if bus is None:                                                         # motors perduts
            print("[J6] motors desconnectats durant la broca")                  # executem print()
            self._timer.stop()                                                  # executem stop()
            self._set_state(self.ARMED)                                         # executem _set_state()
            return                                                              # retornem sense valor
        try:                                                                    # intentem el bloc següent
            if self.state == self.RUNNING:                                      # si gira
                if self._setpoint_fn is not None:                               # l'slider és la font de veritat
                    try:                                                        # intentem el bloc següent
                        self._setpoint = int(self._setpoint_fn())               # self._setpoint ← resultat de int()
                    except Exception:                                           # capturem excepció: Exception
                        pass                                                    # ens quedem amb el valor previ
                self._step_current(self._setpoint, bus)                         # executem _step_current()
                self._check_response(bus)                                       # executem _check_response()
            else:                                                               # STOPPING
                self._step_current(0.0, bus)                                    # executem _step_current()
                quiet = self._cmd == 0.0 and self._motor_quiet(bus)             # quiet ← resultat de _motor_quiet()
                if quiet or time.monotonic() - self._t_stop > STOP_TIMEOUT_S:   # quiet o massa temps
                    if not quiet:                                               # si not quiet
                        print("[J6] no s'ha aturat del tot a temps: el guardo igualment")
                    self._park()                                                # executem _park()
                    return                                                      # retornem sense valor
            self._update_display(bus)                                           # executem _update_display()
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[J6] error al tick: {e} → aturada")                         # executem print()
            self._timer.stop()                                                  # executem stop()
            try:                                                                # intentem el bloc següent
                _send_raw(bus, encode_current(J6_MOTOR_ID, 0))                  # executem _send_raw()
                _send_raw(bus, encode_reset(J6_MOTOR_ID))                       # executem _send_raw()
                self._t_reset = time.monotonic()                                # self._t_reset ← resultat de monotonic()
            except Exception:                                                   # capturem excepció: Exception
                pass                                                            # no fem res
            self._set_state(self.ARMED)                                         # executem _set_state()
