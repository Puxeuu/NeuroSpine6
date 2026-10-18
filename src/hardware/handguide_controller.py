"""
HandGuideController - mode HANDGUIDE (guia manual del cobot).

QUÈ FA:
    Posa els motors en CURRENT_MODE i, a cada tic (50 Hz), els aplica una
    corrent que compensa EXACTAMENT la gravetat. Com que la gravetat queda
    anul·lada, el braç no cau sol i l'usuari el pot moure a mà amb poc
    esforç. A més, afegeix una assistència direccional quan detecta que
    l'usuari ja l'està movent (perquè la fricció interna no es noti tant).

FÓRMULA PER EIX:
    I_cmd = I_grav + k_assist · I_assist

    on:
      I_grav    = corrent necessari per aguantar el pes (Newton-Euler → mA)
      I_assist  = alpha · sign(v) · (I_friccio + I_push)
                  (només si l'usuari ja s'està movent, a prop de v≈0 és 0)
      alpha     = rampa de 0 a 1 entre v_low i v_high (perquè l'assistència
                  no s'engegui bruscament)

CICLE:
    start()  →  bus.reset_all + mode CURRENT per cada motor
    step()   →  llegir q, llegir v, compensar gravetat, enviar I (50 Hz)
    stop()   →  enviar I=0 i tornar a RESET
"""

import time                                                                     # importem el mòdul time (per sleeps curts al start/stop)
import numpy as np                                                              # importem numpy per fer càlculs vectorials

from src.hardware.sitodrive import (Mode, encode_mode_selection, encode_current,)  # Mode = enum amb els modes del motor; les altres són builders de paquets de bytes del protocol SitoDrive
from src.hardware.motor_converter import motor_counts_to_q                      # Converteix counts d'encoder → q en rad (té en compte reduccions)
from src.dynamics.newton_euler import gravity_torques                           # Dóna tau (parell N·m) a cada joint per compensar la gravetat
from src.dynamics.motor_model import torques_to_currents_handguide              # Converteix tau (N·m) a corrent (mA) tenint en compte kt


# ==============================================================================
# CONFIGURACIÓ
# ==============================================================================
MAX_CURRENT_ABS = 1500                                                          # Sostre de seguretat (mA) per joint. Si demanem més, saturem.
TARGET_JOINTS   = [0, 1, 2, 3, 4, 5]                                            # Índexs dels joints que anem a controlar (els 6 del braç)

# Un diccionari de config per cada joint:
#   q_min/max_deg → límits de software (si es passa, aturem)
#   v_low/v_high  → finestra per decidir si l'usuari està movent (rampa alpha)
#   v_alpha       → factor del filtre passa-baixos de la velocitat
#   i_push_up/dn  → mA extra d'empenta quan va contra la gravetat / a favor
#   k_trim        → multiplicador fi de la compensació de gravetat (per axis)
#   k_assist      → multiplicador fi de l'assistència (per axis)
JOINT_CONFIGS = {
    0: {  # J1 - Base
        "name": "j1", "q_min_deg": -170.0, "q_max_deg": 170.0,
        "v_low": 1.2, "v_high": 18.0, "v_alpha": 0.5,
        "i_push_up": 600, "i_push_dn": 600,
        "k_trim": 1.0, "k_assist": 1.0,
    },
    1: {  # J2 - Hombra
        "name": "j2", "q_min_deg": -90.0, "q_max_deg": 90.0,
        "v_low": 1.2, "v_high": 18.0, "v_alpha": 0.5,
        "i_push_up": 200, "i_push_dn": 100,
        "k_trim": 1.0, "k_assist": 1.0,
    },
    2: {  # J3 - Colze
        "name": "j3", "q_min_deg": -130.0, "q_max_deg": 130.0,
        "v_low": 1.2, "v_high": 18.0, "v_alpha": 0.5,
        "i_push_up": 100, "i_push_dn": 600,
        "k_trim": 1.0, "k_assist": 1.0,
    },
    3: {  # J4
        "name": "j1", "q_min_deg": -170.0, "q_max_deg": 170.0,
        "v_low": 0.2, "v_high": 5.0, "v_alpha": 0.85,
        "i_push_up": 400, "i_push_dn": 400,
        "k_trim": 1.0, "k_assist": 1.0,
    },
    4: {  # J5
        "name": "j3", "q_min_deg": -130.0, "q_max_deg": 130.0,
        "v_low": 0.2, "v_high": 5.0, "v_alpha": 0.85,
        "i_push_up": 800, "i_push_dn": 800,
        "k_trim": 1.0, "k_assist": 1.0,
    },
    5: {  # J6 - Broca (reducció harmònica 51:1 → cal empenta molt alta)
        "name": "j6", "q_min_deg": -180.0, "q_max_deg": 180.0,
        "v_low": 0.5, "v_high": 10.0, "v_alpha": 0.5,
        "i_push_up": 1200, "i_push_dn": 1200,
        "k_trim": 1.0, "k_assist": 1.0,
    },
}


# ==============================================================================
# HELPERS DE BAIX NIVELL: enviar bytes al bus
# ==============================================================================
# El MotorBus pot estar connectat a un daemon real (procés separat parlant per
# COM3) o a un simulador en memòria. Ambdós casos accepten el mateix paquet
# de bytes del protocol SitoDrive, només canvia on es "deixa caure".

def _send_raw_to_bus(bus, packet_bytes):                                        # Envia un paquet cru al bus (daemon real o simulador)
    if bus._daemon is not None:                                                 # si hi ha daemon connectat (motors reals)
        bus._daemon.send({"cmd": "raw_bytes", "bytes": list(packet_bytes)})     # li passem els bytes via JSON per stdin
    else:                                                                       # en cas contrari (mode simulació)
        bus._sim.feed_command(packet_bytes)                                     # els injectem al simulador


def _send_current_to_motor(bus, motor_id, current_mA):                          # Envia a UN motor una ordre de corrent (mA)
    pkt = encode_current(motor_id, int(current_mA))                             # construïm el paquet SitoDrive
    _send_raw_to_bus(bus, pkt)                                                  # i l'enviem al bus


def _send_mode_to_motor(bus, motor_id, mode):                                   # Canvia el mode d'un motor (RESET / CURRENT_MODE / ...)
    """
    Canvia el mode d'un motor amb fb_cycles = (10, 10, 10). Això fa que el
    motor retorni feedback dels tres grups (volt/corrent/vel, posicions, i
    temperatura/errors) al ritme que necessitem per al bucle a 50 Hz.
    """
    pkt = encode_mode_selection(motor_id, mode, fb_cycle1=10, fb_cycle2=10, fb_cycle3=10)  # pkt ← paquet amb mode + fb_cycles
    _send_raw_to_bus(bus, pkt)                                                  # l'enviem al bus


def _send_zero_current_to_all(bus, motor_ids):                                  # Helper per "aturar" tots els motors d'un cop (fa falta a stop i a cada abort del bucle)
    for mid in motor_ids:                                                       # iterem: mid per cada motor
        try:                                                                    # intentem el bloc següent
            _send_current_to_motor(bus, mid, 0)                                 # enviem corrent = 0
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[handguide] error current=0 motor {mid}: {e}")


# ==============================================================================
# HANDGUIDECONTROLLER
# ==============================================================================

class HandGuideController:
    """
    Controlador del mode handguide. L'usuari del codi:
        hg = HandGuideController(motor_controller, dyn_params, robot, ...)
        hg.start()
        # a cada tic del QTimer (50 Hz):
        hg.step()
        hg.stop()
    """

    def __init__(self, motor_controller, dyn_params, robot,                     # definim la funció __init__()
                 on_q_updated=None, on_limit_hit=None, on_error=None):
        # Referències als objectes externs que farem servir
        self.mc = motor_controller                                              # wrapper d'alt nivell dels 6 motors
        self.dyn_params = dyn_params                                            # masses, inèrcies, kt, fricció (del YAML dinàmic)
        self.robot = robot                                                      # el model URDF (per fer Newton-Euler)
        self.on_q_updated = on_q_updated                                        # callback(q_rad) per notificar al viewer la q llegida
        self.on_limit_hit = on_limit_hit                                        # callback(j_idx) quan un joint toca límit
        self.on_error = on_error                                                # callback(msg) quan passa qualsevol error greu

        self.cfg = motor_controller.config                                      # motor_config (motor_ids, reduccions, ...)
        self.bus = motor_controller.bus                                         # MotorBus (daemon o simulador)
        self.target_mids = [self.cfg.motor_ids[i] for i in TARGET_JOINTS]       # ids reals dels motors que controlem

        # --- Precalculem la fricció convertida a mA per cada eix ---
        # Al YAML dinàmic tenim fricció en N·m. Com que I = tau / kt (×1000 per mA),
        # ho precalculem un cop a __init__ i reutilitzem a step().
        self._fric_mA = {}                                                      # dict j_idx → mA de fricció
        for j_idx in TARGET_JOINTS:                                             # iterem: j_idx in TARGET_JOINTS
            j_name = f"j{j_idx + 1}"                                            # noms a dyn_params són "j1".."j6"
            kt   = dyn_params.motors[j_name].kt                                 # constant parell del motor (N·m / A)
            fric = dyn_params.friction.get(j_name, 0.0)                         # fricció estimada (N·m)
            self._fric_mA[j_idx] = (fric / kt) * 1000.0                         # conversió a mA

        # --- Estat intern ---
        self._v_filt = {j_idx: 0.0 for j_idx in TARGET_JOINTS}                  # velocitat filtrada per cada joint
        self._first_iter = True                                                 # per inicialitzar el filtre al primer tic
        self._q_last = np.zeros(6, dtype=np.float64)                            # última q coneguda (per si falla un feedback)
        self._active = False                                                    # True entre start() i stop()

        # --- Comptadors de debug (es creen aquí perquè step() no els hagi de crear) ---
        self._dbg_tick = 0                                                      # tic actual (per printar cada N)
        self._dbg_no_feedback_count = 0                                         # cops que ens ha faltat feedback

    # --------------------------------------------------------------------------
    def start(self):                                                            # definim la funció start()
        """
        Prepara els motors per treballar en CURRENT_MODE:
            1) Captura la q inicial per si fa falta
            2) RESET global + RESET individual (deixa tots els motors passius)
            3) Posa els motors target en CURRENT_MODE
            4) Envia corrent 0 inicial
            5) Espera a rebre feedback fresc dels motors
        """
        if self._active:                                                        # si ja estem actius, no fem res
            return                                                              # retornem sense valor

        # Refresquem la referència al bus per si el motor_controller l'ha canviat
        self.bus = self.mc.bus                                                  # assignem self.bus
        if self.bus is None:                                                    # si no hi ha bus (motors no connectats)
            if self.on_error:                                                   # si tenim callback d'error
                self.on_error("motor_controller sense bus")                     # avisem
            return                                                              # retornem sense valor

        # PAS 1: Capturem la q inicial per tenir una referència vàlida
        try:                                                                    # intentem el bloc següent
            self._q_last = self.mc.read_q().copy()                              # self._q_last ← q actual (còpia)
        except Exception as e:                                                  # capturem excepció: Exception as e
            if self.on_error:                                                   # si tenim callback d'error
                self.on_error(f"error llegint q inicial: {e}")
            return                                                              # retornem sense valor

        # PAS 2: RESET global + individual. Això atura qualsevol stream de
        # POSITION_MODE anterior i deixa tots els motors en estat passiu.
        # Funciona tant venint d'un mode "verd" (motors actius) com d'un
        # mode "blau" (ja en RESET).
        try:                                                                    # intentem el bloc següent
            self.bus.reset_all()                                                # RESET global del bus
            print("[handguide] bus.reset_all() enviat")
            time.sleep(0.2)                                                     # deixem que arribi
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[handguide] error reset_all: {e}")

        print(f"[handguide] RESET individual dels motors {self.target_mids}")
        for mid in self.target_mids:                                            # iterem: mid in self.target_mids
            _send_mode_to_motor(self.bus, mid, Mode.RESET)                      # RESET per cada motor
            time.sleep(0.05)                                                    # petit respir
        time.sleep(0.15)                                                        # i un petit extra

        # PAS 3: ara sí, els posem en CURRENT_MODE (fb_cycles = 10,10,10)
        print(f"[handguide] posant motors {self.target_mids} en CURRENT_MODE")
        for mid in self.target_mids:                                            # iterem: mid in self.target_mids
            _send_mode_to_motor(self.bus, mid, Mode.CURRENT_MODE)               # canvi de mode
            time.sleep(0.05)                                                    # respir
        time.sleep(0.2)                                                         # extra

        # PAS 4: Enviem current=0 inicial per "confirmar" l'entrada al mode
        _send_zero_current_to_all(self.bus, self.target_mids)                   # tots a 0 mA
        time.sleep(0.05)                                                        # respir

        # PAS 5: Esperem fins a 1 segon fins que arribi feedback de tots
        for _ in range(20):                                                     # iterem fins a 20 cops (20·50ms = 1s)
            status = self.bus.poll_feedback()                                   # status ← últim estat del bus
            if all(m in status for m in self.target_mids):                      # si ja ens arriben tots
                break                                                           # sortim del bucle
            time.sleep(0.05)                                                    # i tornem a provar

        # Reset estat intern
        self._v_filt = {j_idx: 0.0 for j_idx in TARGET_JOINTS}                  # buidem filtre de velocitat
        self._first_iter = True                                                 # el proper tic serà el primer
        self._dbg_tick = 0                                                      # reset del comptador de debug
        self._dbg_no_feedback_count = 0                                         # idem
        self._active = True                                                     # ja estem actius
        print("[handguide] controller iniciat (J1-J6 en CURRENT_MODE)")

    # --------------------------------------------------------------------------
    def stop(self):                                                             # definim la funció stop()
        """Atura el handguide: current=0 a tots els motors i torna a RESET."""
        if not self._active:                                                    # si no estem actius, no fem res
            return                                                              # retornem sense valor

        # Pas 1: Posem a 0 tots els motors (parem força abans de tocar modes)
        _send_zero_current_to_all(self.bus, self.target_mids)                   # tots a 0 mA
        time.sleep(0.05)                                                        # respir

        # Pas 2: Tornem a RESET per deixar els motors passius
        for mid in self.target_mids:                                            # iterem: mid in self.target_mids
            try:                                                                # intentem el bloc següent
                _send_mode_to_motor(self.bus, mid, Mode.RESET)                  # RESET per cada motor
                time.sleep(0.02)                                                # respir petit
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[handguide] error RESET motor {mid}: {e}")

        time.sleep(0.1)                                                         # deixem que arribi

        self._active = False                                                    # ja no estem actius
        print("[handguide] controller aturat (motors en RESET)")

    # --------------------------------------------------------------------------
    def is_active(self):                                                        # definim la funció is_active()
        return self._active                                                     # retornem: True si estem al bucle

    # --------------------------------------------------------------------------
    def step(self):                                                             # definim la funció step()
        """
        Un tic del bucle de control. Es crida des d'un QTimer a 50 Hz.
        NO fa cap sleep. Si falla alguna lectura o càlcul, envia 0 mA a tots
        i torna a esperar al proper tic.
        """
        if not self._active:                                                    # si no estem actius, no fem res
            return                                                              # retornem sense valor

        self._dbg_tick += 1                                                     # sumem a self._dbg_tick

        # ------------------------------------------------------------------
        # 1) Llegir feedback de tots els motors target
        # ------------------------------------------------------------------
        st = self.bus.poll_feedback()                                           # st ← dict motor_id → MotorStatus
        ids_present = {m for m in self.target_mids if m in st}                  # ids que SÍ ens han contestat

        if not all(m in st for m in self.target_mids):                          # si en falta algun → aborta aquest tic
            self._dbg_no_feedback_count += 1                                    # sumem a self._dbg_no_feedback_count
            if self._dbg_tick % 50 == 0:                                        # cada 50 tics (~1 s)
                missing = set(self.target_mids) - ids_present                   # quins falten
                print(f"[HG debug] tick #{self._dbg_tick}: feedback INCOMPLET · "
                      f"present={sorted(ids_present)} · falten={sorted(missing)} · "
                      f"no_fb_count={self._dbg_no_feedback_count}")
            _send_zero_current_to_all(self.bus, self.target_mids)               # per seguretat, 0 mA a tots
            return                                                              # retornem sense valor

        # ------------------------------------------------------------------
        # 2) Convertir counts d'encoder → q en radians per a tots els joints
        # ------------------------------------------------------------------
        q_counts = np.array(                                                    # array de counts en l'ordre del cfg
            [st[m].actuator_position if m in st else 0
             for m in self.cfg.motor_ids],
            dtype=np.int64,
        )
        q_rad = motor_counts_to_q(q_counts, self.cfg)                           # q_rad ← counts → rad
        q_deg = np.degrees(q_rad)                                               # q_deg ← per imprimir i comprovar límits
        self._q_last = q_rad.copy()                                             # recordem l'última q bona

        # DEBUG puntual
        if self._dbg_tick % 50 == 0:                                            # cada ~1 s
            vels = [st[m].velocity for m in self.cfg.motor_ids]                 # velocitats brutes
            print(f"[HG raw] counts={q_counts.tolist()} vels={vels}")

        # ------------------------------------------------------------------
        # 3) Notificar la q al viewer (callback opcional)
        # ------------------------------------------------------------------
        if self.on_q_updated:                                                   # si tenim callback
            try:                                                                # intentem el bloc següent
                self.on_q_updated(q_rad)                                        # avisem al viewer
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[handguide] callback on_q_updated: {e}")

        # ------------------------------------------------------------------
        # 4) Comprovar límits de software per cada joint
        # ------------------------------------------------------------------
        for j_idx in TARGET_JOINTS:                                             # iterem: j_idx in TARGET_JOINTS
            cfg_j = JOINT_CONFIGS[j_idx]                                        # config d'aquest joint
            if (q_deg[j_idx] <= cfg_j["q_min_deg"]                              # si estem fora dels límits
                    or q_deg[j_idx] >= cfg_j["q_max_deg"]):
                print(f"[handguide] fora de límit a j{j_idx+1}: {q_deg[j_idx]:.2f}°")
                _send_zero_current_to_all(self.bus, self.target_mids)           # parem tot
                if self.on_limit_hit:                                           # si tenim callback de límit
                    self.on_limit_hit(j_idx)                                    # avisem
                return                                                          # i sortim

        # ------------------------------------------------------------------
        # 5) Calcular compensació de gravetat (per tots els joints alhora)
        # ------------------------------------------------------------------
        try:                                                                    # intentem el bloc següent
            tau = gravity_torques(self.robot, q_rad, self.dyn_params)           # tau (N·m) necessari per aguantar el pes
            hg = torques_to_currents_handguide(tau, self.dyn_params)            # tau → mA (té en compte kt per motor)
            I_grav_all = hg["I_grav"]                                           # array amb els 6 corrents de gravetat
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[handguide] error gravetat: {e}")
            _send_zero_current_to_all(self.bus, self.target_mids)               # per seguretat, 0 mA a tots
            if self.on_error:                                                   # avisem si cal
                self.on_error(f"gravetat: {e}")
            return                                                              # retornem sense valor

        # ------------------------------------------------------------------
        # 6) Calcular i enviar la corrent de cada joint
        # ------------------------------------------------------------------
        for j_idx in TARGET_JOINTS:                                             # iterem: j_idx in TARGET_JOINTS
            cfg_j = JOINT_CONFIGS[j_idx]                                        # config d'aquest joint
            mid   = self.cfg.motor_ids[j_idx]                                   # id real del motor
            vel   = st[mid].velocity                                            # velocitat bruta llegida al bus

            # 6a) Filtre passa-baixos exponencial de la velocitat
            # v_filt' = a·v + (1-a)·v_filt
            # Al primer tic, el prefixem al valor cru per no partir de 0.
            if self._first_iter:                                                # si és el primer tic
                self._v_filt[j_idx] = float(vel)                                # prefixem
            else:                                                               # altres tics
                a = cfg_j["v_alpha"]                                            # factor del filtre
                self._v_filt[j_idx] = a * vel + (1.0 - a) * self._v_filt[j_idx]  # EMA

            abs_v = abs(self._v_filt[j_idx])                                    # mòdul de la velocitat filtrada

            # 6b) Rampa d'alpha: factor d'assistència de 0 a 1 entre v_low i v_high
            if abs_v <= cfg_j["v_low"]:                                         # per sota de v_low → sense assistència
                alpha = 0.0
            elif abs_v >= cfg_j["v_high"]:                                      # per sobre de v_high → assistència màxima
                alpha = 1.0
            else:                                                               # al mig → rampa lineal
                alpha = ((abs_v - cfg_j["v_low"])
                         / (cfg_j["v_high"] - cfg_j["v_low"]))

            I_grav = float(I_grav_all[j_idx]) * cfg_j["k_trim"]                 # corrent de gravetat (amb trim fi)

            # 6c) Assistència direccional (només si l'usuari ja es mou)
            I_assist = 0.0                                                      # per defecte, cap assistència
            if alpha > 0.0:                                                     # si hi ha moviment detectat
                sign_v = np.sign(self._v_filt[j_idx])                           # direcció del moviment (+/-1)

                if j_idx == 0:                                                  # J1 (base) és horitzontal → empenta simètrica
                    I_push = cfg_j["i_push_up"]
                else:                                                           # Verticals: diferent empenta segons direcció
                    sign_grav = np.sign(I_grav)                                 # direcció de la gravetat
                    is_against_grav = (sign_v == sign_grav)                     # anem contra la gravetat?
                    I_push = (cfg_j["i_push_up"] if is_against_grav             # si pugem contra gravetat → push_up
                              else cfg_j["i_push_dn"])                          # si baixem a favor → push_dn

                I_fric = self._fric_mA[j_idx]                                   # mA per vèncer fricció interna
                I_assist = alpha * sign_v * (I_fric + I_push)                   # assistència final

            # 6d) Corrent total i saturació
            I_cmd = I_grav + (cfg_j["k_assist"] * I_assist)                     # I = gravetat + assistència
            I_target = int(I_cmd)                                               # a enter per enviar
            if abs(I_target) > MAX_CURRENT_ABS:                                 # si passem del sostre
                I_target = int(np.sign(I_target) * MAX_CURRENT_ABS)             # el saturem

            _send_current_to_motor(self.bus, mid, I_target)                     # ENVIEM al motor

        # ------------------------------------------------------------------
        # 7) Debug periòdic (cada ~1 s)
        # ------------------------------------------------------------------
        if self._dbg_tick % 50 == 0:                                            # cada 50 tics a 50 Hz = 1 s
            v0, v1, v2 = self._v_filt[0], self._v_filt[1], self._v_filt[2]      # agafem 3 velocitats filtrades
            print(f"[HG debug] "
                  f"q(deg)=[{q_deg[0]:+6.1f}, {q_deg[1]:+6.1f}, {q_deg[2]:+6.1f}] | "
                  f"v_filt=[{v0:+5.1f}, {v1:+5.1f}, {v2:+5.1f}] | "
                  f"I_grav(mA)=[{I_grav_all[0]:+5.0f}, {I_grav_all[1]:+5.0f}, {I_grav_all[2]:+5.0f}]")

        self._first_iter = False                                                # ja no és el primer tic
