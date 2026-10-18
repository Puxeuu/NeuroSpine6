"""
Aplicació principal del cobot NeuroSpine6.

Combina en una única finestra:
    - Visualització 3D del robot (PyVista + Qt)
    - Sliders per als 6 joints (mode FK) i per XYZ/RPY del TCP (mode IK)
    - Solver DLS que resol IK en temps real (100 Hz)
    - Detecció de col·lisions amb marge de seguretat
    - Connexió amb els motors reals SitoDrive
    - Connexió amb el mando 6DOF via serial + control teleoperat
    - Visualització 3D del mando al costat del cobot

Modes de control:
    IK      : sliders cartesians del TCP; el DLS resol les q.
    FK      : sliders per cada joint; sense DLS.
    MANDO   : el mando físic controla el target del cobot amb filtre EMA
              i limitador de velocitat cartesiana.

Executa:
    python -m scripts.viewer_cobot
"""

import serial                                                                   # importem el mòdul serial
import sys, os                                                                  # importem el mòdul sys, os
import time                                                                     # importem el mòdul time
import threading                                                                # importem el mòdul threading

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))              # ROOT ← resultat de dirname()
sys.path.insert(0, ROOT)                                                        # executem insert()

import numpy as np                                                              # importem el mòdul numpy as np
import pyvista as pv                                                            # importem el mòdul pyvista as pv
from pyvistaqt import QtInteractor                                              # importem QtInteractor des de pyvistaqt
from qtpy import QtWidgets, QtCore                                              # importem QtWidgets, QtCore des de qtpy
from qtpy.QtCore import Qt                                                      # importem Qt des de qtpy.QtCore

pv.OFF_SCREEN = False                                                           # assignem pv.OFF_SCREEN

from src.model.urdf_loader import load_urdf                                     # importem load_urdf des de src.model.urdf_loader
from src.kinematics.forward import forward_kinematics                           # importem forward_kinematics des de src.kinematics.forward
from src.kinematics.jacobian import compute_jacobian                            # importem compute_jacobian des de src.kinematics.jacobian
from src.kinematics.manipulability import yoshikawa_position, adaptive_damping  # importem yoshikawa_position, adaptive_damping des de src.kinematics.manipulability
from src.control.pose_error import pose_error                                   # importem pose_error des de src.control.pose_error
from src.control.dls_solver import dls                                          # importem dls des de src.control.dls_solver
from src.collision.collision_loader import load_collisions                      # importem load_collisions des de src.collision.collision_loader
from src.collision.checker import check_collisions_with_margin                  # importem check_collisions_with_margin des de src.collision.checker
from src.math.transforms import Rx, Ry, Rz, make_T                              # importem Rx, Ry, Rz, make_T des de src.math.transforms
from src.hardware.motor_config import load_motor_config                         # importem load_motor_config des de src.hardware.motor_config
from src.hardware.motor_controller import MotorController                       # importem MotorController des de src.hardware.motor_controller
from scripts.viewer_capcals_patch import (capcal_manager_init, capcal_add_selector_to_options_panel,)  # importem (capcal_manager_init, capcal_add_selector_to_options_panel,) des de scripts.viewer_capcals_patch


# ==============================================================================
# PALETA DE COLORS (estil Apple)
# ==============================================================================

COLORS = {                                                                      # assignem COLORS
    'bg_window':           '#000000',
    'panel':               '#1c1c1e',
    'panel_border':        '#2c2c2e',
    'accent_q':            '#0a84ff',
    'accent_x':            '#30d158',
    'accent_warn':         '#ff9f0a',
    'accent_err':          '#ff3b30',
    'text':                '#ffffff',
    'text_dim':            '#8e8e93',
    'button_bg':           '#2c2c2e',
    'button_hover':        '#3a3a3c',
    'bg_3d':               '#1c1c1e',
    'robot_color':         '#8e8e93',
    'target_reached_tint': '#a0a0a5',
}


# ==============================================================================
# LÍMITS DELS JOINTS DEL COBOT (rad)
# ==============================================================================

JOINT_LOWER = np.array([-np.pi/2, -np.pi/6, -np.pi/2.2, -np.pi/2, -np.pi/2, -np.pi])  # JOINT_LOWER ← resultat de array()
JOINT_UPPER = np.array([ np.pi/2,  np.pi/2,  np.pi/2,   np.pi/2,  np.pi/6,  np.pi])  # JOINT_UPPER ← resultat de array()


# ==============================================================================
# PARÀMETRES DE CONTROL
# ==============================================================================

CONTROL_DT = 0.01                                                               # assignem CONTROL_DT
KP_POS = 5.0                                                                    # assignem KP_POS
KP_ORI = 3.0                                                                    # assignem KP_ORI
MAX_JOINT_SPEED = np.deg2rad(60)                                              # 60°/s (abans 200°/s). Baixat per fer els moviments d'aproximació i avanç 3-4× més lents (seguretat clínica).

# ---- VELOCITATS LENTES (graus per segon, màxim per joint) -------------------
# Canvia aquests dos números per provar. Referència: la resta de moviments
# van a 60°/s (MAX_JOINT_SPEED). Més petit = més lent.
HOME_SPEED_DEG_S     = 30                                                       # botó Home (2× més lent que 60°/s; abans 20)
APPROACH_SPEED_DEG_S = 20                                                       # "Anar a approach" (1 cm del cargol, 3× més lent)

HOME_JOINT_SPEED     = np.deg2rad(HOME_SPEED_DEG_S)                             # rad/s
APPROACH_JOINT_SPEED = np.deg2rad(APPROACH_SPEED_DEG_S)                         # rad/s

LAMBDA_MIN = 0.005                                                              # assignem LAMBDA_MIN
LAMBDA_MAX = 0.3                                                                # assignem LAMBDA_MAX
W_THRESHOLD = 0.003                                                             # assignem W_THRESHOLD

ERROR_SMOOTH_ALPHA = 0.15                                                       # assignem ERROR_SMOOTH_ALPHA

TOL_POS = 0.001                                                                 # assignem TOL_POS
TOL_ORI = 0.01                                                                  # assignem TOL_ORI

SAFETY_MARGIN = 0.005                                                           # assignem SAFETY_MARGIN

XYZ_RANGE = 0.20                            # sliders XYZ: ±10 cm respecte T_ref (abans ±5 cm)
RPY_RANGE = np.pi / 3                       # sliders RPY: ±60° respecte T_ref (abans ±30°)
XYZ_MULT = 4000                                                                 # 1 pas = 1/4 mm
RPY_MULT = 4000


# ==============================================================================
# VIEWERDLSPRO - Finestra principal
# ==============================================================================

class ViewerDLSPro(QtWidgets.QMainWindow):                                      # definim la classe ViewerDLSPro

    def __init__(self):                                                         # definim la funció __init__()
        super().__init__()                                                      # executem super()

        # ---- Estat del mando serial ------------------------------------------
        self.mando_port = "COM5"                                                # assignem self.mando_port
        self.mando_baud = 115200                                                # assignem self.mando_baud
        self.mando_serial = None                                                # assignem self.mando_serial
        self.mando_thread = None                                                # assignem self.mando_thread
        self.mando_active = False                                               # assignem self.mando_active
        self.mando_raw = [0, 0, 0, 0, 0, 0]                                     # assignem self.mando_raw
        self._mando_svc = None                                                  # referència al MandoService singleton (si es reutilitza)

        # ---- Control del cobot amb el mando ----------------------------------
        # Escala mando → braç (1.0 = 1 mm del mando fa 1 mm al braç).
        # Valors baixos = moviments petits i precisos.
        self.mando_gain_xyz = 0.2                                               # abans 0.8
        self.mando_gain_rpy = 0.2                                               # abans 0.8
        self.mando_ema_alpha = 0.25                                             # assignem self.mando_ema_alpha
        self.mando_comp_filt = None                                             # assignem self.mando_comp_filt
        self.mando_max_lin_speed = 0.03                                         # m/s (abans 0.08)
        self.mando_max_ang_speed = np.deg2rad(25)                               # rad/s (abans 60°/s)
        self.mando_cmd_pose = None                                              # assignem self.mando_cmd_pose
        self.mando_last_tick = None                                             # assignem self.mando_last_tick

        self.mando_zero_raw = None                                              # assignem self.mando_zero_raw
        self.mando_zero_tcp = None                                              # assignem self.mando_zero_tcp

        # ---- Càrrega de fitxers de configuració ------------------------------
        self.robot = load_urdf(os.path.join(ROOT, "config", "cobot.urdf"))      # self.robot ← resultat de load_urdf()
        self.prims = load_collisions(os.path.join(ROOT, "config", "collision_geometry.yaml"))  # self.prims ← resultat de load_collisions()
        capcal_manager_init(self)                                               # executem capcal_manager_init()
        self.motor_config = load_motor_config(os.path.join(ROOT, "config", "motor_config.yaml"))  # self.motor_config ← resultat de load_motor_config()

        # ---- Motors ----------------------------------------------------------
        from scripts.neurospine_app.preferences import load_preferences, DEFAULT_PREFS
        try:                                                                    # port del robot des de Preferències (ex: tcp://10.42.0.2:5000)
            robot_port = load_preferences().get("robot_port", DEFAULT_PREFS["robot_port"])
        except Exception as e:                                                  # sense preferències → el port per defecte (DEFAULT_PREFS)
            print(f"[motors] no s'han pogut llegir les preferències: {e}")
            robot_port = DEFAULT_PREFS["robot_port"]
        print(f"[motors] port del robot: {robot_port}")
        self.motor_controller = MotorController(self.motor_config, port=robot_port)  # self.motor_controller ← resultat de MotorController()
        self.mirror_active = False                                              # assignem self.mirror_active

        # ---- Mando 3D (URDF + offset visual) ---------------------------------
        self.mando_robot = load_urdf(os.path.join(ROOT, "config", "mando6dof.urdf"))  # self.mando_robot ← resultat de load_urdf()
        self.mando_offset = np.array([0.17, -0.2, 0.0])                         # self.mando_offset ← resultat de array()
        self.mando_yaw_deg = 180.0                                              # assignem self.mando_yaw_deg
        self.mando_actors = {}                                                  # assignem self.mando_actors
        self.mando_pulsos_per_volta = 600.0                                     # assignem self.mando_pulsos_per_volta
        self.mando_diametre_corro_mm = 40.0                                     # assignem self.mando_diametre_corro_mm

        # ---- Estat cinemàtic del cobot ---------------------------------------
        self.q = np.zeros(6)                                                    # self.q ← resultat de zeros()
        self.poses = forward_kinematics(self.robot, self.q)                     # self.poses ← resultat de forward_kinematics()

        # NOU MODEL: T_ref és la referència sobre la qual els sliders
        # XYZ+RPY apliquen desplaçaments. S'actualitza cada vegada que
        # es crida sync_target_to_current().
        self.T_ref_pos = self.poses["tcp_link"][:3, 3].copy()                   # self.T_ref_pos ← resultat de copy()
        self.R_target_initial = self.poses["tcp_link"][:3, :3].copy()           # self.R_target_initial ← resultat de copy()

        # T_target inicial = T_ref (sliders a 0 → sense desplaçament)
        self.T_target = self.poses["tcp_link"].copy()                           # self.T_target ← resultat de copy()
        self.target_xyz = np.array([0.0, 0.0, 0.0])      # desplaçament, no posició
        self.target_rpy = np.array([0.0, 0.0, 0.0])                             # self.target_rpy ← resultat de array()

        self.e_smoothed = np.zeros(6)                                           # self.e_smoothed ← resultat de zeros()
        self.current_status = "ok"                                              # assignem self.current_status
        self.blocked_violating = set()                                          # self.blocked_violating ← resultat de set()

        self._updating_sliders = False                                          # assignem self._updating_sliders

        # Mode actual: "IK" (per defecte), "FK" o "MANDO"
        self.mode = "IK"                                                        # assignem self.mode

        # Flag pel mode HAND-GUIDE natiu: quan és True, el bucle 100Hz
        # NO modifica self.q (perquè un controller extern la sobreescriu
        # amb la q real dels motors) ni envia res al bus (mirror OFF).
        self._handguide_active = False                                          # assignem self._handguide_active

        # Broca del J6 (IK5): mentre gira, l'IK fa servir aquest valor fix per
        # a q[5] en comptes del real. Així el gir de la broca no mou J1-J5 i
        # els sliders XYZ/RPY continuen funcionant. None = fes servir q[5].
        # (ho gestiona neurospine_app/modes/j6_drill.py)
        self._j6_ik_override = None                                             # assignem self._j6_ik_override

        # Centre del recorregut del J6 (±π al voltant). En sortir d'IK5 el
        # workspace el posa a la posició on ha quedat la broca: l'slider del
        # J6 queda centrat i es pot moure mitja volta a cada banda.
        self._j6_center = 0.0                                                   # assignem self._j6_center

        # Moviments lents (HOME_SPEED_DEG_S / APPROACH_SPEED_DEG_S):
        #   _homing_goal         Home: q va cap a zero en línia recta
        #   _slow_until_reached  "Anar a approach": IK limitat fins arribar
        self._homing_goal = None                                                # assignem self._homing_goal
        self._slow_until_reached = False                                        # assignem self._slow_until_reached

        # ---- Finestra i timer ------------------------------------------------
        self.setWindowTitle("Cobot OS — DLS Pro Edition")                       # executem setWindowTitle()
        self.resize(1700, 950)                                                  # executem resize()
        self._apply_style()                                                     # executem _apply_style()
        self._build_ui()                                                        # executem _build_ui()
        self._setup_3d_scene()                                                  # executem _setup_3d_scene()
        self._update_mode_ui()                                                  # executem _update_mode_ui()

        self.timer = QtCore.QTimer()                                            # self.timer ← resultat de QTimer()
        self.timer.timeout.connect(self._teleop_step)                           # executem connect()
        self.timer.start(int(CONTROL_DT * 1000))                                # executem start()


    # ==========================================================================
    # SYNC DEL TARGET AL TCP ACTUAL — funció pública
    # ==========================================================================

    def sync_target_to_current(self):                                           # definim la funció sync_target_to_current()
        """
        Estableix la pose ACTUAL del TCP com a nova referència de IK.

        NOU MODEL: els sliders XYZ+RPY són DESPLAÇAMENTS relatius a
        aquesta referència. Després d'aquesta crida:
            - T_ref_pos     = posició actual del TCP
            - R_ref         = orientació actual del TCP
            - Tots els 6 sliders = 0
            - T_target      = T_ref (no hi ha desplaçament)

        Cal cridar-ho SEMPRE que:
          - S'entra en mode IK des d'un altre mode
          - Es reseteja la referència (per exemple, després d'un approach)
          - L'usuari vol "començar de nou" des de la pose actual
        """
        # Recalculem la FK per garantir que poses reflecteix la q actual
        self.poses = forward_kinematics(self.robot, self.q)                     # self.poses ← resultat de forward_kinematics()
        T_actual = self.poses["tcp_link"]                                       # assignem T_actual

        # Nova referència = pose actual del TCP
        self.T_ref_pos = T_actual[:3, 3].copy()                                 # self.T_ref_pos ← resultat de copy()
        self.R_target_initial = T_actual[:3, :3].copy()                         # self.R_target_initial ← resultat de copy()

        # Tots els sliders a 0 (referència = origen del rang)
        self.target_xyz = np.array([0.0, 0.0, 0.0])                             # self.target_xyz ← resultat de array()
        self.target_rpy = np.array([0.0, 0.0, 0.0])                             # self.target_rpy ← resultat de array()

        # Sense desplaçament, T_target = T_ref
        self.T_target = T_actual.copy()                                         # self.T_target ← resultat de copy()
        self.e_smoothed = np.zeros(6)                                           # self.e_smoothed ← resultat de zeros()

        # Refresca el triad target visualment
        for actor in self.target_actors:                                        # iterem: actor in self.target_actors
            actor.user_matrix = self.T_target                                   # assignem actor.user_matrix

        # Sync tots els sliders amb els nous valors (tots a 0)
        self._sync_all_sliders(recenter=True)                                   # assignem self._sync_all_sliders(recenter


    # ==========================================================================
    # ESTIL VISUAL (Qt stylesheet)
    # ==========================================================================

    def _apply_style(self):                                                     # definim la funció _apply_style()
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {COLORS['bg_window']}; }}
            QWidget#CentralWidget {{ background-color: {COLORS['bg_window']}; }}
            QWidget {{ color: {COLORS['text']};
                       font-family: -apple-system, 'Segoe UI', Helvetica, sans-serif;
                       font-size: 13px; }}

            QFrame#ApplePanel {{
                background-color: {COLORS['panel']};
                border-radius: 16px;
                border: 1px solid {COLORS['panel_border']};
            }}
            QLabel, QSlider, QCheckBox {{ background-color: transparent; }}

            QPushButton {{
                background-color: {COLORS['button_bg']};
                border: 1px solid #3a3a3c;
                border-radius: 8px;
                padding: 10px 14px;
                font-weight: 600;
                font-size: 13px;
            }}
            QPushButton:hover {{ background-color: {COLORS['button_hover']}; }}
            QPushButton:pressed {{ background-color: #505055; }}
            QPushButton:checked {{
                background-color: {COLORS['accent_q']};
                border: 1px solid {COLORS['accent_q']};
            }}

            QTextEdit {{
                background-color: #121214;
                border: 1px solid {COLORS['panel_border']};
                border-radius: 12px;
                font-family: 'SF Mono', Consolas, monospace;
                font-size: 12px;
                padding: 12px;
                color: #e5e5ea;
            }}

            QSlider::groove:horizontal {{ background: #3a3a3c; height: 6px; border-radius: 3px; }}
            QSlider::sub-page:horizontal {{ background: {COLORS['accent_q']}; border-radius: 3px; }}
            QSlider::handle:horizontal {{ background: #ffffff; border: 1px solid #8e8e93;
                                           width: 16px; height: 16px;
                                           margin: -5px 0; border-radius: 8px; }}
            QSlider#CartSlider::sub-page:horizontal {{ background: {COLORS['accent_x']}; }}

            QCheckBox {{ spacing: 8px; }}
            QCheckBox::indicator {{ width: 18px; height: 18px;
                                     border-radius: 4px; border: 1px solid #8e8e93; }}
            QCheckBox::indicator:checked {{ background-color: {COLORS['accent_q']}; border: none; }}

            QLabel {{ font-weight: 500; letter-spacing: 0.3px; }}
            QLabel#Title {{ font-size: 14px; font-weight: 700; color: #ffffff;
                            margin-bottom: 8px; text-transform: uppercase;
                            letter-spacing: 1px; }}

            QSlider {{ background-color: transparent; }}
        """)


    # ==========================================================================
    # CONSTRUCCIÓ DE LA UI
    # ==========================================================================

    def _build_ui(self):                                                        # definim la funció _build_ui()
        central = QtWidgets.QWidget()                                           # central ← resultat de QWidget()
        central.setObjectName("CentralWidget")                                  # executem setObjectName()
        self.setCentralWidget(central)                                          # executem setCentralWidget()
        main = QtWidgets.QHBoxLayout(central)                                   # main ← resultat de QHBoxLayout()
        main.setContentsMargins(15, 15, 15, 15)                                 # executem setContentsMargins()
        main.setSpacing(15)                                                     # executem setSpacing()

        left = QtWidgets.QVBoxLayout()                                          # left ← resultat de QVBoxLayout()
        left.setSpacing(15)                                                     # executem setSpacing()

        self.view_container = QtWidgets.QFrame()                                # self.view_container ← resultat de QFrame()
        self.view_container.setObjectName("ApplePanel")                         # executem setObjectName()
        vc_layout = QtWidgets.QVBoxLayout(self.view_container)                  # vc_layout ← resultat de QVBoxLayout()
        vc_layout.setContentsMargins(0, 0, 0, 0)                                # executem setContentsMargins()

        self.plotter = QtInteractor(                                            # assignem self.plotter
            self.view_container,
            multi_samples=0,
            line_smoothing=False,
            point_smoothing=False,
            polygon_smoothing=False,
        )
        vc_layout.addWidget(self.plotter.interactor)                            # executem addWidget()
        left.addWidget(self.view_container, stretch=7)                          # assignem left.addWidget(self.view_container, stretch

        bottom = QtWidgets.QHBoxLayout()                                        # bottom ← resultat de QHBoxLayout()
        bottom.setSpacing(15)                                                   # executem setSpacing()
        bottom.addWidget(self._create_joints_panel(), 1)                        # executem addWidget()
        bottom.addWidget(self._create_cart_panel(), 1)                          # executem addWidget()
        left.addLayout(bottom, stretch=0)                                       # assignem left.addLayout(bottom, stretch

        main.addLayout(left, stretch=7)                                         # assignem main.addLayout(left, stretch

        right = QtWidgets.QVBoxLayout()                                         # right ← resultat de QVBoxLayout()
        right.setSpacing(15)                                                    # executem setSpacing()

        top = QtWidgets.QFrame()                                                # top ← resultat de QFrame()
        top.setObjectName("ApplePanel")                                         # executem setObjectName()
        top_layout = QtWidgets.QHBoxLayout(top)                                 # top_layout ← resultat de QHBoxLayout()
        top_layout.setContentsMargins(15, 15, 15, 15)                           # executem setContentsMargins()
        top_layout.setSpacing(15)                                               # executem setSpacing()

        self.console_text = QtWidgets.QTextEdit()                               # self.console_text ← resultat de QTextEdit()
        self.console_text.setReadOnly(True)                                     # executem setReadOnly()
        top_layout.addWidget(self.console_text, stretch=6)                      # assignem top_layout.addWidget(self.console_text, stretch

        btns = QtWidgets.QVBoxLayout()                                          # btns ← resultat de QVBoxLayout()
        btns.setSpacing(10)                                                     # executem setSpacing()

        self.btn_mode = QtWidgets.QPushButton("Mode: IK")                       # self.btn_mode ← resultat de QPushButton()
        self.btn_mode.setStyleSheet(                                            # executem setStyleSheet()
            f"background-color: {COLORS['accent_x']}; color: white; font-weight: bold;"
        )
        self.btn_mode.clicked.connect(self._on_mode_toggle)                     # executem connect()
        btns.addWidget(self.btn_mode)                                           # executem addWidget()

        self.btn_zero = QtWidgets.QPushButton("Move to zero")                   # self.btn_zero ← resultat de QPushButton()
        self.btn_zero.clicked.connect(self._on_move_to_zero)                    # executem connect()
        btns.addWidget(self.btn_zero)                                           # executem addWidget()

        self.btn_connect = QtWidgets.QPushButton("Connect motors (SIM)")        # self.btn_connect ← resultat de QPushButton()
        self.btn_connect.setCheckable(True)                                     # executem setCheckable()
        self.btn_connect.clicked.connect(self._on_connect_motors)               # executem connect()
        btns.addWidget(self.btn_connect)                                        # executem addWidget()

        self.btn_mirror = QtWidgets.QPushButton("Mirror OFF")                   # self.btn_mirror ← resultat de QPushButton()
        self.btn_mirror.setCheckable(True)                                      # executem setCheckable()
        self.btn_mirror.setEnabled(False)                                       # executem setEnabled()
        self.btn_mirror.clicked.connect(self._on_mirror_toggle)                 # executem connect()
        btns.addWidget(self.btn_mirror)                                         # executem addWidget()

        self.btn_mando = QtWidgets.QPushButton("Mando 6DOF: OFF")               # self.btn_mando ← resultat de QPushButton()
        self.btn_mando.setCheckable(True)                                       # executem setCheckable()
        self.btn_mando.clicked.connect(self._on_mando_toggle)                   # executem connect()
        btns.addWidget(self.btn_mando)                                          # executem addWidget()

        self.btn_mando_control = QtWidgets.QPushButton("Mando Control: OFF")    # self.btn_mando_control ← resultat de QPushButton()
        self.btn_mando_control.setCheckable(True)                               # executem setCheckable()
        self.btn_mando_control.setEnabled(False)                                # executem setEnabled()
        self.btn_mando_control.clicked.connect(self._on_mando_control_toggle)   # executem connect()
        btns.addWidget(self.btn_mando_control)                                  # executem addWidget()

        self.btn_mando_zero = QtWidgets.QPushButton("Set Mando Zero")           # self.btn_mando_zero ← resultat de QPushButton()
        self.btn_mando_zero.setEnabled(False)                                   # executem setEnabled()
        self.btn_mando_zero.clicked.connect(self._on_mando_set_zero)            # executem connect()
        btns.addWidget(self.btn_mando_zero)                                     # executem addWidget()

        self.btn_routes = QtWidgets.QPushButton("Plan Routes")                  # self.btn_routes ← resultat de QPushButton()
        self.btn_routes.setStyleSheet(                                          # executem setStyleSheet()
            f"background-color: {COLORS['accent_q']}; color: white; font-weight: bold;"
        )
        self.btn_routes.setEnabled(False)                                       # executem setEnabled()
        btns.addWidget(self.btn_routes)                                         # executem addWidget()

        self.btn_stop = QtWidgets.QPushButton("STOP")                           # self.btn_stop ← resultat de QPushButton()
        self.btn_stop.setStyleSheet(                                            # executem setStyleSheet()
            f"background-color: {COLORS['accent_err']}; color: white; border: none; font-weight: bold;"
        )
        self.btn_stop.clicked.connect(self._on_stop)                            # executem connect()
        btns.addWidget(self.btn_stop)                                           # executem addWidget()

        btns.addStretch()                                                       # executem addStretch()
        top_layout.addLayout(btns, stretch=4)                                   # assignem top_layout.addLayout(btns, stretch
        right.addWidget(top, stretch=8)                                         # assignem right.addWidget(top, stretch

        options = QtWidgets.QFrame()                                            # options ← resultat de QFrame()
        options.setObjectName("ApplePanel")                                     # executem setObjectName()
        opt = QtWidgets.QVBoxLayout(options)                                    # opt ← resultat de QVBoxLayout()
        opt.setContentsMargins(20, 15, 20, 15)                                  # executem setContentsMargins()
        title = QtWidgets.QLabel("OPTIONS")                                     # title ← resultat de QLabel()
        title.setObjectName("Title")                                            # executem setObjectName()
        opt.addWidget(title)                                                    # executem addWidget()

        opt_row = QtWidgets.QHBoxLayout()                                       # opt_row ← resultat de QHBoxLayout()
        self.chk_collisions = QtWidgets.QCheckBox("Collision detection")        # self.chk_collisions ← resultat de QCheckBox()
        self.chk_collisions.setChecked(True)                                    # executem setChecked()
        opt_row.addWidget(self.chk_collisions)                                  # executem addWidget()
        self.chk_target = QtWidgets.QCheckBox("Show target")                    # self.chk_target ← resultat de QCheckBox()
        self.chk_target.setChecked(True)                                        # executem setChecked()
        self.chk_target.toggled.connect(self._on_target_visibility)             # executem connect()
        opt_row.addWidget(self.chk_target)                                      # executem addWidget()
        opt_row.addStretch()                                                    # executem addStretch()
        opt.addLayout(opt_row)                                                  # executem addLayout()
        capcal_add_selector_to_options_panel(self, opt)                         # executem capcal_add_selector_to_options_panel()
        right.addWidget(options, stretch=2)                                     # assignem right.addWidget(options, stretch

        self._create_overlay_buttons()                                          # executem _create_overlay_buttons()
        main.addLayout(right, stretch=0)                                        # assignem main.addLayout(right, stretch


    # ==========================================================================
    # PANELLS DE SLIDERS
    # ==========================================================================

    def _create_joints_panel(self):                                             # definim la funció _create_joints_panel()
        self.joints_panel = QtWidgets.QFrame()                                  # self.joints_panel ← resultat de QFrame()
        self.joints_panel.setObjectName("ApplePanel")                           # executem setObjectName()
        layout = QtWidgets.QVBoxLayout(self.joints_panel)                       # layout ← resultat de QVBoxLayout()
        layout.setContentsMargins(20, 15, 20, 15)                               # executem setContentsMargins()


        grid = QtWidgets.QGridLayout()                                          # grid ← resultat de QGridLayout()
        grid.setVerticalSpacing(10)                                             # executem setVerticalSpacing()
        grid.setHorizontalSpacing(15)                                           # executem setHorizontalSpacing()

        self.joint_sliders = []                                                 # assignem self.joint_sliders
        self.joint_labels = []                                                  # assignem self.joint_labels

        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]                      # assignem joint_names
        for i in range(6):                                                      # iterem: i in range(6)
            name_lbl = QtWidgets.QLabel(joint_names[i])                         # name_lbl ← resultat de QLabel()
            name_lbl.setFixedWidth(28)                                          # executem setFixedWidth()
            name_lbl.setStyleSheet("font-weight: bold; color: #aeaeb2;")        # executem setStyleSheet()

            slider = QtWidgets.QSlider(Qt.Horizontal)                           # slider ← resultat de QSlider()
            slider.setMinimum(int(JOINT_LOWER[i] * 1000))                       # executem setMinimum()
            slider.setMaximum(int(JOINT_UPPER[i] * 1000))                       # executem setMaximum()
            slider.setValue(0)                                                  # executem setValue()

            val_lbl = QtWidgets.QLabel("0.000")                                 # val_lbl ← resultat de QLabel()
            val_lbl.setFixedWidth(60)                                           # executem setFixedWidth()
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)               # executem setAlignment()
            val_lbl.setStyleSheet("font-family: 'SF Mono', monospace; color: #aeaeb2;")  # executem setStyleSheet()

            grid.addWidget(name_lbl, i, 0)                                      # executem addWidget()
            grid.addWidget(slider, i, 1)                                        # executem addWidget()
            grid.addWidget(val_lbl, i, 2)                                       # executem addWidget()

            self.joint_sliders.append(slider)                                   # executem append()
            self.joint_labels.append(val_lbl)                                   # executem append()
            slider.valueChanged.connect(lambda v, idx=i: self._on_joint_slide(idx, v))  # slider.valueChanged.connect(lambda v, idx ← resultat de _on_joint_slide()

        layout.addLayout(grid)                                                  # executem addLayout()
        layout.addStretch()                                                     # executem addStretch()
        return self.joints_panel                                                # retornem: self.joints_panel


    def _create_cart_panel(self):                                               # definim la funció _create_cart_panel()
        self.cart_panel = QtWidgets.QFrame()                                    # self.cart_panel ← resultat de QFrame()
        self.cart_panel.setObjectName("ApplePanel")                             # executem setObjectName()
        layout = QtWidgets.QVBoxLayout(self.cart_panel)                         # layout ← resultat de QVBoxLayout()
        layout.setContentsMargins(20, 15, 20, 15)                               # executem setContentsMargins()

        grid = QtWidgets.QGridLayout()                                          # grid ← resultat de QGridLayout()
        grid.setVerticalSpacing(10)                                             # executem setVerticalSpacing()
        grid.setHorizontalSpacing(15)                                           # executem setHorizontalSpacing()

        self.xyz_sliders = []                                                   # assignem self.xyz_sliders
        self.xyz_labels = []                                                    # assignem self.xyz_labels
        self.rpy_sliders = []                                                   # assignem self.rpy_sliders
        self.rpy_labels = []                                                    # assignem self.rpy_labels

        xyz_names = ["X", "Y", "Z"]                                             # assignem xyz_names
        for i in range(3):                                                      # iterem: i in range(3)
            name_lbl = QtWidgets.QLabel(xyz_names[i])                           # name_lbl ← resultat de QLabel()
            name_lbl.setFixedWidth(28)                                          # executem setFixedWidth()
            name_lbl.setStyleSheet("font-weight: bold; color: #aeaeb2;")        # executem setStyleSheet()

            slider = QtWidgets.QSlider(Qt.Horizontal)                           # slider ← resultat de QSlider()
            slider.setObjectName("CartSlider")                                  # executem setObjectName()
            # NOU MODEL: sliders XYZ són desplaçaments (±XYZ_RANGE),
            # sempre centrats a 0.
            slider.setMinimum(int(-XYZ_RANGE * XYZ_MULT))                       # executem setMinimum()
            slider.setMaximum(int( XYZ_RANGE * XYZ_MULT))                       # executem setMaximum()
            slider.setValue(0)                                                  # executem setValue()

            val_lbl = QtWidgets.QLabel("0.000")                                 # val_lbl ← resultat de QLabel()
            val_lbl.setFixedWidth(60)                                           # executem setFixedWidth()
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)               # executem setAlignment()
            val_lbl.setStyleSheet("font-family: 'SF Mono', monospace; color: #aeaeb2;")  # executem setStyleSheet()

            grid.addWidget(name_lbl, i, 0)                                      # executem addWidget()
            grid.addWidget(slider, i, 1)                                        # executem addWidget()
            grid.addWidget(val_lbl, i, 2)                                       # executem addWidget()

            self.xyz_sliders.append(slider)                                     # executem append()
            self.xyz_labels.append(val_lbl)                                     # executem append()
            slider.valueChanged.connect(lambda v, idx=i: self._on_xyz_slide(idx, v))  # slider.valueChanged.connect(lambda v, idx ← resultat de _on_xyz_slide()

        rpy_names = ["R", "P", "Y"]                                             # assignem rpy_names
        for i in range(3):                                                      # iterem: i in range(3)
            name_lbl = QtWidgets.QLabel(rpy_names[i])                           # name_lbl ← resultat de QLabel()
            name_lbl.setFixedWidth(28)                                          # executem setFixedWidth()
            name_lbl.setStyleSheet("font-weight: bold; color: #aeaeb2;")        # executem setStyleSheet()

            slider = QtWidgets.QSlider(Qt.Horizontal)                           # slider ← resultat de QSlider()
            slider.setObjectName("CartSlider")                                  # executem setObjectName()
            slider.setMinimum(int(-RPY_RANGE * RPY_MULT))                       # executem setMinimum()
            slider.setMaximum(int(RPY_RANGE * RPY_MULT))                        # executem setMaximum()
            slider.setValue(0)                                                  # executem setValue()

            val_lbl = QtWidgets.QLabel("0.000")                                 # val_lbl ← resultat de QLabel()
            val_lbl.setFixedWidth(60)                                           # executem setFixedWidth()
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)               # executem setAlignment()
            val_lbl.setStyleSheet("font-family: 'SF Mono', monospace; color: #aeaeb2;")  # executem setStyleSheet()

            grid.addWidget(name_lbl, i + 3, 0)                                  # executem addWidget()
            grid.addWidget(slider, i + 3, 1)                                    # executem addWidget()
            grid.addWidget(val_lbl, i + 3, 2)                                   # executem addWidget()

            self.rpy_sliders.append(slider)                                     # executem append()
            self.rpy_labels.append(val_lbl)                                     # executem append()
            slider.valueChanged.connect(lambda v, idx=i: self._on_rpy_slide(idx, v))  # slider.valueChanged.connect(lambda v, idx ← resultat de _on_rpy_slide()

        layout.addLayout(grid)                                                  # executem addLayout()
        layout.addStretch()                                                     # executem addStretch()
        return self.cart_panel                                                  # retornem: self.cart_panel


    def _create_overlay_buttons(self):                                          # definim la funció _create_overlay_buttons()
        self.view_bar = QtWidgets.QFrame(self.view_container)                   # self.view_bar ← resultat de QFrame()
        self.view_bar.setStyleSheet("""
            QFrame {
                background-color: rgba(28, 28, 30, 0.92);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 8px;
            }
        """)
        vb_layout = QtWidgets.QVBoxLayout(self.view_bar)                        # vb_layout ← resultat de QVBoxLayout()
        vb_layout.setContentsMargins(4, 4, 4, 4)                                # executem setContentsMargins()
        vb_layout.setSpacing(2)                                                 # executem setSpacing()

        btn_style = """
            QPushButton {
                background-color: transparent;
                color: #aeaeb2;
                border: none;
                border-radius: 5px;
                padding: 6px 4px;
                font-weight: 600;
                font-size: 10px;
                min-width: 44px;
                max-width: 44px;
                min-height: 26px;
                max-height: 26px;
                letter-spacing: 0.5px;
            }
            QPushButton:hover {
                background-color: rgba(255, 255, 255, 0.06);
                color: #ffffff;
            }
            QPushButton:checked {
                background-color: rgba(255, 255, 255, 0.12);
                color: #ffffff;
            }
        """

        self._view_group = QtWidgets.QButtonGroup(self)                         # self._view_group ← resultat de QButtonGroup()
        self._view_group.setExclusive(True)                                     # executem setExclusive()

        specs = [                                                               # assignem specs
            ("Iso",   "iso"),
            ("Top",   "top"),
            ("Front", "front"),
            ("Left",  "left"),
            ("Right", "right"),
        ]

        self._view_buttons = {}                                                 # assignem self._view_buttons
        for label, tag in specs:                                                # iterem: label, tag in specs
            btn = QtWidgets.QPushButton(label, self.view_bar)                   # btn ← resultat de QPushButton()
            btn.setCheckable(True)                                              # executem setCheckable()
            btn.setCursor(QtCore.Qt.PointingHandCursor)                         # executem setCursor()
            btn.setStyleSheet(btn_style)                                        # executem setStyleSheet()
            btn.clicked.connect(lambda _checked, t=tag: self._set_camera(t))    # btn.clicked.connect(lambda _checked, t ← resultat de _set_camera()
            self._view_group.addButton(btn)                                     # executem addButton()
            vb_layout.addWidget(btn)                                            # executem addWidget()
            self._view_buttons[tag] = btn                                       # assignem self._view_buttons[tag]

        self._view_buttons["iso"].setChecked(True)                              # self._view_buttons["iso"].setChecked(True)
        self.view_bar.adjustSize()                                              # executem adjustSize()


    def resizeEvent(self, event):                                               # definim la funció resizeEvent()
        super().resizeEvent(event)                                              # executem super()
        self._position_view_bar()                                               # executem _position_view_bar()

    def showEvent(self, event):                                                 # definim la funció showEvent()
        super().showEvent(event)                                                # executem super()
        QtCore.QTimer.singleShot(0, self._position_view_bar)                    # executem singleShot()

    def _position_view_bar(self):                                               # definim la funció _position_view_bar()
        if not hasattr(self, "view_bar") or self.view_bar is None:              # si not hasattr(self, "view_bar") or self...
            return                                                              # retornem sense valor
        margin = 12                                                             # assignem margin
        vc_w = self.view_container.width()                                      # vc_w ← resultat de width()
        bar_w = self.view_bar.width()                                           # bar_w ← resultat de width()
        self.view_bar.move(vc_w - bar_w - margin, margin)                       # executem move()
        self.view_bar.raise_()                                                  # executem raise_()


    # ==========================================================================
    # ESCENA 3D
    # ==========================================================================

    def _setup_3d_scene(self):                                                  # definim la funció _setup_3d_scene()
        self.plotter.set_background(COLORS['bg_3d'])                            # executem set_background()

        self.plotter.add_light(pv.Light(position=(3, 3, 4), focal_point=(0, 0, 0.5),  # assignem self.plotter.add_light(pv.Light(position
                                        color='white', intensity=0.6))
        self.plotter.add_light(pv.Light(position=(-3, -1, 2), focal_point=(0, 0, 0.5),  # assignem self.plotter.add_light(pv.Light(position
                                        color='white', intensity=0.4))
        self.plotter.add_light(pv.Light(light_type='headlight', intensity=0.15))  # assignem self.plotter.add_light(pv.Light(light_type

        floor = pv.Plane(center=(0, 0, -0.001), direction=(0, 0, 1),            # assignem floor
                         i_size=3.0, j_size=3.0, i_resolution=25, j_resolution=25)
        self.plotter.add_mesh(floor, color='#2c2c2e', style='wireframe',        # assignem self.plotter.add_mesh(floor, color
                              line_width=1, opacity=0.2)
        slab = pv.Plane(center=(0, 0, -0.005), direction=(0, 0, 1),             # assignem slab
                        i_size=3.0, j_size=3.0)
        self.plotter.add_mesh(slab, color='#0e0e10', opacity=0.9, lighting=False)  # assignem self.plotter.add_mesh(slab, color

        self.stl_actors = {}                                                    # assignem self.stl_actors
        for link_name, link in self.robot.links.items():                        # iterem: link_name, link in self.robot.links.i...
            if not link.mesh:                                                   # si not link.mesh
                continue                                                        # saltem a la següent iteració
            mesh_path = os.path.join(ROOT, "config", link.mesh)                 # mesh_path ← resultat de join()
            mesh = pv.read(mesh_path)                                           # mesh ← resultat de read()

            R_vis = Rz(link.visual_rpy[2]) @ Ry(link.visual_rpy[1]) @ Rx(link.visual_rpy[0])  # R_vis ← resultat de Rz()
            T_vis = make_T(R_vis, link.visual_xyz)                              # T_vis ← resultat de make_T()
            mesh.transform(T_vis, inplace=True)                                 # assignem mesh.transform(T_vis, inplace

            actor = self.plotter.add_mesh(                                      # assignem actor
                mesh, color=COLORS['robot_color'],
                smooth_shading=False,
                specular=0.4, specular_power=10,
                ambient=0.25, diffuse=0.75,
            )
            actor.user_matrix = self.poses[link_name]                           # assignem actor.user_matrix
            self.stl_actors[link_name] = actor                                  # assignem self.stl_actors[link_name]

        self.tip_actors = self._make_triad(scale=0.08, ghost=False)             # self.tip_actors ← resultat de _make_triad()
        self.target_actors = self._make_triad(scale=0.07, ghost=True)           # self.target_actors ← resultat de _make_triad()
        self._update_triads()                                                   # executem _update_triads()

        self._set_camera('iso')                                                 # executem _set_camera()


    def _make_triad(self, scale=0.07, ghost=False):                             # definim la funció _make_triad()
        colors = [COLORS['accent_err'], COLORS['accent_x'], COLORS['accent_q']]  # assignem colors
        opacity = 0.3 if ghost else 0.9                                         # assignem opacity
        actors = []                                                             # assignem actors
        directions = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]                          # assignem directions
        for i, c in enumerate(colors):                                          # iterem: i, c in enumerate(colors)
            arrow = pv.Arrow(start=(0, 0, 0), direction=directions[i],          # assignem arrow
                             tip_length=0.25, shaft_radius=0.02, scale=scale)
            actors.append(self.plotter.add_mesh(arrow, color=c, opacity=opacity, lighting=False))  # assignem actors.append(self.plotter.add_mesh(arrow, color
        return actors                                                           # retornem: actors


    def _update_triads(self):                                                   # definim la funció _update_triads()
        T_ee = self.poses["tcp_link"]                                           # assignem T_ee
        for actor in self.tip_actors:                                           # iterem: actor in self.tip_actors
            actor.user_matrix = T_ee                                            # assignem actor.user_matrix
        for actor in self.target_actors:                                        # iterem: actor in self.target_actors
            actor.user_matrix = self.T_target                                   # assignem actor.user_matrix


    def _set_camera(self, mode):                                                # definim la funció _set_camera()
        focal = (0.0, 0.0, 0.15)                                                # focal ← resultat de ()
        views = {                                                               # assignem views
            'iso':   {'eye': (0.85, 0.85, 0.95), 'up': (0.0, 0.0, 1.0)},
            'top':   {'eye': (0.0,   0.0, 1.25),  'up': (1.0, 0.0, 0.0)},
            'front':  {'eye': (1.05,  0.0, 0.45),  'up': (0.0, 0.0, 1.0)},
            'left':  {'eye': (0.0,   1.05, 0.45), 'up': (0.0, 0.0, 1.0)},
            'right': {'eye': (0.0,  -1.05, 0.45), 'up': (0.0, 0.0, 1.0)},
        }

        v = views.get(mode)                                                     # v ← resultat de get()
        if v is None:                                                           # si v is None
            return                                                              # retornem sense valor

        self.plotter.camera_position = [v['eye'], focal, v['up']]               # assignem self.plotter.camera_position
        self.plotter.render()                                                   # executem render()

        if hasattr(self, "_view_buttons") and mode in self._view_buttons:       # si hasattr(self, "_view_buttons") and mo...
            btn = self._view_buttons[mode]                                      # assignem btn
            if not btn.isChecked():                                             # si not btn.isChecked()
                btn.setChecked(True)                                            # executem setChecked()


    # ==========================================================================
    # MODE FK / IK / MANDO
    # ==========================================================================

    def _on_mode_toggle(self):                                                  # definim la funció _on_mode_toggle()
        if self.mode == "IK":                                                   # si self.mode == "IK"
            self.mode = "FK"                                                    # assignem self.mode
        else:                                                                   # en cas contrari
            self.mode = "IK"                                                    # assignem self.mode
            # En entrar a IK des del botó intern, sincronitzem el target
            self.sync_target_to_current()                                       # executem sync_target_to_current()

        self.e_smoothed = np.zeros(6)                                           # self.e_smoothed ← resultat de zeros()
        self._update_mode_ui()                                                  # executem _update_mode_ui()


    def _update_mode_ui(self):                                                  # definim la funció _update_mode_ui()
        if self.mode == "IK":                                                   # si self.mode == "IK"
            for s in self.joint_sliders:                                        # iterem: s in self.joint_sliders
                s.setEnabled(False)                                             # executem setEnabled()
            for s in self.xyz_sliders + self.rpy_sliders:                       # iterem: s in self.xyz_sliders + self.rpy_sliders
                s.setEnabled(True)                                              # executem setEnabled()
            self.btn_mode.setText("Mode: IK")                                   # executem setText()
            self.btn_mode.setStyleSheet(                                        # executem setStyleSheet()
                f"background-color: {COLORS['accent_x']}; color: white; font-weight: bold;"
            )
        elif self.mode == "FK":                                                 # altrament si self.mode == "FK"
            for s in self.joint_sliders:                                        # iterem: s in self.joint_sliders
                s.setEnabled(True)                                              # executem setEnabled()
            for s in self.xyz_sliders + self.rpy_sliders:                       # iterem: s in self.xyz_sliders + self.rpy_sliders
                s.setEnabled(False)                                             # executem setEnabled()
            self.btn_mode.setText("Mode: FK")                                   # executem setText()
            self.btn_mode.setStyleSheet(                                        # executem setStyleSheet()
                f"background-color: {COLORS['accent_q']}; color: white; font-weight: bold;"
            )
        else:                                                                   # en cas contrari
            for s in self.joint_sliders:                                        # iterem: s in self.joint_sliders
                s.setEnabled(False)                                             # executem setEnabled()
            for s in self.xyz_sliders + self.rpy_sliders:                       # iterem: s in self.xyz_sliders + self.rpy_sliders
                s.setEnabled(False)                                             # executem setEnabled()
            self.btn_mode.setText("Mode: MANDO")                                # executem setText()
            self.btn_mode.setStyleSheet(                                        # executem setStyleSheet()
                f"background-color: {COLORS['accent_warn']}; color: white; font-weight: bold;"
            )

        target_visible = (self.mode == "IK") and self.chk_target.isChecked()    # target_visible ← resultat de ()
        for actor in self.target_actors:                                        # iterem: actor in self.target_actors
            actor.SetVisibility(target_visible)                                 # executem SetVisibility()


    # ==========================================================================
    # COMPROVACIÓ DE COL·LISIONS
    # ==========================================================================

    def _joint_limits(self):                                                    # definim la funció _joint_limits()
        """Límits articulars; els del J6 són ±π al voltant de _j6_center."""
        lower, upper = JOINT_LOWER.copy(), JOINT_UPPER.copy()                   # assignem lower, upper
        lower[5] = self._j6_center - np.pi                                      # assignem lower[5]
        upper[5] = self._j6_center + np.pi                                      # assignem upper[5]
        return lower, upper                                                     # retornem: lower, upper

    def set_j6_center(self, center):                                            # definim la funció set_j6_center()
        """Centra el recorregut (i l'slider) del J6 a `center` rad, ±π a cada banda."""
        self._j6_center = float(center)                                         # assignem self._j6_center
        slider = self.joint_sliders[5]                                          # assignem slider
        self._updating_sliders = True                                           # cap callback mentre canviem el rang
        slider.setMinimum(int(round((self._j6_center - np.pi) * 1000)))         # executem setMinimum()
        slider.setMaximum(int(round((self._j6_center + np.pi) * 1000)))         # executem setMaximum()
        slider.setValue(int(self.q[5] * 1000))                                  # executem setValue()
        self._updating_sliders = False                                          # assignem self._updating_sliders

    def cancel_motion(self):                                                    # definim la funció cancel_motion()
        """Atura un Home o un approach lent en curs (STOP, canvi de mode)."""
        self._homing_goal = None                                                # assignem self._homing_goal
        self._slow_until_reached = False                                        # assignem self._slow_until_reached

    def _check_q_valid(self, q_test):                                           # definim la funció _check_q_valid()
        q_clipped = np.clip(q_test, *self._joint_limits())                   # q_clipped ← resultat de clip()
        if not np.allclose(q_clipped, q_test):                                  # si not np.allclose(q_clipped, q_test)
            return False, set()                                                 # retornem: False, set()

        if self.chk_collisions.isChecked():                                     # si self.chk_collisions.isChecked()
            has_violation, pairs = check_collisions_with_margin(                # assignem has_violation, pairs
                self.robot, q_test, self.prims, margin=SAFETY_MARGIN
            )
            if has_violation:                                                   # si has_violation
                violating = set()                                               # violating ← resultat de set()
                for l1, l2 in pairs:                                            # iterem: l1, l2 in pairs
                    violating.add(l1)                                           # executem add()
                    violating.add(l2)                                           # executem add()
                return False, violating                                         # retornem: False, violating

        return True, set()                                                      # retornem: True, set()


    # ==========================================================================
    # BOTONS
    # ==========================================================================

    def _sync_all_sliders(self, recenter=True):                                 # definim la funció _sync_all_sliders()
        """
        Sincronitza els sliders amb l'estat intern.

        En el nou model IK, els sliders XYZ+RPY són desplaçaments
        respecte a T_ref. El rang sempre és centrat a 0 (±XYZ_RANGE
        i ±RPY_RANGE), no cal recentrar amb el valor.
        """
        self._updating_sliders = True                                           # assignem self._updating_sliders

        for i, slider in enumerate(self.joint_sliders):                         # iterem: i, slider in enumerate(self.joint_sli...
            slider.setValue(int(self.q[i] * 1000))                              # executem setValue()
            self.joint_labels[i].setText(f"{self.q[i]:.3f}")

        # Sliders XYZ: rang fix centrat a 0 (±XYZ_RANGE metres)
        for i, slider in enumerate(self.xyz_sliders):                           # iterem: i, slider in enumerate(self.xyz_sliders)
            if recenter:                                                        # si recenter
                slider.setMinimum(int(-XYZ_RANGE * XYZ_MULT))                   # executem setMinimum()
                slider.setMaximum(int( XYZ_RANGE * XYZ_MULT))                   # executem setMaximum()
            slider.setValue(int(self.target_xyz[i] * XYZ_MULT))                 # executem setValue()
            self.xyz_labels[i].setText(f"{self.target_xyz[i]:.3f}")

        # Sliders RPY: rang fix centrat a 0 (±RPY_RANGE radians)
        for i, slider in enumerate(self.rpy_sliders):                           # iterem: i, slider in enumerate(self.rpy_sliders)
            if recenter:                                                        # si recenter
                slider.setMinimum(int(-RPY_RANGE * RPY_MULT))                   # executem setMinimum()
                slider.setMaximum(int( RPY_RANGE * RPY_MULT))                   # executem setMaximum()
            slider.setValue(int(self.target_rpy[i] * RPY_MULT))                 # executem setValue()
            self.rpy_labels[i].setText(f"{self.target_rpy[i]:.3f}")

        self._updating_sliders = False                                          # assignem self._updating_sliders


    def _on_move_to_zero(self):                                                 # definim la funció _on_move_to_zero()
        """
        Home: porta el robot a q = zeros SUAUMENT. Ja no salta: el bucle de
        100 Hz hi va en línia recta (tots els joints arriben alhora) a
        HOME_SPEED_DEG_S com a màxim per joint (vegeu _homing_step).
        """
        q_zero = np.zeros(6)                                                    # q_zero ← resultat de zeros()
        if getattr(self, "_ik5dof", False):                                     # en IK5 el J6 és de la broca: no el toquem
            q_zero[5] = self.q[5]                                               # assignem q_zero[5]
        is_valid, violating = self._check_q_valid(q_zero)                       # is_valid, violating ← resultat de _check_q_valid()
        if not is_valid:                                                        # si not is_valid
            self.blocked_violating = violating                                  # assignem self.blocked_violating
            return                                                              # retornem sense valor

        self.blocked_violating = set()                                          # self.blocked_violating ← resultat de set()
        self._slow_until_reached = False                                        # assignem self._slow_until_reached
        self._homing_goal = q_zero                                              # el bucle de 100 Hz s'encarrega
        print(f"[home] cap a q = 0 a ≤{HOME_SPEED_DEG_S}°/s per joint")

    def _homing_step(self):                                                     # definim la funció _homing_step()
        """
        Un tic del Home lent. Retorna True si hi ha un Home en curs.
        Avança q cap a _homing_goal en línia recta, com a màxim
        HOME_JOINT_SPEED·dt al joint que més s'ha de moure, i deixa el target
        de l'IK sobre el TCP (perquè l'IK no corregeixi res).
        """
        if self._homing_goal is None:                                           # si no hi ha Home
            return False                                                        # retornem: False
        ik5 = getattr(self, "_ik5dof", False)                                   # ik5 ← resultat de getattr()
        if ik5:                                                                 # el J6 segueix la broca
            self._homing_goal[5] = self.q[5]                                    # assignem self._homing_goal[5]
        delta = self._homing_goal - self.q                                      # assignem delta
        big = float(np.max(np.abs(delta)))                                      # big ← joint que més s'ha de moure
        step = HOME_JOINT_SPEED * CONTROL_DT                                    # assignem step
        done = big <= step                                                      # done ← resultat de <=()
        q_try = self._homing_goal.copy() if done else self.q + delta * (step / big)
        q_final, status, violating = self._try_q_new(self.q, q_try)             # col·lisions i límits
        if status == "blocked":                                                 # alguna cosa ho impedeix
            print("[home] aturat: col·lisió")                                   # executem print()
            self.blocked_violating = violating                                  # assignem self.blocked_violating
            done = True                                                         # assignem done
        else:                                                                   # en cas contrari
            self.q[:] = q_final                                                 # self.q[:] = q_final

        q_m = self.q.copy()                                                     # TCP segons el model de l'IK
        if ik5 and self._j6_ik_override is not None:                            # si ik5 and self._j6_ik_override is not None
            q_m[5] = self._j6_ik_override                                       # assignem q_m[5]
        T = forward_kinematics(self.robot, q_m)["tcp_link"]                     # T ← resultat de forward_kinematics()
        self.T_target = T.copy()                                                # target = TCP: l'IK no fa res
        self.T_ref_pos = T[:3, 3].copy()                                        # self.T_ref_pos ← resultat de copy()
        self.R_target_initial = T[:3, :3].copy()                                # self.R_target_initial ← resultat de copy()
        self.target_xyz = np.zeros(3)                                           # self.target_xyz ← resultat de zeros()
        self.target_rpy = np.zeros(3)                                           # self.target_rpy ← resultat de zeros()
        self.e_smoothed = np.zeros(6)                                           # self.e_smoothed ← resultat de zeros()
        for actor in self.target_actors:                                        # iterem: actor in self.target_actors
            actor.user_matrix = self.T_target                                   # assignem actor.user_matrix

        self._updating_sliders = True                                           # els sliders segueixen el Home
        for i in range(6):                                                      # iterem: i in range(6)
            self.joint_sliders[i].setValue(int(self.q[i] * 1000))               # executem setValue()
            self.joint_labels[i].setText(f"{self.q[i]:.3f}")
        self._updating_sliders = False                                          # assignem self._updating_sliders

        if done:                                                                # arribat (o aturat)
            self._homing_goal = None                                            # assignem self._homing_goal
            self._sync_all_sliders(recenter=True)                               # executem _sync_all_sliders()
            print("[home] fet")                                                 # executem print()
        return True                                                             # retornem: True


    def _on_stop(self):                                                         # definim la funció _on_stop()
        """
        STOP: deixa els motors en RESET (blau, sense torque, sense mantenir
        posició), APAGA el mirror, però NO desconnecta el daemon.

        Marca el btn_connect com no-checked perquè l'ActionBar mostri
        "Connectar" — l'usuari veu clarament que ha de reactivar-los.
        """
        self.mirror_active = False                                              # assignem self.mirror_active
        self.cancel_motion()                                                    # cap Home ni approach lent en curs

        if self.motor_controller.is_connected:                                  # si self.motor_controller.is_connected
            try:                                                                # intentem el bloc següent
                self.motor_controller.bus.reset_all()                           # executem reset_all()
                time.sleep(0.15)                                              # executem sleep()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[STOP] Error al reset del bus: {e}")

        # NO disconnectem el daemon. Els motors segueixen en RESET
        # fins que algun mode els reactivi via enable_position_control().

        # Marca els botons com "no actius" perquè l'ActionBar mostri
        # "Connectar" i "Mirror OFF".
        self.btn_connect.setChecked(False)                                      # executem setChecked()
        self.btn_mirror.setText("Mirror OFF")                                   # executem setText()
        self.btn_mirror.setChecked(False)                                       # executem setChecked()
        self.btn_mirror.setStyleSheet("")                                       # executem setStyleSheet()

        # Actualitza referència del target al TCP actual
        self.sync_target_to_current()                                           # executem sync_target_to_current()

        print("[STOP] motors en RESET (blau). Cliqueu un mode per reactivar-los.")  # executem print()


    def _on_target_visibility(self, checked):                                   # definim la funció _on_target_visibility()
        visible = checked and (self.mode == "IK")                               # visible ← resultat de checked and()
        for actor in self.target_actors:                                        # iterem: actor in self.target_actors
            actor.SetVisibility(visible)                                        # executem SetVisibility()


    def _on_connect_motors(self):                                               # definim la funció _on_connect_motors()
        """
        Toggle de connexió dels motors.

        Tres casos:
          1. Daemon desconnectat (mai s'han obert els motors) →
             connectar + enable_position_control + send_q inicial.
          2. Daemon connectat però motors en RESET (venim d'un STOP) →
             tornar a enable_position_control + send_q per reactivar-los.
             NO desconnectem el daemon.
          3. Daemon connectat i motors actius (l'usuari vol apagar) →
             en aquest cas el btn ja està checked. Sortir directament.
             Per desconnectar de veritat, cal fer STOP i sortir.
        """
        import time as _time                                                    # importem el mòdul time as _time

        # Cas 1: daemon mort → obrir de zero
        if not self.motor_controller.is_connected:                              # si not self.motor_controller.is_connected
            try:                                                                # intentem el bloc següent
                self.motor_controller.connect()                                 # executem connect()
                self.motor_controller.enable_position_control_at(self.q)        # POSITION amb el target enganxat
                _time.sleep(0.1)                                                # executem sleep()
                # send_q amb la q actual per treure els motors del RESET inicial
                try:                                                            # intentem el bloc següent
                    self.motor_controller.send_q(self.q)                        # executem send_q()
                except Exception as e:                                          # capturem excepció: Exception as e
                    print(f"[connect] error send_q inicial: {e}")

                self.btn_connect.setChecked(True)                               # executem setChecked()
                self.btn_mirror.setEnabled(True)                                # executem setEnabled()
                print("[connect] motors connectats i activats")                 # executem print()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[connect] Error connectant: {e}")
                self.btn_connect.setChecked(False)                              # executem setChecked()
            return                                                              # retornem sense valor

        # Daemon connectat.
        # Cas 2: si el btn s'acaba de posar a "checked" (l'usuari volia
        # reactivar), reactivem els motors passant per RESET → POSITION_MODE.
        if self.btn_connect.isChecked():                                        # si self.btn_connect.isChecked()
            try:                                                                # intentem el bloc següent
                # RESET + POSITION_MODE + send_q (mateix patró que sortir de hand-guide)
                self.motor_controller.bus.reset_all()                           # executem reset_all()
                _time.sleep(0.3)                                                # el firmware necessita temps després del RESET
                self.motor_controller.enable_position_control_at(self.q)        # POSITION amb el target enganxat
                _time.sleep(0.1)                                                # executem sleep()
                for _ in range(3):                                              # iterem: _ in range(3)
                    try:                                                        # intentem el bloc següent
                        self.motor_controller.send_q(self.q)                    # executem send_q()
                        _time.sleep(0.03)                                       # executem sleep()
                    except Exception as e:                                      # capturem excepció: Exception as e
                        print(f"[connect] error send_q: {e}")
                        break                                                   # sortim del bucle
                self.btn_mirror.setEnabled(True)                                # executem setEnabled()
                print("[connect] motors reactivats (RESET → POSITION_MODE)")    # executem print()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[connect] error reactivant: {e}")
                self.btn_connect.setChecked(False)                              # executem setChecked()
            return                                                              # retornem sense valor

        # Cas 3: el btn acaba de passar a un-checked → posem els motors en RESET
        # (equivalent a un STOP suau).
        if self.mirror_active:                                                  # si self.mirror_active
            self.mirror_active = False                                          # assignem self.mirror_active
            self.btn_mirror.setText("Mirror OFF")                               # executem setText()
            self.btn_mirror.setChecked(False)                                   # executem setChecked()
            self.btn_mirror.setStyleSheet("")                                   # executem setStyleSheet()

        try:                                                                    # intentem el bloc següent
            self.motor_controller.bus.reset_all()                               # executem reset_all()
            _time.sleep(0.1)                                                    # executem sleep()
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[connect] error reset: {e}")

        self.btn_mirror.setEnabled(False)                                       # executem setEnabled()
        print("[connect] motors en RESET (blau)")                               # executem print()


    def _on_mirror_toggle(self):                                                # definim la funció _on_mirror_toggle()
        self.mirror_active = self.btn_mirror.isChecked()                        # self.mirror_active ← resultat de isChecked()
        if self.mirror_active:                                                  # si self.mirror_active
            self.btn_mirror.setText("Mirror ON")                                # executem setText()
            self.btn_mirror.setStyleSheet(                                      # executem setStyleSheet()
                f"background-color: {COLORS['accent_x']}; color: white; font-weight: bold;"
            )
        else:                                                                   # en cas contrari
            self.btn_mirror.setText("Mirror OFF")                               # executem setText()
            self.btn_mirror.setStyleSheet("")                                   # executem setStyleSheet()


    # ==========================================================================
    # CALLBACKS DELS SLIDERS
    # ==========================================================================

    def _on_joint_slide(self, idx, int_val):                                    # definim la funció _on_joint_slide()
        if self._updating_sliders or self.mode != "FK":                         # si self._updating_sliders or self.mode !...
            return                                                              # retornem sense valor

        val = int_val / 1000.0                                                  # assignem val

        max_delta = MAX_JOINT_SPEED * CONTROL_DT                                # assignem max_delta
        delta = val - self.q[idx]                                               # assignem delta
        if abs(delta) > max_delta:                                              # si abs(delta) > max_delta
            val = self.q[idx] + np.sign(delta) * max_delta                      # assignem val

        q_test = self.q.copy()                                                  # q_test ← resultat de copy()
        q_test[idx] = val                                                       # assignem q_test[idx]

        is_valid, violating = self._check_q_valid(q_test)                       # is_valid, violating ← resultat de _check_q_valid()

        if is_valid:                                                            # si is_valid
            self.q = q_test                                                     # assignem self.q
            self.blocked_violating = set()                                      # self.blocked_violating ← resultat de set()
            self.joint_labels[idx].setText(f"{val:.3f}")

            self._updating_sliders = True                                       # assignem self._updating_sliders
            self.joint_sliders[idx].setValue(int(val * 1000))                   # self.joint_sliders[idx].setValue(int(val * 1000))
            self._updating_sliders = False                                      # assignem self._updating_sliders
        else:                                                                   # en cas contrari
            self.blocked_violating = violating                                  # assignem self.blocked_violating
            self._updating_sliders = True                                       # assignem self._updating_sliders
            self.joint_sliders[idx].setValue(int(self.q[idx] * 1000))           # self.joint_sliders[idx].setValue(int(self.q[idx] * 1000))
            self.joint_labels[idx].setText(f"{self.q[idx]:.3f}")
            self._updating_sliders = False                                      # assignem self._updating_sliders


    def _on_xyz_slide(self, idx, int_val):                                      # definim la funció _on_xyz_slide()
        if self._updating_sliders or self.mode != "IK":                         # si self._updating_sliders or self.mode !...
            return                                                              # retornem sense valor
        val = int_val / XYZ_MULT                                                # convertim enter -> metres (1/3 mm per pas)
        self.target_xyz[idx] = val                                              # assignem self.target_xyz[idx]
        self.xyz_labels[idx].setText(f"{val:.3f}")
        self._rebuild_target()                                                  # executem _rebuild_target()


    def _on_rpy_slide(self, idx, int_val):                                      # definim la funció _on_rpy_slide()
        if self._updating_sliders or self.mode != "IK":                         # si self._updating_sliders or self.mode !...
            return                                                              # retornem sense valor
        val = int_val / RPY_MULT                                                # convertim enter -> radians
        self.target_rpy[idx] = val                                              # assignem self.target_rpy[idx]
        self.rpy_labels[idx].setText(f"{val:.3f}")
        self._rebuild_target()                                                  # executem _rebuild_target()


    def _rebuild_target(self):                                                  # definim la funció _rebuild_target()
        """
        Reconstrueix T_target = T_ref + desplaçament dels sliders.

        target_xyz: desplaçament en metres respecte T_ref_pos
        target_rpy: rotació incremental respecte R_target_initial
        """
        # Base per si mai no s'ha cridat sync_target_to_current
        if not hasattr(self, "T_ref_pos"):                                      # si not hasattr(self, "T_ref_pos")
            self.T_ref_pos = self.T_target[:3, 3].copy()                        # self.T_ref_pos ← resultat de copy()

        # Posició final = referència + desplaçament del slider
        pos_final = self.T_ref_pos + self.target_xyz                            # assignem pos_final

        # Rotació final = rotació incremental del slider aplicada a la referència
        R_delta = Rz(self.target_rpy[2]) @ Ry(self.target_rpy[1]) @ Rx(self.target_rpy[0])  # R_delta ← resultat de Rz()
        R_final = R_delta @ self.R_target_initial                               # assignem R_final

        self.T_target = make_T(R_final, pos_final)                              # self.T_target ← resultat de make_T()
        for actor in self.target_actors:                                        # iterem: actor in self.target_actors
            actor.user_matrix = self.T_target                                   # assignem actor.user_matrix


    # ==========================================================================
    # BUCLE DE CONTROL PRINCIPAL (100 Hz)
    # ==========================================================================

    def _try_q_new(self, q_current, q_new):                                     # definim la funció _try_q_new()
        q_clipped = np.clip(q_new, *self._joint_limits())                    # q_clipped ← resultat de clip()

        if self.chk_collisions.isChecked():                                     # si self.chk_collisions.isChecked()
            has_violation, pairs = check_collisions_with_margin(                # assignem has_violation, pairs
                self.robot, q_clipped, self.prims, margin=SAFETY_MARGIN
            )
            if not has_violation:                                               # si not has_violation
                return q_clipped, "ok", set()                                   # retornem: q_clipped, "ok", set()

            for scale in [0.5, 0.25, 0.125]:                                    # iterem: scale in [0.5, 0.25, 0.125]
                q_partial = q_current + (q_clipped - q_current) * scale         # assignem q_partial
                q_partial = np.clip(q_partial, *self._joint_limits())        # q_partial ← resultat de clip()
                has_violation_p, _ = check_collisions_with_margin(              # només ens cal saber si hi ha col·lisió
                    self.robot, q_partial, self.prims, margin=SAFETY_MARGIN
                )
                if not has_violation_p:                                         # si not has_violation_p
                    return q_partial, "ok", set()                               # retornem: q_partial, "ok", set()

            violating_links = set()                                             # violating_links ← resultat de set()
            for l1, l2 in pairs:                                                # iterem: l1, l2 in pairs
                violating_links.add(l1)                                         # executem add()
                violating_links.add(l2)                                         # executem add()
            return q_current, "blocked", violating_links                        # retornem: q_current, "blocked", violating_links
        else:                                                                   # en cas contrari
            return q_clipped, "ok", set()                                       # retornem: q_clipped, "ok", set()


    def _teleop_step(self):                                                     # definim la funció _teleop_step()
        # Mode HAND-GUIDE natiu: només redibuixem el 3D amb la q actual.
        # NO calculem IK, NO enviem res al bus. També SINCRONITZEM T_target
        # amb el TCP actual perquè en tornar a IK no hi hagi error acumulat.
        # (La broca del J6 NO atura aquest bucle: vegeu IK5 més avall.)
        if getattr(self, "_handguide_active", False):                           # si estem en mode passiu
            self.poses = forward_kinematics(self.robot, self.q)                 # self.poses ← FK amb la q actual
            T_actual_passive = self.poses["tcp_link"]                           # pose actual del TCP
            self.T_target = T_actual_passive.copy()                             # target sincronitzat per no acumular error
            self.T_ref_pos = T_actual_passive[:3, 3].copy()                     # referència actualitzada
            self.R_target_initial = T_actual_passive[:3, :3].copy()             # idem rotació
            for link_name, actor in self.stl_actors.items():                    # iterem: redibuix dels STLs
                actor.user_matrix = self.poses[link_name]
            for actor in self.tip_actors:                                       # idem tip (punta)
                actor.user_matrix = self.poses["tcp_link"]
            self._update_console(self.poses["tcp_link"])                        # consola
            return                                                              # fet, no cal res més

        homing = self._homing_step()                                            # Home lent en curs?
        if self.mode == "MANDO" and not homing:                                 # el mando no s'hi barreja
            self._mando_apply_to_target()                                       # executem _mando_apply_to_target()

        # IK5: el model de l'IK fa servir q[5] congelat mentre la broca gira
        # (self.q[5] segueix el motor real, però només per dibuixar-lo).
        ik5 = getattr(self, "_ik5dof", False)                                   # ik5 ← resultat de getattr()
        q_model = self.q.copy()                                                 # q que veu l'IK
        if ik5 and self._j6_ik_override is not None:                            # broca girant
            q_model[5] = self._j6_ik_override                                   # assignem q_model[5]

        self.poses = forward_kinematics(self.robot, q_model)                    # self.poses ← resultat de forward_kinematics()
        T_actual = self.poses["tcp_link"]                                       # assignem T_actual

        # 3. En mode FK, forcem target = TCP actual perquè el DLS no generi
        # cap moviment quan tornem a IK. NO toquem els sliders CARTESIAN
        # perquè el nou model els fa desaparèixer en FK (l'usuari els posarà
        # a 0 automàticament al canviar a IK, via sync_target_to_current).
        if self.mode == "FK":                                                   # si self.mode == "FK"
            self.T_target = T_actual.copy()                                     # self.T_target ← resultat de copy()
            self.T_ref_pos = T_actual[:3, 3].copy()                             # self.T_ref_pos ← resultat de copy()
            self.R_target_initial = T_actual[:3, :3].copy()                     # self.R_target_initial ← resultat de copy()
            self.target_xyz = np.array([0.0, 0.0, 0.0])                         # self.target_xyz ← resultat de array()
            self.target_rpy = np.array([0.0, 0.0, 0.0])                         # self.target_rpy ← resultat de array()
            for actor in self.target_actors:                                    # iterem: actor in self.target_actors
                actor.user_matrix = self.T_target                               # assignem actor.user_matrix


        e_raw = pose_error(T_actual, self.T_target)                             # e_raw ← resultat de pose_error()

        J = compute_jacobian(self.robot, q_model)                               # J ← resultat de compute_jacobian()

        J_ik = J                                                                # Jacobià que farà servir el DLS
        if ik5:                                                                 # IK de 5 graus de veritat
            # El gir al voltant de l'eix de la broca és el que fa el J6, i
            # J1-J5 no el poden (ni l'han de) corregir. El traiem de la tasca
            # (error I Jacobià, perquè sigui coherent): així l'IK5 controla
            # posició + direcció de la broca amb J1-J5, i res més.
            # L'eix del J6 al món és la seva columna angular del Jacobià.
            J_ik = J[:, :5].copy()                                              # només J1-J5 (6×5)
            axis = J[3:6, 5]                                                    # assignem axis
            n_axis = np.linalg.norm(axis)                                       # n_axis ← resultat de norm()
            if n_axis > 1e-9:                                                   # si n_axis > 1e-9
                a = axis / n_axis                                               # assignem a
                P = np.eye(3) - np.outer(a, a)                                  # projector: treu la component axial
                e_raw[3:] = P @ e_raw[3:]                                       # error sense gir axial
                J_ik[3:6, :] = P @ J_ik[3:6, :]                                 # tasca sense gir axial

        e_raw[:3] *= KP_POS                                                     # multipliquem e_raw[:3]
        e_raw[3:] *= KP_ORI                                                     # multipliquem e_raw[3:]

        self.e_smoothed = self.e_smoothed + ERROR_SMOOTH_ALPHA * (e_raw - self.e_smoothed)  # self.e_smoothed ← resultat de e_smoothed + ERROR_SMOOTH_ALPHA *()

        err_pos = np.linalg.norm(e_raw[:3]) / KP_POS                            # assignem err_pos
        err_ori = np.linalg.norm(e_raw[3:]) / KP_ORI                            # assignem err_ori
        target_reached = (err_pos < TOL_POS) and (err_ori < TOL_ORI)            # target_reached ← resultat de ()

        w = yoshikawa_position(J)                                               # w ← resultat de yoshikawa_position()
        lam = adaptive_damping(w, LAMBDA_MIN, LAMBDA_MAX, W_THRESHOLD)          # lam ← resultat de adaptive_damping()

        if ik5:                                                                 # només J1-J5 (Jacobià 6×5)
            q_dot = np.zeros(6)                                                 # q_dot[5] = 0: el J6 no el toca l'IK
            q_dot[:5] = dls(J_ik, self.e_smoothed, lam)                         # el J6 no entra a la solució
        else:                                                                   # IK de 6 graus
            q_dot = dls(J, self.e_smoothed, lam)                                # q_dot ← resultat de dls()
        if self._slow_until_reached:                                            # "Anar a approach": 6× més lent
            peak = float(np.max(np.abs(q_dot)))                                 # peak ← joint més ràpid
            if peak > APPROACH_JOINT_SPEED:                                         # escalem TOTS els joints igual:
                q_dot = q_dot * (APPROACH_JOINT_SPEED / peak)                       # la trajectòria no es deforma
            elif target_reached:                                                # arribat i ja va a poc a poc: velocitat normal
                self._slow_until_reached = False                                # assignem self._slow_until_reached
                print("[approach] arribat")                                     # executem print()
        q_dot = np.clip(q_dot, -MAX_JOINT_SPEED, MAX_JOINT_SPEED)               # q_dot ← resultat de clip()

        q_new = self.q + q_dot * CONTROL_DT                                     # assignem q_new
        q_final, status, violating = self._try_q_new(self.q, q_new)             # q_final, status, violating ← resultat de _try_q_new()
        self.q[:] = q_final                                                     # self.q[:] = q_final

        if status == "blocked":                                                 # si status == "blocked"
            display_violating = violating                                       # assignem display_violating
        elif self.blocked_violating:                                            # altrament si self.blocked_violating
            display_violating = self.blocked_violating                          # assignem display_violating
            status = "blocked"                                                  # assignem status
        else:                                                                   # en cas contrari
            display_violating = set()                                           # display_violating ← resultat de set()

        if self.mode == "IK":                                                   # si self.mode == "IK"
            self._updating_sliders = True                                       # assignem self._updating_sliders
            for i in range(6):                                                  # iterem: i in range(6)
                self.joint_sliders[i].setValue(int(self.q[i] * 1000))           # self.joint_sliders[i].setValue(int(self.q[i] * 1000))
                self.joint_labels[i].setText(f"{self.q[i]:.3f}")
            self._updating_sliders = False                                      # assignem self._updating_sliders

        self.poses = forward_kinematics(self.robot, self.q)                     # self.poses ← resultat de forward_kinematics()
        for link_name, actor in self.stl_actors.items():                        # iterem: link_name, actor in self.stl_actors.i...
            actor.user_matrix = self.poses[link_name]                           # assignem actor.user_matrix
        for actor in self.tip_actors:                                           # iterem: actor in self.tip_actors
            actor.user_matrix = self.poses["tcp_link"]                          # assignem actor.user_matrix

        if status == "blocked":                                                 # si status == "blocked"
            for link_name, actor in self.stl_actors.items():                    # iterem: link_name, actor in self.stl_actors.i...
                if link_name in display_violating:                              # si link_name in display_violating
                    actor.prop.color = COLORS['accent_err']                     # assignem actor.prop.color
            self.current_status = "BLOCKED"                                     # assignem self.current_status
        elif target_reached:                                                    # altrament si target_reached
            for actor in self.stl_actors.values():                              # iterem: actor in self.stl_actors.values()
                actor.prop.color = COLORS['target_reached_tint']                # assignem actor.prop.color
            self.current_status = "TARGET REACHED"                              # assignem self.current_status
        else:                                                                   # en cas contrari
            for actor in self.stl_actors.values():                              # iterem: actor in self.stl_actors.values()
                actor.prop.color = COLORS['robot_color']                        # assignem actor.prop.color
            self.current_status = "OK"                                          # assignem self.current_status

        if status == "ok":                                                      # si status == "ok"
            self.blocked_violating = set()                                      # self.blocked_violating ← resultat de set()

        self._update_console(T_actual)                                          # executem _update_console()

        if self.mirror_active and self.motor_controller.is_connected:           # si self.mirror_active and self.motor_con...
            try:                                                                # intentem el bloc següent
                self.motor_controller.send_q(self.q)                            # executem send_q()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"Error enviant q: {e}")
                self.mirror_active = False                                      # assignem self.mirror_active
                self.btn_mirror.setText("Mirror OFF")                           # executem setText()
                self.btn_mirror.setChecked(False)                               # executem setChecked()

        if self.mando_active:                                                   # si self.mando_active
            self._mando_update_poses()                                          # executem _mando_update_poses()


    # ==========================================================================
    # MANDO - CONNEXIÓ SERIAL I LECTURA
    # ==========================================================================

    def _on_mando_toggle(self):                                                 # definim la funció _on_mando_toggle()
        if not self.mando_active:                                               # si not self.mando_active
            self._mando_connect()                                               # executem _mando_connect()
        else:                                                                   # en cas contrari
            self._mando_disconnect()                                            # executem _mando_disconnect()


    def _mando_connect(self):                                                   # definim la funció _mando_connect()
        """
        Dues vies de connexió, en ordre de preferència:

        1) MandoService central (quan corre dins l'app NeuroSpine): ja hi
           ha una instància singleton que té el port obert i rep dades.
           Nosaltres NO obrim cap port: ens subscrivim al seu signal
           `reading` i actualitzem self.mando_raw des d'allà.

        2) Serial propi (quan el viewer corre sol, ex: `python -m scripts.viewer_cobot`):
           obrim el port directament, com es feia fins ara.

        Això evita el conflicte on tant MandoService com el viewer intenten
        obrir COM5 a la vegada → el segon falla amb "accés denegat".
        """
        # ---- Via 1: reutilitzar el MandoService singleton si existeix ----
        try:                                                                    # intentem localitzar el singleton
            from neurospine_app.mando_service import get_mando_service          # import local
            svc = get_mando_service()                                           # retorna la instància existent (o en crea una)
        except Exception:                                                       # si no es pot importar (viewer sol)
            svc = None

        if svc is not None and svc.is_connected():                              # si el servei ja té el port
            self._mando_svc = svc                                               # recordem referència
            # Subscripció: cada lectura del servei actualitza self.mando_raw.
            try:                                                                # intentem connectar signal
                svc.reading.connect(self._on_mando_service_reading)
            except Exception as e:                                              # per si ja estava connectat
                print(f"[viewer_cobot] connect signal mando: {e}")
            self.mando_active = True                                            # flag ON
            self.mando_thread = None                                            # NO hi ha thread propi
            self.mando_serial = None                                            # NO hi ha serial propi
            self._mando_load_scene()                                            # dibuixem el mando 3D
            self.btn_mando.setText("Mando 6DOF: ON")
            self.btn_mando.setStyleSheet(
                f"background-color: {COLORS['accent_x']}; color: white; font-weight: bold;"
            )
            self.btn_mando_zero.setEnabled(True)
            self.btn_mando_control.setEnabled(True)
            print("[viewer_cobot] Mando connectat via MandoService (compartit)")
            return

        # ---- Via 2: obrir el serial propi (viewer standalone) ----
        if serial is None:                                                      # pyserial no instal·lat
            print("pyserial no instal·lat")
            self.btn_mando.setChecked(False)
            return

        try:                                                                    # intentem obrir port
            self.mando_serial = serial.Serial(
                self.mando_port, self.mando_baud, timeout=0.1
            )
        except Exception as e:                                                  # port ocupat / inexistent
            print(f"No s'ha pogut obrir {self.mando_port}: {e}")
            self.btn_mando.setChecked(False)
            return

        self._mando_svc = None                                                  # no usem servei
        self.mando_active = True
        self.mando_thread = threading.Thread(target=self._mando_read_loop, daemon=True)
        self.mando_thread.start()
        self._mando_load_scene()
        self.btn_mando.setText("Mando 6DOF: ON")
        self.btn_mando.setStyleSheet(
            f"background-color: {COLORS['accent_x']}; color: white; font-weight: bold;"
        )
        self.btn_mando_zero.setEnabled(True)
        self.btn_mando_control.setEnabled(True)
        print(f"[viewer_cobot] Mando connectat (serial propi) a {self.mando_port}")

    def _on_mando_service_reading(self, data):                                  # definim la funció _on_mando_service_reading()
        """
        Slot connectat al signal 'reading' del MandoService.
        Converteix la dada rebuda al format que el viewer_cobot espera a
        self.mando_raw (llista de 6 ints: els 'pols' crus dels encoders).
        """
        try:
            pols = data.get("pols")                                             # llista de 6 enters crus
            if pols is not None and len(pols) >= 6:                             # mínim 6 valors
                self.mando_raw = list(pols[:6])                                 # còpia
        except Exception as e:
            print(f"[viewer_cobot] parse mando service: {e}")


    def _mando_disconnect(self):                                                # definim la funció _mando_disconnect()
        self.mando_active = False                                               # assignem self.mando_active

        # Si estàvem subscrits al MandoService, desconnectem el signal
        # (però NO parem el servei: continua corrent per si altres workspaces
        # el necessiten; només nosaltres deixem d'escoltar).
        svc = getattr(self, "_mando_svc", None)
        if svc is not None:
            try:
                svc.reading.disconnect(self._on_mando_service_reading)
            except Exception:
                pass
            self._mando_svc = None

        if self.mando_thread is not None:                                       # si self.mando_thread is not None
            self.mando_thread.join(timeout=0.5)                                 # assignem self.mando_thread.join(timeout
            self.mando_thread = None                                            # assignem self.mando_thread
        if self.mando_serial is not None:                                       # si self.mando_serial is not None
            try:                                                                # intentem el bloc següent
                self.mando_serial.close()                                       # executem close()
            except Exception:                                                   # capturem excepció: Exception
                pass                                                            # no fem res (placeholder)
            self.mando_serial = None                                            # assignem self.mando_serial

        self._mando_unload_scene()                                              # executem _mando_unload_scene()

        self.btn_mando.setText("Mando 6DOF: OFF")                               # executem setText()
        self.btn_mando.setStyleSheet("")                                        # executem setStyleSheet()
        print("Mando desconnectat")                                             # executem print()

        self.btn_mando_zero.setEnabled(False)                                   # executem setEnabled()
        self.btn_mando_control.setEnabled(False)                                # executem setEnabled()
        self.mando_zero_raw = None                                              # assignem self.mando_zero_raw
        self.mando_zero_tcp = None                                              # assignem self.mando_zero_tcp

        if self.mode == "MANDO":                                                # si self.mode == "MANDO"
            self.mode = "IK"                                                    # assignem self.mode
            self.btn_mando_control.setText("Mando Control: OFF")                # executem setText()
            self.btn_mando_control.setStyleSheet("")                            # executem setStyleSheet()
            self.btn_mando_control.setChecked(False)                            # executem setChecked()
            self._update_mode_ui()                                              # executem _update_mode_ui()


    def _mando_read_loop(self):                                                 # definim la funció _mando_read_loop()
        buf = b""                                                               # assignem buf
        while self.mando_active and self.mando_serial is not None:              # mentre self.mando_active and self.mando_seri...
            try:                                                                # intentem el bloc següent
                data = self.mando_serial.read(128)                              # data ← resultat de read()
                if not data:                                                    # si not data
                    continue                                                    # saltem a la següent iteració
                buf += data                                                     # sumem a buf
                while b"\n" in buf:                                             # mentre b"\n" in buf
                    line, buf = buf.split(b"\n", 1)                             # line, buf ← resultat de split()
                    self._mando_parse(line.decode("utf-8", "ignore").strip())   # executem _mando_parse()
            except Exception:                                                   # capturem excepció: Exception
                time.sleep(0.05)                                                # executem sleep()


    def _mando_parse(self, line):                                               # definim la funció _mando_parse()
        if not line or ":" not in line:                                         # si not line or ":" not in line
            return                                                              # retornem sense valor
        try:                                                                    # intentem el bloc següent
            # Mateix parser que el MandoService (format nou "E1: n   E2: n ...",
            # reordenat a l'ordre de 'pols' que espera la resta del codi).
            try:
                from neurospine_app.mando_service import parse_mando_line       # import local
            except ImportError:
                from scripts.neurospine_app.mando_service import parse_mando_line
            vals = parse_mando_line(line)                                       # llista de 6 o None
            if vals is not None:                                                # línia vàlida
                self.mando_raw = vals                                           # assignem self.mando_raw
        except Exception:                                                       # capturem excepció: Exception
            pass                                                                # no fem res (placeholder)


    # ==========================================================================
    # MANDO - VISUALITZACIÓ 3D
    # ==========================================================================

    def _mando_load_scene(self):                                                # definim la funció _mando_load_scene()
        if self.mando_actors:                                                   # si self.mando_actors
            return                                                              # retornem sense valor

        q_init = np.zeros(7)                                                    # q_init ← resultat de zeros()
        poses = forward_kinematics(self.mando_robot, q_init)                    # poses ← resultat de forward_kinematics()

        T_offset = self._mando_get_offset_matrix()                              # T_offset ← resultat de _mando_get_offset_matrix()

        for link_name, link in self.mando_robot.links.items():                  # iterem: link_name, link in self.mando_robot.l...
            if not link.mesh:                                                   # si not link.mesh
                continue                                                        # saltem a la següent iteració
            mesh_path = os.path.join(ROOT, "config", link.mesh)                 # mesh_path ← resultat de join()
            try:                                                                # intentem el bloc següent
                mesh = pv.read(mesh_path)                                       # mesh ← resultat de read()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"No s'ha pogut carregar {mesh_path}: {e}")
                continue                                                        # saltem a la següent iteració

            actor = self.plotter.add_mesh(                                      # assignem actor
                mesh,
                color=COLORS['robot_color'],
                smooth_shading=False,
                specular=0.4, specular_power=10,
                ambient=0.25, diffuse=0.75,
                name=f"mando_{link_name}",
            )
            actor.user_matrix = T_offset @ poses[link_name]                     # assignem actor.user_matrix
            self.mando_actors[link_name] = actor                                # assignem self.mando_actors[link_name]

        self.plotter.render()                                                   # executem render()


    def _mando_unload_scene(self):                                              # definim la funció _mando_unload_scene()
        for name in list(self.mando_actors.keys()):                             # iterem: name in list(self.mando_actors.keys())
            try:                                                                # intentem el bloc següent
                self.plotter.remove_actor(f"mando_{name}")
            except Exception:                                                   # capturem excepció: Exception
                pass                                                            # no fem res (placeholder)
        self.mando_actors.clear()                                               # executem clear()
        self.plotter.render()                                                   # executem render()


    def _mando_get_offset_matrix(self):                                         # definim la funció _mando_get_offset_matrix()
        yaw_rad = np.deg2rad(self.mando_yaw_deg)                                # yaw_rad ← resultat de deg2rad()
        T_offset = np.eye(4)                                                    # T_offset ← resultat de eye()
        T_offset[:3, :3] = Rz(yaw_rad)                                          # T_offset[:3, :3] = Rz(yaw_rad)
        T_offset[:3,  3] = self.mando_offset                                    # T_offset[:3,  3] = self.mando_offset
        return T_offset                                                         # retornem: T_offset


    def _mando_update_poses(self):                                              # definim la funció _mando_update_poses()
        if not self.mando_actors:                                               # si not self.mando_actors
            return                                                              # retornem sense valor

        conv_g = 360.0 / self.mando_pulsos_per_volta                            # assignem conv_g
        conv_m = (np.pi * self.mando_diametre_corro_mm) / self.mando_pulsos_per_volta  # assignem conv_m

        rot_idx = (4, 0, 5)                                                     # rot_idx ← resultat de ()
        rot_sign = (1, -1, 1)                                                   # rot_sign ← resultat de ()
        transl_idx = (3, 2, 1)                                                  # transl_idx ← resultat de ()
        transl_sign = (1, 1, -1)                                                # transl_sign ← resultat de ()

        # Respecte al Set Zero (si n'hi ha): en entrar al mode el mando surt a 0
        zero = self.mando_zero_raw if self.mando_zero_raw is not None else [0] * 6
        raw = [self.mando_raw[i] - zero[i] for i in range(6)]                   # assignem raw

        rx_deg = raw[rot_idx[0]] * rot_sign[0] * conv_g                         # assignem rx_deg
        ry_deg = raw[rot_idx[1]] * rot_sign[1] * conv_g                         # assignem ry_deg
        rz_deg = raw[rot_idx[2]] * rot_sign[2] * conv_g                         # assignem rz_deg
        x_mm   = raw[transl_idx[0]] * transl_sign[0] * conv_m                   # assignem x_mm
        y_mm   = raw[transl_idx[1]] * transl_sign[1] * conv_m                   # assignem y_mm
        z_mm   = raw[transl_idx[2]] * transl_sign[2] * conv_m                   # assignem z_mm

        # E6 (Z) comença a dalt de tot del recorregut, no al mig: en fer
        # Set Zero el mando surt a dalt i només pot baixar.
        # La resta del mando penja de part3 (j2 = -j1), per això "dalt" és j1 = -17.5 mm.
        j1 = float(np.clip(-0.0175 + z_mm / 1000.0, -0.0175, 0.0175))           # j1 ← resultat de float()
        j2 = -j1                                                                # assignem j2
        j3 = float(np.clip(x_mm / 1000.0, -0.0276, 0.0276))                     # j3 ← resultat de float()
        j4 = float(np.clip(y_mm / 1000.0, -0.0276, 0.0276))                     # j4 ← resultat de float()
        j5 = float(np.deg2rad(rz_deg))                                          # j5 ← resultat de float()
        j6 = float(np.deg2rad(ry_deg))                                          # j6 ← resultat de float()
        j7 = float(np.deg2rad(rx_deg))                                          # j7 ← resultat de float()

        q_mando = np.array([j1, j2, j3, j4, j5, j6, j7])                        # q_mando ← resultat de array()

        poses = forward_kinematics(self.mando_robot, q_mando)                   # poses ← resultat de forward_kinematics()
        T_offset = self._mando_get_offset_matrix()                              # T_offset ← resultat de _mando_get_offset_matrix()

        for link_name, actor in self.mando_actors.items():                      # iterem: link_name, actor in self.mando_actors...
            T = poses.get(link_name)                                            # T ← resultat de get()
            if T is None:                                                       # si T is None
                continue                                                        # saltem a la següent iteració
            actor.user_matrix = T_offset @ T                                    # assignem actor.user_matrix


    # ==========================================================================
    # MANDO - CONTROL DEL COBOT (mode MANDO)
    # ==========================================================================

    def _on_mando_set_zero(self):                                               # definim la funció _on_mando_set_zero()
        self.mando_zero_raw = list(self.mando_raw)                              # self.mando_zero_raw ← resultat de list()
        self.mando_zero_tcp = self.poses["tcp_link"].copy()                     # self.mando_zero_tcp ← resultat de copy()

        # Arrencada neta: oblidem el filtre i la pose comandada de la sessió
        # anterior (si no, en tornar a entrar el robot llisca cap a l'antiga).
        self.mando_comp_filt = None                                             # assignem self.mando_comp_filt
        self.mando_cmd_pose = None                                              # assignem self.mando_cmd_pose
        self.mando_last_tick = None                                             # assignem self.mando_last_tick

        print("Set Zero:")                                                      # executem print()
        print(f"  Mando raw: {self.mando_zero_raw}")
        print(f"  TCP pos:   {self.mando_zero_tcp[:3, 3] * 1000} mm")


    def _on_mando_control_toggle(self):                                         # definim la funció _on_mando_control_toggle()
        if self.btn_mando_control.isChecked():                                  # si self.btn_mando_control.isChecked()
            if self.mando_zero_raw is None or self.mando_zero_tcp is None:      # si self.mando_zero_raw is None or self.m...
                print("Primer fes Set Mando Zero!")                             # executem print()
                self.btn_mando_control.setChecked(False)                        # executem setChecked()
                return                                                          # retornem sense valor

            self.mode = "MANDO"                                                 # assignem self.mode
            self.mando_comp_filt = None                                         # assignem self.mando_comp_filt
            self.mando_cmd_pose = None                                          # assignem self.mando_cmd_pose
            self.mando_last_tick = None                                         # assignem self.mando_last_tick

            self.btn_mando_control.setText("Mando Control: ON")                 # executem setText()
            self.btn_mando_control.setStyleSheet(                               # executem setStyleSheet()
                f"background-color: {COLORS['accent_warn']}; color: white; font-weight: bold;"
            )
            print("MANDO CONTROL ACTIVAT")                                      # executem print()
        else:                                                                   # en cas contrari
            self.mode = "IK"                                                    # assignem self.mode
            self.btn_mando_control.setText("Mando Control: OFF")                # executem setText()
            self.btn_mando_control.setStyleSheet("")                            # executem setStyleSheet()
            print("MANDO CONTROL DESACTIVAT")                                   # executem print()

        self._update_mode_ui()                                                  # executem _update_mode_ui()


    def _mando_apply_to_target(self):                                           # definim la funció _mando_apply_to_target()
        if self.mando_zero_raw is None or self.mando_zero_tcp is None:          # si self.mando_zero_raw is None or self.m...
            return                                                              # retornem sense valor

        delta = [self.mando_raw[i] - self.mando_zero_raw[i] for i in range(6)]  # assignem delta

        conv_g = 360.0 / self.mando_pulsos_per_volta                            # assignem conv_g
        conv_m = (np.pi * self.mando_diametre_corro_mm) / self.mando_pulsos_per_volta  # assignem conv_m

        rot_idx = (0, 4, 5)                                                     # rot_idx ← resultat de ()
        rot_sign = (1, 1, 1)                                                    # rot_sign ← resultat de ()
        transl_idx = (3, 2, 1)                                                  # transl_idx ← resultat de ()
        transl_sign = (1, 1, 1)                                                 # transl_sign ← resultat de ()

        rx_deg = delta[rot_idx[0]] * rot_sign[0] * conv_g                       # assignem rx_deg
        ry_deg = delta[rot_idx[1]] * rot_sign[1] * conv_g                       # assignem ry_deg
        rz_deg = delta[rot_idx[2]] * rot_sign[2] * conv_g                       # assignem rz_deg
        x_mm   = delta[transl_idx[0]] * transl_sign[0] * conv_m                 # assignem x_mm
        y_mm   = delta[transl_idx[1]] * transl_sign[1] * conv_m                 # assignem y_mm
        z_mm   = delta[transl_idx[2]] * transl_sign[2] * conv_m                 # assignem z_mm

        yaw_rad = np.deg2rad(self.mando_yaw_deg)                                # yaw_rad ← resultat de deg2rad()
        R_yaw = Rz(yaw_rad)                                                     # R_yaw ← resultat de Rz()

        comps_raw = np.array([                                                  # assignem comps_raw
            np.deg2rad(rx_deg),
            np.deg2rad(ry_deg),
            np.deg2rad(rz_deg),
            x_mm / 1000.0,
            y_mm / 1000.0,
            z_mm / 1000.0,
        ])

        if self.mando_comp_filt is None:                                        # si self.mando_comp_filt is None
            self.mando_comp_filt = comps_raw.copy()                             # self.mando_comp_filt ← resultat de copy()
        else:                                                                   # en cas contrari
            a = self.mando_ema_alpha                                            # assignem a
            self.mando_comp_filt = a * comps_raw + (1.0 - a) * self.mando_comp_filt  # assignem self.mando_comp_filt

        rx, ry, rz, dx, dy, dz = self.mando_comp_filt                           # assignem rx, ry, rz, dx, dy, dz

        dx *= self.mando_gain_xyz                                               # multipliquem dx
        dy *= self.mando_gain_xyz                                               # multipliquem dy
        dz *= self.mando_gain_xyz                                               # multipliquem dz
        rx *= self.mando_gain_rpy                                               # multipliquem rx
        ry *= self.mando_gain_rpy                                               # multipliquem ry
        rz *= self.mando_gain_rpy                                               # multipliquem rz

        delta_trans = R_yaw @ np.array([dx, dy, dz])                            # delta_trans ← resultat de array()
        R_delta = R_yaw @ (Rz(rz) @ Ry(ry) @ Rx(rx)) @ R_yaw.T                  # assignem R_delta

        T_goal = self.mando_zero_tcp.copy()                                     # T_goal ← resultat de copy()
        T_goal[:3, 3] += delta_trans                                            # sumem a T_goal[:3, 3]
        T_goal[:3, :3] = R_delta @ self.mando_zero_tcp[:3, :3]                  # T_goal[:3, :3] = R_delta @ self.mando_zero_tcp[:3, :3]

        now = time.monotonic()                                                  # now ← resultat de monotonic()
        if self.mando_last_tick is None or self.mando_cmd_pose is None:         # si self.mando_last_tick is None or self....
            self.mando_cmd_pose = T_goal.copy()                                 # self.mando_cmd_pose ← resultat de copy()
        else:                                                                   # en cas contrari
            dt = max(now - self.mando_last_tick, 1e-3)                          # dt ← resultat de max()
            max_lin = self.mando_max_lin_speed * dt                             # assignem max_lin
            max_ang = self.mando_max_ang_speed * dt                             # assignem max_ang

            d = T_goal[:3, 3] - self.mando_cmd_pose[:3, 3]                      # assignem d
            d_norm = float(np.linalg.norm(d))                                   # d_norm ← resultat de float()
            if d_norm > max_lin:                                                # si d_norm > max_lin
                self.mando_cmd_pose[:3, 3] += d * (max_lin / d_norm)            # sumem a self.mando_cmd_pose[:3, 3]
            else:                                                               # en cas contrari
                self.mando_cmd_pose[:3, 3] = T_goal[:3, 3]                      # self.mando_cmd_pose[:3, 3] = T_goal[:3, 3]

            R_err = T_goal[:3, :3] @ self.mando_cmd_pose[:3, :3].T              # assignem R_err
            cos_angle = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)       # cos_angle ← resultat de clip()
            angle_err = np.arccos(cos_angle)                                    # angle_err ← resultat de arccos()
            if angle_err > max_ang and angle_err > 1e-6:                        # si angle_err > max_ang and angle_err > 1e-6
                axis = np.array([                                               # assignem axis
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ]) / (2 * np.sin(angle_err))
                K = np.array([                                                  # assignem K
                    [       0, -axis[2],  axis[1]],
                    [ axis[2],        0, -axis[0]],
                    [-axis[1],  axis[0],        0],
                ])
                R_step = np.eye(3) + np.sin(max_ang) * K + (1 - np.cos(max_ang)) * (K @ K)  # R_step ← resultat de eye()
                self.mando_cmd_pose[:3, :3] = R_step @ self.mando_cmd_pose[:3, :3]  # self.mando_cmd_pose[:3, :3] = R_step @ self.mando_cmd_pos...
            else:                                                               # en cas contrari
                self.mando_cmd_pose[:3, :3] = T_goal[:3, :3]                    # self.mando_cmd_pose[:3, :3] = T_goal[:3, :3]

        self.mando_last_tick = now                                              # assignem self.mando_last_tick

        self.T_target = self.mando_cmd_pose.copy()                              # self.T_target ← resultat de copy()


    # ==========================================================================
    # CONSOLA
    # ==========================================================================

    def _update_console(self, T_actual):                                        # definim la funció _update_console()
        x, y, z = T_actual[:3, 3] * 1000                                        # assignem x, y, z
        R = T_actual[:3, :3]                                                    # assignem R
        pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))                         # pitch ← resultat de arcsin()
        roll = np.arctan2(R[2, 1], R[2, 2])                                     # roll ← resultat de arctan2()
        yaw = np.arctan2(R[1, 0], R[0, 0])                                      # yaw ← resultat de arctan2()
        rx, ry, rz = np.degrees([roll, pitch, yaw])                             # rx, ry, rz ← resultat de degrees()

        q_deg = np.degrees(self.q)                                              # q_deg ← resultat de degrees()
        display_order = [1, 3, 2, 5, 0, 4]                                      # assignem display_order
        q_ordered = [q_deg[i] for i in display_order]                           # assignem q_ordered
        q_str = "  ".join(f"{v:+7.1f}" for v in q_ordered)

        if self.current_status == "BLOCKED":                                    # si self.current_status == "BLOCKED"
            status_text = "BLOCKED"                                             # assignem status_text
        elif self.current_status == "TARGET REACHED":                           # altrament si self.current_status == "TARGET REACHED"
            status_text = "TARGET REACHED"                                      # assignem status_text
        else:                                                                   # en cas contrari
            status_text = "OK"                                                  # assignem status_text

        text = (                                                                # assignem text
            f"─── COBOT OS — DLS PRO ───\n\n"
            f"MODE: {self.mode}\n\n"
            f"TCP POSE\n"
            f"  X:  {x:+7.1f} mm    Rx: {rx:+7.1f}°\n"
            f"  Y:  {y:+7.1f} mm    Ry: {ry:+7.1f}°\n"
            f"  Z:  {z:+7.1f} mm    Rz: {rz:+7.1f}°\n\n"
            f"JOINTS (deg)\n"
            f"  [{q_str}]\n\n"
            f"STATUS\n"
            f"  {status_text}\n"
        )

        if self.mando_active:                                                   # si self.mando_active
            raw_str = "  ".join(f"{v:+6d}" for v in self.mando_raw)
            text += f"\nMANDO 6DOF ({self.mando_port})\n  [{raw_str}]\n"

        if self.motor_controller.is_connected:                                  # si self.motor_controller.is_connected
            try:                                                                # intentem el bloc següent
                q_motors = self.motor_controller.read_q()                       # q_motors ← resultat de read_q()
                q_motors_deg = np.degrees(q_motors)                             # q_motors_deg ← resultat de degrees()
                q_ordered = [q_motors_deg[i] for i in display_order]            # assignem q_ordered
                motor_str = " ".join(f"{v:5.1f}" for v in q_ordered)
                text += (                                                       # sumem a text
                    f"\nMOTORS (deg)\n"
                    f"  [{motor_str}]\n"
                    f"  Mirror: {'ON' if self.mirror_active else 'OFF'}\n"
                )
            except Exception:                                                   # capturem excepció: Exception
                pass                                                            # no fem res (placeholder)

        self.console_text.setText(text)                                         # executem setText()


# ==============================================================================
# ENTRY POINT
# ==============================================================================

def main():                                                                     # definim la funció main()
    app = QtWidgets.QApplication(sys.argv)                                      # app ← resultat de QApplication()
    viewer = ViewerDLSPro()                                                     # viewer ← resultat de ViewerDLSPro()
    viewer.show()                                                               # executem show()
    sys.exit(app.exec_())                                                       # executem exit()


if __name__ == '__main__':                                                      # si __name__ == '__main__'
    main()                                                                      # executem main()