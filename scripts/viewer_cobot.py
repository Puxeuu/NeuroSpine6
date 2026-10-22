import serial
import sys, os
import time
import threading

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from qtpy import QtWidgets, QtCore
from qtpy.QtCore import Qt

pv.OFF_SCREEN = False

from src.model.urdf_loader import load_urdf
from src.kinematics.forward import forward_kinematics
from src.kinematics.jacobian import compute_jacobian
from src.kinematics.manipulability import yoshikawa_position, adaptive_damping
from src.control.pose_error import pose_error
from src.control.dls_solver import dls
from src.collision.collision_loader import load_collisions
from src.collision.checker import check_collisions_with_margin
from src.math.transforms import Rx, Ry, Rz, make_T
from src.hardware.motor_config import load_motor_config
from src.hardware.motor_controller import MotorController
from scripts.viewer_capcals_patch import (capcal_manager_init, capcal_add_selector_to_options_panel,)


COLORS = {
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


JOINT_LOWER = np.array([-np.pi/2, -np.pi/6, -np.pi/2.2, -np.pi/2, -np.pi/2, -np.pi])
JOINT_UPPER = np.array([ np.pi/2,  np.pi/2,  np.pi/2,   np.pi/2,  np.pi/6,  np.pi])


CONTROL_DT = 0.01
KP_POS = 5.0
KP_ORI = 3.0
MAX_JOINT_SPEED = np.deg2rad(60)

HOME_SPEED_DEG_S     = 30
APPROACH_SPEED_DEG_S = 20

HOME_JOINT_SPEED     = np.deg2rad(HOME_SPEED_DEG_S)
APPROACH_JOINT_SPEED = np.deg2rad(APPROACH_SPEED_DEG_S)

LAMBDA_MIN = 0.005
LAMBDA_MAX = 0.3
W_THRESHOLD = 0.003

ERROR_SMOOTH_ALPHA = 0.15

TOL_POS = 0.001
TOL_ORI = 0.01

SAFETY_MARGIN = 0.005

XYZ_RANGE = 0.20
RPY_RANGE = np.pi / 3
XYZ_MULT = 4000
RPY_MULT = 4000


class ViewerDLSPro(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.mando_port = "COM5"
        self.mando_baud = 115200
        self.mando_serial = None
        self.mando_thread = None
        self.mando_active = False
        self.mando_raw = [0, 0, 0, 0, 0, 0]
        self._mando_svc = None

        self.mando_gain_xyz = 0.2
        self.mando_gain_rpy = 0.2
        self.mando_ema_alpha = 0.25
        self.mando_comp_filt = None
        self.mando_max_lin_speed = 0.03
        self.mando_max_ang_speed = np.deg2rad(25)
        self.mando_cmd_pose = None
        self.mando_last_tick = None

        self.mando_zero_raw = None
        self.mando_zero_tcp = None

        self.robot = load_urdf(os.path.join(ROOT, "config", "cobot.urdf"))
        self.prims = load_collisions(os.path.join(ROOT, "config", "collision_geometry.yaml"))
        capcal_manager_init(self)
        self.motor_config = load_motor_config(os.path.join(ROOT, "config", "motor_config.yaml"))

        from scripts.neurospine_app.preferences import load_preferences, DEFAULT_PREFS
        try:
            robot_port = load_preferences().get("robot_port", DEFAULT_PREFS["robot_port"])
        except Exception as e:
            print(f"[motors] no s'han pogut llegir les preferències: {e}")
            robot_port = DEFAULT_PREFS["robot_port"]
        print(f"[motors] port del robot: {robot_port}")
        self.motor_controller = MotorController(self.motor_config, port=robot_port)
        self.mirror_active = False

        self.mando_robot = load_urdf(os.path.join(ROOT, "config", "mando6dof.urdf"))
        self.mando_offset = np.array([0.17, -0.2, 0.0])
        self.mando_yaw_deg = 180.0
        self.mando_actors = {}
        self.mando_pulsos_per_volta = 600.0
        self.mando_diametre_corro_mm = 40.0

        self.q = np.zeros(6)
        self.poses = forward_kinematics(self.robot, self.q)

        self.T_ref_pos = self.poses["tcp_link"][:3, 3].copy()
        self.R_target_initial = self.poses["tcp_link"][:3, :3].copy()

        self.T_target = self.poses["tcp_link"].copy()
        self.target_xyz = np.array([0.0, 0.0, 0.0])
        self.target_rpy = np.array([0.0, 0.0, 0.0])

        self.e_smoothed = np.zeros(6)
        self.current_status = "ok"
        self.blocked_violating = set()

        self._updating_sliders = False

        self.mode = "IK"

        self._handguide_active = False

        self._j6_ik_override = None

        self._j6_center = 0.0

        self._homing_goal = None
        self._slow_until_reached = False

        self.setWindowTitle("Cobot OS — DLS Pro Edition")
        self.resize(1700, 950)
        self._apply_style()
        self._build_ui()
        self._setup_3d_scene()
        self._update_mode_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._teleop_step)
        self.timer.start(int(CONTROL_DT * 1000))

    def sync_target_to_current(self):
        self.poses = forward_kinematics(self.robot, self.q)
        T_actual = self.poses["tcp_link"]

        self.T_ref_pos = T_actual[:3, 3].copy()
        self.R_target_initial = T_actual[:3, :3].copy()

        self.target_xyz = np.array([0.0, 0.0, 0.0])
        self.target_rpy = np.array([0.0, 0.0, 0.0])

        self.T_target = T_actual.copy()
        self.e_smoothed = np.zeros(6)

        for actor in self.target_actors:
            actor.user_matrix = self.T_target

        self._sync_all_sliders(recenter=True)

    def _apply_style(self):
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

    def _build_ui(self):
        central = QtWidgets.QWidget()
        central.setObjectName("CentralWidget")
        self.setCentralWidget(central)
        main = QtWidgets.QHBoxLayout(central)
        main.setContentsMargins(15, 15, 15, 15)
        main.setSpacing(15)

        left = QtWidgets.QVBoxLayout()
        left.setSpacing(15)

        self.view_container = QtWidgets.QFrame()
        self.view_container.setObjectName("ApplePanel")
        vc_layout = QtWidgets.QVBoxLayout(self.view_container)
        vc_layout.setContentsMargins(0, 0, 0, 0)

        self.plotter = QtInteractor(
            self.view_container,
            multi_samples=0,
            line_smoothing=False,
            point_smoothing=False,
            polygon_smoothing=False,
        )
        vc_layout.addWidget(self.plotter.interactor)
        left.addWidget(self.view_container, stretch=7)

        bottom = QtWidgets.QHBoxLayout()
        bottom.setSpacing(15)
        bottom.addWidget(self._create_joints_panel(), 1)
        bottom.addWidget(self._create_cart_panel(), 1)
        left.addLayout(bottom, stretch=0)

        main.addLayout(left, stretch=7)

        right = QtWidgets.QVBoxLayout()
        right.setSpacing(15)

        top = QtWidgets.QFrame()
        top.setObjectName("ApplePanel")
        top_layout = QtWidgets.QHBoxLayout(top)
        top_layout.setContentsMargins(15, 15, 15, 15)
        top_layout.setSpacing(15)

        self.console_text = QtWidgets.QTextEdit()
        self.console_text.setReadOnly(True)
        top_layout.addWidget(self.console_text, stretch=6)

        btns = QtWidgets.QVBoxLayout()
        btns.setSpacing(10)

        self.btn_mode = QtWidgets.QPushButton("Mode: IK")
        self.btn_mode.setStyleSheet(
            f"background-color: {COLORS['accent_x']}; color: white; font-weight: bold;"
        )
        self.btn_mode.clicked.connect(self._on_mode_toggle)
        btns.addWidget(self.btn_mode)

        self.btn_zero = QtWidgets.QPushButton("Move to zero")
        self.btn_zero.clicked.connect(self._on_move_to_zero)
        btns.addWidget(self.btn_zero)

        self.btn_connect = QtWidgets.QPushButton("Connect motors (SIM)")
        self.btn_connect.setCheckable(True)
        self.btn_connect.clicked.connect(self._on_connect_motors)
        btns.addWidget(self.btn_connect)

        self.btn_mirror = QtWidgets.QPushButton("Mirror OFF")
        self.btn_mirror.setCheckable(True)
        self.btn_mirror.setEnabled(False)
        self.btn_mirror.clicked.connect(self._on_mirror_toggle)
        btns.addWidget(self.btn_mirror)

        self.btn_mando = QtWidgets.QPushButton("Mando 6DOF: OFF")
        self.btn_mando.setCheckable(True)
        self.btn_mando.clicked.connect(self._on_mando_toggle)
        btns.addWidget(self.btn_mando)

        self.btn_mando_control = QtWidgets.QPushButton("Mando Control: OFF")
        self.btn_mando_control.setCheckable(True)
        self.btn_mando_control.setEnabled(False)
        self.btn_mando_control.clicked.connect(self._on_mando_control_toggle)
        btns.addWidget(self.btn_mando_control)

        self.btn_mando_zero = QtWidgets.QPushButton("Set Mando Zero")
        self.btn_mando_zero.setEnabled(False)
        self.btn_mando_zero.clicked.connect(self._on_mando_set_zero)
        btns.addWidget(self.btn_mando_zero)

        self.btn_routes = QtWidgets.QPushButton("Plan Routes")
        self.btn_routes.setStyleSheet(
            f"background-color: {COLORS['accent_q']}; color: white; font-weight: bold;"
        )
        self.btn_routes.setEnabled(False)
        btns.addWidget(self.btn_routes)

        self.btn_stop = QtWidgets.QPushButton("STOP")
        self.btn_stop.setStyleSheet(
            f"background-color: {COLORS['accent_err']}; color: white; border: none; font-weight: bold;"
        )
        self.btn_stop.clicked.connect(self._on_stop)
        btns.addWidget(self.btn_stop)

        btns.addStretch()
        top_layout.addLayout(btns, stretch=4)
        right.addWidget(top, stretch=8)

        options = QtWidgets.QFrame()
        options.setObjectName("ApplePanel")
        opt = QtWidgets.QVBoxLayout(options)
        opt.setContentsMargins(20, 15, 20, 15)
        title = QtWidgets.QLabel("OPTIONS")
        title.setObjectName("Title")
        opt.addWidget(title)

        opt_row = QtWidgets.QHBoxLayout()
        self.chk_collisions = QtWidgets.QCheckBox("Collision detection")
        self.chk_collisions.setChecked(True)
        opt_row.addWidget(self.chk_collisions)
        self.chk_target = QtWidgets.QCheckBox("Show target")
        self.chk_target.setChecked(True)
        self.chk_target.toggled.connect(self._on_target_visibility)
        opt_row.addWidget(self.chk_target)
        opt_row.addStretch()
        opt.addLayout(opt_row)
        capcal_add_selector_to_options_panel(self, opt)
        right.addWidget(options, stretch=2)

        self._create_overlay_buttons()
        main.addLayout(right, stretch=0)

    def _create_joints_panel(self):
        self.joints_panel = QtWidgets.QFrame()
        self.joints_panel.setObjectName("ApplePanel")
        layout = QtWidgets.QVBoxLayout(self.joints_panel)
        layout.setContentsMargins(20, 15, 20, 15)

        grid = QtWidgets.QGridLayout()
        grid.setVerticalSpacing(10)
        grid.setHorizontalSpacing(15)

        self.joint_sliders = []
        self.joint_labels = []

        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        for i in range(6):
            name_lbl = QtWidgets.QLabel(joint_names[i])
            name_lbl.setFixedWidth(28)
            name_lbl.setStyleSheet("font-weight: bold; color: #aeaeb2;")

            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setMinimum(int(JOINT_LOWER[i] * 1000))
            slider.setMaximum(int(JOINT_UPPER[i] * 1000))
            slider.setValue(0)

            val_lbl = QtWidgets.QLabel("0.000")
            val_lbl.setFixedWidth(60)
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val_lbl.setStyleSheet("font-family: 'SF Mono', monospace; color: #aeaeb2;")

            grid.addWidget(name_lbl, i, 0)
            grid.addWidget(slider, i, 1)
            grid.addWidget(val_lbl, i, 2)

            self.joint_sliders.append(slider)
            self.joint_labels.append(val_lbl)
            slider.valueChanged.connect(lambda v, idx=i: self._on_joint_slide(idx, v))

        layout.addLayout(grid)
        layout.addStretch()
        return self.joints_panel

    def _create_cart_panel(self):
        self.cart_panel = QtWidgets.QFrame()
        self.cart_panel.setObjectName("ApplePanel")
        layout = QtWidgets.QVBoxLayout(self.cart_panel)
        layout.setContentsMargins(20, 15, 20, 15)

        grid = QtWidgets.QGridLayout()
        grid.setVerticalSpacing(10)
        grid.setHorizontalSpacing(15)

        self.xyz_sliders = []
        self.xyz_labels = []
        self.rpy_sliders = []
        self.rpy_labels = []

        xyz_names = ["X", "Y", "Z"]
        for i in range(3):
            name_lbl = QtWidgets.QLabel(xyz_names[i])
            name_lbl.setFixedWidth(28)
            name_lbl.setStyleSheet("font-weight: bold; color: #aeaeb2;")

            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setObjectName("CartSlider")
            slider.setMinimum(int(-XYZ_RANGE * XYZ_MULT))
            slider.setMaximum(int( XYZ_RANGE * XYZ_MULT))
            slider.setValue(0)

            val_lbl = QtWidgets.QLabel("0.000")
            val_lbl.setFixedWidth(60)
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val_lbl.setStyleSheet("font-family: 'SF Mono', monospace; color: #aeaeb2;")

            grid.addWidget(name_lbl, i, 0)
            grid.addWidget(slider, i, 1)
            grid.addWidget(val_lbl, i, 2)

            self.xyz_sliders.append(slider)
            self.xyz_labels.append(val_lbl)
            slider.valueChanged.connect(lambda v, idx=i: self._on_xyz_slide(idx, v))

        rpy_names = ["R", "P", "Y"]
        for i in range(3):
            name_lbl = QtWidgets.QLabel(rpy_names[i])
            name_lbl.setFixedWidth(28)
            name_lbl.setStyleSheet("font-weight: bold; color: #aeaeb2;")

            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setObjectName("CartSlider")
            slider.setMinimum(int(-RPY_RANGE * RPY_MULT))
            slider.setMaximum(int(RPY_RANGE * RPY_MULT))
            slider.setValue(0)

            val_lbl = QtWidgets.QLabel("0.000")
            val_lbl.setFixedWidth(60)
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val_lbl.setStyleSheet("font-family: 'SF Mono', monospace; color: #aeaeb2;")

            grid.addWidget(name_lbl, i + 3, 0)
            grid.addWidget(slider, i + 3, 1)
            grid.addWidget(val_lbl, i + 3, 2)

            self.rpy_sliders.append(slider)
            self.rpy_labels.append(val_lbl)
            slider.valueChanged.connect(lambda v, idx=i: self._on_rpy_slide(idx, v))

        layout.addLayout(grid)
        layout.addStretch()
        return self.cart_panel

    def _create_overlay_buttons(self):
        self.view_bar = QtWidgets.QFrame(self.view_container)
        self.view_bar.setStyleSheet("""
            QFrame {
                background-color: rgba(28, 28, 30, 0.92);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 8px;
            }
        """)
        vb_layout = QtWidgets.QVBoxLayout(self.view_bar)
        vb_layout.setContentsMargins(4, 4, 4, 4)
        vb_layout.setSpacing(2)

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

        self._view_group = QtWidgets.QButtonGroup(self)
        self._view_group.setExclusive(True)

        specs = [
            ("Iso",   "iso"),
            ("Top",   "top"),
            ("Front", "front"),
            ("Left",  "left"),
            ("Right", "right"),
        ]

        self._view_buttons = {}
        for label, tag in specs:
            btn = QtWidgets.QPushButton(label, self.view_bar)
            btn.setCheckable(True)
            btn.setCursor(QtCore.Qt.PointingHandCursor)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(lambda _checked, t=tag: self._set_camera(t))
            self._view_group.addButton(btn)
            vb_layout.addWidget(btn)
            self._view_buttons[tag] = btn

        self._view_buttons["iso"].setChecked(True)
        self.view_bar.adjustSize()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._position_view_bar()

    def showEvent(self, event):
        super().showEvent(event)
        QtCore.QTimer.singleShot(0, self._position_view_bar)

    def _position_view_bar(self):
        if not hasattr(self, "view_bar") or self.view_bar is None:
            return
        margin = 12
        vc_w = self.view_container.width()
        bar_w = self.view_bar.width()
        self.view_bar.move(vc_w - bar_w - margin, margin)
        self.view_bar.raise_()

    def _setup_3d_scene(self):
        self.plotter.set_background(COLORS['bg_3d'])

        self.plotter.add_light(pv.Light(position=(3, 3, 4), focal_point=(0, 0, 0.5),
                                        color='white', intensity=0.6))
        self.plotter.add_light(pv.Light(position=(-3, -1, 2), focal_point=(0, 0, 0.5),
                                        color='white', intensity=0.4))
        self.plotter.add_light(pv.Light(light_type='headlight', intensity=0.15))

        floor = pv.Plane(center=(0, 0, -0.001), direction=(0, 0, 1),
                         i_size=3.0, j_size=3.0, i_resolution=25, j_resolution=25)
        self.plotter.add_mesh(floor, color='#2c2c2e', style='wireframe',
                              line_width=1, opacity=0.2)
        slab = pv.Plane(center=(0, 0, -0.005), direction=(0, 0, 1),
                        i_size=3.0, j_size=3.0)
        self.plotter.add_mesh(slab, color='#0e0e10', opacity=0.9, lighting=False)

        self.stl_actors = {}
        for link_name, link in self.robot.links.items():
            if not link.mesh:
                continue
            mesh_path = os.path.join(ROOT, "config", link.mesh)
            mesh = pv.read(mesh_path)

            R_vis = Rz(link.visual_rpy[2]) @ Ry(link.visual_rpy[1]) @ Rx(link.visual_rpy[0])
            T_vis = make_T(R_vis, link.visual_xyz)
            mesh.transform(T_vis, inplace=True)

            actor = self.plotter.add_mesh(
                mesh, color=COLORS['robot_color'],
                smooth_shading=False,
                specular=0.4, specular_power=10,
                ambient=0.25, diffuse=0.75,
            )
            actor.user_matrix = self.poses[link_name]
            self.stl_actors[link_name] = actor

        self.tip_actors = self._make_triad(scale=0.08, ghost=False)
        self.target_actors = self._make_triad(scale=0.07, ghost=True)
        self._update_triads()

        self._set_camera('iso')

    def _make_triad(self, scale=0.07, ghost=False):
        colors = [COLORS['accent_err'], COLORS['accent_x'], COLORS['accent_q']]
        opacity = 0.3 if ghost else 0.9
        actors = []
        directions = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
        for i, c in enumerate(colors):
            arrow = pv.Arrow(start=(0, 0, 0), direction=directions[i],
                             tip_length=0.25, shaft_radius=0.02, scale=scale)
            actors.append(self.plotter.add_mesh(arrow, color=c, opacity=opacity, lighting=False))
        return actors

    def _update_triads(self):
        T_ee = self.poses["tcp_link"]
        for actor in self.tip_actors:
            actor.user_matrix = T_ee
        for actor in self.target_actors:
            actor.user_matrix = self.T_target

    def _set_camera(self, mode):
        focal = (0.0, 0.0, 0.15)
        views = {
            'iso':   {'eye': (0.85, 0.85, 0.95), 'up': (0.0, 0.0, 1.0)},
            'top':   {'eye': (0.0,   0.0, 1.25),  'up': (1.0, 0.0, 0.0)},
            'front':  {'eye': (1.05,  0.0, 0.45),  'up': (0.0, 0.0, 1.0)},
            'left':  {'eye': (0.0,   1.05, 0.45), 'up': (0.0, 0.0, 1.0)},
            'right': {'eye': (0.0,  -1.05, 0.45), 'up': (0.0, 0.0, 1.0)},
        }

        v = views.get(mode)
        if v is None:
            return

        self.plotter.camera_position = [v['eye'], focal, v['up']]
        self.plotter.render()

        if hasattr(self, "_view_buttons") and mode in self._view_buttons:
            btn = self._view_buttons[mode]
            if not btn.isChecked():
                btn.setChecked(True)

    def _on_mode_toggle(self):
        if self.mode == "IK":
            self.mode = "FK"
        else:
            self.mode = "IK"
            self.sync_target_to_current()

        self.e_smoothed = np.zeros(6)
        self._update_mode_ui()

    def _update_mode_ui(self):
        if self.mode == "IK":
            for s in self.joint_sliders:
                s.setEnabled(False)
            for s in self.xyz_sliders + self.rpy_sliders:
                s.setEnabled(True)
            self.btn_mode.setText("Mode: IK")
            self.btn_mode.setStyleSheet(
                f"background-color: {COLORS['accent_x']}; color: white; font-weight: bold;"
            )
        elif self.mode == "FK":
            for s in self.joint_sliders:
                s.setEnabled(True)
            for s in self.xyz_sliders + self.rpy_sliders:
                s.setEnabled(False)
            self.btn_mode.setText("Mode: FK")
            self.btn_mode.setStyleSheet(
                f"background-color: {COLORS['accent_q']}; color: white; font-weight: bold;"
            )
        else:
            for s in self.joint_sliders:
                s.setEnabled(False)
            for s in self.xyz_sliders + self.rpy_sliders:
                s.setEnabled(False)
            self.btn_mode.setText("Mode: MANDO")
            self.btn_mode.setStyleSheet(
                f"background-color: {COLORS['accent_warn']}; color: white; font-weight: bold;"
            )

        target_visible = (self.mode == "IK") and self.chk_target.isChecked()
        for actor in self.target_actors:
            actor.SetVisibility(target_visible)

    def _joint_limits(self):
        lower, upper = JOINT_LOWER.copy(), JOINT_UPPER.copy()
        lower[5] = self._j6_center - np.pi
        upper[5] = self._j6_center + np.pi
        return lower, upper

    def set_j6_center(self, center):
        self._j6_center = float(center)
        slider = self.joint_sliders[5]
        self._updating_sliders = True
        slider.setMinimum(int(round((self._j6_center - np.pi) * 1000)))
        slider.setMaximum(int(round((self._j6_center + np.pi) * 1000)))
        slider.setValue(int(self.q[5] * 1000))
        self._updating_sliders = False

    def cancel_motion(self):
        self._homing_goal = None
        self._slow_until_reached = False

    def _check_q_valid(self, q_test):
        q_clipped = np.clip(q_test, *self._joint_limits())
        if not np.allclose(q_clipped, q_test):
            return False, set()

        if self.chk_collisions.isChecked():
            has_violation, pairs = check_collisions_with_margin(
                self.robot, q_test, self.prims, margin=SAFETY_MARGIN
            )
            if has_violation:
                violating = set()
                for l1, l2 in pairs:
                    violating.add(l1)
                    violating.add(l2)
                return False, violating

        return True, set()

    def _sync_all_sliders(self, recenter=True):
        self._updating_sliders = True

        for i, slider in enumerate(self.joint_sliders):
            slider.setValue(int(self.q[i] * 1000))
            self.joint_labels[i].setText(f"{self.q[i]:.3f}")

        for i, slider in enumerate(self.xyz_sliders):
            if recenter:
                slider.setMinimum(int(-XYZ_RANGE * XYZ_MULT))
                slider.setMaximum(int( XYZ_RANGE * XYZ_MULT))
            slider.setValue(int(self.target_xyz[i] * XYZ_MULT))
            self.xyz_labels[i].setText(f"{self.target_xyz[i]:.3f}")

        for i, slider in enumerate(self.rpy_sliders):
            if recenter:
                slider.setMinimum(int(-RPY_RANGE * RPY_MULT))
                slider.setMaximum(int( RPY_RANGE * RPY_MULT))
            slider.setValue(int(self.target_rpy[i] * RPY_MULT))
            self.rpy_labels[i].setText(f"{self.target_rpy[i]:.3f}")

        self._updating_sliders = False

    def _on_move_to_zero(self):
        q_zero = np.zeros(6)
        if getattr(self, "_ik5dof", False):
            q_zero[5] = self.q[5]
        is_valid, violating = self._check_q_valid(q_zero)
        if not is_valid:
            self.blocked_violating = violating
            return

        self.blocked_violating = set()
        self._slow_until_reached = False
        self._homing_goal = q_zero
        print(f"[home] cap a q = 0 a ≤{HOME_SPEED_DEG_S}°/s per joint")

    def _homing_step(self):
        if self._homing_goal is None:
            return False
        ik5 = getattr(self, "_ik5dof", False)
        if ik5:
            self._homing_goal[5] = self.q[5]
        delta = self._homing_goal - self.q
        big = float(np.max(np.abs(delta)))
        step = HOME_JOINT_SPEED * CONTROL_DT
        done = big <= step
        q_try = self._homing_goal.copy() if done else self.q + delta * (step / big)
        q_final, status, violating = self._try_q_new(self.q, q_try)
        if status == "blocked":
            print("[home] aturat: col·lisió")
            self.blocked_violating = violating
            done = True
        else:
            self.q[:] = q_final

        q_m = self.q.copy()
        if ik5 and self._j6_ik_override is not None:
            q_m[5] = self._j6_ik_override
        T = forward_kinematics(self.robot, q_m)["tcp_link"]
        self.T_target = T.copy()
        self.T_ref_pos = T[:3, 3].copy()
        self.R_target_initial = T[:3, :3].copy()
        self.target_xyz = np.zeros(3)
        self.target_rpy = np.zeros(3)
        self.e_smoothed = np.zeros(6)
        for actor in self.target_actors:
            actor.user_matrix = self.T_target

        self._updating_sliders = True
        for i in range(6):
            self.joint_sliders[i].setValue(int(self.q[i] * 1000))
            self.joint_labels[i].setText(f"{self.q[i]:.3f}")
        self._updating_sliders = False

        if done:
            self._homing_goal = None
            self._sync_all_sliders(recenter=True)
            print("[home] fet")
        return True

    def _on_stop(self):
        self.mirror_active = False
        self.cancel_motion()

        if self.motor_controller.is_connected:
            try:
                self.motor_controller.bus.reset_all()
                time.sleep(0.15)
            except Exception as e:
                print(f"[STOP] Error al reset del bus: {e}")

        self.btn_connect.setChecked(False)
        self.btn_mirror.setText("Mirror OFF")
        self.btn_mirror.setChecked(False)
        self.btn_mirror.setStyleSheet("")

        self.sync_target_to_current()

        print("[STOP] motors en RESET (blau). Cliqueu un mode per reactivar-los.")

    def _on_target_visibility(self, checked):
        visible = checked and (self.mode == "IK")
        for actor in self.target_actors:
            actor.SetVisibility(visible)

    def _on_connect_motors(self):
        import time as _time

        if not self.motor_controller.is_connected:
            try:
                self.motor_controller.connect()
                self.motor_controller.enable_position_control_at(self.q)
                _time.sleep(0.1)
                try:
                    self.motor_controller.send_q(self.q)
                except Exception as e:
                    print(f"[connect] error send_q inicial: {e}")

                self.btn_connect.setChecked(True)
                self.btn_mirror.setEnabled(True)
                print("[connect] motors connectats i activats")
            except Exception as e:
                print(f"[connect] Error connectant: {e}")
                self.btn_connect.setChecked(False)
            return

        if self.btn_connect.isChecked():
            try:
                self.motor_controller.bus.reset_all()
                _time.sleep(0.3)
                self.motor_controller.enable_position_control_at(self.q)
                _time.sleep(0.1)
                for _ in range(3):
                    try:
                        self.motor_controller.send_q(self.q)
                        _time.sleep(0.03)
                    except Exception as e:
                        print(f"[connect] error send_q: {e}")
                        break
                self.btn_mirror.setEnabled(True)
                print("[connect] motors reactivats (RESET → POSITION_MODE)")
            except Exception as e:
                print(f"[connect] error reactivant: {e}")
                self.btn_connect.setChecked(False)
            return

        if self.mirror_active:
            self.mirror_active = False
            self.btn_mirror.setText("Mirror OFF")
            self.btn_mirror.setChecked(False)
            self.btn_mirror.setStyleSheet("")

        try:
            self.motor_controller.bus.reset_all()
            _time.sleep(0.1)
        except Exception as e:
            print(f"[connect] error reset: {e}")

        self.btn_mirror.setEnabled(False)
        print("[connect] motors en RESET (blau)")

    def _on_mirror_toggle(self):
        self.mirror_active = self.btn_mirror.isChecked()
        if self.mirror_active:
            self.btn_mirror.setText("Mirror ON")
            self.btn_mirror.setStyleSheet(
                f"background-color: {COLORS['accent_x']}; color: white; font-weight: bold;"
            )
        else:
            self.btn_mirror.setText("Mirror OFF")
            self.btn_mirror.setStyleSheet("")

    def _on_joint_slide(self, idx, int_val):
        if self._updating_sliders or self.mode != "FK":
            return

        val = int_val / 1000.0

        max_delta = MAX_JOINT_SPEED * CONTROL_DT
        delta = val - self.q[idx]
        if abs(delta) > max_delta:
            val = self.q[idx] + np.sign(delta) * max_delta

        q_test = self.q.copy()
        q_test[idx] = val

        is_valid, violating = self._check_q_valid(q_test)

        if is_valid:
            self.q = q_test
            self.blocked_violating = set()
            self.joint_labels[idx].setText(f"{val:.3f}")

            self._updating_sliders = True
            self.joint_sliders[idx].setValue(int(val * 1000))
            self._updating_sliders = False
        else:
            self.blocked_violating = violating
            self._updating_sliders = True
            self.joint_sliders[idx].setValue(int(self.q[idx] * 1000))
            self.joint_labels[idx].setText(f"{self.q[idx]:.3f}")
            self._updating_sliders = False

    def _on_xyz_slide(self, idx, int_val):
        if self._updating_sliders or self.mode != "IK":
            return
        val = int_val / XYZ_MULT
        self.target_xyz[idx] = val
        self.xyz_labels[idx].setText(f"{val:.3f}")
        self._rebuild_target()

    def _on_rpy_slide(self, idx, int_val):
        if self._updating_sliders or self.mode != "IK":
            return
        val = int_val / RPY_MULT
        self.target_rpy[idx] = val
        self.rpy_labels[idx].setText(f"{val:.3f}")
        self._rebuild_target()

    def _rebuild_target(self):
        if not hasattr(self, "T_ref_pos"):
            self.T_ref_pos = self.T_target[:3, 3].copy()

        pos_final = self.T_ref_pos + self.target_xyz

        R_delta = Rz(self.target_rpy[2]) @ Ry(self.target_rpy[1]) @ Rx(self.target_rpy[0])
        R_final = R_delta @ self.R_target_initial

        self.T_target = make_T(R_final, pos_final)
        for actor in self.target_actors:
            actor.user_matrix = self.T_target

    def _try_q_new(self, q_current, q_new):
        q_clipped = np.clip(q_new, *self._joint_limits())

        if self.chk_collisions.isChecked():
            has_violation, pairs = check_collisions_with_margin(
                self.robot, q_clipped, self.prims, margin=SAFETY_MARGIN
            )
            if not has_violation:
                return q_clipped, "ok", set()

            for scale in [0.5, 0.25, 0.125]:
                q_partial = q_current + (q_clipped - q_current) * scale
                q_partial = np.clip(q_partial, *self._joint_limits())
                has_violation_p, _ = check_collisions_with_margin(
                    self.robot, q_partial, self.prims, margin=SAFETY_MARGIN
                )
                if not has_violation_p:
                    return q_partial, "ok", set()

            violating_links = set()
            for l1, l2 in pairs:
                violating_links.add(l1)
                violating_links.add(l2)
            return q_current, "blocked", violating_links
        else:
            return q_clipped, "ok", set()

    def _teleop_step(self):
        if getattr(self, "_handguide_active", False):
            self.poses = forward_kinematics(self.robot, self.q)
            T_actual_passive = self.poses["tcp_link"]
            self.T_target = T_actual_passive.copy()
            self.T_ref_pos = T_actual_passive[:3, 3].copy()
            self.R_target_initial = T_actual_passive[:3, :3].copy()
            for link_name, actor in self.stl_actors.items():
                actor.user_matrix = self.poses[link_name]
            for actor in self.tip_actors:
                actor.user_matrix = self.poses["tcp_link"]
            self._update_console(self.poses["tcp_link"])
            return

        homing = self._homing_step()
        if self.mode == "MANDO" and not homing:
            self._mando_apply_to_target()

        ik5 = getattr(self, "_ik5dof", False)
        q_model = self.q.copy()
        if ik5 and self._j6_ik_override is not None:
            q_model[5] = self._j6_ik_override

        self.poses = forward_kinematics(self.robot, q_model)
        T_actual = self.poses["tcp_link"]

        if self.mode == "FK":
            self.T_target = T_actual.copy()
            self.T_ref_pos = T_actual[:3, 3].copy()
            self.R_target_initial = T_actual[:3, :3].copy()
            self.target_xyz = np.array([0.0, 0.0, 0.0])
            self.target_rpy = np.array([0.0, 0.0, 0.0])
            for actor in self.target_actors:
                actor.user_matrix = self.T_target

        e_raw = pose_error(T_actual, self.T_target)

        J = compute_jacobian(self.robot, q_model)

        J_ik = J
        if ik5:
            J_ik = J[:, :5].copy()
            axis = J[3:6, 5]
            n_axis = np.linalg.norm(axis)
            if n_axis > 1e-9:
                a = axis / n_axis
                P = np.eye(3) - np.outer(a, a)
                e_raw[3:] = P @ e_raw[3:]
                J_ik[3:6, :] = P @ J_ik[3:6, :]

        e_raw[:3] *= KP_POS
        e_raw[3:] *= KP_ORI

        self.e_smoothed = self.e_smoothed + ERROR_SMOOTH_ALPHA * (e_raw - self.e_smoothed)

        err_pos = np.linalg.norm(e_raw[:3]) / KP_POS
        err_ori = np.linalg.norm(e_raw[3:]) / KP_ORI
        target_reached = (err_pos < TOL_POS) and (err_ori < TOL_ORI)

        w = yoshikawa_position(J)
        lam = adaptive_damping(w, LAMBDA_MIN, LAMBDA_MAX, W_THRESHOLD)

        if ik5:
            q_dot = np.zeros(6)
            q_dot[:5] = dls(J_ik, self.e_smoothed, lam)
        else:
            q_dot = dls(J, self.e_smoothed, lam)
        if self._slow_until_reached:
            peak = float(np.max(np.abs(q_dot)))
            if peak > APPROACH_JOINT_SPEED:
                q_dot = q_dot * (APPROACH_JOINT_SPEED / peak)
            elif target_reached:
                self._slow_until_reached = False
                print("[approach] arribat")
        q_dot = np.clip(q_dot, -MAX_JOINT_SPEED, MAX_JOINT_SPEED)

        q_new = self.q + q_dot * CONTROL_DT
        q_final, status, violating = self._try_q_new(self.q, q_new)
        self.q[:] = q_final

        if status == "blocked":
            display_violating = violating
        elif self.blocked_violating:
            display_violating = self.blocked_violating
            status = "blocked"
        else:
            display_violating = set()

        if self.mode == "IK":
            self._updating_sliders = True
            for i in range(6):
                self.joint_sliders[i].setValue(int(self.q[i] * 1000))
                self.joint_labels[i].setText(f"{self.q[i]:.3f}")
            self._updating_sliders = False

        self.poses = forward_kinematics(self.robot, self.q)
        for link_name, actor in self.stl_actors.items():
            actor.user_matrix = self.poses[link_name]
        for actor in self.tip_actors:
            actor.user_matrix = self.poses["tcp_link"]

        if status == "blocked":
            for link_name, actor in self.stl_actors.items():
                if link_name in display_violating:
                    actor.prop.color = COLORS['accent_err']
            self.current_status = "BLOCKED"
        elif target_reached:
            for actor in self.stl_actors.values():
                actor.prop.color = COLORS['target_reached_tint']
            self.current_status = "TARGET REACHED"
        else:
            for actor in self.stl_actors.values():
                actor.prop.color = COLORS['robot_color']
            self.current_status = "OK"

        if status == "ok":
            self.blocked_violating = set()

        self._update_console(T_actual)

        if self.mirror_active and self.motor_controller.is_connected:
            try:
                self.motor_controller.send_q(self.q)
            except Exception as e:
                print(f"Error enviant q: {e}")
                self.mirror_active = False
                self.btn_mirror.setText("Mirror OFF")
                self.btn_mirror.setChecked(False)

        if self.mando_active:
            self._mando_update_poses()

    def _on_mando_toggle(self):
        if not self.mando_active:
            self._mando_connect()
        else:
            self._mando_disconnect()

    def _mando_connect(self):
        try:
            from neurospine_app.mando_service import get_mando_service
            svc = get_mando_service()
        except Exception:
            svc = None

        if svc is not None and svc.is_connected():
            self._mando_svc = svc
            try:
                svc.reading.connect(self._on_mando_service_reading)
            except Exception as e:
                print(f"[viewer_cobot] connect signal mando: {e}")
            self.mando_active = True
            self.mando_thread = None
            self.mando_serial = None
            self._mando_load_scene()
            self.btn_mando.setText("Mando 6DOF: ON")
            self.btn_mando.setStyleSheet(
                f"background-color: {COLORS['accent_x']}; color: white; font-weight: bold;"
            )
            self.btn_mando_zero.setEnabled(True)
            self.btn_mando_control.setEnabled(True)
            print("[viewer_cobot] Mando connectat via MandoService (compartit)")
            return

        if serial is None:
            print("pyserial no instal·lat")
            self.btn_mando.setChecked(False)
            return

        try:
            self.mando_serial = serial.Serial(
                self.mando_port, self.mando_baud, timeout=0.1
            )
        except Exception as e:
            print(f"No s'ha pogut obrir {self.mando_port}: {e}")
            self.btn_mando.setChecked(False)
            return

        self._mando_svc = None
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

    def _on_mando_service_reading(self, data):
        try:
            pols = data.get("pols")
            if pols is not None and len(pols) >= 6:
                self.mando_raw = list(pols[:6])
        except Exception as e:
            print(f"[viewer_cobot] parse mando service: {e}")

    def _mando_disconnect(self):
        self.mando_active = False

        svc = getattr(self, "_mando_svc", None)
        if svc is not None:
            try:
                svc.reading.disconnect(self._on_mando_service_reading)
            except Exception:
                pass
            self._mando_svc = None

        if self.mando_thread is not None:
            self.mando_thread.join(timeout=0.5)
            self.mando_thread = None
        if self.mando_serial is not None:
            try:
                self.mando_serial.close()
            except Exception:
                pass
            self.mando_serial = None

        self._mando_unload_scene()

        self.btn_mando.setText("Mando 6DOF: OFF")
        self.btn_mando.setStyleSheet("")
        print("Mando desconnectat")

        self.btn_mando_zero.setEnabled(False)
        self.btn_mando_control.setEnabled(False)
        self.mando_zero_raw = None
        self.mando_zero_tcp = None

        if self.mode == "MANDO":
            self.mode = "IK"
            self.btn_mando_control.setText("Mando Control: OFF")
            self.btn_mando_control.setStyleSheet("")
            self.btn_mando_control.setChecked(False)
            self._update_mode_ui()

    def _mando_read_loop(self):
        buf = b""
        while self.mando_active and self.mando_serial is not None:
            try:
                data = self.mando_serial.read(128)
                if not data:
                    continue
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._mando_parse(line.decode("utf-8", "ignore").strip())
            except Exception:
                time.sleep(0.05)

    def _mando_parse(self, line):
        if not line or ":" not in line:
            return
        try:
            try:
                from neurospine_app.mando_service import parse_mando_line
            except ImportError:
                from scripts.neurospine_app.mando_service import parse_mando_line
            vals = parse_mando_line(line)
            if vals is not None:
                self.mando_raw = vals
        except Exception:
            pass

    def _mando_load_scene(self):
        if self.mando_actors:
            return

        q_init = np.zeros(7)
        poses = forward_kinematics(self.mando_robot, q_init)

        T_offset = self._mando_get_offset_matrix()

        for link_name, link in self.mando_robot.links.items():
            if not link.mesh:
                continue
            mesh_path = os.path.join(ROOT, "config", link.mesh)
            try:
                mesh = pv.read(mesh_path)
            except Exception as e:
                print(f"No s'ha pogut carregar {mesh_path}: {e}")
                continue

            actor = self.plotter.add_mesh(
                mesh,
                color=COLORS['robot_color'],
                smooth_shading=False,
                specular=0.4, specular_power=10,
                ambient=0.25, diffuse=0.75,
                name=f"mando_{link_name}",
            )
            actor.user_matrix = T_offset @ poses[link_name]
            self.mando_actors[link_name] = actor

        self.plotter.render()

    def _mando_unload_scene(self):
        for name in list(self.mando_actors.keys()):
            try:
                self.plotter.remove_actor(f"mando_{name}")
            except Exception:
                pass
        self.mando_actors.clear()
        self.plotter.render()

    def _mando_get_offset_matrix(self):
        yaw_rad = np.deg2rad(self.mando_yaw_deg)
        T_offset = np.eye(4)
        T_offset[:3, :3] = Rz(yaw_rad)
        T_offset[:3,  3] = self.mando_offset
        return T_offset

    def _mando_update_poses(self):
        if not self.mando_actors:
            return

        conv_g = 360.0 / self.mando_pulsos_per_volta
        conv_m = (np.pi * self.mando_diametre_corro_mm) / self.mando_pulsos_per_volta

        rot_idx = (4, 0, 5)
        rot_sign = (1, -1, 1)
        transl_idx = (3, 2, 1)
        transl_sign = (1, 1, -1)

        zero = self.mando_zero_raw if self.mando_zero_raw is not None else [0] * 6
        raw = [self.mando_raw[i] - zero[i] for i in range(6)]

        rx_deg = raw[rot_idx[0]] * rot_sign[0] * conv_g
        ry_deg = raw[rot_idx[1]] * rot_sign[1] * conv_g
        rz_deg = raw[rot_idx[2]] * rot_sign[2] * conv_g
        x_mm   = raw[transl_idx[0]] * transl_sign[0] * conv_m
        y_mm   = raw[transl_idx[1]] * transl_sign[1] * conv_m
        z_mm   = raw[transl_idx[2]] * transl_sign[2] * conv_m

        j1 = float(np.clip(-0.0175 + z_mm / 1000.0, -0.0175, 0.0175))
        j2 = -j1
        j3 = float(np.clip(x_mm / 1000.0, -0.0276, 0.0276))
        j4 = float(np.clip(y_mm / 1000.0, -0.0276, 0.0276))
        j5 = float(np.deg2rad(rz_deg))
        j6 = float(np.deg2rad(ry_deg))
        j7 = float(np.deg2rad(rx_deg))

        q_mando = np.array([j1, j2, j3, j4, j5, j6, j7])

        poses = forward_kinematics(self.mando_robot, q_mando)
        T_offset = self._mando_get_offset_matrix()

        for link_name, actor in self.mando_actors.items():
            T = poses.get(link_name)
            if T is None:
                continue
            actor.user_matrix = T_offset @ T

    def _on_mando_set_zero(self):
        self.mando_zero_raw = list(self.mando_raw)
        self.mando_zero_tcp = self.poses["tcp_link"].copy()

        self.mando_comp_filt = None
        self.mando_cmd_pose = None
        self.mando_last_tick = None

        print("Set Zero:")
        print(f"  Mando raw: {self.mando_zero_raw}")
        print(f"  TCP pos:   {self.mando_zero_tcp[:3, 3] * 1000} mm")

    def _on_mando_control_toggle(self):
        if self.btn_mando_control.isChecked():
            if self.mando_zero_raw is None or self.mando_zero_tcp is None:
                print("Primer fes Set Mando Zero!")
                self.btn_mando_control.setChecked(False)
                return

            self.mode = "MANDO"
            self.mando_comp_filt = None
            self.mando_cmd_pose = None
            self.mando_last_tick = None

            self.btn_mando_control.setText("Mando Control: ON")
            self.btn_mando_control.setStyleSheet(
                f"background-color: {COLORS['accent_warn']}; color: white; font-weight: bold;"
            )
            print("MANDO CONTROL ACTIVAT")
        else:
            self.mode = "IK"
            self.btn_mando_control.setText("Mando Control: OFF")
            self.btn_mando_control.setStyleSheet("")
            print("MANDO CONTROL DESACTIVAT")

        self._update_mode_ui()

    def _mando_apply_to_target(self):
        if self.mando_zero_raw is None or self.mando_zero_tcp is None:
            return

        delta = [self.mando_raw[i] - self.mando_zero_raw[i] for i in range(6)]

        conv_g = 360.0 / self.mando_pulsos_per_volta
        conv_m = (np.pi * self.mando_diametre_corro_mm) / self.mando_pulsos_per_volta

        rot_idx = (0, 4, 5)
        rot_sign = (1, 1, 1)
        transl_idx = (3, 2, 1)
        transl_sign = (1, 1, 1)

        rx_deg = delta[rot_idx[0]] * rot_sign[0] * conv_g
        ry_deg = delta[rot_idx[1]] * rot_sign[1] * conv_g
        rz_deg = delta[rot_idx[2]] * rot_sign[2] * conv_g
        x_mm   = delta[transl_idx[0]] * transl_sign[0] * conv_m
        y_mm   = delta[transl_idx[1]] * transl_sign[1] * conv_m
        z_mm   = delta[transl_idx[2]] * transl_sign[2] * conv_m

        yaw_rad = np.deg2rad(self.mando_yaw_deg)
        R_yaw = Rz(yaw_rad)

        comps_raw = np.array([
            np.deg2rad(rx_deg),
            np.deg2rad(ry_deg),
            np.deg2rad(rz_deg),
            x_mm / 1000.0,
            y_mm / 1000.0,
            z_mm / 1000.0,
        ])

        if self.mando_comp_filt is None:
            self.mando_comp_filt = comps_raw.copy()
        else:
            a = self.mando_ema_alpha
            self.mando_comp_filt = a * comps_raw + (1.0 - a) * self.mando_comp_filt

        rx, ry, rz, dx, dy, dz = self.mando_comp_filt

        dx *= self.mando_gain_xyz
        dy *= self.mando_gain_xyz
        dz *= self.mando_gain_xyz
        rx *= self.mando_gain_rpy
        ry *= self.mando_gain_rpy
        rz *= self.mando_gain_rpy

        delta_trans = R_yaw @ np.array([dx, dy, dz])
        R_delta = R_yaw @ (Rz(rz) @ Ry(ry) @ Rx(rx)) @ R_yaw.T

        T_goal = self.mando_zero_tcp.copy()
        T_goal[:3, 3] += delta_trans
        T_goal[:3, :3] = R_delta @ self.mando_zero_tcp[:3, :3]

        now = time.monotonic()
        if self.mando_last_tick is None or self.mando_cmd_pose is None:
            self.mando_cmd_pose = T_goal.copy()
        else:
            dt = max(now - self.mando_last_tick, 1e-3)
            max_lin = self.mando_max_lin_speed * dt
            max_ang = self.mando_max_ang_speed * dt

            d = T_goal[:3, 3] - self.mando_cmd_pose[:3, 3]
            d_norm = float(np.linalg.norm(d))
            if d_norm > max_lin:
                self.mando_cmd_pose[:3, 3] += d * (max_lin / d_norm)
            else:
                self.mando_cmd_pose[:3, 3] = T_goal[:3, 3]

            R_err = T_goal[:3, :3] @ self.mando_cmd_pose[:3, :3].T
            cos_angle = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
            angle_err = np.arccos(cos_angle)
            if angle_err > max_ang and angle_err > 1e-6:
                axis = np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ]) / (2 * np.sin(angle_err))
                K = np.array([
                    [       0, -axis[2],  axis[1]],
                    [ axis[2],        0, -axis[0]],
                    [-axis[1],  axis[0],        0],
                ])
                R_step = np.eye(3) + np.sin(max_ang) * K + (1 - np.cos(max_ang)) * (K @ K)
                self.mando_cmd_pose[:3, :3] = R_step @ self.mando_cmd_pose[:3, :3]
            else:
                self.mando_cmd_pose[:3, :3] = T_goal[:3, :3]

        self.mando_last_tick = now

        self.T_target = self.mando_cmd_pose.copy()

    def _update_console(self, T_actual):
        x, y, z = T_actual[:3, 3] * 1000
        R = T_actual[:3, :3]
        pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
        rx, ry, rz = np.degrees([roll, pitch, yaw])

        q_deg = np.degrees(self.q)
        display_order = [1, 3, 2, 5, 0, 4]
        q_ordered = [q_deg[i] for i in display_order]
        q_str = "  ".join(f"{v:+7.1f}" for v in q_ordered)

        if self.current_status == "BLOCKED":
            status_text = "BLOCKED"
        elif self.current_status == "TARGET REACHED":
            status_text = "TARGET REACHED"
        else:
            status_text = "OK"

        text = (
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

        if self.mando_active:
            raw_str = "  ".join(f"{v:+6d}" for v in self.mando_raw)
            text += f"\nMANDO 6DOF ({self.mando_port})\n  [{raw_str}]\n"

        if self.motor_controller.is_connected:
            try:
                q_motors = self.motor_controller.read_q()
                q_motors_deg = np.degrees(q_motors)
                q_ordered = [q_motors_deg[i] for i in display_order]
                motor_str = " ".join(f"{v:5.1f}" for v in q_ordered)
                text += (
                    f"\nMOTORS (deg)\n"
                    f"  [{motor_str}]\n"
                    f"  Mirror: {'ON' if self.mirror_active else 'OFF'}\n"
                )
            except Exception:
                pass

        self.console_text.setText(text)


def main():
    app = QtWidgets.QApplication(sys.argv)
    viewer = ViewerDLSPro()
    viewer.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

