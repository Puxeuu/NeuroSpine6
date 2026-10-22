import os
import sys
import time
import importlib.util
import numpy as np

from qtpy import QtWidgets, QtCore, QtGui

from ..theme import (
    C_BG_WINDOW, C_PANEL, C_BORDER, C_TEXT_DIM, C_WARN,
)
from ..modes.mode_selector import ModeSelector
from ..modes.modes_config import allow_broca
from ..modes.coordinator import (
    hide_redundant_widgets, enter_idle, MODE_ENTER,
)
from ..panels.context_panel import ContextPanel
from ..panels.action_bar import ActionBar
from .j6_control import J6ControlPanel
from ..panels.screw_panel import ScrewPanel
from ..panels.currents_panel import CurrentsPanel
from ..modes.j6_drill import J6Drill, ensure_j6_position_hold

from src.hardware.handguide_controller import HandGuideController
from src.dynamics.dynamic_loader import load_dynamic_params


J6_SPIN_HZ = 40

HANDGUIDE_HZ = 50

CONTEXT_PANEL_HEIGHT = 663
SCREW_PANEL_HEIGHT   = 260

COLOR_RESETTING = "#0a84ff"
COLOR_OK        = "#30d158"
COLOR_ERROR     = "#ff3b30"


def _load_viewer_cobot():
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    path = os.path.join(root, "scripts", "viewer_cobot.py")

    if "viewer_cobot" in sys.modules:
        return sys.modules["viewer_cobot"]

    spec = importlib.util.spec_from_file_location("viewer_cobot", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["viewer_cobot"] = module
    spec.loader.exec_module(module)
    return module


class RobotWorkspace(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )

        self._inner            = None
        self._j6_panel         = None
        self._j6_container     = None
        self._context_panel    = None
        self._mode_selector    = None
        self._current_mode     = "ik6"
        self._reg_screws_m     = []
        self._active_screw_idx = None
        self._vector_actors    = []
        self._spin_timer       = QtCore.QTimer(self)
        self._spin_timer.timeout.connect(self._spin_j6_tick)
        self._spin_velocity_rad_s = 0.0
        self._j6_angle         = 0.0
        self._tick_count       = 0
        self._mirror_was_on_before_spin = False

        self._drill            = None

        self._hg_controller = None

        self._hg_timer = QtCore.QTimer(self)
        self._hg_timer.timeout.connect(self._hg_tick)

        root_layout = QtWidgets.QHBoxLayout(self)
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(10)

        try:
            mod = _load_viewer_cobot()
            self._inner = mod.ViewerDLSPro()
            self._inner.setWindowFlags(QtCore.Qt.Widget)

            central = self._inner.centralWidget()
            if central is None:
                raise RuntimeError("ViewerDLSPro no té centralWidget")

            hide_redundant_widgets(self._inner)

            left_col = QtWidgets.QWidget()
            left_col.setStyleSheet("background: transparent;")
            lc = QtWidgets.QVBoxLayout(left_col)
            lc.setContentsMargins(0, 0, 0, 0)
            lc.setSpacing(8)

            self._mode_selector = ModeSelector(initial_mode=self._current_mode)
            self._mode_selector.modeChanged.connect(self._on_mode_changed)
            lc.addWidget(self._mode_selector)

            self._view_split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
            self._view_split.setChildrenCollapsible(False)
            self._view_split.setHandleWidth(8)
            self._view_split.setStyleSheet(f"""
                QSplitter::handle {{ background: transparent; }}
                QSplitter::handle:hover {{ background: {C_BORDER}; border-radius: 3px; }}
            """)
            self._view_split.addWidget(central)

            self._currents_panel = CurrentsPanel(self._inner)
            self._currents_panel.hide()
            self._view_split.addWidget(self._currents_panel)
            self._view_split.setStretchFactor(0, 1)
            self._view_split.setStretchFactor(1, 0)
            lc.addWidget(self._view_split, 1)

            self._action_bar = ActionBar(self._inner)
            self._action_bar.currentsToggled.connect(self._on_currents_toggled)
            lc.addWidget(self._action_bar)

            root_layout.addWidget(left_col, 1)

            right = QtWidgets.QWidget()
            right.setFixedWidth(340)
            right.setStyleSheet("background: transparent;")
            right_layout = QtWidgets.QVBoxLayout(right)
            right_layout.setContentsMargins(0, 0, 0, 0)
            right_layout.setSpacing(10)

            self._context_panel = ContextPanel(self._inner)
            self._context_panel.setFixedHeight(CONTEXT_PANEL_HEIGHT)
            right_layout.addWidget(self._context_panel)

            self._screw_panel = ScrewPanel()
            self._screw_panel.setFixedHeight(SCREW_PANEL_HEIGHT)
            self._screw_panel.showVectorRequested.connect(self._on_show_vector)
            self._screw_panel.goApproachRequested.connect(self._on_go_approach)
            self._screw_panel.selectionChanged.connect(self._on_screw_selection)
            self._screw_panel.trajectoryDepthChanged.connect(self._on_trajectory_depth)
            right_layout.addWidget(self._screw_panel)

            right_layout.addStretch(1)

            root_layout.addWidget(right)

            j6_container = QtWidgets.QFrame()
            j6_container.setStyleSheet(f"""
                QFrame {{
                    background-color: {C_PANEL};
                    border: 1px solid {C_BORDER};
                    border-radius: 12px;
                }}
            """)
            j6_container.setMinimumHeight(220)

            j6_layout = QtWidgets.QVBoxLayout(j6_container)
            j6_layout.setContentsMargins(14, 12, 14, 12)
            j6_layout.setSpacing(6)

            self._j6_panel = J6ControlPanel(j6_container)
            self._j6_panel.velocityRequested.connect(self._on_j6_velocity)
            self._j6_panel.torqueRequested.connect(self._on_j6_torque)
            self._j6_panel.stopRequested.connect(self._on_j6_stop)
            j6_layout.addWidget(self._j6_panel)

            self._drill = J6Drill(
                self._inner,
                setpoint_fn=lambda: self._j6_panel.slider_tor.value(),
                parent=self,
            )

            self._action_bar.btn_stop.clicked.disconnect()
            self._action_bar.btn_stop.clicked.connect(self.emergency_stop)

            self._j6_container = j6_container

            self._install_broca_in_ik5_panel()
            self._install_shortcuts()

            self._activate_mode(self._current_mode, first_time=True)

        except Exception as e:
            self._show_error(root_layout, e)

    def _install_broca_in_ik5_panel(self):
        ik5_panel = self._context_panel._panels.get("ik5")
        if ik5_panel is None:
            print("[layout] no s'ha trobat IK5Panel al context_panel")
            return
        layout = ik5_panel.layout()
        if layout is None:
            return
        insert_at = layout.count()
        for i in range(layout.count() - 1, -1, -1):
            item = layout.itemAt(i)
            if item and item.spacerItem() is not None:
                insert_at = i
                break
        layout.insertWidget(insert_at, self._j6_container)
        print("[layout] broca instal·lada dins del IK5Panel")

    def _show_error(self, layout, exc):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setAlignment(QtCore.Qt.AlignCenter)
        v.setSpacing(16)

        icon = QtWidgets.QLabel("⚠")
        icon.setAlignment(QtCore.Qt.AlignCenter)
        icon.setStyleSheet(f"color: {C_WARN}; font-size: 48px;")
        v.addWidget(icon)

        title = QtWidgets.QLabel("No s'ha pogut carregar el workspace del robot")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("color: white; font-size: 15px; font-weight: 600;")
        v.addWidget(title)

        msg = QtWidgets.QLabel(f"{type(exc).__name__}: {exc}")
        msg.setAlignment(QtCore.Qt.AlignCenter)
        msg.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: 12px; padding: 20px;")
        msg.setWordWrap(True)
        v.addWidget(msg)

        layout.addWidget(w)

    def _install_shortcuts(self):
        for i, m_id in enumerate(["fk", "ik6", "ik5", "mando", "handguide"], start=1):
            sc = QtWidgets.QShortcut(QtGui.QKeySequence(str(i)), self)
            sc.activated.connect(lambda mid=m_id: self._mode_selector.set_mode(mid))
        sc_i = QtWidgets.QShortcut(QtGui.QKeySequence("I"), self)
        sc_i.activated.connect(self._action_bar.btn_currents.toggle)

    def _on_currents_toggled(self, on):
        self._currents_panel.setVisible(on)
        if on:
            total = sum(self._view_split.sizes())
            h = min(380, max(300, int(total * 0.42)))
            self._view_split.setSizes([max(total - h, 1), h])

    def get_viewer(self):
        return self._inner

    def is_robot_connected(self):
        if self._inner is None:
            return False
        mc = getattr(self._inner, "motor_controller", None)
        if mc is None:
            return False
        try:
            attr = getattr(mc, "_connected", None)
            if attr is not None:
                return bool(attr)
            return bool(mc.is_connected())
        except Exception:
            return False

    def _btn_connect_set_style(self, color, text):
        btn = getattr(self._inner, "btn_connect", None)
        if btn is None:
            return
        btn.setText(text)
        btn.setStyleSheet(
            f"background-color: {color}; color: white; font-weight: bold;"
        )
        btn.repaint()
        QtWidgets.QApplication.processEvents()

    def _reset_and_reconnect_motors(self):
        if self._inner is None:
            return False

        mc = getattr(self._inner, "motor_controller", None)
        if mc is None:
            return False

        self._btn_connect_set_style(COLOR_RESETTING, "Resetting motors...")
        print("[motors] reset lògic")

        self._safe_set_mirror(False)

        if self._spin_timer.isActive():
            self._spin_timer.stop()
            self._spin_velocity_rad_s = 0.0

        if self._j6_panel is not None:
            self._j6_panel.set_stopped()
        if self._drill is not None:
            self._drill.disarm()

        if not self.is_robot_connected():
            try:
                mc.connect()
                mc.enable_position_control_at(self._inner.q)
                self._btn_connect_set_style(COLOR_OK, "Motors connectats")
                btn = getattr(self._inner, "btn_connect", None)
                if btn is not None:
                    btn.setChecked(True)
                btn_mirror = getattr(self._inner, "btn_mirror", None)
                if btn_mirror is not None:
                    btn_mirror.setEnabled(True)

                if self._current_mode in ("fk", "ik6", "ik5"):
                    self._safe_set_mirror(True)
                    print("[motors] mirror activat automàticament")

                print("[motors] connectats de nou (daemon nou)")
                return True
            except Exception as e:
                self._handle_reset_failure(e)
                return False

        try:
            if hasattr(mc, "bus") and mc.bus is not None:
                mc.bus.reset_all()

            time.sleep(0.3)

            mc.enable_position_control_at(self._inner.q)

            self._btn_connect_set_style(COLOR_OK, "Motors connectats")
            btn = getattr(self._inner, "btn_connect", None)
            if btn is not None:
                btn.setChecked(True)
            btn_mirror = getattr(self._inner, "btn_mirror", None)
            if btn_mirror is not None:
                btn_mirror.setEnabled(True)

            if self._current_mode in ("fk", "ik6", "ik5"):
                self._safe_set_mirror(True)
                print("[motors] mirror activat automàticament")

            print("[motors] reset OK · motors operatius")
            return True

        except Exception as e:
            self._handle_reset_failure(e)
            return False

    def _handle_reset_failure(self, exception):
        self._btn_connect_set_style(COLOR_ERROR, "Error de reset")
        btn = getattr(self._inner, "btn_connect", None)
        if btn is not None:
            btn.setChecked(False)
        btn_mirror = getattr(self._inner, "btn_mirror", None)
        if btn_mirror is not None:
            btn_mirror.setEnabled(False)

        print(f"[motors] FALLADA del reset: {exception}")
        QtWidgets.QMessageBox.warning(
            self, "Motors",
            f"No s'ha pogut resetejar els motors.\n\n"
            f"Error: {type(exception).__name__}: {exception}\n\n"
            f"Comprova que el cable USB estigui connectat.\n"
            f"Torna a canviar de mode per reintentar."
        )

    def _on_mode_changed(self, mode_id):
        self._activate_mode(mode_id)

    def _activate_mode(self, mode_id, first_time=False):
        old = self._current_mode
        print(f"[mode] {old} → {mode_id}")

        if self._inner is not None and hasattr(self._inner, "cancel_motion"):
            self._inner.cancel_motion()

        if self._j6_panel is not None and self._j6_panel.is_running():
            self._j6_panel.set_stopped()
        if self._drill is not None:
            if old == "ik5" and mode_id == "ik5":
                self._drill.finish_now()
            else:
                self._drill.disarm()
        if self._spin_timer.isActive():
            self._on_j6_stop()

        if self._hg_controller is not None:
            self._stop_handguide_native()

        if not first_time:
            try:
                enter_idle(self._inner)
            except Exception as e:
                print(f"[mode] error a enter_idle: {e}")

        SOFT_MODES = {"fk", "ik6", "ik5", "mando", "handguide"}
        needs_reset = (
            not first_time
            and not (old in SOFT_MODES and mode_id in SOFT_MODES)
        )

        if needs_reset:
            ok = self._reset_and_reconnect_motors()
            if not ok:
                print(f"[mode] mode {mode_id} NO activat per fallada de motors")
                return
        else:
            print("[mode] transició suau, sense reset")
            if (mode_id in ("fk", "ik6", "ik5", "mando")
                    and self.is_robot_connected()):
                mc = getattr(self._inner, "motor_controller", None)
                if mode_id != "ik5":
                    ensure_j6_position_hold(self._inner)
                if mc is not None:
                    try:
                        mc.enable_position_control_at(self._inner.q)
                    except Exception as e:
                        print(f"[mode] error re-activant POSITION_MODE: {e}")

                if not self._safe_is_mirror_on():
                    self._safe_set_mirror(True)
                    print("[mode] mirror activat automàticament (transició suau)")

        self._current_mode = mode_id

        if mode_id == "handguide":
            enter_fn = MODE_ENTER.get("handguide")
            if enter_fn is not None:
                try:
                    enter_fn(self._inner)
                except Exception as e:
                    print(f"[mode] error entrant a handguide: {e}")

            ok = self._start_handguide_native()
            if not ok:
                self._current_mode = old
                if self._mode_selector is not None:
                    self._mode_selector.set_mode(old)
                return
        else:
            enter_fn = MODE_ENTER.get(mode_id)
            if enter_fn is not None:
                try:
                    enter_fn(self._inner)
                except Exception as e:
                    print(f"[mode] error entrant a {mode_id}: {e}")

        if mode_id == "ik5" and self._drill is not None:
            try:
                self._drill.arm()
            except Exception as e:
                print(f"[mode] error armant la broca: {e}")

        if mode_id == "mando":
            mando_ok = bool(getattr(self._inner, "mando_active", False))
            if not mando_ok:
                QtWidgets.QMessageBox.warning(
                    self, "Mando 6DOF",
                    "No s'ha pogut connectar amb el mando al port COM5.\n\n"
                    "Comprova que està endollat i que cap altre programa\n"
                    "no està usant el port. Tornant al mode anterior."
                )
                self._current_mode = old
                if self._mode_selector is not None:
                    self._mode_selector.set_mode(old)
                return

        if self._context_panel is not None:
            self._context_panel.set_mode(mode_id)

        self._sync_j6_availability()

        self._apply_trajectory_slider_visibility()

    def _sync_j6_availability(self):
        can_use = allow_broca(self._current_mode)
        if self._j6_panel:
            self._j6_panel.setEnabled(can_use)

    def _on_j6_velocity(self, current_mA, velocity_counts):
        if not allow_broca(self._current_mode):
            self._j6_panel._on_stop()
            return

        if not self.is_robot_connected():
            QtWidgets.QMessageBox.warning(
                self, "Robot",
                "El robot no està connectat. Connecta el port sèrie primer."
            )
            self._j6_panel._on_stop()
            return

        rpm = float(velocity_counts)
        self._spin_velocity_rad_s = rpm * 2.0 * np.pi / 60.0

        if not self._spin_timer.isActive():
            self._j6_angle = float(self._inner.q[5])

            self._mirror_was_on_before_spin = self._safe_is_mirror_on()
            if self._mirror_was_on_before_spin:
                print("[J6] apagant mirror del viewer per evitar conflicte")
                self._safe_set_mirror(False)

            print(f"[J6] START · RPM={rpm:.1f} · rad/s={self._spin_velocity_rad_s:.3f}")
            self._spin_timer.start(int(1000 / J6_SPIN_HZ))
            self._tick_count = 0

    def _on_j6_torque(self, current_mA):
        if self._drill is not None and self._drill.state == J6Drill.RUNNING:
            self._drill.set_current(current_mA)
            return

        if not allow_broca(self._current_mode):
            self._j6_panel.set_stopped()
            return

        if not self.is_robot_connected():
            QtWidgets.QMessageBox.warning(
                self, "Robot",
                "El robot no està connectat. Connecta el port sèrie primer."
            )
            self._j6_panel.set_stopped()
            return

        if self._drill is None or not self._drill.start(current_mA):
            print("[J6] no s'ha pogut engegar la broca (cal el daemon real)")
            self._j6_panel.set_stopped()

    def _on_j6_stop(self):
        was_spin = self._spin_timer.isActive()
        self._spin_timer.stop()
        self._spin_velocity_rad_s = 0.0

        if was_spin and self._inner is not None:
            try:
                angle_wrapped = float(np.remainder(
                    self._j6_angle + np.pi, 2.0 * np.pi) - np.pi)

                self._inner.q[5] = angle_wrapped
                self._j6_angle   = angle_wrapped

                print(f"[J6] STOP · viewer.q[5] re-sincronitzat a "
                      f"{angle_wrapped:+.3f} rad "
                      f"({np.degrees(angle_wrapped):+.1f}°)")
            except Exception as e:
                print(f"[J6 stop] error sincronitzant: {e}")

        if self._mirror_was_on_before_spin:
            print("[J6] restaurant mirror del viewer")
            self._safe_set_mirror(True)
            self._mirror_was_on_before_spin = False

        if self._drill is not None:
            self._drill.stop()

    def emergency_stop(self):
        print("[STOP] aturada de tots els motors")
        if self._j6_panel is not None:
            self._j6_panel.set_stopped()
        self._spin_timer.stop()
        if self._drill is not None:
            self._drill.abort()
        if self._inner is not None and hasattr(self._inner, "_on_stop"):
            try:
                self._inner._on_stop()
            except Exception as e:
                print(f"[STOP] error: {e}")

    def _spin_j6_tick(self):
        if not self.is_robot_connected():
            self._on_j6_stop()
            return

        dt = 1.0 / J6_SPIN_HZ
        dq = self._spin_velocity_rad_s * dt
        self._j6_angle += dq

        try:
            viewer = self._inner
            q_send = viewer.q.copy()
            q_send[5] = self._j6_angle
            viewer.motor_controller.send_q(q_send)

            self._tick_count += 1
            if self._tick_count % 40 == 0:
                revs = self._j6_angle / (2.0 * np.pi)
                print(f"[J6 #{self._tick_count}] {revs:+.2f} voltes")
        except Exception as e:
            print(f"[J6 tick] error: {e}")
            self._on_j6_stop()

    def _safe_is_mirror_on(self):
        try:
            return bool(getattr(self._inner, "mirror_active", False))
        except Exception:
            return False

    def _safe_set_mirror(self, on):
        try:
            btn = getattr(self._inner, "btn_mirror", None)
            if btn is None:
                return
            if btn.isChecked() != on:
                btn.setChecked(on)
                if hasattr(self._inner, "_on_mirror_toggle"):
                    self._inner._on_mirror_toggle()
        except Exception as e:
            print(f"[mirror] {e}")

    def _start_handguide_native(self):
        if not self.is_robot_connected():
            QtWidgets.QMessageBox.warning(
                self, "Hand-Guide",
                "Els motors no estan connectats. Torna a un altre mode\n"
                "primer per connectar-los."
            )
            return False

        self._inner._handguide_active = True

        self._safe_set_mirror(False)

        deadline = time.monotonic() + 0.2
        while self._safe_is_mirror_on() and time.monotonic() < deadline:
            QtWidgets.QApplication.processEvents()
            time.sleep(0.01)
        if self._safe_is_mirror_on():
            print("[handguide] ATENCIÓ: mirror no s'ha apagat!")
        else:
            print("[handguide] mirror apagat correctament")

        try:
            root = os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.abspath(__file__)))))
            dyn_path = os.path.join(root, "config", "cobot.urdf")
            dyn_params = load_dynamic_params(dyn_path)
        except Exception as e:
            QtWidgets.QMessageBox.warning(
                self, "Hand-Guide",
                f"No s'han pogut carregar els paràmetres dinàmics del URDF:\n{e}"
            )
            self._inner._handguide_active = False
            return False

        def _on_q(q_rad):
            try:
                self._inner.q[:] = q_rad
            except Exception as e:
                print(f"[handguide] on_q_updated: {e}")

        def _on_limit(j_idx):
            print(f"[handguide] límit assolit a j{j_idx+1}, atura't")

        def _on_error(msg):
            print(f"[handguide] error: {msg}")

        try:
            self._hg_controller = HandGuideController(
                motor_controller=self._inner.motor_controller,
                dyn_params=dyn_params,
                robot=self._inner.robot,
                on_q_updated=_on_q,
                on_limit_hit=_on_limit,
                on_error=_on_error,
            )
            self._hg_controller.start()
        except Exception as e:
            QtWidgets.QMessageBox.warning(
                self, "Hand-Guide",
                f"Error iniciant el controller:\n{type(e).__name__}: {e}"
            )
            self._hg_controller = None
            self._inner._handguide_active = False
            return False

        if not self._hg_controller.is_active():
            QtWidgets.QMessageBox.warning(
                self, "Hand-Guide",
                "No s'ha pogut activar el controller.\n"
                "Comprova els logs de la consola."
            )
            self._hg_controller = None
            self._inner._handguide_active = False
            return False

        mirror_ok = not self._safe_is_mirror_on()
        viewer_mode = getattr(self._inner, "mode", "?")
        print("[handguide] estats abans d'arrencar timer:")
        print(f"    mirror off:   {mirror_ok}")
        print(f"    viewer.mode:  {viewer_mode}")
        print(f"    motors conn:  {self.is_robot_connected()}")

        self._hg_timer.start(int(1000 / HANDGUIDE_HZ))
        print(f"[handguide] mode natiu actiu ({HANDGUIDE_HZ} Hz)")
        return True

    def _stop_handguide_native(self):
        import time as _time

        if self._hg_timer.isActive():
            self._hg_timer.stop()

        if self._hg_controller is not None:
            try:
                self._hg_controller.stop()
            except Exception as e:
                print(f"[handguide] error a stop(): {e}")
            self._hg_controller = None

        if self._inner is not None:
            self._inner._handguide_active = False

        if self._inner is None:
            return
        mc = getattr(self._inner, "motor_controller", None)
        if mc is None or not mc.is_connected:
            return

        try:
            mc.bus.reset_all()
            _time.sleep(0.15)

            q_actual = self._inner.q.copy()
            mc.enable_position_control_at(q_actual)
            _time.sleep(0.1)

            for _ in range(3):
                try:
                    mc.send_q(q_actual)
                    _time.sleep(0.03)
                except Exception as e:
                    print(f"[handguide] error send_q: {e}")
                    break

            print("[handguide] motors reset+POSITION_MODE+send_q OK")

        except Exception as e:
            print(f"[handguide] error reconfigurant: {e}")

    def _hg_tick(self):
        if self._hg_controller is None:
            return
        try:
            self._hg_controller.step()
        except Exception as e:
            print(f"[handguide] tick error: {e}")
            self._stop_handguide_native()

    def load_registered_scene(self, vertebra_mesh, screws, T_ct_to_robot,
                              vertebres=None):
        import numpy as np
        import pyvista as pv

        if self._inner is None:
            return

        plotter = getattr(self._inner, "plotter", None)
        if plotter is None:
            print("[robot] load_registered_scene: viewer sense plotter")
            return

        self._remove_registered_scene()
        self._reg_actors = []

        if vertebres is None:
            if vertebra_mesh is None or T_ct_to_robot is None:
                vertebres_dict = {}
            else:
                vertebres_dict = {"default": {
                    "mesh": vertebra_mesh,
                    "T":    np.asarray(T_ct_to_robot, dtype=np.float64),
                }}
        else:
            vertebres_dict = {k: {"mesh": v["mesh"],
                                   "T": np.asarray(v["T"], dtype=np.float64)}
                              for k, v in vertebres.items() if v.get("mesh") is not None
                              and v.get("T") is not None}

        if not vertebres_dict:
            print("[robot] load_registered_scene: cap vèrtebra amb T vàlida")
            return

        def make_transformer(T):
            R = T[:3, :3]
            t_mm = T[:3, 3]
            def ct_mm_to_robot_m(pts_ct_mm):
                pts = np.asarray(pts_ct_mm, dtype=np.float64)
                single = (pts.ndim == 1)
                if single:
                    pts = pts[None, :]
                pts_robot_mm = (R @ pts.T).T + t_mm
                pts_robot_m = pts_robot_mm * 0.001
                return pts_robot_m[0] if single else pts_robot_m
            return ct_mm_to_robot_m

        vertebra_info = {}
        for code, data in vertebres_dict.items():
            mesh = data["mesh"]
            T    = data["T"]
            xform = make_transformer(T)
            try:
                pts_ct   = np.asarray(mesh.points, dtype=np.float64)
                centroid = pts_ct.mean(axis=0)
                pts_robot = xform(pts_ct)
                mesh_r = mesh.copy()
                mesh_r.points = pts_robot
                actor_name = f"reg_vertebra_{code}"
                plotter.add_mesh(
                    mesh_r,
                    color=(207, 200, 188),
                    opacity=1.0,
                    smooth_shading=True,
                    specular=0.0,
                    specular_power=1,
                    ambient=0.65,
                    diffuse=0.45,
                    name=actor_name,
                )
                self._reg_actors.append(actor_name)
                vertebra_info[code] = {"T": T, "centroid_ct": centroid, "xform": xform}
            except Exception as e:
                print(f"[robot] Error afegint vèrtebra '{code}': {e}")

        def pick_vertebra_for_screw(p0_ct):
            if len(vertebra_info) == 1:
                only = next(iter(vertebra_info.values()))
                return only["xform"], only
            best = None
            best_d = float("inf")
            for info in vertebra_info.values():
                d = float(np.linalg.norm(np.asarray(p0_ct) - info["centroid_ct"]))
                if d < best_d:
                    best_d = d
                    best = info
            return best["xform"], best

        APPROACH_OFFSET_M = 0.01
        self._reg_screws_m = []
        if screws:
            for i, cg in enumerate(screws):
                try:
                    p0_mm_ct, p1_mm_ct = self._screw_endpoints_mm(cg)
                    if p0_mm_ct is None:
                        continue
                    xform, _info = pick_vertebra_for_screw(p0_mm_ct)
                    p0_m = np.asarray(xform(np.array(p0_mm_ct)), dtype=np.float64)
                    p1_m = np.asarray(xform(np.array(p1_mm_ct)), dtype=np.float64)
                    axis = p1_m - p0_m
                    L = float(np.linalg.norm(axis))
                    if L < 1e-6:
                        continue
                    axis_unit = axis / L
                    diam_mm = float(cg.get("diam", 5.5))
                    radius_m = (diam_mm / 2.0) * 0.001
                    tube = pv.Line(p0_m, p1_m).tube(radius=radius_m)
                    color = cg.get("color", "#e0e0e5")
                    name = f"reg_screw_{i}"
                    plotter.add_mesh(
                        tube, color=color, name=name,
                        smooth_shading=True,
                        specular=0.4, specular_power=15,
                        ambient=0.25, diffuse=0.75,
                    )
                    self._reg_actors.append(name)
                    p_approach_m = p0_m - APPROACH_OFFSET_M * axis_unit
                    self._reg_screws_m.append({
                        "idx": i,
                        "name": f"Cargol {i + 1}",
                        "p0_m": p0_m,
                        "p1_m": p1_m,
                        "axis_unit": axis_unit,
                        "length_m": L,
                        "diam_m": diam_mm * 0.001,
                        "color": color,
                        "p_approach_m": p_approach_m,
                    })
                except Exception as e:
                    print(f"[robot] Error preparant cargol {i}: {e}")

        if hasattr(self, "_screw_panel") and self._screw_panel is not None:
            self._screw_panel.set_screws(self._reg_screws_m)

        plotter.render()
        print(f"[robot] Escena registrada: vèrtebres={list(vertebra_info.keys())}, "
              f"cargols={len(self._reg_screws_m)}")

    def _screw_endpoints_mm(self, cg):
        main = self.window()
        planning_ws = None
        if hasattr(main, "_workspaces"):
            planning_ws = main._workspaces.get("planning")
        pw = None
        if planning_ws is not None:
            pw = getattr(planning_ws, "widget", None) or planning_ws
        if pw is None or not hasattr(pw, "spacing") or not hasattr(pw, "origin"):
            print("[robot] no puc trobar spacing/origin del CT")
            return None, None

        spacing = pw.spacing
        origin = pw.origin

        try:
            p0 = cg["p0"]
            p1 = cg["p1"]
            z0 = cg.get("z_p0", cg.get("z", 0))
            z1 = cg.get("z_p1", cg.get("z", 0))
            p0_mm = (origin[0] + p0[0] * spacing[0],
                     origin[1] + p0[1] * spacing[1],
                     origin[2] + z0   * spacing[2])
            p1_mm = (origin[0] + p1[0] * spacing[0],
                     origin[1] + p1[1] * spacing[1],
                     origin[2] + z1   * spacing[2])
            return p0_mm, p1_mm
        except (KeyError, IndexError, TypeError) as e:
            print(f"[robot] estructura de cargol inesperada: {e}")
            return None, None

    def _on_screw_selection(self, idx):
        if idx < 0 or idx >= len(self._reg_screws_m):
            self._active_screw_idx = None
        else:
            self._active_screw_idx = idx

    def _on_show_vector(self, idx):
        import pyvista as pv

        if idx < 0 or idx >= len(self._reg_screws_m):
            return
        cg = self._reg_screws_m[idx]
        plotter = getattr(self._inner, "plotter", None)
        if plotter is None:
            return

        self._clear_vector_actors()

        p0 = cg["p0_m"]
        p1 = cg["p1_m"]
        p_app = cg["p_approach_m"]
        color = cg["color"]

        axis_arrow = pv.Arrow(
            start=p0, direction=p1 - p0, scale=float(cg["length_m"]),
            tip_length=0.15, tip_radius=0.006, shaft_radius=0.003,
        )
        plotter.add_mesh(axis_arrow, color=color, opacity=0.9,
                          name="reg_vector_axis", lighting=False)
        self._vector_actors.append("reg_vector_axis")

        app_arrow = pv.Arrow(
            start=p_app, direction=p0 - p_app, scale=0.03,
            tip_length=0.20, tip_radius=0.004, shaft_radius=0.0015,
        )
        plotter.add_mesh(app_arrow, color="#ffffff", opacity=0.8,
                          name="reg_vector_approach", lighting=False)
        self._vector_actors.append("reg_vector_approach")

        sphere = pv.Sphere(radius=0.004, center=p_app)
        plotter.add_mesh(sphere, color="#ffffff", opacity=0.95,
                          name="reg_vector_sphere",
                          smooth_shading=True, specular=0.4)
        self._vector_actors.append("reg_vector_sphere")

        plotter.render()
        print(f"[robot] Vector mostrat: {cg['name']} · "
              f"approach a {p_app[0]*1000:+.1f}, "
              f"{p_app[1]*1000:+.1f}, {p_app[2]*1000:+.1f} mm")

    def _on_go_approach(self, idx):
        if idx < 0 or idx >= len(self._reg_screws_m):
            return
        cg = self._reg_screws_m[idx]
        viewer = self._inner
        if viewer is None:
            return

        if self._current_mode != "ik6":
            QtWidgets.QMessageBox.information(
                self, "Anar a approach",
                "Canvia al mode IK 6DOF per enviar el robot al punt d'approach."
            )
            return

        x_new = cg["axis_unit"]
        R_current = viewer.poses["tcp_link"][:3, :3]
        x_current = R_current[:, 0]

        cos_theta = float(np.clip(np.dot(x_current, x_new), -1.0, 1.0))
        if cos_theta > 0.9999:
            R_align = np.eye(3)
        elif cos_theta < -0.9999:
            axis = R_current[:, 1]
            if abs(np.dot(axis, x_new)) > 0.9:
                axis = R_current[:, 2]
            R_align = self._rot_matrix_from_axis_angle(axis, np.pi)
        else:
            rot_axis = np.cross(x_current, x_new)
            rot_axis /= np.linalg.norm(rot_axis)
            theta = np.arccos(cos_theta)
            R_align = self._rot_matrix_from_axis_angle(rot_axis, theta)

        R_new = R_align @ R_current

        T_new = np.eye(4)
        T_new[:3, :3] = R_new
        T_new[:3,  3] = cg["p_approach_m"]

        viewer.T_target = T_new
        viewer.T_ref_pos = T_new[:3, 3].copy()
        viewer.R_target_initial = R_new.copy()
        viewer.target_xyz = np.array([0.0, 0.0, 0.0])
        viewer.target_rpy = np.array([0.0, 0.0, 0.0])
        viewer.e_smoothed = np.zeros(6)

        for actor in viewer.target_actors:
            actor.user_matrix = T_new

        if hasattr(viewer, "_sync_all_sliders"):
            viewer._sync_all_sliders(recenter=True)

        viewer._homing_goal = None
        viewer._slow_until_reached = True

        print(f"[robot] T_target → approach de {cg['name']}: "
              f"{cg['p_approach_m'][0]*1000:+.1f}, "
              f"{cg['p_approach_m'][1]*1000:+.1f}, "
              f"{cg['p_approach_m'][2]*1000:+.1f} mm")

    def _on_trajectory_depth(self, idx, depth_mm):
        if idx < 0 or idx >= len(self._reg_screws_m):
            return
        if self._current_mode not in ("ik5", "ik6"):
            return
        viewer = self._inner
        if viewer is None:
            return

        cg = self._reg_screws_m[idx]
        axis_unit = cg["axis_unit"]
        p_approach = cg["p_approach_m"]
        depth_m = depth_mm * 0.001

        p_target_new = p_approach + depth_m * axis_unit

        T_new = viewer.T_target.copy()
        T_new[:3, 3] = p_target_new

        viewer.T_target = T_new
        viewer.T_ref_pos = T_new[:3, 3].copy()
        viewer.target_xyz = np.array([0.0, 0.0, 0.0])

        for actor in viewer.target_actors:
            actor.user_matrix = T_new

        if hasattr(viewer, "_sync_all_sliders"):
            viewer._sync_all_sliders(recenter=True)

        if depth_mm % 5 == 0:
            print(f"[trajectory] {cg['name']} avanç {depth_mm} mm → "
                  f"({p_target_new[0]*1000:+.1f}, {p_target_new[1]*1000:+.1f}, "
                  f"{p_target_new[2]*1000:+.1f}) mm")

    def _apply_trajectory_slider_visibility(self):
        if not hasattr(self, "_screw_panel") or self._screw_panel is None:
            return
        visible = (self._current_mode in ("ik5", "ik6"))
        self._screw_panel.set_trajectory_slider_visible(visible)

    @staticmethod
    def _rot_matrix_from_axis_angle(axis, angle):
        axis = np.asarray(axis, dtype=np.float64)
        axis = axis / np.linalg.norm(axis)
        K = np.array([
            [       0.0, -axis[2],  axis[1]],
            [ axis[2],       0.0, -axis[0]],
            [-axis[1],  axis[0],      0.0],
        ])
        return (np.eye(3)
                + np.sin(angle) * K
                + (1.0 - np.cos(angle)) * (K @ K))

    def _clear_vector_actors(self):
        plotter = getattr(self._inner, "plotter", None)
        if plotter is None:
            return
        for name in self._vector_actors:
            try:
                plotter.remove_actor(name)
            except Exception:
                pass
        self._vector_actors = []
        plotter.render()

    def _remove_registered_scene(self):
        if self._inner is None:
            return
        plotter = getattr(self._inner, "plotter", None)
        if plotter is None:
            return
        for name in getattr(self, "_reg_actors", []):
            try:
                plotter.remove_actor(name)
            except Exception:
                pass
        self._reg_actors = []
        self._clear_vector_actors()
        self._reg_screws_m = []
        self._active_screw_idx = None
        if hasattr(self, "_screw_panel") and self._screw_panel is not None:
            self._screw_panel.set_screws([])

    def cleanup(self):
        import time as _time

        self._spin_timer.stop()
        if self._drill is not None:
            self._drill.abort()
        if getattr(self, "_currents_panel", None) is not None:
            self._currents_panel.stop()
        if self._hg_timer.isActive():
            self._hg_timer.stop()

        if self._hg_controller is not None:
            try:
                self._hg_controller.stop()
            except Exception as e:
                print(f"[cleanup] error aturant hand-guide: {e}")
            self._hg_controller = None

        if self._inner is not None:
            mc = getattr(self._inner, "motor_controller", None)
            if mc is not None and mc.is_connected:
                try:
                    mc.bus.reset_all()
                    _time.sleep(0.15)
                    print("[cleanup] motors en RESET (blau)")
                except Exception as e:
                    print(f"[cleanup] error reset bus: {e}")
                try:
                    mc.disconnect()
                    print("[cleanup] daemon dels motors tancat")
                except Exception as e:
                    print(f"[cleanup] error disconnect: {e}")

        if self._inner is not None:
            try:
                self._inner.close()
            except Exception:
                pass

