"""
Workspace del Robot amb selector de 5 modes.

TRANSICIONS SUAUS ENTRE FK / IK6 / IK5:
    Els tres modes comparteixen la mateixa configuració del bus.
    Canviar entre ells NO requereix reset ni reconnexió.
    El sync del target ↔ TCP actual el fa el coordinator via el mètode
    viewer.sync_target_to_current() als enter_ik6/enter_ik5.
    Així els sliders RPY sempre parteixen de 0 respecte de la pose actual
    del TCP, i mai no fan salts.

RESET + RECONNEXIÓ DE MOTORS:
    Només es fa quan cal (entrar/sortir de MANDO o HAND-GUIDE), amb un
    reset lògic al bus (mc.bus.reset_all() + enable_position_control())
    sense matar el daemon ni tancar el port sèrie.
"""

import os                                                                       # importem el mòdul os
import sys                                                                      # importem el mòdul sys
import time                                                                     # importem el mòdul time
import importlib.util                                                           # importem el mòdul importlib.util
import numpy as np                                                              # importem el mòdul numpy as np

from qtpy import QtWidgets, QtCore, QtGui                                       # importem QtWidgets, QtCore, QtGui des de qtpy

from ..theme import (                                                           # importem ( des de ..theme
    C_BG_WINDOW, C_PANEL, C_BORDER, C_TEXT_DIM, C_WARN,
)
from ..modes.mode_selector import ModeSelector                                  # importem ModeSelector des de ..modes.mode_selector
from ..modes.modes_config import allow_broca                                    # importem allow_broca des de ..modes.modes_config
from ..modes.coordinator import (                                               # importem ( des de ..modes.coordinator
    hide_redundant_widgets, enter_idle, MODE_ENTER,
)
from ..panels.context_panel import ContextPanel                                 # importem ContextPanel des de ..panels.context_panel
from ..panels.action_bar import ActionBar                                       # importem ActionBar des de ..panels.action_bar
from .j6_control import J6ControlPanel                                          # importem J6ControlPanel des de .j6_control
from ..panels.screw_panel import ScrewPanel                                     # importem ScrewPanel des de ..panels.screw_panel
from ..panels.currents_panel import CurrentsPanel                               # gràfic d'intensitats dels 6 motors
from ..modes.j6_drill import J6Drill, ensure_j6_position_hold                   # broca del J6 (IK5)

from src.hardware.handguide_controller import HandGuideController               # importem HandGuideController des de src.hardware.handguide_controller
from src.dynamics.dynamic_loader import load_dynamic_params                     # importem load_dynamic_params des de src.dynamics.dynamic_loader


J6_SPIN_HZ = 40                                                                 # assignem J6_SPIN_HZ
# (Els paràmetres de la broca en TORQUE són a modes/j6_drill.py)

# Freqüència del bucle de hand-guide (Hz). 50 Hz és el valor original.
HANDGUIDE_HZ = 50                                                               # assignem HANDGUIDE_HZ

CONTEXT_PANEL_HEIGHT = 663                                                      # assignem CONTEXT_PANEL_HEIGHT
SCREW_PANEL_HEIGHT   = 260                                                      # assignem SCREW_PANEL_HEIGHT

COLOR_RESETTING = "#0a84ff"                                                     # assignem COLOR_RESETTING
COLOR_OK        = "#30d158"                                                     # assignem COLOR_OK
COLOR_ERROR     = "#ff3b30"                                                     # assignem COLOR_ERROR


def _load_viewer_cobot():                                                       # definim la funció _load_viewer_cobot()
    root = os.path.dirname(os.path.dirname(os.path.dirname(                     # assignem root
        os.path.dirname(os.path.abspath(__file__)))))
    path = os.path.join(root, "scripts", "viewer_cobot.py")                     # path ← resultat de join()

    if "viewer_cobot" in sys.modules:                                           # si "viewer_cobot" in sys.modules
        return sys.modules["viewer_cobot"]                                      # retornem: sys.modules["viewer_cobot"]

    spec = importlib.util.spec_from_file_location("viewer_cobot", path)         # spec ← resultat de spec_from_file_location()
    module = importlib.util.module_from_spec(spec)                              # module ← resultat de module_from_spec()
    sys.modules["viewer_cobot"] = module                                        # sys.modules["viewer_cobot"] = module
    spec.loader.exec_module(module)                                             # executem exec_module()
    return module                                                               # retornem: module


class RobotWorkspace(QtWidgets.QWidget):                                        # definim la classe RobotWorkspace

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")
        self.setSizePolicy(                                                     # executem setSizePolicy()
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )

        self._inner            = None                                           # assignem self._inner
        self._j6_panel         = None                                           # assignem self._j6_panel
        self._j6_container     = None                                           # assignem self._j6_container
        self._context_panel    = None                                           # assignem self._context_panel
        self._mode_selector    = None                                           # assignem self._mode_selector
        self._current_mode     = "ik6"                                          # assignem self._current_mode
        self._reg_screws_m     = []                                             # assignem self._reg_screws_m
        self._active_screw_idx = None                                           # assignem self._active_screw_idx
        self._vector_actors    = []                                             # assignem self._vector_actors
        self._spin_timer       = QtCore.QTimer(self)                            # self._spin_timer ← resultat de QTimer()
        self._spin_timer.timeout.connect(self._spin_j6_tick)                    # executem connect()
        self._spin_velocity_rad_s = 0.0                                         # assignem self._spin_velocity_rad_s
        self._j6_angle         = 0.0                                            # assignem self._j6_angle
        self._tick_count       = 0                                              # assignem self._tick_count
        self._mirror_was_on_before_spin = False                                 # assignem self._mirror_was_on_before_spin

        # Broca del J6 (CURRENT_MODE només mentre gira). Es crea quan tenim viewer.
        self._drill            = None                                           # j6_drill.J6Drill

        # ---- Hand-guide natiu (QTimer, no subprocés) --------------------
        self._hg_controller = None                                              # assignem self._hg_controller

        self._hg_timer = QtCore.QTimer(self)                                    # self._hg_timer ← resultat de QTimer()
        self._hg_timer.timeout.connect(self._hg_tick)                           # executem connect()

        root_layout = QtWidgets.QHBoxLayout(self)                               # root_layout ← resultat de QHBoxLayout()
        root_layout.setContentsMargins(10, 10, 10, 10)                          # executem setContentsMargins()
        root_layout.setSpacing(10)                                              # executem setSpacing()

        try:                                                                    # intentem el bloc següent
            mod = _load_viewer_cobot()                                          # mod ← resultat de _load_viewer_cobot()
            self._inner = mod.ViewerDLSPro()                                    # self._inner ← resultat de ViewerDLSPro()
            self._inner.setWindowFlags(QtCore.Qt.Widget)                        # executem setWindowFlags()

            central = self._inner.centralWidget()                               # central ← resultat de centralWidget()
            if central is None:                                                 # si central is None
                raise RuntimeError("ViewerDLSPro no té centralWidget")          # llencem excepció: RuntimeError("ViewerDLSPro no té cent...

            hide_redundant_widgets(self._inner)                                 # executem hide_redundant_widgets()

            left_col = QtWidgets.QWidget()                                      # left_col ← resultat de QWidget()
            left_col.setStyleSheet("background: transparent;")                  # executem setStyleSheet()
            lc = QtWidgets.QVBoxLayout(left_col)                                # lc ← resultat de QVBoxLayout()
            lc.setContentsMargins(0, 0, 0, 0)                                   # executem setContentsMargins()
            lc.setSpacing(8)                                                    # executem setSpacing()

            self._mode_selector = ModeSelector(initial_mode=self._current_mode)  # self._mode_selector ← resultat de ModeSelector()
            self._mode_selector.modeChanged.connect(self._on_mode_changed)      # executem connect()
            lc.addWidget(self._mode_selector)                                   # executem addWidget()

            # Vista 3D a dalt + gràfic d'intensitats a baix, amb divisor arrossegable
            self._view_split = QtWidgets.QSplitter(QtCore.Qt.Vertical)          # self._view_split ← resultat de QSplitter()
            self._view_split.setChildrenCollapsible(False)                      # cap dels dos es pot col·lapsar a 0
            self._view_split.setHandleWidth(8)                                  # executem setHandleWidth()
            self._view_split.setStyleSheet(f"""
                QSplitter::handle {{ background: transparent; }}
                QSplitter::handle:hover {{ background: {C_BORDER}; border-radius: 3px; }}
            """)
            self._view_split.addWidget(central)                                 # executem addWidget()

            self._currents_panel = CurrentsPanel(self._inner)                   # self._currents_panel ← resultat de CurrentsPanel()
            self._currents_panel.hide()                                         # amagat per defecte (botó "Intensitats")
            self._view_split.addWidget(self._currents_panel)                    # executem addWidget()
            self._view_split.setStretchFactor(0, 1)                             # el 3D s'emporta l'espai sobrant
            self._view_split.setStretchFactor(1, 0)                             # executem setStretchFactor()
            lc.addWidget(self._view_split, 1)                                   # executem addWidget()

            self._action_bar = ActionBar(self._inner)                           # self._action_bar ← resultat de ActionBar()
            self._action_bar.currentsToggled.connect(self._on_currents_toggled)  # executem connect()
            lc.addWidget(self._action_bar)                                      # executem addWidget()

            root_layout.addWidget(left_col, 1)                                  # executem addWidget()

            right = QtWidgets.QWidget()                                         # right ← resultat de QWidget()
            right.setFixedWidth(340)                                            # executem setFixedWidth()
            right.setStyleSheet("background: transparent;")                     # executem setStyleSheet()
            right_layout = QtWidgets.QVBoxLayout(right)                         # right_layout ← resultat de QVBoxLayout()
            right_layout.setContentsMargins(0, 0, 0, 0)                         # executem setContentsMargins()
            right_layout.setSpacing(10)                                         # executem setSpacing()

            self._context_panel = ContextPanel(self._inner)                     # self._context_panel ← resultat de ContextPanel()
            self._context_panel.setFixedHeight(CONTEXT_PANEL_HEIGHT)            # executem setFixedHeight()
            right_layout.addWidget(self._context_panel)                         # executem addWidget()

            self._screw_panel = ScrewPanel()                                    # self._screw_panel ← resultat de ScrewPanel()
            self._screw_panel.setFixedHeight(SCREW_PANEL_HEIGHT)                # executem setFixedHeight()
            self._screw_panel.showVectorRequested.connect(self._on_show_vector)  # executem connect()
            self._screw_panel.goApproachRequested.connect(self._on_go_approach)  # executem connect()
            self._screw_panel.selectionChanged.connect(self._on_screw_selection)  # executem connect()
            self._screw_panel.trajectoryDepthChanged.connect(self._on_trajectory_depth)  # slider profunditat
            right_layout.addWidget(self._screw_panel)                           # executem addWidget()

            right_layout.addStretch(1)                                          # executem addStretch()

            root_layout.addWidget(right)                                        # executem addWidget()

            j6_container = QtWidgets.QFrame()                                   # j6_container ← resultat de QFrame()
            j6_container.setStyleSheet(f"""
                QFrame {{
                    background-color: {C_PANEL};
                    border: 1px solid {C_BORDER};
                    border-radius: 12px;
                }}
            """)
            j6_container.setMinimumHeight(220)                                  # executem setMinimumHeight()

            j6_layout = QtWidgets.QVBoxLayout(j6_container)                     # j6_layout ← resultat de QVBoxLayout()
            j6_layout.setContentsMargins(14, 12, 14, 12)                        # executem setContentsMargins()
            j6_layout.setSpacing(6)                                             # executem setSpacing()

            self._j6_panel = J6ControlPanel(j6_container)                       # self._j6_panel ← resultat de J6ControlPanel()
            self._j6_panel.velocityRequested.connect(self._on_j6_velocity)      # executem connect()
            self._j6_panel.torqueRequested.connect(self._on_j6_torque)          # executem connect()
            self._j6_panel.stopRequested.connect(self._on_j6_stop)              # executem connect()
            j6_layout.addWidget(self._j6_panel)                                 # executem addWidget()

            # Controlador de la broca: l'slider del panell és la font de veritat del corrent
            self._drill = J6Drill(                                              # self._drill ← resultat de J6Drill()
                self._inner,
                setpoint_fn=lambda: self._j6_panel.slider_tor.value(),
                parent=self,
            )

            # STOP de la barra d'accions: abans d'aturar tots els motors, la
            # broca s'atura i el J6 es fixa on és (així en reconnectar no
            # desfà voltes). Treiem les connexions originals (n'hi havia dues).
            self._action_bar.btn_stop.clicked.disconnect()                      # executem disconnect()
            self._action_bar.btn_stop.clicked.connect(self.emergency_stop)      # executem connect()

            self._j6_container = j6_container                                   # assignem self._j6_container

            self._install_broca_in_ik5_panel()                                  # executem _install_broca_in_ik5_panel()
            self._install_shortcuts()                                           # executem _install_shortcuts()

            # Aplicar mode inicial
            self._activate_mode(self._current_mode, first_time=True)            # assignem self._activate_mode(self._current_mode, first_time

            # El diàleg modal es crearà sota demanda al primer showEvent.

        except Exception as e:                                                  # capturem excepció: Exception as e
            self._show_error(root_layout, e)                                    # executem _show_error()

    def _install_broca_in_ik5_panel(self):                                      # definim la funció _install_broca_in_ik5_panel()
        ik5_panel = self._context_panel._panels.get("ik5")                      # ik5_panel ← resultat de get()
        if ik5_panel is None:                                                   # si ik5_panel is None
            print("[layout] no s'ha trobat IK5Panel al context_panel")          # executem print()
            return                                                              # retornem sense valor
        layout = ik5_panel.layout()                                             # layout ← resultat de layout()
        if layout is None:                                                      # si layout is None
            return                                                              # retornem sense valor
        insert_at = layout.count()                                              # insert_at ← resultat de count()
        for i in range(layout.count() - 1, -1, -1):                             # iterem: i in range(layout.count() - 1, -1, -1)
            item = layout.itemAt(i)                                             # item ← resultat de itemAt()
            if item and item.spacerItem() is not None:                          # si item and item.spacerItem() is not None
                insert_at = i                                                   # assignem insert_at
                break                                                           # sortim del bucle
        layout.insertWidget(insert_at, self._j6_container)                      # executem insertWidget()
        print("[layout] broca instal·lada dins del IK5Panel")                   # executem print()

    def _show_error(self, layout, exc):                                         # definim la funció _show_error()
        w = QtWidgets.QWidget()                                                 # w ← resultat de QWidget()
        v = QtWidgets.QVBoxLayout(w)                                            # v ← resultat de QVBoxLayout()
        v.setAlignment(QtCore.Qt.AlignCenter)                                   # executem setAlignment()
        v.setSpacing(16)                                                        # executem setSpacing()

        icon = QtWidgets.QLabel("⚠")                                            # icon ← resultat de QLabel()
        icon.setAlignment(QtCore.Qt.AlignCenter)                                # executem setAlignment()
        icon.setStyleSheet(f"color: {C_WARN}; font-size: 48px;")
        v.addWidget(icon)                                                       # executem addWidget()

        title = QtWidgets.QLabel("No s'ha pogut carregar el workspace del robot")  # title ← resultat de QLabel()
        title.setAlignment(QtCore.Qt.AlignCenter)                               # executem setAlignment()
        title.setStyleSheet("color: white; font-size: 15px; font-weight: 600;")  # executem setStyleSheet()
        v.addWidget(title)                                                      # executem addWidget()

        msg = QtWidgets.QLabel(f"{type(exc).__name__}: {exc}")
        msg.setAlignment(QtCore.Qt.AlignCenter)                                 # executem setAlignment()
        msg.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: 12px; padding: 20px;")
        msg.setWordWrap(True)                                                   # executem setWordWrap()
        v.addWidget(msg)                                                        # executem addWidget()

        layout.addWidget(w)                                                     # executem addWidget()

    def _install_shortcuts(self):                                               # definim la funció _install_shortcuts()
        for i, m_id in enumerate(["fk", "ik6", "ik5", "mando", "handguide"], start=1):  # iterem: i, m_id in enumerate(["fk", "ik6", "i...
            sc = QtWidgets.QShortcut(QtGui.QKeySequence(str(i)), self)          # sc ← resultat de QShortcut()
            sc.activated.connect(lambda mid=m_id: self._mode_selector.set_mode(mid))  # sc.activated.connect(lambda mid ← resultat de set_mode()
        sc_i = QtWidgets.QShortcut(QtGui.QKeySequence("I"), self)               # tecla I → mostra/amaga intensitats
        sc_i.activated.connect(self._action_bar.btn_currents.toggle)            # executem connect()

    def _on_currents_toggled(self, on):                                         # definim la funció _on_currents_toggled()
        """Mostra/amaga el gràfic d'intensitats sota la vista 3D."""
        self._currents_panel.setVisible(on)                                     # executem setVisible()
        if on:                                                                  # en obrir: ~380 px per als 6 gràfics
            total = sum(self._view_split.sizes())                               # total ← resultat de sum()
            h = min(380, max(300, int(total * 0.42)))                           # h ← resultat de min()
            self._view_split.setSizes([max(total - h, 1), h])                   # executem setSizes()

    def get_viewer(self):                                                       # definim la funció get_viewer()
        return self._inner                                                      # retornem: self._inner

    def is_robot_connected(self):                                               # definim la funció is_robot_connected()
        if self._inner is None:                                                 # si self._inner is None
            return False                                                        # retornem: False
        mc = getattr(self._inner, "motor_controller", None)                     # mc ← resultat de getattr()
        if mc is None:                                                          # si mc is None
            return False                                                        # retornem: False
        try:                                                                    # intentem el bloc següent
            attr = getattr(mc, "_connected", None)                              # attr ← resultat de getattr()
            if attr is not None:                                                # si attr is not None
                return bool(attr)                                               # retornem: bool(attr)
            return bool(mc.is_connected())                                      # retornem: bool(mc.is_connected())
        except Exception:                                                       # capturem excepció: Exception
            return False                                                        # retornem: False

    def _btn_connect_set_style(self, color, text):                              # definim la funció _btn_connect_set_style()
        btn = getattr(self._inner, "btn_connect", None)                         # btn ← resultat de getattr()
        if btn is None:                                                         # si btn is None
            return                                                              # retornem sense valor
        btn.setText(text)                                                       # executem setText()
        btn.setStyleSheet(                                                      # executem setStyleSheet()
            f"background-color: {color}; color: white; font-weight: bold;"
        )
        btn.repaint()                                                           # executem repaint()
        QtWidgets.QApplication.processEvents()                                  # executem processEvents()

    def _reset_and_reconnect_motors(self):                                      # definim la funció _reset_and_reconnect_motors()
        """Reset lògic sense tocar el port sèrie ni matar el daemon."""         # """Reset lògic sense tocar el port sèrie ni matar el daem...
        if self._inner is None:                                                 # si self._inner is None
            return False                                                        # retornem: False

        mc = getattr(self._inner, "motor_controller", None)                     # mc ← resultat de getattr()
        if mc is None:                                                          # si mc is None
            return False                                                        # retornem: False

        self._btn_connect_set_style(COLOR_RESETTING, "Resetting motors...")     # executem _btn_connect_set_style()
        print("[motors] reset lògic")                                           # executem print()

        self._safe_set_mirror(False)                                            # executem _safe_set_mirror()

        if self._spin_timer.isActive():                                         # si self._spin_timer.isActive()
            self._spin_timer.stop()                                             # executem stop()
            self._spin_velocity_rad_s = 0.0                                     # assignem self._spin_velocity_rad_s

        if self._j6_panel is not None:                                          # UI de la broca a "aturat"
            self._j6_panel.set_stopped()                                        # executem set_stopped()
        if self._drill is not None:                                             # broca aturada i J6 a POSITION abans del reset
            self._drill.disarm()                                                # executem disarm()

        if not self.is_robot_connected():                                     # si not self.is_robot_connected()
            try:                                                                # intentem el bloc següent
                mc.connect()                                                    # executem connect()
                mc.enable_position_control_at(self._inner.q)                    # POSITION amb el target enganxat
                self._btn_connect_set_style(COLOR_OK, "Motors connectats")      # executem _btn_connect_set_style()
                btn = getattr(self._inner, "btn_connect", None)                 # btn ← resultat de getattr()
                if btn is not None:                                             # si btn is not None
                    btn.setChecked(True)                                        # executem setChecked()
                btn_mirror = getattr(self._inner, "btn_mirror", None)           # btn_mirror ← resultat de getattr()
                if btn_mirror is not None:                                      # si btn_mirror is not None
                    btn_mirror.setEnabled(True)                                 # executem setEnabled()

                if self._current_mode in ("fk", "ik6", "ik5"):                  # si self._current_mode in ("fk", "ik6", "...
                    self._safe_set_mirror(True)                                 # executem _safe_set_mirror()
                    print("[motors] mirror activat automàticament")             # executem print()

                print("[motors] connectats de nou (daemon nou)")                # executem print()
                return True                                                     # retornem: True
            except Exception as e:                                              # capturem excepció: Exception as e
                self._handle_reset_failure(e)                                   # executem _handle_reset_failure()
                return False                                                    # retornem: False

        try:                                                                    # intentem el bloc següent
            if hasattr(mc, "bus") and mc.bus is not None:                       # si hasattr(mc, "bus") and mc.bus is not ...
                mc.bus.reset_all()                                              # executem reset_all()

            # El firmware necessita temps després d'un RESET per acceptar el
            # canvi de mode (≥0.2 s, com fa el hand-guide).
            time.sleep(0.3)                                                     # executem sleep()

            mc.enable_position_control_at(self._inner.q)                        # POSITION amb el target enganxat

            self._btn_connect_set_style(COLOR_OK, "Motors connectats")          # executem _btn_connect_set_style()
            btn = getattr(self._inner, "btn_connect", None)                     # btn ← resultat de getattr()
            if btn is not None:                                                 # si btn is not None
                btn.setChecked(True)                                            # executem setChecked()
            btn_mirror = getattr(self._inner, "btn_mirror", None)               # btn_mirror ← resultat de getattr()
            if btn_mirror is not None:                                          # si btn_mirror is not None
                btn_mirror.setEnabled(True)                                     # executem setEnabled()

            if self._current_mode in ("fk", "ik6", "ik5"):                      # si self._current_mode in ("fk", "ik6", "...
                self._safe_set_mirror(True)                                     # executem _safe_set_mirror()
                print("[motors] mirror activat automàticament")                 # executem print()

            print("[motors] reset OK · motors operatius")                       # executem print()
            return True                                                         # retornem: True

        except Exception as e:                                                  # capturem excepció: Exception as e
            self._handle_reset_failure(e)                                       # executem _handle_reset_failure()
            return False                                                        # retornem: False

    def _handle_reset_failure(self, exception):                                 # definim la funció _handle_reset_failure()
        self._btn_connect_set_style(COLOR_ERROR, "Error de reset")              # executem _btn_connect_set_style()
        btn = getattr(self._inner, "btn_connect", None)                         # btn ← resultat de getattr()
        if btn is not None:                                                     # si btn is not None
            btn.setChecked(False)                                               # executem setChecked()
        btn_mirror = getattr(self._inner, "btn_mirror", None)                   # btn_mirror ← resultat de getattr()
        if btn_mirror is not None:                                              # si btn_mirror is not None
            btn_mirror.setEnabled(False)                                        # executem setEnabled()

        print(f"[motors] FALLADA del reset: {exception}")
        QtWidgets.QMessageBox.warning(                                          # executem warning()
            self, "Motors",
            f"No s'ha pogut resetejar els motors.\n\n"
            f"Error: {type(exception).__name__}: {exception}\n\n"
            f"Comprova que el cable USB estigui connectat.\n"
            f"Torna a canviar de mode per reintentar."
        )

    def _on_mode_changed(self, mode_id):                                        # definim la funció _on_mode_changed()
        self._activate_mode(mode_id)                                            # executem _activate_mode()

    def _activate_mode(self, mode_id, first_time=False):                        # definim la funció _activate_mode()
        """
        Aplica un mode nou fent primer el shutdown del mode anterior.

        NOTA: el sync del target ↔ TCP actual ja NO es fa aquí.
        Ara viu al coordinator (enter_ik6/enter_ik5), que crida
        viewer.sync_target_to_current() dins de la mateixa transacció
        de canvi de mode. Això elimina els salts en tocar RPY.
        """
        old = self._current_mode                                                # assignem old
        print(f"[mode] {old} → {mode_id}")

        # 0. Un Home o un "Anar a approach" lents en curs s'aturen aquí
        if self._inner is not None and hasattr(self._inner, "cancel_motion"):   # si el viewer ho suporta
            self._inner.cancel_motion()                                         # executem cancel_motion()

        # 1. Broca / J6:
        #    · si tornem a clicar IK5 estant en IK5: només aturem la broca
        #      (el J6 es queda en RESET, armat)
        #    · si sortim d'IK5: parada completa i el J6 torna a POSITION a la
        #      posició guardada (disarm). A partir d'aquí és un motor normal.
        if self._j6_panel is not None and self._j6_panel.is_running():          # UI a "aturat" sense emetre senyals
            self._j6_panel.set_stopped()                                        # executem set_stopped()
        if self._drill is not None:                                             # si tenim broca
            if old == "ik5" and mode_id == "ik5":                               # mateix mode
                self._drill.finish_now()                                        # executem finish_now()
            else:                                                               # sortim d'IK5 (o no hi érem)
                self._drill.disarm()                                            # executem disarm()
        if self._spin_timer.isActive():                                         # spin sintètic antic (si n'hi hagués)
            self._on_j6_stop()                                                  # executem _on_j6_stop()

        # 2. Aturar hand-guide natiu si estava viu
        if self._hg_controller is not None:                                     # si self._hg_controller is not None
            self._stop_handguide_native()                                       # executem _stop_handguide_native()

        # 3. Reset net del viewer intern (mando control OFF, mirror OFF)
        if not first_time:                                                      # si not first_time
            try:                                                                # intentem el bloc següent
                enter_idle(self._inner)                                         # executem enter_idle()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[mode] error a enter_idle: {e}")

        # 4. RESET DE MOTORS — només quan cal
        # =============================================================
        # SOFT_MODES són els modes que comparteixen POSITION_MODE + streaming
        # de q via send_q. Canviar entre ells NO requereix reset del daemon.
        #
        # Handguide també entra al conjunt PER LA SORTIDA (perquè el
        # _stop_handguide_native() ja s'encarrega de re-configurar els
        # motors a POSITION_MODE net). En canvi, per l'ENTRADA a handguide,
        # el controller ja fa el seu propi RESET al start(), així que
        # tampoc no cal reset previ del daemon.
        # =============================================================
        SOFT_MODES = {"fk", "ik6", "ik5", "mando", "handguide"}                 # assignem SOFT_MODES
        needs_reset = (                                                         # assignem needs_reset
            not first_time
            and not (old in SOFT_MODES and mode_id in SOFT_MODES)
        )

        if needs_reset:                                                         # si needs_reset
            ok = self._reset_and_reconnect_motors()                             # ok ← resultat de _reset_and_reconnect_motors()
            if not ok:                                                          # si not ok
                print(f"[mode] mode {mode_id} NO activat per fallada de motors")
                return                                                          # retornem sense valor
        else:                                                                   # en cas contrari
            print("[mode] transició suau, sense reset")
            # Si venim d'un STOP (motors en RESET), cal reactivar POSITION_MODE
            # abans d'activar mirror.
            #
            # enable_position_control_at: el target (q actual) va just abans i
            # just després del mode_selection, així cap motor va cap a un
            # target vell que el firmware tingués guardat. El J6, si encara
            # és armat en IK5, és fora del send_q i no es toca.
            if (mode_id in ("fk", "ik6", "ik5", "mando")                        # si (mode_id in ("fk", "ik6", "ik5", "man...
                    and self.is_robot_connected()):
                mc = getattr(self._inner, "motor_controller", None)             # mc ← resultat de getattr()
                if mode_id != "ik5":                                            # fora d'IK5 el J6 ha de ser al send_q
                    ensure_j6_position_hold(self._inner)                      # seguretat (normalment no fa res)
                if mc is not None:                                              # si mc is not None
                    try:                                                        # intentem el bloc següent
                        mc.enable_position_control_at(self._inner.q)            # executem enable_position_control_at()
                    except Exception as e:                                      # capturem excepció: Exception as e
                        print(f"[mode] error re-activant POSITION_MODE: {e}")

                if not self._safe_is_mirror_on():                             # si not self._safe_is_mirror_on()
                    self._safe_set_mirror(True)                                 # executem _safe_set_mirror()
                    print("[mode] mirror activat automàticament (transició suau)")  # executem print()

        # 5. Aplicar el mode nou
        # =============================================================
        # MODE_ENTER[ik6]/MODE_ENTER[ik5] criden internament
        # viewer.sync_target_to_current() → sliders RPY sense salts.
        # MODE_ENTER[mando] intenta connectar el mando + auto Set Zero
        # + activar control.
        # handguide arrenca el QTimer + controller natiu.
        # =============================================================
        self._current_mode = mode_id                                            # assignem self._current_mode

        if mode_id == "handguide":                                              # si mode_id == "handguide"
            # PRIMER: fixem viewer.mode = "FK" perquè el bucle 100Hz
            # NO calculi IK ni modifiqui self.q (el hand-guide és qui
            # actualitza self.q via callback des dels motors reals).
            # Això ho fa enter_handguide() del coordinator.
            enter_fn = MODE_ENTER.get("handguide")                              # enter_fn ← resultat de get()
            if enter_fn is not None:                                            # si enter_fn is not None
                try:                                                            # intentem el bloc següent
                    enter_fn(self._inner)                                       # executem enter_fn()
                except Exception as e:                                          # capturem excepció: Exception as e
                    print(f"[mode] error entrant a handguide: {e}")

            # SEGON: arrenca el QTimer + controller natiu.
            ok = self._start_handguide_native()                                 # ok ← resultat de _start_handguide_native()
            if not ok:                                                          # si not ok
                self._current_mode = old                                        # assignem self._current_mode
                if self._mode_selector is not None:                             # si self._mode_selector is not None
                    self._mode_selector.set_mode(old)                           # executem set_mode()
                return                                                          # retornem sense valor
        else:                                                                   # en cas contrari
            enter_fn = MODE_ENTER.get(mode_id)                                  # enter_fn ← resultat de get()
            if enter_fn is not None:                                            # si enter_fn is not None
                try:                                                            # intentem el bloc següent
                    enter_fn(self._inner)                                       # executem enter_fn()
                except Exception as e:                                          # capturem excepció: Exception as e
                    print(f"[mode] error entrant a {mode_id}: {e}")


        # 5b. IK5: el J6 queda guardat i en RESET (blau) fins que es premi Start
        if mode_id == "ik5" and self._drill is not None:                        # si mode_id == "ik5"
            try:                                                                # intentem el bloc següent
                self._drill.arm()                                               # executem arm()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[mode] error armant la broca: {e}")

        # 6. Si hem entrat a MANDO però no s'ha pogut connectar, avisem
        if mode_id == "mando":                                                  # si mode_id == "mando"
            mando_ok = bool(getattr(self._inner, "mando_active", False))        # mando_ok ← resultat de bool()
            if not mando_ok:                                                    # si not mando_ok
                QtWidgets.QMessageBox.warning(                                  # executem warning()
                    self, "Mando 6DOF",
                    "No s'ha pogut connectar amb el mando al port COM5.\n\n"
                    "Comprova que està endollat i que cap altre programa\n"
                    "no està usant el port. Tornant al mode anterior."
                )
                self._current_mode = old                                        # assignem self._current_mode
                if self._mode_selector is not None:                             # si self._mode_selector is not None
                    self._mode_selector.set_mode(old)                           # executem set_mode()
                return                                                          # retornem sense valor

        # 7. Actualitzar panell contextual dret
        if self._context_panel is not None:                                     # si self._context_panel is not None
            self._context_panel.set_mode(mode_id)                               # executem set_mode()

        # 8. Sincronitzar habilitat de la broca (dins del IK5Panel)
        self._sync_j6_availability()                                            # executem _sync_j6_availability()

        # 9. Visibilitat de l'slider de trajectòria (només IK5)
        self._apply_trajectory_slider_visibility()                              # executem _apply_trajectory_slider_visibility()

    def _sync_j6_availability(self):                                            # definim la funció _sync_j6_availability()
        can_use = allow_broca(self._current_mode)                               # can_use ← resultat de allow_broca()
        if self._j6_panel:                                                      # si self._j6_panel
            self._j6_panel.setEnabled(can_use)                                  # executem setEnabled()

    def _on_j6_velocity(self, current_mA, velocity_counts):                     # definim la funció _on_j6_velocity()
        if not allow_broca(self._current_mode):                                 # si not allow_broca(self._current_mode)
            self._j6_panel._on_stop()                                           # executem _on_stop()
            return                                                              # retornem sense valor

        if not self.is_robot_connected():                                       # si not self.is_robot_connected()
            QtWidgets.QMessageBox.warning(                                      # executem warning()
                self, "Robot",
                "El robot no està connectat. Connecta el port sèrie primer."
            )
            self._j6_panel._on_stop()                                           # executem _on_stop()
            return                                                              # retornem sense valor

        rpm = float(velocity_counts)                                            # rpm ← resultat de float()
        self._spin_velocity_rad_s = rpm * 2.0 * np.pi / 60.0                    # assignem self._spin_velocity_rad_s

        if not self._spin_timer.isActive():                                     # si not self._spin_timer.isActive()
            self._j6_angle = float(self._inner.q[5])                            # self._j6_angle ← resultat de float()

            self._mirror_was_on_before_spin = self._safe_is_mirror_on()         # self._mirror_was_on_before_spin ← resultat de _safe_is_mirror_on()
            if self._mirror_was_on_before_spin:                                 # si self._mirror_was_on_before_spin
                print("[J6] apagant mirror del viewer per evitar conflicte")    # executem print()
                self._safe_set_mirror(False)                                    # executem _safe_set_mirror()

            print(f"[J6] START · RPM={rpm:.1f} · rad/s={self._spin_velocity_rad_s:.3f}")
            self._spin_timer.start(int(1000 / J6_SPIN_HZ))                      # executem start()
            self._tick_count = 0                                                # assignem self._tick_count

    def _on_j6_torque(self, current_mA):                                        # definim la funció _on_j6_torque()
        """
        Start de la broca (o canvi de corrent si ja gira).
        Tota la lògica del motor viu a modes/j6_drill.py (J6Drill).
        """
        if self._drill is not None and self._drill.state == J6Drill.RUNNING:    # ja gira: només canvi de corrent
            self._drill.set_current(current_mA)                                 # executem set_current()
            return                                                              # retornem sense valor

        if not allow_broca(self._current_mode):                                 # si not allow_broca(self._current_mode)
            self._j6_panel.set_stopped()                                        # executem set_stopped()
            return                                                              # retornem sense valor

        if not self.is_robot_connected():                                       # si not self.is_robot_connected()
            QtWidgets.QMessageBox.warning(                                      # executem warning()
                self, "Robot",
                "El robot no està connectat. Connecta el port sèrie primer."
            )
            self._j6_panel.set_stopped()                                        # executem set_stopped()
            return                                                              # retornem sense valor

        if self._drill is None or not self._drill.start(current_mA):            # sense daemon real
            print("[J6] no s'ha pogut engegar la broca (cal el daemon real)")   # executem print()
            self._j6_panel.set_stopped()                                        # executem set_stopped()

    def _on_j6_stop(self):                                                      # definim la funció _on_j6_stop()
        # ---- Cas 1: estàvem en spin sintètic (VELOCITY mode del panell) ----
        was_spin = self._spin_timer.isActive()                                  # was_spin
        self._spin_timer.stop()                                                 # executem stop()
        self._spin_velocity_rad_s = 0.0                                         # assignem self._spin_velocity_rad_s

        if was_spin and self._inner is not None:                                # si estàvem fent spin sintètic
            try:                                                                # intentem el bloc següent
                angle_wrapped = float(np.remainder(                             # assignem angle_wrapped
                    self._j6_angle + np.pi, 2.0 * np.pi) - np.pi)

                self._inner.q[5] = angle_wrapped                                # assignem self._inner.q[5]
                self._j6_angle   = angle_wrapped                                # assignem self._j6_angle

                print(f"[J6] STOP · viewer.q[5] re-sincronitzat a "
                      f"{angle_wrapped:+.3f} rad "
                      f"({np.degrees(angle_wrapped):+.1f}°)")
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[J6 stop] error sincronitzant: {e}")

        if self._mirror_was_on_before_spin:                                     # si el mirror estava on abans del spin
            print("[J6] restaurant mirror del viewer")                          # executem print()
            self._safe_set_mirror(True)                                         # executem _safe_set_mirror()
            self._mirror_was_on_before_spin = False                             # assignem self._mirror_was_on_before_spin

        # ---- Cas 2: broca en TORQUE → parada suau (no bloqueja la UI) ----
        # Rampa a 0 mA, espera que el motor estigui quiet, guarda la posició
        # i RESET (blau): preparat per a un altre Start. Mentrestant J1-J5
        # continuen controlats per l'IK5 amb normalitat.
        if self._drill is not None:                                             # si tenim broca
            self._drill.stop()                                                  # executem stop()

    def emergency_stop(self):                                                   # definim la funció emergency_stop()
        """
        STOP de la barra d'accions i E-STOP de la capçalera.
          1. Broca: 0 mA i J6 fixat on és, a l'instant (el firmware guarda
             aquesta posició com a target → en reconnectar no desfà voltes).
          2. RESET de tots els motors (viewer._on_stop: blau, sense parell).
        """
        print("[STOP] aturada de tots els motors")                              # executem print()
        if self._j6_panel is not None:                                          # si tenim panell
            self._j6_panel.set_stopped()                                        # executem set_stopped()
        self._spin_timer.stop()                                                 # executem stop()
        if self._drill is not None:                                             # si tenim broca
            self._drill.abort()                                                 # executem abort()
        if self._inner is not None and hasattr(self._inner, "_on_stop"):        # RESET global
            try:                                                                # intentem el bloc següent
                self._inner._on_stop()                                          # executem _on_stop()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[STOP] error: {e}")

    def _spin_j6_tick(self):                                                    # definim la funció _spin_j6_tick()
        if not self.is_robot_connected():                                       # si not self.is_robot_connected()
            self._on_j6_stop()                                                  # executem _on_j6_stop()
            return                                                              # retornem sense valor

        dt = 1.0 / J6_SPIN_HZ                                                   # assignem dt
        dq = self._spin_velocity_rad_s * dt                                     # assignem dq
        self._j6_angle += dq                                                    # sumem a self._j6_angle

        try:                                                                    # intentem el bloc següent
            viewer = self._inner                                                # assignem viewer
            q_send = viewer.q.copy()                                            # q_send ← resultat de copy()
            q_send[5] = self._j6_angle                                          # assignem q_send[5]
            viewer.motor_controller.send_q(q_send)                              # executem send_q()

            self._tick_count += 1                                               # sumem a self._tick_count
            if self._tick_count % 40 == 0:                                      # si self._tick_count % 40 == 0
                revs = self._j6_angle / (2.0 * np.pi)                           # revs ← resultat de _j6_angle /()
                print(f"[J6 #{self._tick_count}] {revs:+.2f} voltes")
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[J6 tick] error: {e}")
            self._on_j6_stop()                                                  # executem _on_j6_stop()

    def _safe_is_mirror_on(self):                                               # definim la funció _safe_is_mirror_on()
        try:                                                                    # intentem el bloc següent
            return bool(getattr(self._inner, "mirror_active", False))           # retornem: bool(getattr(self._inner, "mirror_act...
        except Exception:                                                       # capturem excepció: Exception
            return False                                                        # retornem: False

    def _safe_set_mirror(self, on):                                             # definim la funció _safe_set_mirror()
        try:                                                                    # intentem el bloc següent
            btn = getattr(self._inner, "btn_mirror", None)                      # btn ← resultat de getattr()
            if btn is None:                                                     # si btn is None
                return                                                          # retornem sense valor
            if btn.isChecked() != on:                                           # si btn.isChecked() != on
                btn.setChecked(on)                                              # executem setChecked()
                if hasattr(self._inner, "_on_mirror_toggle"):                   # si hasattr(self._inner, "_on_mirror_togg...
                    self._inner._on_mirror_toggle()                             # executem _on_mirror_toggle()
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[mirror] {e}")

    # ==========================================================================
    # HAND-GUIDE NATIU (dins del procés Qt, sense subprocés)
    # ==========================================================================
    def _start_handguide_native(self):                                          # definim la funció _start_handguide_native()
        """
        Inicia el mode hand-guide dins del procés Qt.

        Pasos:
          1. Comprova que els motors estan connectats.
          2. Apaga el mirror (imprescindible: mirror envia POSITION_MODE
             als motors, i el hand-guide els vol en CURRENT_MODE).
          3. Carrega els paràmetres dinàmics (gravetat + fricció).
          4. Crea el controller amb un callback que actualitza viewer.q
             a cada tic (així el robot virtual segueix el físic).
          5. Arrenca el QTimer a 50 Hz.
        """
        if not self.is_robot_connected():                                       # si not self.is_robot_connected()
            QtWidgets.QMessageBox.warning(                                      # executem warning()
                self, "Hand-Guide",
                "Els motors no estan connectats. Torna a un altre mode\n"
                "primer per connectar-los."
            )
            return False                                                        # retornem: False

        # PRIMER DE TOT: activar el flag perquè el bucle 100Hz del viewer
        # es curtcircuiti immediatament i no interfereixi amb el nostre
        # canvi de mode dels motors. Sense això, entre el moment que
        # apaguem mirror i el moment que arrenca el controller, el
        # viewer envia POSITION als motors i anul·la el CURRENT_MODE.
        self._inner._handguide_active = True                                    # assignem self._inner._handguide_active

        # Apaguem el mirror
        self._safe_set_mirror(False)                                            # executem _safe_set_mirror()

        # VERIFICACIÓ crítica: si el mirror encara està on, el bucle del
        # viewer enviarà POSITION als motors i anul·larà els nostres
        # CURRENT. Cal apagar-lo de veritat.
        deadline = time.monotonic() + 0.2                                       # assignem deadline
        while self._safe_is_mirror_on() and time.monotonic() < deadline:        # mentre self._safe_is_mirror_on() and time.mo...
            QtWidgets.QApplication.processEvents()                              # executem processEvents()
            time.sleep(0.01)                                                    # executem sleep()
        if self._safe_is_mirror_on():                                           # si self._safe_is_mirror_on()
            print("[handguide] ATENCIÓ: mirror no s'ha apagat!")                # executem print()
        else:                                                                   # en cas contrari
            print("[handguide] mirror apagat correctament")                     # executem print()

        # Carreguem paràmetres dinàmics (gravetat + fricció)
        try:                                                                    # intentem el bloc següent
            root = os.path.dirname(os.path.dirname(os.path.dirname(             # assignem root
                os.path.dirname(os.path.abspath(__file__)))))
            dyn_path = os.path.join(root, "config", "cobot.urdf")               # dyn_path ← resultat de join()
            dyn_params = load_dynamic_params(dyn_path)                          # dyn_params ← resultat de load_dynamic_params()
        except Exception as e:                                                  # capturem excepció: Exception as e
            QtWidgets.QMessageBox.warning(                                      # executem warning()
                self, "Hand-Guide",
                f"No s'han pogut carregar els paràmetres dinàmics del URDF:\n{e}"
            )
            self._inner._handguide_active = False                               # assignem self._inner._handguide_active
            return False                                                        # retornem: False

        # Callback per sincronitzar el robot virtual amb el físic
        def _on_q(q_rad):                                                       # definim la funció _on_q()
            try:                                                                # intentem el bloc següent
                # Actualitzem la q del viewer. El bucle 100 Hz del viewer
                # ja recalcularà FK i actualitzarà STLs, triads, etc.
                self._inner.q[:] = q_rad                                        # self._inner.q[:] = q_rad
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[handguide] on_q_updated: {e}")

        def _on_limit(j_idx):                                                   # definim la funció _on_limit()
            print(f"[handguide] límit assolit a j{j_idx+1}, atura't")

        def _on_error(msg):                                                     # definim la funció _on_error()
            print(f"[handguide] error: {msg}")

        # Creem i arrenquem el controller
        try:                                                                    # intentem el bloc següent
            self._hg_controller = HandGuideController(                          # assignem self._hg_controller
                motor_controller=self._inner.motor_controller,
                dyn_params=dyn_params,
                robot=self._inner.robot,
                on_q_updated=_on_q,
                on_limit_hit=_on_limit,
                on_error=_on_error,
            )
            self._hg_controller.start()                                         # executem start()
        except Exception as e:                                                  # capturem excepció: Exception as e
            QtWidgets.QMessageBox.warning(                                      # executem warning()
                self, "Hand-Guide",
                f"Error iniciant el controller:\n{type(e).__name__}: {e}"
            )
            self._hg_controller = None                                          # assignem self._hg_controller
            self._inner._handguide_active = False                               # assignem self._inner._handguide_active
            return False                                                        # retornem: False

        if not self._hg_controller.is_active():                                 # si not self._hg_controller.is_active()
            QtWidgets.QMessageBox.warning(                                      # executem warning()
                self, "Hand-Guide",
                "No s'ha pogut activar el controller.\n"
                "Comprova els logs de la consola."
            )
            self._hg_controller = None                                          # assignem self._hg_controller
            self._inner._handguide_active = False                               # assignem self._inner._handguide_active
            return False                                                        # retornem: False

        # DIAGNÒSTIC: comprova estats crítics abans d'arrencar el timer
        mirror_ok = not self._safe_is_mirror_on()                               # mirror_ok ← resultat de _safe_is_mirror_on()
        viewer_mode = getattr(self._inner, "mode", "?")                         # viewer_mode ← resultat de getattr()
        print("[handguide] estats abans d'arrencar timer:")
        print(f"    mirror off:   {mirror_ok}")
        print(f"    viewer.mode:  {viewer_mode}")
        print(f"    motors conn:  {self.is_robot_connected()}")

        # (Ja hem activat _handguide_active al principi)

        # Arrenca el QTimer (50 Hz per defecte)
        self._hg_timer.start(int(1000 / HANDGUIDE_HZ))                          # executem start()
        print(f"[handguide] mode natiu actiu ({HANDGUIDE_HZ} Hz)")
        return True                                                             # retornem: True

    def _stop_handguide_native(self):                                           # definim la funció _stop_handguide_native()
        """
        Atura el hand-guide de forma neta i deixa els motors llestos.

        Ordre crític:
          1. Aturar el timer perquè no arribi cap step més.
          2. Aturar el controller (envia currents=0 als joints hg).
          3. Desactivar el flag _handguide_active perquè el bucle 100Hz
             pugui tornar a interactuar.
          4. RESET GLOBAL del bus (mateix que el botó STOP fa). Això és
             el que garanteix que els motors passin per un estat "net"
             abans de reactivar POSITION_MODE.
          5. Cridar enable_position_control() per tornar-los a POSITION_MODE.
          6. Enviar send_q(q_actual) per donar-los un objectiu concret
             i que surtin del "blau" (RESET) al "verd" (actiu).
          7. Enviar send_q un parell de vegades més amb petites pauses,
             perquè els motors s'assegurin de rebre l'objectiu i comencin
             a fer feedback POSITION.
        """
        import time as _time                                                    # importem el mòdul time as _time

        # 1. Aturar timer
        if self._hg_timer.isActive():                                           # si self._hg_timer.isActive()
            self._hg_timer.stop()                                               # executem stop()

        # 2. Aturar controller (currents=0 als joints hg)
        if self._hg_controller is not None:                                     # si self._hg_controller is not None
            try:                                                                # intentem el bloc següent
                self._hg_controller.stop()                                      # executem stop()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[handguide] error a stop(): {e}")
            self._hg_controller = None                                          # assignem self._hg_controller

        # 3. Desmarca la flag
        if self._inner is not None:                                             # si self._inner is not None
            self._inner._handguide_active = False                               # assignem self._inner._handguide_active

        # 4-7. Reconfiguració neta
        if self._inner is None:                                                 # si self._inner is None
            return                                                              # retornem sense valor
        mc = getattr(self._inner, "motor_controller", None)                     # mc ← resultat de getattr()
        if mc is None or not mc.is_connected:                                   # si mc is None or not mc.is_connected
            return                                                              # retornem sense valor

        try:                                                                    # intentem el bloc següent
            # 4. RESET global del bus (mateix efecte que el botó STOP)
            mc.bus.reset_all()                                                  # executem reset_all()
            _time.sleep(0.15)                                                   # executem sleep()

            # 5. Tornar a POSITION_MODE amb el target = q actual enganxat al
            #    mode_selection (sense finestra cap a un target vell)
            q_actual = self._inner.q.copy()                                     # q_actual ← resultat de copy()
            mc.enable_position_control_at(q_actual)                             # executem enable_position_control_at()
            _time.sleep(0.1)                                                    # executem sleep()

            # 6-7. send_q amb la q actual, un parell de cops per
            # assegurar-nos que els motors rebi la comanda i surtin del "blau"
            for _ in range(3):                                                  # iterem: _ in range(3)
                try:                                                            # intentem el bloc següent
                    mc.send_q(q_actual)                                         # executem send_q()
                    _time.sleep(0.03)                                           # executem sleep()
                except Exception as e:                                          # capturem excepció: Exception as e
                    print(f"[handguide] error send_q: {e}")
                    break                                                       # sortim del bucle

            print("[handguide] motors reset+POSITION_MODE+send_q OK")           # executem print()

        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[handguide] error reconfigurant: {e}")



    def _hg_tick(self):                                                         # definim la funció _hg_tick()
        """Tic del QTimer del hand-guide: delega al controller."""              # """Tic del QTimer del hand-guide: delega al controller."""
        if self._hg_controller is None:                                         # si self._hg_controller is None
            return                                                              # retornem sense valor
        try:                                                                    # intentem el bloc següent
            self._hg_controller.step()                                          # executem step()
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[handguide] tick error: {e}")
            self._stop_handguide_native()                                       # executem _stop_handguide_native()

    def load_registered_scene(self, vertebra_mesh, screws, T_ct_to_robot,       # definim la funció load_registered_scene()
                              vertebres=None):
        """
        Dibuixa l'escena registrada al 3D del workspace Robot.

        Dues formes de crida:

          A) Una sola vèrtebra (compatibilitat):
             load_registered_scene(mesh, screws, T)
             → tots els cargols es transformen amb la mateixa T.

          B) Múltiples vèrtebres (nou):
             load_registered_scene(None, screws, None,
                                   vertebres={"T12": {"mesh": m1, "T": T1},
                                              "L1":  {"mesh": m2, "T": T2}, ...})
             → cada vèrtebra es dibuixa amb la seva T.
             → cada cargol s'associa automàticament a la vèrtebra del
               centroid més proper al punt d'entrada (p0 en coord CT),
               i es transforma amb la T d'aquella vèrtebra.
        """
        import numpy as np                                                      # importem el mòdul numpy as np
        import pyvista as pv                                                    # importem el mòdul pyvista as pv

        if self._inner is None:                                                 # si self._inner is None
            return                                                              # retornem sense valor

        plotter = getattr(self._inner, "plotter", None)                         # plotter ← resultat de getattr()
        if plotter is None:                                                     # si plotter is None
            print("[robot] load_registered_scene: viewer sense plotter")
            return                                                              # retornem sense valor

        self._remove_registered_scene()                                         # neteja escena prèvia
        self._reg_actors = []                                                   # llista d'actors nous

        # ----- Normalitzem a sempre treballar amb un dict {code: {mesh, T}} -----
        if vertebres is None:                                                   # mode compat: una sola vèrtebra
            if vertebra_mesh is None or T_ct_to_robot is None:
                vertebres_dict = {}
            else:
                vertebres_dict = {"default": {
                    "mesh": vertebra_mesh,
                    "T":    np.asarray(T_ct_to_robot, dtype=np.float64),
                }}
        else:                                                                   # multi-vèrtebra
            vertebres_dict = {k: {"mesh": v["mesh"],
                                   "T": np.asarray(v["T"], dtype=np.float64)}
                              for k, v in vertebres.items() if v.get("mesh") is not None
                              and v.get("T") is not None}

        if not vertebres_dict:                                                  # si no hi ha res a dibuixar
            print("[robot] load_registered_scene: cap vèrtebra amb T vàlida")
            return

        # ----- Helpers per transformar punts CT → robot amb una T concreta -----
        def make_transformer(T):                                                # factory de funcions
            R = T[:3, :3]
            t_mm = T[:3, 3]
            def ct_mm_to_robot_m(pts_ct_mm):
                pts = np.asarray(pts_ct_mm, dtype=np.float64)
                single = (pts.ndim == 1)
                if single:
                    pts = pts[None, :]
                pts_robot_mm = (R @ pts.T).T + t_mm                             # aplica rot + trans (mm)
                pts_robot_m = pts_robot_mm * 0.001                              # mm → m
                return pts_robot_m[0] if single else pts_robot_m
            return ct_mm_to_robot_m

        # Dibuixem CADA vèrtebra amb la seva T i guardem també el seu centroid
        # (en coord CT) per poder associar cargols → vèrtebra per proximitat.
        vertebra_info = {}                                                      # {code: {"T": T, "centroid_ct": np.array, "xform": fn}}
        for code, data in vertebres_dict.items():
            mesh = data["mesh"]
            T    = data["T"]
            xform = make_transformer(T)
            try:
                pts_ct   = np.asarray(mesh.points, dtype=np.float64)
                centroid = pts_ct.mean(axis=0)                                  # centroid CT per associar cargols
                pts_robot = xform(pts_ct)                                       # transformem al frame robot
                mesh_r = mesh.copy()
                mesh_r.points = pts_robot
                actor_name = f"reg_vertebra_{code}"
                plotter.add_mesh(
                    mesh_r,
                    color=(207, 200, 188),                                      # pedra càlida mate (mateix color que al registre)
                    opacity=1.0,
                    smooth_shading=True,
                    specular=0.0,
                    specular_power=1,
                    ambient=0.65,                                               # base clara uniforme
                    diffuse=0.45,                                               # volum suau
                    name=actor_name,
                )
                self._reg_actors.append(actor_name)
                vertebra_info[code] = {"T": T, "centroid_ct": centroid, "xform": xform}
            except Exception as e:
                print(f"[robot] Error afegint vèrtebra '{code}': {e}")

        # ----- Associació cargol → vèrtebra: la del centroid més proper al p0 -----
        def pick_vertebra_for_screw(p0_ct):                                     # retorna xform de la vèrtebra escollida
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

        # ----- Dibuixem cargols amb la T de la seva vèrtebra associada -----
        APPROACH_OFFSET_M = 0.01                                                # 1 cm d'approach des de p0
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
                    # Dibuix 3D del cargol
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
                    # Dades per al panell de cargols (navegació robot)
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

    def _screw_endpoints_mm(self, cg):                                          # definim la funció _screw_endpoints_mm()
        main = self.window()                                                    # main ← resultat de window()
        planning_ws = None                                                      # assignem planning_ws
        if hasattr(main, "_workspaces"):                                        # si hasattr(main, "_workspaces")
            planning_ws = main._workspaces.get("planning")                      # planning_ws ← resultat de get()
        pw = None                                                               # assignem pw
        if planning_ws is not None:                                             # si planning_ws is not None
            pw = getattr(planning_ws, "widget", None) or planning_ws            # assignem pw
        if pw is None or not hasattr(pw, "spacing") or not hasattr(pw, "origin"):  # si pw is None or not hasattr(pw, "spacin...
            print("[robot] no puc trobar spacing/origin del CT")                # executem print()
            return None, None                                                   # retornem: None, None

        spacing = pw.spacing                                                    # assignem spacing
        origin = pw.origin                                                      # assignem origin

        try:                                                                    # intentem el bloc següent
            p0 = cg["p0"]                                                       # assignem p0
            p1 = cg["p1"]                                                       # assignem p1
            z0 = cg.get("z_p0", cg.get("z", 0))                                 # z0 ← resultat de get()
            z1 = cg.get("z_p1", cg.get("z", 0))                                 # z1 ← resultat de get()
            p0_mm = (origin[0] + p0[0] * spacing[0],                            # assignem p0_mm
                     origin[1] + p0[1] * spacing[1],
                     origin[2] + z0   * spacing[2])
            p1_mm = (origin[0] + p1[0] * spacing[0],                            # assignem p1_mm
                     origin[1] + p1[1] * spacing[1],
                     origin[2] + z1   * spacing[2])
            return p0_mm, p1_mm                                                 # retornem: p0_mm, p1_mm
        except (KeyError, IndexError, TypeError) as e:                          # capturem excepció: (KeyError, IndexError, TypeError) as e
            print(f"[robot] estructura de cargol inesperada: {e}")
            return None, None                                                   # retornem: None, None

    def _on_screw_selection(self, idx):                                         # definim la funció _on_screw_selection()
        if idx < 0 or idx >= len(self._reg_screws_m):                           # si idx < 0 or idx >= len(self._reg_screw...
            self._active_screw_idx = None                                       # assignem self._active_screw_idx
        else:                                                                   # en cas contrari
            self._active_screw_idx = idx                                        # assignem self._active_screw_idx

    def _on_show_vector(self, idx):                                             # definim la funció _on_show_vector()
        import pyvista as pv                                                    # importem el mòdul pyvista as pv

        if idx < 0 or idx >= len(self._reg_screws_m):                           # si idx < 0 or idx >= len(self._reg_screw...
            return                                                              # retornem sense valor
        cg = self._reg_screws_m[idx]                                            # assignem cg
        plotter = getattr(self._inner, "plotter", None)                         # plotter ← resultat de getattr()
        if plotter is None:                                                     # si plotter is None
            return                                                              # retornem sense valor

        self._clear_vector_actors()                                             # executem _clear_vector_actors()

        p0 = cg["p0_m"]                                                         # assignem p0
        p1 = cg["p1_m"]                                                         # assignem p1
        p_app = cg["p_approach_m"]                                              # assignem p_app
        color = cg["color"]                                                     # assignem color

        axis_arrow = pv.Arrow(                                                  # assignem axis_arrow
            start=p0, direction=p1 - p0, scale=float(cg["length_m"]),
            tip_length=0.15, tip_radius=0.006, shaft_radius=0.003,
        )
        plotter.add_mesh(axis_arrow, color=color, opacity=0.9,                  # assignem plotter.add_mesh(axis_arrow, color
                          name="reg_vector_axis", lighting=False)
        self._vector_actors.append("reg_vector_axis")                           # executem append()

        app_arrow = pv.Arrow(                                                   # assignem app_arrow
            start=p_app, direction=p0 - p_app, scale=0.03,
            tip_length=0.20, tip_radius=0.004, shaft_radius=0.0015,
        )
        plotter.add_mesh(app_arrow, color="#ffffff", opacity=0.8,               # assignem plotter.add_mesh(app_arrow, color
                          name="reg_vector_approach", lighting=False)
        self._vector_actors.append("reg_vector_approach")                       # executem append()

        sphere = pv.Sphere(radius=0.004, center=p_app)                          # sphere ← resultat de Sphere()
        plotter.add_mesh(sphere, color="#ffffff", opacity=0.95,                 # assignem plotter.add_mesh(sphere, color
                          name="reg_vector_sphere",
                          smooth_shading=True, specular=0.4)
        self._vector_actors.append("reg_vector_sphere")                         # executem append()

        plotter.render()                                                        # executem render()
        print(f"[robot] Vector mostrat: {cg['name']} · "
              f"approach a {p_app[0]*1000:+.1f}, "
              f"{p_app[1]*1000:+.1f}, {p_app[2]*1000:+.1f} mm")

    def _on_go_approach(self, idx):                                             # definim la funció _on_go_approach()
        if idx < 0 or idx >= len(self._reg_screws_m):                           # si idx < 0 or idx >= len(self._reg_screw...
            return                                                              # retornem sense valor
        cg = self._reg_screws_m[idx]                                            # assignem cg
        viewer = self._inner                                                    # assignem viewer
        if viewer is None:                                                      # si viewer is None
            return                                                              # retornem sense valor

        if self._current_mode != "ik6":                                         # si self._current_mode != "ik6"
            QtWidgets.QMessageBox.information(                                  # executem information()
                self, "Anar a approach",
                "Canvia al mode IK 6DOF per enviar el robot al punt d'approach."
            )
            return                                                              # retornem sense valor

        x_new = cg["axis_unit"]                                                 # assignem x_new
        R_current = viewer.poses["tcp_link"][:3, :3]                            # assignem R_current
        x_current = R_current[:, 0]                                             # assignem x_current

        cos_theta = float(np.clip(np.dot(x_current, x_new), -1.0, 1.0))         # cos_theta ← resultat de float()
        if cos_theta > 0.9999:                                                  # si cos_theta > 0.9999
            R_align = np.eye(3)                                                 # R_align ← resultat de eye()
        elif cos_theta < -0.9999:                                               # altrament si cos_theta < -0.9999
            axis = R_current[:, 1]                                              # assignem axis
            if abs(np.dot(axis, x_new)) > 0.9:                                  # si abs(np.dot(axis, x_new)) > 0.9
                axis = R_current[:, 2]                                          # assignem axis
            R_align = self._rot_matrix_from_axis_angle(axis, np.pi)             # R_align ← resultat de _rot_matrix_from_axis_angle()
        else:                                                                   # en cas contrari
            rot_axis = np.cross(x_current, x_new)                               # rot_axis ← resultat de cross()
            rot_axis /= np.linalg.norm(rot_axis)                                # dividim rot_axis
            theta = np.arccos(cos_theta)                                        # theta ← resultat de arccos()
            R_align = self._rot_matrix_from_axis_angle(rot_axis, theta)         # R_align ← resultat de _rot_matrix_from_axis_angle()

        R_new = R_align @ R_current                                             # assignem R_new

        T_new = np.eye(4)                                                       # T_new ← resultat de eye()
        T_new[:3, :3] = R_new                                                   # T_new[:3, :3] = R_new
        T_new[:3,  3] = cg["p_approach_m"]                                      # T_new[:3,  3] = cg["p_approach_m"]

        # NOU MODEL: fem que T_new sigui la nova referència. El DLS
        # portarà el braç allà, i quan hi arribi els sliders ja estaran
        # a 0 (perquè la referència s'ha reset). Si l'usuari mou llavors
        # un slider, es mourà relatiu al punt d'approach.
        viewer.T_target = T_new                                                 # assignem viewer.T_target
        viewer.T_ref_pos = T_new[:3, 3].copy()                                  # viewer.T_ref_pos ← resultat de copy()
        viewer.R_target_initial = R_new.copy()                                  # viewer.R_target_initial ← resultat de copy()
        viewer.target_xyz = np.array([0.0, 0.0, 0.0])                           # viewer.target_xyz ← resultat de array()
        viewer.target_rpy = np.array([0.0, 0.0, 0.0])                           # viewer.target_rpy ← resultat de array()
        viewer.e_smoothed = np.zeros(6)                                         # viewer.e_smoothed ← resultat de zeros()

        for actor in viewer.target_actors:                                      # iterem: actor in viewer.target_actors
            actor.user_matrix = T_new                                           # assignem actor.user_matrix

        if hasattr(viewer, "_sync_all_sliders"):                                # si hasattr(viewer, "_sync_all_sliders")
            viewer._sync_all_sliders(recenter=True)                             # assignem viewer._sync_all_sliders(recenter

        viewer._homing_goal = None                                              # si hi havia un Home, mana l'approach
        viewer._slow_until_reached = True                                       # IK a ≤10°/s per joint fins arribar

        print(f"[robot] T_target → approach de {cg['name']}: "
              f"{cg['p_approach_m'][0]*1000:+.1f}, "
              f"{cg['p_approach_m'][1]*1000:+.1f}, "
              f"{cg['p_approach_m'][2]*1000:+.1f} mm")

    def _on_trajectory_depth(self, idx, depth_mm):                              # definim la funció _on_trajectory_depth()
        """
        L'usuari mou l'slider de profunditat al panell del cargol.
        Avança el TCP al llarg del vector del cargol des de l'approach.

        Permès en IK6 (mantenir orientació) i IK5 (J6 lliure, p.ex. amb
        broca girant). En altres modes (FK, MANDO, HANDGUIDE) no té sentit.

        idx      : cargol seleccionat (a self._reg_screws_m)
        depth_mm : mil·límetres avançats des de l'approach (0..TRAJECTORY_MAX_MM)
        """
        if idx < 0 or idx >= len(self._reg_screws_m):                           # si idx no vàlid
            return                                                              # sortim
        if self._current_mode not in ("ik5", "ik6"):                            # permès en ambdós IK
            return                                                              # sortim
        viewer = self._inner                                                    # viewer
        if viewer is None:                                                      # sense viewer
            return                                                              # sortim

        # Amb la broca girant l'IK5 continua actiu (el gir del J6 no afecta
        # J1-J5), així que l'avanç funciona igual amb o sense broca.
        cg = self._reg_screws_m[idx]                                           # cargol actiu
        axis_unit = cg["axis_unit"]                                             # direcció del cargol
        p_approach = cg["p_approach_m"]                                         # punt d'approach
        depth_m = depth_mm * 0.001                                              # a metres

        # Nova posició objectiu: approach + depth · axis_unit
        # (positiu = cap a l'os al llarg del vector del cargol)
        p_target_new = p_approach + depth_m * axis_unit                         # posició nova

        # Actualitzem T_target mantenint l'orientació que ja té viewer.T_target.
        # Això vol dir que el DLS IK5 mourà el TCP suaument fins al nou punt
        # sense reorientar-lo (perquè ja està alineat amb el vector des de
        # l'approach).
        T_new = viewer.T_target.copy()                                          # còpia
        T_new[:3, 3] = p_target_new                                             # nova posició

        viewer.T_target = T_new                                                 # assignem
        # Actualitzem la referència perquè els sliders XYZ del panell IK
        # segueixin funcionant relatius a la nova posició
        viewer.T_ref_pos = T_new[:3, 3].copy()                                  # nova ref
        viewer.target_xyz = np.array([0.0, 0.0, 0.0])                           # reset sliders XYZ

        # Actualitzem els actors del target (guies visuals)
        for actor in viewer.target_actors:                                      # iterem
            actor.user_matrix = T_new                                           # assignem

        # Sincronitzem els sliders del panell IK amb la nova referència
        if hasattr(viewer, "_sync_all_sliders"):                                # si té el mètode
            viewer._sync_all_sliders(recenter=True)                             # sync

        # Log breu (només cada 5 mm per no saturar)
        if depth_mm % 5 == 0:                                                   # cada 5 mm
            print(f"[trajectory] {cg['name']} avanç {depth_mm} mm → "
                  f"({p_target_new[0]*1000:+.1f}, {p_target_new[1]*1000:+.1f}, "
                  f"{p_target_new[2]*1000:+.1f}) mm")

    def _apply_trajectory_slider_visibility(self):                              # definim la funció _apply_trajectory_slider_visibility()
        """
        Aplica la visibilitat de l'slider de trajectòria segons el mode
        actual. Visible en IK5 i IK6 (els modes on té sentit avançar al
        llarg del vector del cargol); amagat en FK/MANDO/HANDGUIDE.
        """
        if not hasattr(self, "_screw_panel") or self._screw_panel is None:      # sense panell
            return                                                              # sortim
        visible = (self._current_mode in ("ik5", "ik6"))                        # IK5 i IK6
        self._screw_panel.set_trajectory_slider_visible(visible)                # apliquem

    @staticmethod                                                               # decorador staticmethod
    def _rot_matrix_from_axis_angle(axis, angle):                               # definim la funció _rot_matrix_from_axis_angle()
        axis = np.asarray(axis, dtype=np.float64)                               # axis ← resultat de asarray()
        axis = axis / np.linalg.norm(axis)                                      # axis ← resultat de norm()
        K = np.array([                                                          # assignem K
            [       0.0, -axis[2],  axis[1]],
            [ axis[2],       0.0, -axis[0]],
            [-axis[1],  axis[0],      0.0],
        ])
        return (np.eye(3)                                                       # retornem: (np.eye(3)
                + np.sin(angle) * K
                + (1.0 - np.cos(angle)) * (K @ K))

    def _clear_vector_actors(self):                                             # definim la funció _clear_vector_actors()
        plotter = getattr(self._inner, "plotter", None)                         # plotter ← resultat de getattr()
        if plotter is None:                                                     # si plotter is None
            return                                                              # retornem sense valor
        for name in self._vector_actors:                                        # iterem: name in self._vector_actors
            try:                                                                # intentem el bloc següent
                plotter.remove_actor(name)                                      # executem remove_actor()
            except Exception:                                                   # capturem excepció: Exception
                pass                                                            # no fem res (placeholder)
        self._vector_actors = []                                                # assignem self._vector_actors
        plotter.render()                                                        # executem render()

    def _remove_registered_scene(self):                                         # definim la funció _remove_registered_scene()
        if self._inner is None:                                                 # si self._inner is None
            return                                                              # retornem sense valor
        plotter = getattr(self._inner, "plotter", None)                         # plotter ← resultat de getattr()
        if plotter is None:                                                     # si plotter is None
            return                                                              # retornem sense valor
        for name in getattr(self, "_reg_actors", []):                           # iterem: name in getattr(self, "_reg_actors", [])
            try:                                                                # intentem el bloc següent
                plotter.remove_actor(name)                                      # executem remove_actor()
            except Exception:                                                   # capturem excepció: Exception
                pass                                                            # no fem res (placeholder)
        self._reg_actors = []                                                   # assignem self._reg_actors
        self._clear_vector_actors()                                             # executem _clear_vector_actors()
        self._reg_screws_m = []                                                 # assignem self._reg_screws_m
        self._active_screw_idx = None                                           # assignem self._active_screw_idx
        if hasattr(self, "_screw_panel") and self._screw_panel is not None:     # si hasattr(self, "_screw_panel") and sel...
            self._screw_panel.set_screws([])                                    # executem set_screws()

    # ==========================================================================
    # (RETIRAT) Abans aquí hi havia un overlay modal inicial de selecció
    # de capçal que cobria tot el workspace. Ara es confia que l'usuari
    # vegi el selector del panell OPTIONS, que té vora vermella pulsant
    # per cridar l'atenció. L'usuari pot canviar el capçal en qualsevol
    # moment des d'aquell selector.
    # ==========================================================================


    def cleanup(self):                                                          # definim la funció cleanup()
        """
        Shutdown net del workspace del robot en tancar l'aplicació.

        Ordre crític:
          1. Aturar timers propis (broca, hand-guide) perquè no puguin
             disparar events durant el tancament.
          2. Aturar hand-guide si estava viu.
          3. RESET del bus per deixar els motors en mode passiu (blau).
             Això és el que garanteix que en tancar l'app, els motors NO
             es queden actius aguantant torque.
          4. Disconnect del daemon per matar el subprocés netament.
          5. Tancar el viewer intern.
        """
        import time as _time                                                    # importem el mòdul time as _time

        # 1. Aturar timers (i la broca: 0 mA + J6 fixat on és)
        self._spin_timer.stop()                                                 # executem stop()
        if self._drill is not None:                                             # si tenim broca
            self._drill.abort()                                                 # executem abort()
        if getattr(self, "_currents_panel", None) is not None:                  # el gràfic deixa de llegir el bus
            self._currents_panel.stop()                                         # executem stop()
        if self._hg_timer.isActive():                                           # si self._hg_timer.isActive()
            self._hg_timer.stop()                                               # executem stop()

        # 2. Aturar hand-guide si estava viu
        if self._hg_controller is not None:                                     # si self._hg_controller is not None
            try:                                                                # intentem el bloc següent
                self._hg_controller.stop()                                      # executem stop()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[cleanup] error aturant hand-guide: {e}")
            self._hg_controller = None                                          # assignem self._hg_controller

        # 3-4. Shutdown net dels motors (blau + daemon fora)
        if self._inner is not None:                                             # si self._inner is not None
            mc = getattr(self._inner, "motor_controller", None)                 # mc ← resultat de getattr()
            if mc is not None and mc.is_connected:                              # si mc is not None and mc.is_connected
                try:                                                            # intentem el bloc següent
                    mc.bus.reset_all()                                          # executem reset_all()
                    _time.sleep(0.15)                                           # executem sleep()
                    print("[cleanup] motors en RESET (blau)")                   # executem print()
                except Exception as e:                                          # capturem excepció: Exception as e
                    print(f"[cleanup] error reset bus: {e}")
                try:                                                            # intentem el bloc següent
                    mc.disconnect()                                             # executem disconnect()
                    print("[cleanup] daemon dels motors tancat")                # executem print()
                except Exception as e:                                          # capturem excepció: Exception as e
                    print(f"[cleanup] error disconnect: {e}")

        # 5. Tancar viewer intern
        if self._inner is not None:                                             # si self._inner is not None
            try:                                                                # intentem el bloc següent
                self._inner.close()                                             # executem close()
            except Exception:                                                   # capturem excepció: Exception
                pass                                                            # no fem res (placeholder)