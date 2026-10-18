"""
Panell de control manual del motor J6 (broca).

Disseny:
    - Minimalista
    - Gris fosc, coherent amb ViewerDLSPro
    - Sense colors d'estat
    - Sense elements decoratius innecessaris
"""

from qtpy import QtWidgets, QtCore                                              # importem QtWidgets, QtCore des de qtpy
from qtpy.QtCore import Qt                                                      # importem Qt des de qtpy.QtCore

from ..theme import (                                                           # importem ( des de ..theme
    C_PANEL,
    C_PANEL_ALT,
    C_BORDER,
    C_BORDER_STRONG,
    C_TEXT,
    C_TEXT_DIM,
    C_TEXT_MUTED,
    SIZE_XS,
    SIZE_SM,
    FONT_MONO,
)


# =============================================================================
# RANGS
# =============================================================================

MIN_VELOCITY = -3000                                                            # assignem MIN_VELOCITY
MAX_VELOCITY = 3000                                                             # assignem MAX_VELOCITY

MIN_CURRENT_MA = 100                                                            # assignem MIN_CURRENT_MA
MAX_CURRENT_MA = 5000                                                           # assignem MAX_CURRENT_MA

DEFAULT_CURRENT_MA = 2000                                                       # assignem DEFAULT_CURRENT_MA


# =============================================================================
# PANELL J6
# =============================================================================

class J6ControlPanel(QtWidgets.QFrame):                                         # definim la classe J6ControlPanel

    velocityRequested = QtCore.Signal(int, int)                                 # velocityRequested ← resultat de Signal()
    torqueRequested = QtCore.Signal(int)                                        # torqueRequested ← resultat de Signal()
    stopRequested = QtCore.Signal()                                             # stopRequested ← resultat de Signal()

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()

        # ---------------------------------------------------------------------
        # ESTAT
        # ---------------------------------------------------------------------

        self.mode = "torque"                                                    # únic mode disponible: TORQUE (control per corrent)
        self.running = False                                                    # assignem self.running

        self.setObjectName("J6ControlPanel")                                    # executem setObjectName()
        self.setFixedWidth(280)                                                 # executem setFixedWidth()

        # ---------------------------------------------------------------------
        # ESTIL GENERAL
        # ---------------------------------------------------------------------

        self.setStyleSheet(f"""
            QFrame#J6ControlPanel {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 12px;
            }}
        """)

        # ---------------------------------------------------------------------
        # LAYOUT
        # ---------------------------------------------------------------------

        layout = QtWidgets.QVBoxLayout(self)                                    # layout ← resultat de QVBoxLayout()
        layout.setContentsMargins(18, 16, 18, 16)                               # executem setContentsMargins()
        layout.setSpacing(10)                                                   # executem setSpacing()

        # =====================================================================
        # TITOL
        # =====================================================================

        self.title = QtWidgets.QLabel("BROCA J6")                               # self.title ← resultat de QLabel()

        self.title.setStyleSheet(f"""
            QLabel {{
                color: {C_TEXT};
                background: transparent;
                border: none;
                font-size: {SIZE_SM}px;
                font-weight: 700;
                letter-spacing: 1px;
            }}
        """)

        layout.addWidget(self.title)                                            # executem addWidget()

        # =====================================================================
        # MODE — Panell simplificat: NOMÉS TORQUE (control per corrent).
        # Els botons VELOCITY/TORQUE es creen igual per no trencar codi que
        # els referencia, però NO s'afegeixen al layout (invisibles).
        # =====================================================================

        self.btn_vel = self._make_mode_button("VELOCITY")                       # self.btn_vel ← resultat de _make_mode_button()
        self.btn_tor = self._make_mode_button("TORQUE")                         # self.btn_tor ← resultat de _make_mode_button()

        # Deixem TORQUE marcat des del principi (és l'únic mode disponible)
        self.btn_tor.setChecked(True)                                           # executem setChecked()

        self.btn_vel.clicked.connect(                                           # executem connect()
            lambda: self._set_mode("velocity")
        )

        self.btn_tor.clicked.connect(                                           # executem connect()
            lambda: self._set_mode("torque")
        )

        self.mode_group = QtWidgets.QButtonGroup(self)                          # self.mode_group ← resultat de QButtonGroup()
        self.mode_group.setExclusive(True)                                      # executem setExclusive()
        self.mode_group.addButton(self.btn_vel)                                 # executem addButton()
        self.mode_group.addButton(self.btn_tor)                                 # executem addButton()

        # NO afegim mode_row al layout: els botons queden ocults
        self.btn_vel.hide()                                                     # amaguem VELOCITY
        self.btn_tor.hide()                                                     # amaguem TORQUE (el mode és fix)

        # =====================================================================
        # VELOCITY
        # =====================================================================

        self.vel_panel = QtWidgets.QWidget()                                    # self.vel_panel ← resultat de QWidget()

        vp = QtWidgets.QVBoxLayout(self.vel_panel)                              # vp ← resultat de QVBoxLayout()
        vp.setContentsMargins(0, 4, 0, 0)                                       # executem setContentsMargins()
        vp.setSpacing(6)                                                        # executem setSpacing()

        # Label
        vp.addWidget(                                                           # executem addWidget()
            self._make_caption("VELOCITAT")
        )

        # Valor
        self.vel_value = self._make_value("0")                                  # self.vel_value ← resultat de _make_value()
        vp.addWidget(self.vel_value)                                            # executem addWidget()

        # Slider
        self.slider_vel = self._make_slider(                                    # assignem self.slider_vel
            MIN_VELOCITY,
            MAX_VELOCITY,
            0,
            500
        )

        self.slider_vel.valueChanged.connect(                                   # executem connect()
            self._on_vel_slider
        )

        vp.addWidget(self.slider_vel)                                           # executem addWidget()

        # -------------------------------------------------------------
        # CORRENT MAX
        # -------------------------------------------------------------

        vp.addSpacing(4)                                                        # executem addSpacing()

        vp.addWidget(                                                           # executem addWidget()
            self._make_caption("CORRENT MÀXIM")
        )

        self.vel_current_value = self._make_value(                              # assignem self.vel_current_value
            str(DEFAULT_CURRENT_MA),
            small=True
        )

        vp.addWidget(self.vel_current_value)                                    # executem addWidget()

        self.slider_vel_current = self._make_slider(                            # assignem self.slider_vel_current
            MIN_CURRENT_MA,
            MAX_CURRENT_MA,
            DEFAULT_CURRENT_MA,
            1000
        )

        self.slider_vel_current.valueChanged.connect(                           # executem connect()
            self._on_vel_current_slider
        )

        vp.addWidget(self.slider_vel_current)                                   # executem addWidget()

        layout.addWidget(self.vel_panel)                                        # executem addWidget()
        self.vel_panel.hide()                                                   # amagat: mode VELOCITY no disponible

        # =====================================================================
        # TORQUE
        # =====================================================================

        self.tor_panel = QtWidgets.QWidget()                                    # self.tor_panel ← resultat de QWidget()

        tp = QtWidgets.QVBoxLayout(self.tor_panel)                              # tp ← resultat de QVBoxLayout()
        tp.setContentsMargins(0, 4, 0, 0)                                       # executem setContentsMargins()
        tp.setSpacing(6)                                                        # executem setSpacing()

        tp.addWidget(                                                           # executem addWidget()
            self._make_caption("CORRENT")
        )

        self.tor_value = self._make_value("0")                                  # self.tor_value ← resultat de _make_value()
        tp.addWidget(self.tor_value)                                            # executem addWidget()

        self.slider_tor = self._make_slider(                                    # assignem self.slider_tor
            -MAX_CURRENT_MA,
            MAX_CURRENT_MA,
            0,
            1000
        )

        self.slider_tor.valueChanged.connect(                                   # executem connect()
            self._on_tor_slider
        )

        tp.addWidget(self.slider_tor)                                           # executem addWidget()

        layout.addWidget(self.tor_panel)                                        # executem addWidget()

        # tor_panel visible per defecte: només aquest mode està disponible
        # (no cridem .hide(), a diferència de vel_panel)

        # =====================================================================
        # RPM
        # =====================================================================

        self.lbl_rpm = QtWidgets.QLabel(                                        # assignem self.lbl_rpm
            "0 RPM"
        )

        self.lbl_rpm.setAlignment(Qt.AlignCenter)                               # executem setAlignment()

        self.lbl_rpm.setStyleSheet(f"""
            QLabel {{
                color: {C_TEXT_MUTED};
                background: transparent;
                border: none;
                font-size: {SIZE_XS}px;
                font-family: {FONT_MONO};
            }}
        """)

        layout.addWidget(self.lbl_rpm)                                          # executem addWidget()
        self.lbl_rpm.hide()                                                     # amagat: era indicador del mode velocity

        # =====================================================================
        # BOTONS
        # =====================================================================

        buttons = QtWidgets.QHBoxLayout()                                       # buttons ← resultat de QHBoxLayout()
        buttons.setSpacing(6)                                                   # executem setSpacing()

        self.btn_start = self._make_action_button("START")                      # self.btn_start ← resultat de _make_action_button()
        self.btn_stop = self._make_action_button("STOP")                        # self.btn_stop ← resultat de _make_action_button()

        self.btn_start.clicked.connect(                                         # executem connect()
            self._on_start
        )

        self.btn_stop.clicked.connect(                                          # executem connect()
            self._on_stop
        )

        self.btn_stop.setEnabled(False)                                         # executem setEnabled()

        buttons.addWidget(self.btn_start)                                       # executem addWidget()
        buttons.addWidget(self.btn_stop)                                        # executem addWidget()

        layout.addLayout(buttons)                                               # executem addLayout()

        # ---------------------------------------------------------------------
        # INIT
        # ---------------------------------------------------------------------

        self._update_rpm_label()                                                # executem _update_rpm_label()

    # =========================================================================
    # HELPERS
    # =========================================================================

    def _make_caption(self, text):                                              # definim la funció _make_caption()

        label = QtWidgets.QLabel(text)                                          # label ← resultat de QLabel()

        label.setStyleSheet(f"""
            QLabel {{
                color: {C_TEXT_MUTED};
                background: transparent;
                border: none;
                font-size: {SIZE_XS}px;
                font-weight: 700;
                letter-spacing: 0.8px;
            }}
        """)

        return label                                                            # retornem: label

    # -------------------------------------------------------------------------

    def _make_value(self, text, small=False):                                   # definim la funció _make_value()

        label = QtWidgets.QLabel(text)                                          # label ← resultat de QLabel()

        label.setAlignment(Qt.AlignCenter)                                      # executem setAlignment()

        size = 19 if small else 24                                              # assignem size

        label.setStyleSheet(f"""
            QLabel {{
                color: {C_TEXT};
                background: transparent;
                border: none;
                font-size: {size}px;
                font-weight: 700;
                font-family: {FONT_MONO};
            }}
        """)

        return label                                                            # retornem: label

    # -------------------------------------------------------------------------

    def _make_mode_button(self, text):                                          # definim la funció _make_mode_button()

        button = QtWidgets.QPushButton(text)                                    # button ← resultat de QPushButton()

        button.setCheckable(True)                                               # executem setCheckable()
        button.setMinimumHeight(28)                                             # executem setMinimumHeight()

        button.setStyleSheet(f"""
            QPushButton {{
                background-color: {C_PANEL_ALT};
                color: {C_TEXT_MUTED};
                border: 1px solid {C_BORDER};
                border-radius: 6px;
                font-size: {SIZE_XS}px;
                font-weight: 700;
                letter-spacing: 0.7px;
            }}

            QPushButton:hover {{
                background-color: {C_BORDER};
                color: {C_TEXT_DIM};
            }}

            QPushButton:checked {{
                background-color: {C_BORDER_STRONG};
                color: {C_TEXT};
                border: 1px solid {C_BORDER_STRONG};
            }}
        """)

        return button                                                           # retornem: button

    # -------------------------------------------------------------------------

    def _make_action_button(self, text):                                        # definim la funció _make_action_button()

        button = QtWidgets.QPushButton(text)                                    # button ← resultat de QPushButton()

        button.setMinimumHeight(34)                                             # executem setMinimumHeight()

        button.setStyleSheet(f"""
            QPushButton {{
                background-color: {C_PANEL_ALT};
                color: {C_TEXT_DIM};
                border: 1px solid {C_BORDER_STRONG};
                border-radius: 6px;
                font-size: {SIZE_XS}px;
                font-weight: 700;
                letter-spacing: 1px;
            }}

            QPushButton:hover {{
                background-color: {C_BORDER};
                color: {C_TEXT};
            }}

            QPushButton:pressed {{
                background-color: {C_BORDER_STRONG};
            }}

            QPushButton:disabled {{
                background-color: {C_PANEL_ALT};
                color: {C_TEXT_MUTED};
                border: 1px solid {C_BORDER};
            }}
        """)

        return button                                                           # retornem: button

    # -------------------------------------------------------------------------

    def _make_slider(                                                           # definim la funció _make_slider()
        self,
        minimum,
        maximum,
        value,
        tick_interval
    ):

        slider = QtWidgets.QSlider(Qt.Horizontal)                               # slider ← resultat de QSlider()

        slider.setRange(                                                        # executem setRange()
            minimum,
            maximum
        )

        slider.setValue(value)                                                  # executem setValue()

        slider.setTickPosition(                                                 # executem setTickPosition()
            QtWidgets.QSlider.TicksBelow
        )

        slider.setTickInterval(                                                 # executem setTickInterval()
            tick_interval
        )

        slider.setStyleSheet(f"""
            QSlider {{
                background: transparent;
            }}

            QSlider::groove:horizontal {{
                background: {C_BORDER};
                height: 4px;
                border-radius: 2px;
            }}

            QSlider::sub-page:horizontal {{
                background: {C_BORDER_STRONG};
                border-radius: 2px;
            }}

            QSlider::add-page:horizontal {{
                background: {C_PANEL_ALT};
                border-radius: 2px;
            }}

            QSlider::handle:horizontal {{
                background: {C_TEXT_DIM};
                border: 1px solid {C_PANEL};
                width: 12px;
                height: 12px;
                margin: -4px 0;
                border-radius: 6px;
            }}

            QSlider::handle:horizontal:hover {{
                background: {C_TEXT};
            }}

            QSlider::tick-mark:horizontal {{
                background: {C_BORDER_STRONG};
                width: 1px;
                height: 3px;
            }}
        """)

        return slider                                                           # retornem: slider

    # =========================================================================
    # VELOCITY
    # =========================================================================

    def _on_vel_slider(self, value):                                            # definim la funció _on_vel_slider()

        if value > 0:                                                           # si value > 0
            text = f"+{value}"
        else:                                                                   # en cas contrari
            text = str(value)                                                   # text ← resultat de str()

        self.vel_value.setText(text)                                            # executem setText()

        self._update_rpm_label()                                                # executem _update_rpm_label()

        if self.running and self.mode == "velocity":                            # si self.running and self.mode == "velocity"
            self._emit_velocity()                                               # executem _emit_velocity()

    # -------------------------------------------------------------------------

    def _on_vel_current_slider(self, value):                                    # definim la funció _on_vel_current_slider()

        self.vel_current_value.setText(                                         # executem setText()
            str(value)
        )

        if self.running and self.mode == "velocity":                            # si self.running and self.mode == "velocity"
            self._emit_velocity()                                               # executem _emit_velocity()

    # =========================================================================
    # TORQUE
    # =========================================================================

    def _on_tor_slider(self, value):                                            # definim la funció _on_tor_slider()

        if value > 0:                                                           # si value > 0
            text = f"+{value}"
        else:                                                                   # en cas contrari
            text = str(value)                                                   # text ← resultat de str()

        self.tor_value.setText(text)                                            # executem setText()

        if self.running and self.mode == "torque":                              # si self.running and self.mode == "torque"
            self.torqueRequested.emit(value)                                    # executem emit()

    # =========================================================================
    # RPM
    # =========================================================================

    def _update_rpm_label(self):                                                # definim la funció _update_rpm_label()

        rpm_sortida = self.slider_vel.value()                                   # rpm_sortida ← resultat de value()

        rpm_motor = rpm_sortida * 51.0                                          # assignem rpm_motor

        self.lbl_rpm.setText(                                                   # executem setText()
            f"{rpm_motor:+.0f} RPM · motor"
        )

    # =========================================================================
    # MODE
    # =========================================================================

    def _set_mode(self, mode):                                                  # definim la funció _set_mode()

        if self.running:                                                        # si self.running
            self._on_stop()                                                     # executem _on_stop()

        self.mode = mode                                                        # assignem self.mode

        self.vel_panel.setVisible(                                              # executem setVisible()
            mode == "velocity"
        )

        self.tor_panel.setVisible(                                              # executem setVisible()
            mode == "torque"
        )

    # =========================================================================
    # START
    # =========================================================================

    def _on_start(self):                                                        # definim la funció _on_start()

        self.running = True                                                     # assignem self.running

        self.btn_start.setEnabled(False)                                        # executem setEnabled()
        self.btn_stop.setEnabled(True)                                          # executem setEnabled()

        if self.mode == "velocity":                                             # si self.mode == "velocity"
            self._emit_velocity()                                               # executem _emit_velocity()
        else:                                                                   # en cas contrari
            self.torqueRequested.emit(                                          # executem emit()
                self.slider_tor.value()
            )

    # =========================================================================
    # STOP
    # =========================================================================

    def _on_stop(self):                                                         # definim la funció _on_stop()

        self.set_stopped()                                                      # executem set_stopped()

        self.stopRequested.emit()                                               # executem emit()

    def set_stopped(self):                                                      # definim la funció set_stopped()
        """
        Posa la UI en estat "aturat" SENSE emetre stopRequested.
        El workspace la fa servir quan ja s'encarrega ell de parar la broca
        (canvi de mode, STOP general), per no disparar dues parades.
        """

        self.running = False                                                    # assignem self.running

        self.btn_start.setEnabled(True)                                         # executem setEnabled()
        self.btn_stop.setEnabled(False)                                         # executem setEnabled()

        # Reset sense generar comandes intermèdies
        self.slider_vel.blockSignals(True)                                      # executem blockSignals()
        self.slider_tor.blockSignals(True)                                      # executem blockSignals()

        self.slider_vel.setValue(0)                                             # executem setValue()
        self.slider_tor.setValue(0)                                             # executem setValue()

        self.slider_vel.blockSignals(False)                                     # executem blockSignals()
        self.slider_tor.blockSignals(False)                                     # executem blockSignals()

        self.vel_value.setText("0")                                             # executem setText()
        self.tor_value.setText("0")                                             # executem setText()

        self._update_rpm_label()                                                # executem _update_rpm_label()

    # =========================================================================
    # EMISSIÓ
    # =========================================================================

    def _emit_velocity(self):                                                   # definim la funció _emit_velocity()

        self.velocityRequested.emit(                                            # executem emit()
            self.slider_vel_current.value(),
            self.slider_vel.value(),
        )

    # =========================================================================
    # ESTAT
    # =========================================================================

    def is_running(self):                                                       # definim la funció is_running()

        return self.running                                                     # retornem: self.running