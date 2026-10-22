from qtpy import QtWidgets, QtCore
from qtpy.QtCore import Qt

from ..theme import (
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


MIN_VELOCITY = -3000
MAX_VELOCITY = 3000

MIN_CURRENT_MA = 100
MAX_CURRENT_MA = 5000

DEFAULT_CURRENT_MA = 2000


class J6ControlPanel(QtWidgets.QFrame):
    velocityRequested = QtCore.Signal(int, int)
    torqueRequested = QtCore.Signal(int)
    stopRequested = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.mode = "torque"
        self.running = False

        self.setObjectName("J6ControlPanel")
        self.setFixedWidth(280)

        self.setStyleSheet(f"""
            QFrame#J6ControlPanel {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 12px;
            }}
        """)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)

        self.title = QtWidgets.QLabel("BROCA J6")

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

        layout.addWidget(self.title)

        self.btn_vel = self._make_mode_button("VELOCITY")
        self.btn_tor = self._make_mode_button("TORQUE")

        self.btn_tor.setChecked(True)

        self.btn_vel.clicked.connect(
            lambda: self._set_mode("velocity")
        )

        self.btn_tor.clicked.connect(
            lambda: self._set_mode("torque")
        )

        self.mode_group = QtWidgets.QButtonGroup(self)
        self.mode_group.setExclusive(True)
        self.mode_group.addButton(self.btn_vel)
        self.mode_group.addButton(self.btn_tor)

        self.btn_vel.hide()
        self.btn_tor.hide()

        self.vel_panel = QtWidgets.QWidget()

        vp = QtWidgets.QVBoxLayout(self.vel_panel)
        vp.setContentsMargins(0, 4, 0, 0)
        vp.setSpacing(6)

        vp.addWidget(
            self._make_caption("VELOCITAT")
        )

        self.vel_value = self._make_value("0")
        vp.addWidget(self.vel_value)

        self.slider_vel = self._make_slider(
            MIN_VELOCITY,
            MAX_VELOCITY,
            0,
            500
        )

        self.slider_vel.valueChanged.connect(
            self._on_vel_slider
        )

        vp.addWidget(self.slider_vel)

        vp.addSpacing(4)

        vp.addWidget(
            self._make_caption("CORRENT MÀXIM")
        )

        self.vel_current_value = self._make_value(
            str(DEFAULT_CURRENT_MA),
            small=True
        )

        vp.addWidget(self.vel_current_value)

        self.slider_vel_current = self._make_slider(
            MIN_CURRENT_MA,
            MAX_CURRENT_MA,
            DEFAULT_CURRENT_MA,
            1000
        )

        self.slider_vel_current.valueChanged.connect(
            self._on_vel_current_slider
        )

        vp.addWidget(self.slider_vel_current)

        layout.addWidget(self.vel_panel)
        self.vel_panel.hide()

        self.tor_panel = QtWidgets.QWidget()

        tp = QtWidgets.QVBoxLayout(self.tor_panel)
        tp.setContentsMargins(0, 4, 0, 0)
        tp.setSpacing(6)

        tp.addWidget(
            self._make_caption("CORRENT")
        )

        self.tor_value = self._make_value("0")
        tp.addWidget(self.tor_value)

        self.slider_tor = self._make_slider(
            -MAX_CURRENT_MA,
            MAX_CURRENT_MA,
            0,
            1000
        )

        self.slider_tor.valueChanged.connect(
            self._on_tor_slider
        )

        tp.addWidget(self.slider_tor)

        layout.addWidget(self.tor_panel)

        self.lbl_rpm = QtWidgets.QLabel(
            "0 RPM"
        )

        self.lbl_rpm.setAlignment(Qt.AlignCenter)

        self.lbl_rpm.setStyleSheet(f"""
            QLabel {{
                color: {C_TEXT_MUTED};
                background: transparent;
                border: none;
                font-size: {SIZE_XS}px;
                font-family: {FONT_MONO};
            }}
        """)

        layout.addWidget(self.lbl_rpm)
        self.lbl_rpm.hide()

        buttons = QtWidgets.QHBoxLayout()
        buttons.setSpacing(6)

        self.btn_start = self._make_action_button("START")
        self.btn_stop = self._make_action_button("STOP")

        self.btn_start.clicked.connect(
            self._on_start
        )

        self.btn_stop.clicked.connect(
            self._on_stop
        )

        self.btn_stop.setEnabled(False)

        buttons.addWidget(self.btn_start)
        buttons.addWidget(self.btn_stop)

        layout.addLayout(buttons)

        self._update_rpm_label()

    def _make_caption(self, text):
        label = QtWidgets.QLabel(text)

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

        return label

    def _make_value(self, text, small=False):
        label = QtWidgets.QLabel(text)

        label.setAlignment(Qt.AlignCenter)

        size = 19 if small else 24

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

        return label

    def _make_mode_button(self, text):
        button = QtWidgets.QPushButton(text)

        button.setCheckable(True)
        button.setMinimumHeight(28)

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

        return button

    def _make_action_button(self, text):
        button = QtWidgets.QPushButton(text)

        button.setMinimumHeight(34)

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

        return button

    def _make_slider(
        self,
        minimum,
        maximum,
        value,
        tick_interval
    ):
        slider = QtWidgets.QSlider(Qt.Horizontal)

        slider.setRange(
            minimum,
            maximum
        )

        slider.setValue(value)

        slider.setTickPosition(
            QtWidgets.QSlider.TicksBelow
        )

        slider.setTickInterval(
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

        return slider

    def _on_vel_slider(self, value):
        if value > 0:
            text = f"+{value}"
        else:
            text = str(value)

        self.vel_value.setText(text)

        self._update_rpm_label()

        if self.running and self.mode == "velocity":
            self._emit_velocity()

    def _on_vel_current_slider(self, value):
        self.vel_current_value.setText(
            str(value)
        )

        if self.running and self.mode == "velocity":
            self._emit_velocity()

    def _on_tor_slider(self, value):
        if value > 0:
            text = f"+{value}"
        else:
            text = str(value)

        self.tor_value.setText(text)

        if self.running and self.mode == "torque":
            self.torqueRequested.emit(value)

    def _update_rpm_label(self):
        rpm_sortida = self.slider_vel.value()

        rpm_motor = rpm_sortida * 51.0

        self.lbl_rpm.setText(
            f"{rpm_motor:+.0f} RPM · motor"
        )

    def _set_mode(self, mode):
        if self.running:
            self._on_stop()

        self.mode = mode

        self.vel_panel.setVisible(
            mode == "velocity"
        )

        self.tor_panel.setVisible(
            mode == "torque"
        )

    def _on_start(self):
        self.running = True

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        if self.mode == "velocity":
            self._emit_velocity()
        else:
            self.torqueRequested.emit(
                self.slider_tor.value()
            )

    def _on_stop(self):
        self.set_stopped()

        self.stopRequested.emit()

    def set_stopped(self):
        self.running = False

        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

        self.slider_vel.blockSignals(True)
        self.slider_tor.blockSignals(True)

        self.slider_vel.setValue(0)
        self.slider_tor.setValue(0)

        self.slider_vel.blockSignals(False)
        self.slider_tor.blockSignals(False)

        self.vel_value.setText("0")
        self.tor_value.setText("0")

        self._update_rpm_label()

    def _emit_velocity(self):
        self.velocityRequested.emit(
            self.slider_vel_current.value(),
            self.slider_vel.value(),
        )

    def is_running(self):
        return self.running

