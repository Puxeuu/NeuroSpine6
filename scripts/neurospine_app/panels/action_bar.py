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
)


class ActionButton(QtWidgets.QPushButton):
    HEIGHT = 34

    def __init__(self, text, icon=None, parent=None):
        super().__init__(parent)

        self.setText(
            f"{icon}  {text}" if icon else text
        )

        self.setCursor(
            Qt.PointingHandCursor
        )

        self.setCheckable(False)

        self.setFixedHeight(
            self.HEIGHT
        )

        self.setMinimumWidth(84)

        self._apply_style()

    def _apply_style(self):
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {C_PANEL_ALT};
                color: {C_TEXT_DIM};
                border: 1px solid {C_BORDER_STRONG};
                border-radius: 7px;
                padding: 0 12px;
                font-size: {SIZE_XS}px;
                font-weight: 700;
                letter-spacing: 0.4px;
            }}

            QPushButton:hover {{
                background-color: {C_BORDER};
                color: {C_TEXT};
            }}

            QPushButton:pressed {{
                background-color: {C_BORDER_STRONG};
                color: {C_TEXT};
            }}

            QPushButton:checked {{
                background-color: {C_BORDER_STRONG};
                color: {C_TEXT};
                border: 1px solid #5a5d62;
            }}

            QPushButton:disabled {{
                background-color: {C_PANEL_ALT};
                color: {C_TEXT_MUTED};
                border: 1px solid {C_BORDER};
            }}
        """)

    def set_label(self, text):
        self.setText(text)


class ActionBar(QtWidgets.QFrame):
    HEIGHT = 52

    currentsToggled = QtCore.Signal(bool)

    def __init__(self, viewer, parent=None):
        super().__init__(parent)

        self.viewer = viewer

        self.setObjectName(
            "ActionBar"
        )

        self.setFixedHeight(
            self.HEIGHT
        )

        self.setStyleSheet(f"""
            QFrame#ActionBar {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)

        h = QtWidgets.QHBoxLayout(self)

        h.setContentsMargins(
            8, 8, 8, 8
        )

        h.setSpacing(6)

        self.btn_connect = ActionButton(
            "Connectar"
        )

        self.btn_connect.clicked.connect(
            self._on_connect
        )

        h.addWidget(
            self.btn_connect
        )

        self.btn_mirror = ActionButton(
            "Mirror"
        )

        self.btn_mirror.setEnabled(
            False
        )

        self.btn_mirror.clicked.connect(
            self._on_mirror
        )

        h.addWidget(
            self.btn_mirror
        )

        self.btn_home = ActionButton(
            "Home"
        )

        self.btn_home.clicked.connect(
            self._on_home
        )

        h.addWidget(
            self.btn_home
        )

        h.addWidget(
            self._separator()
        )

        self.btn_stop = ActionButton("STOP")

        self.btn_stop.setStyleSheet("""
            QPushButton {
                background-color: #C94A45;
                color: white;
                border: 1px solid #C94A45;
                border-radius: 7px;
                padding: 0 12px;
                font-size: 11px;
                font-weight: 700;
                letter-spacing: 0.4px;
            }

            QPushButton:hover {
                background-color: #D6534E;
                border: 1px solid #D6534E;
            }

            QPushButton:pressed {
                background-color: #A83C38;
                border: 1px solid #A83C38;
            }

            QPushButton:disabled {
                background-color: #2C2C2E;
                color: #636366;
                border: 1px solid #3A3A3C;
            }
        """)

        self.btn_stop.clicked.connect(self._on_stop)

        self.btn_stop.clicked.connect(
            self._on_stop
        )

        h.addWidget(
            self.btn_stop
        )

        h.addStretch()

        self.btn_collisions = ActionButton(
            "Collisions"
        )

        self.btn_collisions.setCheckable(
            True
        )

        chk = getattr(
            viewer,
            "chk_collisions",
            None
        )

        if chk is not None:
            self.btn_collisions.setChecked(
                chk.isChecked()
            )

        self.btn_collisions.clicked.connect(
            self._on_toggle_collisions
        )

        h.addWidget(
            self.btn_collisions
        )

        self.btn_target = ActionButton(
            "Target"
        )

        self.btn_target.setCheckable(
            True
        )

        chk_target = getattr(
            viewer,
            "chk_target",
            None
        )

        if chk_target is not None:
            self.btn_target.setChecked(
                chk_target.isChecked()
            )

        self.btn_target.clicked.connect(
            self._on_toggle_target
        )

        h.addWidget(
            self.btn_target
        )

        self.btn_currents = ActionButton(
            "Intensitats"
        )

        self.btn_currents.setCheckable(True)
        self.btn_currents.setToolTip("Gràfic del corrent de tots els motors (I)")

        self.btn_currents.toggled.connect(
            self.currentsToggled.emit
        )

        h.addWidget(
            self.btn_currents
        )

        self.capcal_frame = QtWidgets.QFrame()

        self.capcal_frame.setFixedHeight(
            ActionButton.HEIGHT
        )

        self.capcal_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {C_PANEL_ALT};
                border: 1.5px solid #ff3b30;
                border-radius: 7px;
            }}
        """)

        capcal_layout = QtWidgets.QHBoxLayout(
            self.capcal_frame
        )

        capcal_layout.setContentsMargins(
            3, 3, 3, 3
        )

        capcal_layout.setSpacing(
            2
        )

        capcal_info = {
            "bola": "Bola",
            "punxo": "Punxó",
            "broca": "Broca",
        }

        self.capcal_buttons = {}

        for cap_id, label in capcal_info.items():
            button = QtWidgets.QPushButton(
                label
            )

            button.setCheckable(
                True
            )

            button.setFixedHeight(
                26
            )

            button.setCursor(
                Qt.PointingHandCursor
            )

            button.setStyleSheet(f"""
                QPushButton {{
                    background-color: transparent;
                    color: {C_TEXT_MUTED};
                    border: none;
                    border-radius: 5px;
                    padding: 0 10px;
                    font-size: {SIZE_XS}px;
                    font-weight: 700;
                }}

                QPushButton:hover {{
                    background-color: rgba(255, 59, 48, 0.15);
                    color: {C_TEXT};
                }}

                QPushButton:checked {{
                    background-color: #ff3b30;
                    color: white;
                }}
            """)

            button.clicked.connect(
                lambda checked,
                cid=cap_id:
                self._on_capcal_button_clicked(cid)
            )

            self.capcal_buttons[
                cap_id
            ] = button

            capcal_layout.addWidget(
                button
            )

        manager = getattr(
            self.viewer,
            "capcal_manager",
            None
        )

        if manager is not None:
            active = getattr(
                manager,
                "active_nom",
                "bola"
            )

            if active in self.capcal_buttons:
                self.capcal_buttons[
                    active
                ].setChecked(True)

        else:
            self.capcal_buttons[
                "bola"
            ].setChecked(True)

        h.addWidget(
            self.capcal_frame
        )

        self._timer = QtCore.QTimer(
            self
        )

        self._timer.timeout.connect(
            self._refresh_state
        )

        self._timer.start(
            400
        )

    def _separator(self):
        sep = QtWidgets.QFrame()

        sep.setFixedWidth(
            1
        )

        sep.setFixedHeight(
            24
        )

        sep.setStyleSheet(
            f"""
            QFrame {{
                background-color: {C_BORDER};
                border: none;
            }}
            """
        )

        return sep

    def _on_capcal_button_clicked(
        self,
        cap_id
    ):
        for cid, button in self.capcal_buttons.items():
            button.setChecked(
                cid == cap_id
            )

        selector = getattr(
            self.viewer,
            "capcal_selector",
            None
        )

        if selector is not None:
            if hasattr(
                selector,
                "cards"
            ):
                try:
                    for card_id, card in selector.cards.items():
                        card.setChecked(
                            card_id == cap_id
                        )

                except Exception as e:
                    print(
                        f"[actionbar] capcal cards: {e}"
                    )

            if hasattr(
                selector,
                "capcalChanged"
            ):
                try:
                    selector.capcalChanged.emit(
                        cap_id
                    )

                except Exception as e:
                    print(
                        f"[actionbar] capcal switch: {e}"
                    )

    def _on_connect(self):
        button = getattr(
            self.viewer,
            "btn_connect",
            None
        )

        if button is None:
            return

        button.setChecked(
            not button.isChecked()
        )

        if hasattr(
            self.viewer,
            "_on_connect_motors"
        ):
            self.viewer._on_connect_motors()

    def _on_mirror(self):
        button = getattr(
            self.viewer,
            "btn_mirror",
            None
        )

        if button is None:
            return

        if not button.isEnabled():
            return

        button.setChecked(
            not button.isChecked()
        )

        if hasattr(
            self.viewer,
            "_on_mirror_toggle"
        ):
            self.viewer._on_mirror_toggle()

    def _on_home(self):
        if hasattr(
            self.viewer,
            "_on_move_to_zero"
        ):
            self.viewer._on_move_to_zero()

    def _on_stop(self):
        if hasattr(
            self.viewer,
            "_on_stop"
        ):
            self.viewer._on_stop()

    def _on_toggle_collisions(self):
        checkbox = getattr(
            self.viewer,
            "chk_collisions",
            None
        )

        if checkbox is None:
            return

        checkbox.setChecked(
            self.btn_collisions.isChecked()
        )

    def _on_toggle_target(self):
        checkbox = getattr(
            self.viewer,
            "chk_target",
            None
        )

        if checkbox is None:
            return

        value = self.btn_target.isChecked()

        checkbox.setChecked(
            value
        )

        if hasattr(
            self.viewer,
            "_on_target_visibility"
        ):
            self.viewer._on_target_visibility(
                value
            )

    def _refresh_state(self):
        button = getattr(
            self.viewer,
            "btn_connect",
            None
        )

        if button is not None:
            connected = button.isChecked()

            self.btn_connect.setText(
                "Desconnectar"
                if connected
                else "Connectar"
            )

            self.btn_mirror.setEnabled(
                button.isChecked()
            )

        mirror = getattr(
            self.viewer,
            "btn_mirror",
            None
        )

        if mirror is not None:
            self.btn_mirror.setText(
                "Mirror ON"
                if mirror.isChecked()
                else "Mirror"
            )

            self.btn_mirror.setChecked(
                mirror.isChecked()
            )

        collisions = getattr(
            self.viewer,
            "chk_collisions",
            None
        )

        if collisions is not None:
            self.btn_collisions.setChecked(
                collisions.isChecked()
            )

        target = getattr(
            self.viewer,
            "chk_target",
            None
        )

        if target is not None:
            self.btn_target.setChecked(
                target.isChecked()
            )

