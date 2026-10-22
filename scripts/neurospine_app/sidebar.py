from qtpy import QtWidgets, QtCore

from .theme import (
    C_BG_SIDEBAR, C_BORDER, C_TEXT, C_TEXT_DIM,
)


class SidebarButton(QtWidgets.QPushButton):
    WIDTH  = 84
    HEIGHT = 76

    def __init__(self, icon_text, label, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setFixedSize(self.WIDTH, self.HEIGHT)
        self.setCursor(QtCore.Qt.PointingHandCursor)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 8, 0, 8)
        layout.setSpacing(4)
        layout.setAlignment(QtCore.Qt.AlignCenter)

        self.icon_label = QtWidgets.QLabel(icon_text)
        self.icon_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.icon_label)

        self.text_label = QtWidgets.QLabel(label)
        self.text_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.text_label)

        self._sync_colors()
        self._apply_style()

    def _apply_style(self):
        self.setStyleSheet(f"""
            SidebarButton {{
                background-color: transparent;
                border: none;
                border-left: 3px solid transparent;
                border-radius: 0px;
            }}
            SidebarButton:hover {{
                background-color: rgba(255, 255, 255, 0.04);
            }}
            SidebarButton:checked {{
                background-color: rgba(255, 255, 255, 0.08);
                border-left: 3px solid {C_TEXT_DIM};
            }}
        """)

    def _sync_colors(self):
        color = C_TEXT if self.isChecked() else C_TEXT_DIM
        self.icon_label.setStyleSheet(
            f"color: {color}; background: transparent; border: none; font-size: 22px;"
        )
        self.text_label.setStyleSheet(
            f"color: {color}; background: transparent; border: none; "
            f"font-size: 9px; font-weight: 600; letter-spacing: 0.5px;"
        )

    def setChecked(self, checked):
        super().setChecked(checked)
        self._sync_colors()


class Sidebar(QtWidgets.QFrame):
    WIDTH = 92
    workspaceRequested = QtCore.Signal(str)
    settingsRequested  = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(self.WIDTH)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_SIDEBAR};
                border: none;
                border-right: 1px solid {C_BORDER};
            }}
        """)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 16, 0, 16)
        layout.setSpacing(2)
        layout.setAlignment(QtCore.Qt.AlignTop)

        self._group = QtWidgets.QButtonGroup(self)
        self._group.setExclusive(True)
        self._buttons = {}

        specs = [
            ("planning",     "▦",  "PLANNING"),
            ("robot",        "▤",  "ROBOT"),
            ("registration", "◈",  "REGISTER"),
            ("navigation",   "◎",  "NAVIGATION"),
        ]
        for name, icon, label in specs:
            btn = SidebarButton(icon, label)
            btn.clicked.connect(lambda _checked, n=name: self.workspaceRequested.emit(n))
            self._group.addButton(btn)
            self._buttons[name] = btn
            layout.addWidget(btn)

        layout.addStretch()

        sep = QtWidgets.QFrame()
        sep.setFixedHeight(1)
        sep.setStyleSheet(f"background-color: {C_BORDER}; border: none; margin: 0 12px;")
        layout.addWidget(sep)

        settings_btn = SidebarButton("⚙", "SETTINGS")
        settings_btn.setCheckable(False)
        settings_btn.clicked.connect(self.settingsRequested.emit)
        layout.addWidget(settings_btn)

    def set_active(self, name):
        if name in self._buttons:
            self._buttons[name].setChecked(True)

