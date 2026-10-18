"""
Barra lateral esquerra amb els botons dels workspaces.
"""

from qtpy import QtWidgets, QtCore                                              # importem QtWidgets, QtCore des de qtpy

from .theme import (                                                            # importem ( des de .theme
    C_BG_SIDEBAR, C_BORDER, C_TEXT, C_TEXT_DIM,
)


# ==============================================================================
# BOTÓ DE LA BARRA LATERAL
# ==============================================================================

class SidebarButton(QtWidgets.QPushButton):                                     # definim la classe SidebarButton
    """Botó de la barra lateral: icona a dalt + label a baix."""                # """Botó de la barra lateral: icona a dalt + label a baix."""

    WIDTH  = 84                                                                 # assignem WIDTH
    HEIGHT = 76                                                                 # assignem HEIGHT

    def __init__(self, icon_text, label, parent=None):                          # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setCheckable(True)                                                 # executem setCheckable()
        self.setFixedSize(self.WIDTH, self.HEIGHT)                              # executem setFixedSize()
        self.setCursor(QtCore.Qt.PointingHandCursor)                            # executem setCursor()

        layout = QtWidgets.QVBoxLayout(self)                                    # layout ← resultat de QVBoxLayout()
        layout.setContentsMargins(0, 8, 0, 8)                                   # executem setContentsMargins()
        layout.setSpacing(4)                                                    # executem setSpacing()
        layout.setAlignment(QtCore.Qt.AlignCenter)                              # executem setAlignment()

        self.icon_label = QtWidgets.QLabel(icon_text)                           # self.icon_label ← resultat de QLabel()
        self.icon_label.setAlignment(QtCore.Qt.AlignCenter)                     # executem setAlignment()
        layout.addWidget(self.icon_label)                                       # executem addWidget()

        self.text_label = QtWidgets.QLabel(label)                               # self.text_label ← resultat de QLabel()
        self.text_label.setAlignment(QtCore.Qt.AlignCenter)                     # executem setAlignment()
        layout.addWidget(self.text_label)                                       # executem addWidget()

        self._sync_colors()                                                     # executem _sync_colors()
        self._apply_style()                                                     # executem _apply_style()

    def _apply_style(self):                                                     # definim la funció _apply_style()
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

    def _sync_colors(self):                                                     # definim la funció _sync_colors()
        color = C_TEXT if self.isChecked() else C_TEXT_DIM                      # assignem color
        self.icon_label.setStyleSheet(                                          # executem setStyleSheet()
            f"color: {color}; background: transparent; border: none; font-size: 22px;"
        )
        self.text_label.setStyleSheet(                                          # executem setStyleSheet()
            f"color: {color}; background: transparent; border: none; "
            f"font-size: 9px; font-weight: 600; letter-spacing: 0.5px;"
        )

    def setChecked(self, checked):                                              # definim la funció setChecked()
        super().setChecked(checked)                                             # executem super()
        self._sync_colors()                                                     # executem _sync_colors()


# ==============================================================================
# SIDEBAR
# ==============================================================================

class Sidebar(QtWidgets.QFrame):                                                # definim la classe Sidebar

    WIDTH = 92                                                                  # assignem WIDTH
    workspaceRequested = QtCore.Signal(str)                                     # workspaceRequested ← resultat de Signal()
    settingsRequested  = QtCore.Signal()                                        # settingsRequested ← resultat de Signal()

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setFixedWidth(self.WIDTH)                                          # executem setFixedWidth()
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_SIDEBAR};
                border: none;
                border-right: 1px solid {C_BORDER};
            }}
        """)

        layout = QtWidgets.QVBoxLayout(self)                                    # layout ← resultat de QVBoxLayout()
        layout.setContentsMargins(0, 16, 0, 16)                                 # executem setContentsMargins()
        layout.setSpacing(2)                                                    # executem setSpacing()
        layout.setAlignment(QtCore.Qt.AlignTop)                                 # executem setAlignment()

        self._group = QtWidgets.QButtonGroup(self)                              # self._group ← resultat de QButtonGroup()
        self._group.setExclusive(True)                                          # executem setExclusive()
        self._buttons = {}                                                      # assignem self._buttons

        # Especificació dels workspaces:
        #   nom intern      icona     label
        specs = [                                                               # assignem specs
            ("planning",     "▦",  "PLANNING"),
            ("robot",        "▤",  "ROBOT"),
            ("registration", "◈",  "REGISTER"),
            ("navigation",   "◎",  "NAVIGATION"),
        ]
        for name, icon, label in specs:                                         # iterem: name, icon, label in specs
            btn = SidebarButton(icon, label)                                    # btn ← resultat de SidebarButton()
            btn.clicked.connect(lambda _checked, n=name: self.workspaceRequested.emit(n))  # btn.clicked.connect(lambda _checked, n ← resultat de emit()
            self._group.addButton(btn)                                          # executem addButton()
            self._buttons[name] = btn                                           # assignem self._buttons[name]
            layout.addWidget(btn)                                               # executem addWidget()

        layout.addStretch()                                                     # executem addStretch()

        # Separador
        sep = QtWidgets.QFrame()                                                # sep ← resultat de QFrame()
        sep.setFixedHeight(1)                                                   # executem setFixedHeight()
        sep.setStyleSheet(f"background-color: {C_BORDER}; border: none; margin: 0 12px;")
        layout.addWidget(sep)                                                   # executem addWidget()

        # Botó settings
        settings_btn = SidebarButton("⚙", "SETTINGS")                           # settings_btn ← resultat de SidebarButton()
        settings_btn.setCheckable(False)                                        # executem setCheckable()
        settings_btn.clicked.connect(self.settingsRequested.emit)               # executem connect()
        layout.addWidget(settings_btn)                                          # executem addWidget()

    def set_active(self, name):                                                 # definim la funció set_active()
        if name in self._buttons:                                               # si name in self._buttons
            self._buttons[name].setChecked(True)                                # self._buttons[name].setChecked(True)
