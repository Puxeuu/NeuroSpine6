from qtpy import QtWidgets, QtGui
from qtpy.QtCore import Qt, Signal

from ..theme import (
    C_PANEL, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    SIZE_SM, FONT_MONO,
)
from .modes_config import MODES, get_mode


class ModeCard(QtWidgets.QFrame):
    clicked = Signal(str)

    HEIGHT = 46

    def __init__(self, mode_dict, parent=None):
        super().__init__(parent)
        self.mode = mode_dict
        self._active = False
        self._hover  = False
        self.setFixedHeight(self.HEIGHT)
        self.setCursor(Qt.PointingHandCursor)
        self.setAttribute(Qt.WA_Hover, True)

        hk = mode_dict.get("hotkey", "")
        hk_txt = f"  [{hk}]" if hk else ""
        self.setToolTip(
            f"<b>{mode_dict['name']}{hk_txt}</b><br>"
            f"<span style='color:#8e8e93'>{mode_dict['long']}</span>"
        )

        self._build_ui()
        self._refresh_style()

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        main = QtWidgets.QHBoxLayout()
        main.setContentsMargins(16, 0, 16, 0)
        main.setSpacing(10)
        main.setAlignment(Qt.AlignCenter)

        self.icon_label = QtWidgets.QLabel(self.mode["icon"])
        self.icon_label.setAlignment(Qt.AlignCenter)
        f = QtGui.QFont()
        f.setPointSize(15)
        self.icon_label.setFont(f)
        main.addWidget(self.icon_label)

        self.name_label = QtWidgets.QLabel(self.mode["name"])
        self.name_label.setStyleSheet(
            f"background: transparent; border: none; "
            f"font-size: {SIZE_SM}px; font-weight: 700; letter-spacing: 1.2px;"
        )
        main.addWidget(self.name_label)

        if self.mode.get("hotkey"):
            hk = QtWidgets.QLabel(self.mode["hotkey"])
            hk.setAlignment(Qt.AlignCenter)
            hk.setFixedSize(18, 18)
            hk.setStyleSheet(
                f"background: transparent; border: 1px solid {C_BORDER_STRONG}; "
                f"border-radius: 3px; color: {C_TEXT_MUTED}; "
                f"font-size: 9px; font-weight: 700; font-family: {FONT_MONO};"
            )
            self.hk_label = hk
            main.addWidget(hk)
        else:
            self.hk_label = None

        outer.addLayout(main, 1)

        self.accent_bar = QtWidgets.QFrame()
        self.accent_bar.setFixedHeight(3)
        outer.addWidget(self.accent_bar)

    def set_active(self, active):
        if self._active == active:
            return
        self._active = active
        self._refresh_style()

    def _refresh_style(self):
        accent = self.mode["accent"]

        if self._active:
            bg = f"rgba({self._hex_rgb(accent)}, 0.10)"
            self.setStyleSheet(f"""
                ModeCard {{
                    background-color: {bg};
                    border: none;
                    border-radius: 6px;
                }}
            """)
            self.accent_bar.setStyleSheet(f"background-color: {accent}; border: none;")
            self.icon_label.setStyleSheet(
                f"background: transparent; border: none; color: {accent};"
            )
            self.name_label.setStyleSheet(
                f"background: transparent; border: none; color: {C_TEXT}; "
                f"font-size: {SIZE_SM}px; font-weight: 700; letter-spacing: 1.2px;"
            )
        else:
            hover_bg = "rgba(255,255,255,0.03)" if self._hover else "transparent"
            self.setStyleSheet(f"""
                ModeCard {{
                    background-color: {hover_bg};
                    border: none;
                    border-radius: 6px;
                }}
            """)
            self.accent_bar.setStyleSheet("background-color: transparent; border: none;")
            icon_col = C_TEXT_DIM if self._hover else C_TEXT_MUTED
            self.icon_label.setStyleSheet(
                f"background: transparent; border: none; color: {icon_col};"
            )
            name_col = C_TEXT if self._hover else C_TEXT_DIM
            self.name_label.setStyleSheet(
                f"background: transparent; border: none; color: {name_col}; "
                f"font-size: {SIZE_SM}px; font-weight: 700; letter-spacing: 1.2px;"
            )

    @staticmethod
    def _hex_rgb(hex_color):
        h = hex_color.lstrip("#")
        return f"{int(h[0:2],16)},{int(h[2:4],16)},{int(h[4:6],16)}"

    def enterEvent(self, e):
        self._hover = True
        self._refresh_style()
        super().enterEvent(e)

    def leaveEvent(self, e):
        self._hover = False
        self._refresh_style()
        super().leaveEvent(e)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.clicked.emit(self.mode["id"])
        super().mousePressEvent(e)


class ModeSelector(QtWidgets.QFrame):
    modeChanged = Signal(str)

    HEIGHT = 58

    def __init__(self, initial_mode="ik6", parent=None):
        super().__init__(parent)
        self.setFixedHeight(self.HEIGHT)
        self.setStyleSheet(f"""
            ModeSelector {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)

        self._current = initial_mode
        self._cards = {}

        self._build_ui()
        self.set_mode(initial_mode, silent=True)

    def _build_ui(self):
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(6, 4, 6, 4)
        h.setSpacing(4)

        for mode in MODES:
            card = ModeCard(mode)
            card.clicked.connect(self._on_card_clicked)
            self._cards[mode["id"]] = card
            h.addWidget(card, 1)

    def _on_card_clicked(self, mode_id):
        if mode_id == self._current:
            return
        self.set_mode(mode_id)

    def set_mode(self, mode_id, silent=False):
        m = get_mode(mode_id)
        if not m:
            return
        self._current = mode_id

        for mid, card in self._cards.items():
            card.set_active(mid == mode_id)

        if not silent:
            self.modeChanged.emit(mode_id)

    def current_mode(self):
        return self._current

