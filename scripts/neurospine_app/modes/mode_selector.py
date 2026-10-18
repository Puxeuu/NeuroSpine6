"""
Selector visual dels 5 modes de control del cobot.

Disseny: panell vertical estret (~180 px) a l'esquerra del viewer del
robot, amb 5 targetes apilades una sota l'altra. Cada targeta té:
    - Icona geomètrica gran (Unicode simple, sense emoji)
    - Nom del mode en majúscules amb letter-spacing
    - Subtítol curt en to atenuat
    - Línia lateral vertical de color quan el mode és actiu
    - Fons amb accent subtil al mode actiu
    - Hover state elegant

Estètica: Apple × Linear × Framer. Fosc, tipografia neta, transicions
subtils, cap sorpresa visual, tot cohesionat.
"""

from qtpy import QtWidgets, QtGui                                               # importem QtWidgets, QtGui des de qtpy
from qtpy.QtCore import Qt, Signal                                              # importem Qt, Signal des de qtpy.QtCore

from ..theme import (                                                           # importem ( des de ..theme
    C_PANEL, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    SIZE_SM, FONT_MONO,
)
from .modes_config import MODES, get_mode                                       # importem MODES, get_mode des de .modes_config


# ==============================================================================
# TARGETA D'UN MODE
# ==============================================================================

class ModeCard(QtWidgets.QFrame):                                               # definim la classe ModeCard
    """
    Targeta horitzontal d'un mode. Icona + nom en una fila compacta.
    Barra d'accent inferior (no lateral) quan és el mode actiu.
    """

    clicked = Signal(str)                                                       # clicked ← resultat de Signal()

    HEIGHT = 46                                                                 # assignem HEIGHT

    def __init__(self, mode_dict, parent=None):                                 # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.mode = mode_dict                                                   # assignem self.mode
        self._active = False                                                    # assignem self._active
        self._hover  = False                                                    # assignem self._hover
        self.setFixedHeight(self.HEIGHT)                                        # executem setFixedHeight()
        self.setCursor(Qt.PointingHandCursor)                                   # executem setCursor()
        self.setAttribute(Qt.WA_Hover, True)                                    # executem setAttribute()

        # Tooltip enriquit
        hk = mode_dict.get("hotkey", "")                                        # hk ← resultat de get()
        hk_txt = f"  [{hk}]" if hk else ""
        self.setToolTip(                                                        # executem setToolTip()
            f"<b>{mode_dict['name']}{hk_txt}</b><br>"
            f"<span style='color:#8e8e93'>{mode_dict['long']}</span>"
        )

        self._build_ui()                                                        # executem _build_ui()
        self._refresh_style()                                                   # executem _refresh_style()

    def _build_ui(self):                                                        # definim la funció _build_ui()
        outer = QtWidgets.QVBoxLayout(self)                                     # outer ← resultat de QVBoxLayout()
        outer.setContentsMargins(0, 0, 0, 0)                                    # executem setContentsMargins()
        outer.setSpacing(0)                                                     # executem setSpacing()

        # Contingut principal
        main = QtWidgets.QHBoxLayout()                                          # main ← resultat de QHBoxLayout()
        main.setContentsMargins(16, 0, 16, 0)                                   # executem setContentsMargins()
        main.setSpacing(10)                                                     # executem setSpacing()
        main.setAlignment(Qt.AlignCenter)                                       # executem setAlignment()

        self.icon_label = QtWidgets.QLabel(self.mode["icon"])                   # self.icon_label ← resultat de QLabel()
        self.icon_label.setAlignment(Qt.AlignCenter)                            # executem setAlignment()
        f = QtGui.QFont()                                                       # f ← resultat de QFont()
        f.setPointSize(15)                                                      # executem setPointSize()
        self.icon_label.setFont(f)                                              # executem setFont()
        main.addWidget(self.icon_label)                                         # executem addWidget()

        self.name_label = QtWidgets.QLabel(self.mode["name"])                   # self.name_label ← resultat de QLabel()
        self.name_label.setStyleSheet(                                          # executem setStyleSheet()
            f"background: transparent; border: none; "
            f"font-size: {SIZE_SM}px; font-weight: 700; letter-spacing: 1.2px;"
        )
        main.addWidget(self.name_label)                                         # executem addWidget()

        # Hotkey petit a la dreta (opcional)
        if self.mode.get("hotkey"):                                             # si self.mode.get("hotkey")
            hk = QtWidgets.QLabel(self.mode["hotkey"])                          # hk ← resultat de QLabel()
            hk.setAlignment(Qt.AlignCenter)                                     # executem setAlignment()
            hk.setFixedSize(18, 18)                                             # executem setFixedSize()
            hk.setStyleSheet(                                                   # executem setStyleSheet()
                f"background: transparent; border: 1px solid {C_BORDER_STRONG}; "
                f"border-radius: 3px; color: {C_TEXT_MUTED}; "
                f"font-size: 9px; font-weight: 700; font-family: {FONT_MONO};"
            )
            self.hk_label = hk                                                  # assignem self.hk_label
            main.addWidget(hk)                                                  # executem addWidget()
        else:                                                                   # en cas contrari
            self.hk_label = None                                                # assignem self.hk_label

        outer.addLayout(main, 1)                                                # executem addLayout()

        # Barra d'accent inferior (3px)
        self.accent_bar = QtWidgets.QFrame()                                    # self.accent_bar ← resultat de QFrame()
        self.accent_bar.setFixedHeight(3)                                       # executem setFixedHeight()
        outer.addWidget(self.accent_bar)                                        # executem addWidget()

    # -----------------------------------------------------------
    def set_active(self, active):                                               # definim la funció set_active()
        if self._active == active:                                              # si self._active == active
            return                                                              # retornem sense valor
        self._active = active                                                   # assignem self._active
        self._refresh_style()                                                   # executem _refresh_style()

    def _refresh_style(self):                                                   # definim la funció _refresh_style()
        accent = self.mode["accent"]                                            # assignem accent

        if self._active:                                                        # si self._active
            bg = f"rgba({self._hex_rgb(accent)}, 0.10)"
            self.setStyleSheet(f"""
                ModeCard {{
                    background-color: {bg};
                    border: none;
                    border-radius: 6px;
                }}
            """)
            self.accent_bar.setStyleSheet(f"background-color: {accent}; border: none;")
            self.icon_label.setStyleSheet(                                      # executem setStyleSheet()
                f"background: transparent; border: none; color: {accent};"
            )
            self.name_label.setStyleSheet(                                      # executem setStyleSheet()
                f"background: transparent; border: none; color: {C_TEXT}; "
                f"font-size: {SIZE_SM}px; font-weight: 700; letter-spacing: 1.2px;"
            )
        else:                                                                   # en cas contrari
            hover_bg = "rgba(255,255,255,0.03)" if self._hover else "transparent"  # assignem hover_bg
            self.setStyleSheet(f"""
                ModeCard {{
                    background-color: {hover_bg};
                    border: none;
                    border-radius: 6px;
                }}
            """)
            self.accent_bar.setStyleSheet("background-color: transparent; border: none;")  # executem setStyleSheet()
            icon_col = C_TEXT_DIM if self._hover else C_TEXT_MUTED              # assignem icon_col
            self.icon_label.setStyleSheet(                                      # executem setStyleSheet()
                f"background: transparent; border: none; color: {icon_col};"
            )
            name_col = C_TEXT if self._hover else C_TEXT_DIM                    # assignem name_col
            self.name_label.setStyleSheet(                                      # executem setStyleSheet()
                f"background: transparent; border: none; color: {name_col}; "
                f"font-size: {SIZE_SM}px; font-weight: 700; letter-spacing: 1.2px;"
            )

    @staticmethod                                                               # decorador staticmethod
    def _hex_rgb(hex_color):                                                    # definim la funció _hex_rgb()
        h = hex_color.lstrip("#")                                               # h ← resultat de lstrip()
        return f"{int(h[0:2],16)},{int(h[2:4],16)},{int(h[4:6],16)}"

    # -----------------------------------------------------------
    def enterEvent(self, e):                                                    # definim la funció enterEvent()
        self._hover = True                                                      # assignem self._hover
        self._refresh_style()                                                   # executem _refresh_style()
        super().enterEvent(e)                                                   # executem super()

    def leaveEvent(self, e):                                                    # definim la funció leaveEvent()
        self._hover = False                                                     # assignem self._hover
        self._refresh_style()                                                   # executem _refresh_style()
        super().leaveEvent(e)                                                   # executem super()

    def mousePressEvent(self, e):                                               # definim la funció mousePressEvent()
        if e.button() == Qt.LeftButton:                                         # si e.button() == Qt.LeftButton
            self.clicked.emit(self.mode["id"])                                  # executem emit()
        super().mousePressEvent(e)                                              # executem super()


# ==============================================================================
# SELECTOR HORITZONTAL
# ==============================================================================

class ModeSelector(QtWidgets.QFrame):                                           # definim la classe ModeSelector
    """
    Barra horitzontal amb les 5 targetes de mode, disposades en fila.
    Emet `modeChanged(str)` quan l'usuari canvia de mode.
    """

    modeChanged = Signal(str)                                                   # modeChanged ← resultat de Signal()

    HEIGHT = 58                                                                 # assignem HEIGHT

    def __init__(self, initial_mode="ik6", parent=None):                        # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setFixedHeight(self.HEIGHT)                                        # executem setFixedHeight()
        self.setStyleSheet(f"""
            ModeSelector {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)

        self._current = initial_mode                                            # assignem self._current
        self._cards = {}                                                        # assignem self._cards

        self._build_ui()                                                        # executem _build_ui()
        self.set_mode(initial_mode, silent=True)                                # assignem self.set_mode(initial_mode, silent

    def _build_ui(self):                                                        # definim la funció _build_ui()
        h = QtWidgets.QHBoxLayout(self)                                         # h ← resultat de QHBoxLayout()
        h.setContentsMargins(6, 4, 6, 4)                                        # executem setContentsMargins()
        h.setSpacing(4)                                                         # executem setSpacing()

        # Targetes distribuïdes uniformement
        for mode in MODES:                                                      # iterem: mode in MODES
            card = ModeCard(mode)                                               # card ← resultat de ModeCard()
            card.clicked.connect(self._on_card_clicked)                         # executem connect()
            self._cards[mode["id"]] = card                                      # self._cards[mode["id"]] = card
            h.addWidget(card, 1)   # stretch=1 → totes iguals

    # -----------------------------------------------------------
    def _on_card_clicked(self, mode_id):                                        # definim la funció _on_card_clicked()
        if mode_id == self._current:                                            # si mode_id == self._current
            return                                                              # retornem sense valor
        self.set_mode(mode_id)                                                  # executem set_mode()

    def set_mode(self, mode_id, silent=False):                                  # definim la funció set_mode()
        m = get_mode(mode_id)                                                   # m ← resultat de get_mode()
        if not m:                                                               # si not m
            return                                                              # retornem sense valor
        self._current = mode_id                                                 # assignem self._current

        for mid, card in self._cards.items():                                   # iterem: mid, card in self._cards.items()
            card.set_active(mid == mode_id)                                     # assignem card.set_active(mid

        if not silent:                                                          # si not silent
            self.modeChanged.emit(mode_id)                                      # executem emit()

    def current_mode(self):                                                     # definim la funció current_mode()
        return self._current                                                    # retornem: self._current

    # Tecles ràpides (1-5) es gestionen des del RobotWorkspace mitjançant
    # QShortcut que crida self.set_mode(...) directament.
