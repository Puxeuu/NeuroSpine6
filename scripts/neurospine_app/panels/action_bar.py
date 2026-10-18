"""
Barra d'accions del RobotWorkspace.

Disseny:
    - Compacte i minimalista.
    - Tots els botons tenen la mateixa alçada.
    - Estètica gris / negre coherent amb ViewerDLSPro.
    - Sense colors blau, verd o vermell.
    - Selector de capçal integrat en la mateixa barra.

Contingut:

    [ Connectar ] [ Mirror ] [ Home ] [ STOP ]    [ Collisions ]
    [ Target ]    [ Bola | Punxó | Broca ]
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
)


# ==============================================================================
# BOTÓ D'ACCIÓ
# ==============================================================================

class ActionButton(QtWidgets.QPushButton):                                      # definim la classe ActionButton
    """
    Botó compacte i uniforme.

    Tots els botons comparteixen:
        - mateixa alçada
        - mateixa tipografia
        - mateixa paleta
        - mateix border-radius
    """

    HEIGHT = 34                                                                 # assignem HEIGHT

    def __init__(self, text, icon=None, parent=None):                           # definim la funció __init__()
        super().__init__(parent)                                                # executem super()

        self.setText(                                                           # executem setText()
            f"{icon}  {text}" if icon else text
        )

        self.setCursor(                                                         # executem setCursor()
            Qt.PointingHandCursor
        )

        self.setCheckable(False)                                                # executem setCheckable()

        self.setFixedHeight(                                                    # executem setFixedHeight()
            self.HEIGHT
        )

        self.setMinimumWidth(84)                                                # executem setMinimumWidth()

        self._apply_style()                                                     # executem _apply_style()

    # --------------------------------------------------------------------------

    def _apply_style(self):                                                     # definim la funció _apply_style()

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

    # --------------------------------------------------------------------------

    def set_label(self, text):                                                  # definim la funció set_label()

        self.setText(text)                                                      # executem setText()


# ==============================================================================
# BARRA D'ACCIONS
# ==============================================================================

class ActionBar(QtWidgets.QFrame):                                              # definim la classe ActionBar
    """
    Barra horitzontal d'accions del RobotWorkspace.
    """

    HEIGHT = 52                                                                 # assignem HEIGHT

    currentsToggled = QtCore.Signal(bool)                                       # mostra / amaga el gràfic d'intensitats

    def __init__(self, viewer, parent=None):                                    # definim la funció __init__()

        super().__init__(parent)                                                # executem super()

        self.viewer = viewer                                                    # assignem self.viewer

        self.setObjectName(                                                     # executem setObjectName()
            "ActionBar"
        )

        self.setFixedHeight(                                                    # executem setFixedHeight()
            self.HEIGHT
        )

        self.setStyleSheet(f"""
            QFrame#ActionBar {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)

        # ----------------------------------------------------------------------
        # LAYOUT
        # ----------------------------------------------------------------------

        h = QtWidgets.QHBoxLayout(self)                                         # h ← resultat de QHBoxLayout()

        h.setContentsMargins(                                                   # executem setContentsMargins()
            8, 8, 8, 8
        )

        h.setSpacing(6)                                                         # executem setSpacing()

        # ======================================================================
        # ESQUERRA — CONTROL DEL ROBOT
        # ======================================================================

        self.btn_connect = ActionButton(                                        # assignem self.btn_connect
            "Connectar"
        )

        self.btn_connect.clicked.connect(                                       # executem connect()
            self._on_connect
        )

        h.addWidget(                                                            # executem addWidget()
            self.btn_connect
        )

        # ----------------------------------------------------------------------

        self.btn_mirror = ActionButton(                                         # assignem self.btn_mirror
            "Mirror"
        )

        self.btn_mirror.setEnabled(                                             # executem setEnabled()
            False
        )

        self.btn_mirror.clicked.connect(                                        # executem connect()
            self._on_mirror
        )

        h.addWidget(                                                            # executem addWidget()
            self.btn_mirror
        )

        # ----------------------------------------------------------------------

        self.btn_home = ActionButton(                                           # assignem self.btn_home
            "Home"
        )

        self.btn_home.clicked.connect(                                          # executem connect()
            self._on_home
        )

        h.addWidget(                                                            # executem addWidget()
            self.btn_home
        )

        # ----------------------------------------------------------------------
        # SEPARADOR
        # ----------------------------------------------------------------------

        h.addWidget(                                                            # executem addWidget()
            self._separator()
        )

        # ----------------------------------------------------------------------
        # STOP
        # ----------------------------------------------------------------------

        self.btn_stop = ActionButton("STOP")                                    # self.btn_stop ← resultat de ActionButton()

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

        self.btn_stop.clicked.connect(self._on_stop)                            # executem connect()

        self.btn_stop.clicked.connect(                                          # executem connect()
            self._on_stop
        )

        h.addWidget(                                                            # executem addWidget()
            self.btn_stop
        )

        # ======================================================================
        # ESPAI
        # ======================================================================

        h.addStretch()                                                          # executem addStretch()

        # ======================================================================
        # DRETA — OPCIONS
        # ======================================================================

        self.btn_collisions = ActionButton(                                     # assignem self.btn_collisions
            "Collisions"
        )

        self.btn_collisions.setCheckable(                                       # executem setCheckable()
            True
        )

        chk = getattr(                                                          # assignem chk
            viewer,
            "chk_collisions",
            None
        )

        if chk is not None:                                                     # si chk is not None

            self.btn_collisions.setChecked(                                     # executem setChecked()
                chk.isChecked()
            )

        self.btn_collisions.clicked.connect(                                    # executem connect()
            self._on_toggle_collisions
        )

        h.addWidget(                                                            # executem addWidget()
            self.btn_collisions
        )

        # ----------------------------------------------------------------------

        self.btn_target = ActionButton(                                         # assignem self.btn_target
            "Target"
        )

        self.btn_target.setCheckable(                                           # executem setCheckable()
            True
        )

        chk_target = getattr(                                                   # assignem chk_target
            viewer,
            "chk_target",
            None
        )

        if chk_target is not None:                                              # si chk_target is not None

            self.btn_target.setChecked(                                         # executem setChecked()
                chk_target.isChecked()
            )

        self.btn_target.clicked.connect(                                        # executem connect()
            self._on_toggle_target
        )

        h.addWidget(                                                            # executem addWidget()
            self.btn_target
        )

        # ----------------------------------------------------------------------

        self.btn_currents = ActionButton(                                       # gràfic d'intensitats dels motors
            "Intensitats"
        )

        self.btn_currents.setCheckable(True)                                    # executem setCheckable()
        self.btn_currents.setToolTip("Gràfic del corrent de tots els motors (I)")  # executem setToolTip()

        self.btn_currents.toggled.connect(                                      # executem connect()
            self.currentsToggled.emit
        )

        h.addWidget(                                                            # executem addWidget()
            self.btn_currents
        )

        # ======================================================================
        # SELECTOR DE CAPÇAL
        # ======================================================================

        self.capcal_frame = QtWidgets.QFrame()                                  # self.capcal_frame ← resultat de QFrame()

        self.capcal_frame.setFixedHeight(                                       # executem setFixedHeight()
            ActionButton.HEIGHT
        )

        # Vora VERMELLA perquè l'operador vegi clarament aquest selector:
        # cal confirmar quin TCP (bola/punxó/broca) està muntat físicament
        # al robot. S'evita així haver de mostrar un modal al principi.
        self.capcal_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {C_PANEL_ALT};
                border: 1.5px solid #ff3b30;
                border-radius: 7px;
            }}
        """)

        capcal_layout = QtWidgets.QHBoxLayout(                                  # assignem capcal_layout
            self.capcal_frame
        )

        capcal_layout.setContentsMargins(                                       # executem setContentsMargins()
            3, 3, 3, 3
        )

        capcal_layout.setSpacing(                                               # executem setSpacing()
            2
        )

        capcal_info = {                                                         # assignem capcal_info
            "bola": "Bola",
            "punxo": "Punxó",
            "broca": "Broca",
        }

        self.capcal_buttons = {}                                                # assignem self.capcal_buttons

        for cap_id, label in capcal_info.items():                               # iterem: cap_id, label in capcal_info.items()

            button = QtWidgets.QPushButton(                                     # assignem button
                label
            )

            button.setCheckable(                                                # executem setCheckable()
                True
            )

            button.setFixedHeight(                                              # executem setFixedHeight()
                26
            )

            button.setCursor(                                                   # executem setCursor()
                Qt.PointingHandCursor
            )

            # Botó actiu (checked) també en vermell, consistent amb la vora
            # del recuadre: així es veu clarament quin capçal està seleccionat.
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

            button.clicked.connect(                                             # executem connect()
                lambda checked,
                cid=cap_id:
                self._on_capcal_button_clicked(cid)
            )

            self.capcal_buttons[                                                # self.capcal_buttons[
                cap_id
            ] = button

            capcal_layout.addWidget(                                            # executem addWidget()
                button
            )

        # ----------------------------------------------------------------------
        # CAPÇAL ACTIU INICIAL
        # ----------------------------------------------------------------------

        manager = getattr(                                                      # assignem manager
            self.viewer,
            "capcal_manager",
            None
        )

        if manager is not None:                                                 # si manager is not None

            active = getattr(                                                   # assignem active
                manager,
                "active_nom",
                "bola"
            )

            if active in self.capcal_buttons:                                   # si active in self.capcal_buttons

                self.capcal_buttons[                                            # self.capcal_buttons[
                    active
                ].setChecked(True)

        else:                                                                   # en cas contrari

            self.capcal_buttons[                                                # self.capcal_buttons[
                "bola"
            ].setChecked(True)

        h.addWidget(                                                            # executem addWidget()
            self.capcal_frame
        )

        # ======================================================================
        # TIMER DE SINCRONITZACIÓ
        # ======================================================================

        self._timer = QtCore.QTimer(                                            # assignem self._timer
            self
        )

        self._timer.timeout.connect(                                            # executem connect()
            self._refresh_state
        )

        self._timer.start(                                                      # executem start()
            400
        )

    # ==========================================================================
    # HELPERS
    # ==========================================================================

    def _separator(self):                                                       # definim la funció _separator()

        sep = QtWidgets.QFrame()                                                # sep ← resultat de QFrame()

        sep.setFixedWidth(                                                      # executem setFixedWidth()
            1
        )

        sep.setFixedHeight(                                                     # executem setFixedHeight()
            24
        )

        sep.setStyleSheet(                                                      # executem setStyleSheet()
            f"""
            QFrame {{
                background-color: {C_BORDER};
                border: none;
            }}
            """
        )

        return sep                                                              # retornem: sep

    # ==========================================================================
    # CAPÇALS
    # ==========================================================================

    def _on_capcal_button_clicked(                                              # definim la funció _on_capcal_button_clicked()
        self,
        cap_id
    ):

        # Només un capçal actiu
        for cid, button in self.capcal_buttons.items():                         # iterem: cid, button in self.capcal_buttons.it...

            button.setChecked(                                                  # executem setChecked()
                cid == cap_id
            )

        # Aplica el canvi al selector intern
        selector = getattr(                                                     # assignem selector
            self.viewer,
            "capcal_selector",
            None
        )

        if selector is not None:                                                # si selector is not None

            if hasattr(                                                         # si hasattr(
                selector,
                "cards"
            ):

                try:                                                            # intentem el bloc següent

                    for card_id, card in selector.cards.items():                # iterem: card_id, card in selector.cards.items()

                        card.setChecked(                                        # executem setChecked()
                            card_id == cap_id
                        )

                except Exception as e:                                          # capturem excepció: Exception as e

                    print(                                                      # executem print()
                        f"[actionbar] capcal cards: {e}"
                    )

            if hasattr(                                                         # si hasattr(
                selector,
                "capcalChanged"
            ):

                try:                                                            # intentem el bloc següent

                    selector.capcalChanged.emit(                                # executem emit()
                        cap_id
                    )

                except Exception as e:                                          # capturem excepció: Exception as e

                    print(                                                      # executem print()
                        f"[actionbar] capcal switch: {e}"
                    )

    # ==========================================================================
    # BOTONS PRINCIPALS
    # ==========================================================================

    def _on_connect(self):                                                      # definim la funció _on_connect()

        button = getattr(                                                       # assignem button
            self.viewer,
            "btn_connect",
            None
        )

        if button is None:                                                      # si button is None
            return                                                              # retornem sense valor

        button.setChecked(                                                      # executem setChecked()
            not button.isChecked()
        )

        if hasattr(                                                             # si hasattr(
            self.viewer,
            "_on_connect_motors"
        ):

            self.viewer._on_connect_motors()                                    # executem _on_connect_motors()

    # --------------------------------------------------------------------------

    def _on_mirror(self):                                                       # definim la funció _on_mirror()

        button = getattr(                                                       # assignem button
            self.viewer,
            "btn_mirror",
            None
        )

        if button is None:                                                      # si button is None
            return                                                              # retornem sense valor

        if not button.isEnabled():                                              # si not button.isEnabled()
            return                                                              # retornem sense valor

        button.setChecked(                                                      # executem setChecked()
            not button.isChecked()
        )

        if hasattr(                                                             # si hasattr(
            self.viewer,
            "_on_mirror_toggle"
        ):

            self.viewer._on_mirror_toggle()                                     # executem _on_mirror_toggle()

    # --------------------------------------------------------------------------

    def _on_home(self):                                                         # definim la funció _on_home()

        if hasattr(                                                             # si hasattr(
            self.viewer,
            "_on_move_to_zero"
        ):

            self.viewer._on_move_to_zero()                                      # executem _on_move_to_zero()

    # --------------------------------------------------------------------------

    def _on_stop(self):                                                         # definim la funció _on_stop()

        if hasattr(                                                             # si hasattr(
            self.viewer,
            "_on_stop"
        ):

            self.viewer._on_stop()                                              # executem _on_stop()

    # ==========================================================================
    # TOGGLES
    # ==========================================================================

    def _on_toggle_collisions(self):                                            # definim la funció _on_toggle_collisions()

        checkbox = getattr(                                                     # assignem checkbox
            self.viewer,
            "chk_collisions",
            None
        )

        if checkbox is None:                                                    # si checkbox is None
            return                                                              # retornem sense valor

        checkbox.setChecked(                                                    # executem setChecked()
            self.btn_collisions.isChecked()
        )

    # --------------------------------------------------------------------------

    def _on_toggle_target(self):                                                # definim la funció _on_toggle_target()

        checkbox = getattr(                                                     # assignem checkbox
            self.viewer,
            "chk_target",
            None
        )

        if checkbox is None:                                                    # si checkbox is None
            return                                                              # retornem sense valor

        value = self.btn_target.isChecked()                                     # value ← resultat de isChecked()

        checkbox.setChecked(                                                    # executem setChecked()
            value
        )

        if hasattr(                                                             # si hasattr(
            self.viewer,
            "_on_target_visibility"
        ):

            self.viewer._on_target_visibility(                                  # executem _on_target_visibility()
                value
            )

    # ==========================================================================
    # SINCRONITZACIÓ
    # ==========================================================================

    def _refresh_state(self):                                                   # definim la funció _refresh_state()

        # ----------------------------------------------------------------------
        # CONNECT
        # ----------------------------------------------------------------------

        button = getattr(                                                       # assignem button
            self.viewer,
            "btn_connect",
            None
        )

        if button is not None:                                                  # si button is not None

            connected = button.isChecked()                                      # connected ← resultat de isChecked()

            self.btn_connect.setText(                                           # executem setText()
                "Desconnectar"
                if connected
                else "Connectar"
            )

            self.btn_mirror.setEnabled(                                         # executem setEnabled()
                button.isChecked()
            )

        # ----------------------------------------------------------------------
        # MIRROR
        # ----------------------------------------------------------------------

        mirror = getattr(                                                       # assignem mirror
            self.viewer,
            "btn_mirror",
            None
        )

        if mirror is not None:                                                  # si mirror is not None

            self.btn_mirror.setText(                                            # executem setText()
                "Mirror ON"
                if mirror.isChecked()
                else "Mirror"
            )

            self.btn_mirror.setChecked(                                         # executem setChecked()
                mirror.isChecked()
            )

        # ----------------------------------------------------------------------
        # COLLISIONS
        # ----------------------------------------------------------------------

        collisions = getattr(                                                   # assignem collisions
            self.viewer,
            "chk_collisions",
            None
        )

        if collisions is not None:                                              # si collisions is not None

            self.btn_collisions.setChecked(                                     # executem setChecked()
                collisions.isChecked()
            )

        # ----------------------------------------------------------------------
        # TARGET
        # ----------------------------------------------------------------------

        target = getattr(                                                       # assignem target
            self.viewer,
            "chk_target",
            None
        )

        if target is not None:                                                  # si target is not None

            self.btn_target.setChecked(                                         # executem setChecked()
                target.isChecked()
            )