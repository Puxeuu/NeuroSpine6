"""
Header superior de l'aplicació.

Sempre visible, mostra:
    - Logo + nom aplicació
    - Breadcrumb del workspace actiu
    - Info pacient
    - Pastilles d'estat: MANDO, ROBOT, REGISTRE
    - Botó Emergency Stop
"""

from qtpy import QtWidgets, QtCore                                              # importem QtWidgets, QtCore des de qtpy

from .theme import (                                                            # importem ( des de .theme
    C_BG_HEADER, C_BORDER, C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_ACCENT,
    SIZE_SM, SIZE_BASE,
    FONT_MONO, style_pill,
)


class Header(QtWidgets.QFrame):                                                 # definim la classe Header

    HEIGHT = 56                                                                 # assignem HEIGHT
    emergencyStop = QtCore.Signal()                                             # emergencyStop ← resultat de Signal()

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setFixedHeight(self.HEIGHT)                                        # executem setFixedHeight()
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_HEADER};
                border: none;
                border-bottom: 1px solid {C_BORDER};
            }}
        """)

        layout = QtWidgets.QHBoxLayout(self)                                    # layout ← resultat de QHBoxLayout()
        layout.setContentsMargins(20, 0, 20, 0)                                 # executem setContentsMargins()
        layout.setSpacing(16)                                                   # executem setSpacing()

        # ---- Logo + nom app ----
        logo = QtWidgets.QLabel("◈")                                            # logo ← resultat de QLabel()
        logo.setStyleSheet(f"color: {C_ACCENT}; font-size: 22px;")
        layout.addWidget(logo)                                                  # executem addWidget()

        app_name = QtWidgets.QLabel("NEUROSPINE NAVIGATOR")                     # app_name ← resultat de QLabel()
        app_name.setStyleSheet(                                                 # executem setStyleSheet()
            f"color: {C_TEXT}; font-size: {SIZE_BASE}px; font-weight: 700; "
            f"letter-spacing: 1.5px;"
        )
        layout.addWidget(app_name)                                              # executem addWidget()

        # ---- Breadcrumb del workspace ----
        self.workspace_label = QtWidgets.QLabel("")                             # self.workspace_label ← resultat de QLabel()
        self.workspace_label.setStyleSheet(                                     # executem setStyleSheet()
            f"color: {C_TEXT_DIM}; font-size: {SIZE_SM}px; font-weight: 500; "
            f"letter-spacing: 1px; padding-left: 12px; margin-left: 8px; "
            f"border-left: 1px solid {C_BORDER};"
        )
        layout.addWidget(self.workspace_label)                                  # executem addWidget()

        layout.addStretch()                                                     # executem addStretch()

        # ---- Info pacient ----
        self.patient_label = QtWidgets.QLabel("PACIENT  ID 839201")             # self.patient_label ← resultat de QLabel()
        self.patient_label.setStyleSheet(                                       # executem setStyleSheet()
            f"color: {C_TEXT_DIM}; font-size: {SIZE_SM}px; font-weight: 500; "
            f"letter-spacing: 0.5px; font-family: {FONT_MONO};"
        )
        layout.addWidget(self.patient_label)                                    # executem addWidget()

        # ---- Pastilles d'estat ----
        self.status_mando = QtWidgets.QLabel("●  MANDO")                        # self.status_mando ← resultat de QLabel()
        self.status_robot = QtWidgets.QLabel("●  ROBOT")                        # self.status_robot ← resultat de QLabel()
        self.status_reg   = QtWidgets.QLabel("●  REGISTRE")                     # self.status_reg ← resultat de QLabel()

        self.set_mando_status(False)                                            # executem set_mando_status()
        self.set_robot_status(False)                                            # executem set_robot_status()
        self.set_registration_status(False)                                     # executem set_registration_status()

        layout.addWidget(self.status_mando)                                     # executem addWidget()
        layout.addWidget(self.status_robot)                                     # executem addWidget()
        layout.addWidget(self.status_reg)                                       # executem addWidget()

        # Emergency Stop tret: no ho volem al header (l'stop lògic el
        # tenim a l'ActionBar de cada workspace, i l'stop físic és el
        # bot vermell hardware de la mesa).

    # --------------------------------------------------------------------
    def set_workspace_name(self, name):                                         # definim la funció set_workspace_name()
        self.workspace_label.setText(name.upper())                              # executem setText()

    def set_patient(self, patient_id, procediment=""):                          # definim la funció set_patient()
        text = f"PACIENT  {patient_id}"
        if procediment:                                                         # si procediment
            text += f"   ·   {procediment.upper()}"
        self.patient_label.setText(text)                                        # executem setText()

    def set_mando_status(self, connected):                                      # definim la funció set_mando_status()
        # Sense colors verds/vermells: només fem servir opacitat/intensitat
        # perquè el header quedi net i minimal. Quan està connectat es veu
        # més clar (text blanc), quan no, es difumina (gris fosc).
        color = C_TEXT if connected else C_TEXT_MUTED                           # assignem color
        self.status_mando.setStyleSheet(style_pill(color))                      # executem setStyleSheet()

    def set_robot_status(self, connected):                                      # definim la funció set_robot_status()
        color = C_TEXT if connected else C_TEXT_MUTED                           # assignem color
        self.status_robot.setStyleSheet(style_pill(color))                      # executem setStyleSheet()

    def set_registration_status(self, ok):                                      # definim la funció set_registration_status()
        color = C_TEXT if ok else C_TEXT_MUTED                                  # assignem color
        self.status_reg.setStyleSheet(style_pill(color))                        # executem setStyleSheet()
