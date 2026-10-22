from qtpy import QtWidgets, QtCore

from .theme import (
    C_BG_HEADER, C_BORDER, C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_ACCENT,
    SIZE_SM, SIZE_BASE,
    FONT_MONO, style_pill,
)


class Header(QtWidgets.QFrame):
    HEIGHT = 56
    emergencyStop = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(self.HEIGHT)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_HEADER};
                border: none;
                border-bottom: 1px solid {C_BORDER};
            }}
        """)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(20, 0, 20, 0)
        layout.setSpacing(16)

        logo = QtWidgets.QLabel("◈")
        logo.setStyleSheet(f"color: {C_ACCENT}; font-size: 22px;")
        layout.addWidget(logo)

        app_name = QtWidgets.QLabel("NEUROSPINE NAVIGATOR")
        app_name.setStyleSheet(
            f"color: {C_TEXT}; font-size: {SIZE_BASE}px; font-weight: 700; "
            f"letter-spacing: 1.5px;"
        )
        layout.addWidget(app_name)

        self.workspace_label = QtWidgets.QLabel("")
        self.workspace_label.setStyleSheet(
            f"color: {C_TEXT_DIM}; font-size: {SIZE_SM}px; font-weight: 500; "
            f"letter-spacing: 1px; padding-left: 12px; margin-left: 8px; "
            f"border-left: 1px solid {C_BORDER};"
        )
        layout.addWidget(self.workspace_label)

        layout.addStretch()

        self.patient_label = QtWidgets.QLabel("PACIENT  ID 839201")
        self.patient_label.setStyleSheet(
            f"color: {C_TEXT_DIM}; font-size: {SIZE_SM}px; font-weight: 500; "
            f"letter-spacing: 0.5px; font-family: {FONT_MONO};"
        )
        layout.addWidget(self.patient_label)

        self.status_mando = QtWidgets.QLabel("●  MANDO")
        self.status_robot = QtWidgets.QLabel("●  ROBOT")
        self.status_reg   = QtWidgets.QLabel("●  REGISTRE")

        self.set_mando_status(False)
        self.set_robot_status(False)
        self.set_registration_status(False)

        layout.addWidget(self.status_mando)
        layout.addWidget(self.status_robot)
        layout.addWidget(self.status_reg)

    def set_workspace_name(self, name):
        self.workspace_label.setText(name.upper())

    def set_patient(self, patient_id, procediment=""):
        text = f"PACIENT  {patient_id}"
        if procediment:
            text += f"   ·   {procediment.upper()}"
        self.patient_label.setText(text)

    def set_mando_status(self, connected):
        color = C_TEXT if connected else C_TEXT_MUTED
        self.status_mando.setStyleSheet(style_pill(color))

    def set_robot_status(self, connected):
        color = C_TEXT if connected else C_TEXT_MUTED
        self.status_robot.setStyleSheet(style_pill(color))

    def set_registration_status(self, ok):
        color = C_TEXT if ok else C_TEXT_MUTED
        self.status_reg.setStyleSheet(style_pill(color))

