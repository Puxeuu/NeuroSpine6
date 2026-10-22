import os
import json

from qtpy import QtWidgets
from qtpy.QtCore import Qt

from .theme import (
    C_BG_WINDOW, C_PANEL_ALT, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED, C_ACCENT,
    SIZE_XS, SIZE_SM, SIZE_BASE, SIZE_LG, FONT_MONO,
)


DEFAULT_PREFS = {
    "mando_port":     "COM5",
    "mando_baudrate": 115200,
    "robot_port":     "tcp://10.42.0.2:5000",
    "pacient_id":     "839201",
    "auto_connect_mando": False,
}


def _prefs_path():
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    return os.path.join(root, "data", "preferences.json")


def load_preferences():
    path = _prefs_path()
    if not os.path.exists(path):
        return dict(DEFAULT_PREFS)

    try:
        with open(path, "r", encoding="utf-8") as f:
            saved = json.load(f)
        prefs = dict(DEFAULT_PREFS)
        prefs.update(saved)
        return prefs
    except Exception as e:
        print(f"[prefs] Error llegint {path}: {e}")
        return dict(DEFAULT_PREFS)


def save_preferences(prefs):
    path = _prefs_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(prefs, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"[prefs] Error guardant {path}: {e}")


class PreferencesDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Preferències")
        self.setMinimumWidth(520)
        self.setStyleSheet(f"""
            QDialog {{ background-color: {C_BG_WINDOW}; }}
            QWidget {{ color: {C_TEXT}; font-size: {SIZE_BASE}px; }}
            QLineEdit, QSpinBox, QComboBox {{
                background-color: {C_PANEL_ALT};
                border: 1px solid {C_BORDER_STRONG};
                border-radius: 6px;
                padding: 6px 10px;
                font-family: {FONT_MONO};
                font-size: {SIZE_SM}px;
            }}
            QLineEdit:focus, QSpinBox:focus, QComboBox:focus {{
                border: 1px solid {C_ACCENT};
            }}
            QCheckBox {{ spacing: 8px; }}
            QCheckBox::indicator {{
                width: 18px; height: 18px; border-radius: 4px;
                border: 1px solid #8e8e93; background: {C_PANEL_ALT};
            }}
            QCheckBox::indicator:checked {{
                background: {C_ACCENT}; border: 1px solid {C_ACCENT};
            }}
        """)

        self.prefs = load_preferences()

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(24, 24, 24, 20)
        v.setSpacing(16)

        title = QtWidgets.QLabel("PREFERÈNCIES")
        title.setStyleSheet(
            f"color: {C_TEXT}; font-size: {SIZE_LG}px; font-weight: 700; "
            f"letter-spacing: 1.5px;"
        )
        v.addWidget(title)

        sub = QtWidgets.QLabel(
            "Ajusta la configuració de l'aplicació. Els canvis es guarden "
            "automàticament i s'apliquen al reiniciar l'app."
        )
        sub.setWordWrap(True)
        sub.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: {SIZE_SM}px;")
        v.addWidget(sub)

        v.addWidget(self._divider())

        v.addWidget(self._section_title("CONNEXIONS HARDWARE"))

        form = QtWidgets.QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        form.setHorizontalSpacing(20)
        form.setVerticalSpacing(10)

        self.edit_mando_port = QtWidgets.QLineEdit(str(self.prefs["mando_port"]))
        self.edit_mando_port.setPlaceholderText("Ex: COM5")
        form.addRow(self._label("Port serie mando:"), self.edit_mando_port)

        self.spin_mando_baud = QtWidgets.QSpinBox()
        self.spin_mando_baud.setRange(9600, 921600)
        self.spin_mando_baud.setSingleStep(9600)
        self.spin_mando_baud.setValue(int(self.prefs["mando_baudrate"]))
        form.addRow(self._label("Baudrate mando:"), self.spin_mando_baud)

        self.edit_robot_port = QtWidgets.QLineEdit(str(self.prefs["robot_port"]))
        self.edit_robot_port.setPlaceholderText("Ex: tcp://10.42.0.2:5000 o /dev/cu.usbmodem...")
        form.addRow(self._label("Port serie robot:"), self.edit_robot_port)

        self.chk_auto_mando = QtWidgets.QCheckBox("Connectar el mando automàticament a l'arrencada")
        self.chk_auto_mando.setChecked(bool(self.prefs.get("auto_connect_mando", False)))
        form.addRow("", self.chk_auto_mando)

        v.addLayout(form)

        v.addWidget(self._divider())

        v.addWidget(self._section_title("PACIENT ACTIU"))

        form2 = QtWidgets.QFormLayout()
        form2.setHorizontalSpacing(20)
        form2.setVerticalSpacing(10)

        self.edit_pacient = QtWidgets.QLineEdit(str(self.prefs["pacient_id"]))
        form2.addRow(self._label("ID del pacient:"), self.edit_pacient)

        hint = QtWidgets.QLabel(
            "El pacient actiu determina on es busquen els fitxers TAC, "
            "STL de la vèrtebra, punts fiducials i matriu de registre "
            "(a data/pacients/{ID}/)."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-size: {SIZE_XS}px; "
            f"font-style: italic; padding-top: 6px;"
        )
        form2.addRow("", hint)
        v.addLayout(form2)

        v.addWidget(self._divider())

        info = QtWidgets.QLabel(
            "NeuroSpine Navigator · TFG Jordi Puxeu · UdG 2027"
        )
        info.setAlignment(Qt.AlignCenter)
        info.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-size: {SIZE_XS}px; "
            f"font-style: italic; padding: 6px;"
        )
        v.addWidget(info)

        v.addStretch()
        buttons = QtWidgets.QHBoxLayout()
        buttons.addStretch()

        self.btn_cancel = QtWidgets.QPushButton("Cancel·lar")
        self.btn_cancel.setStyleSheet(self._btn_secondary())
        self.btn_cancel.clicked.connect(self.reject)
        buttons.addWidget(self.btn_cancel)

        self.btn_save = QtWidgets.QPushButton("Guardar")
        self.btn_save.setStyleSheet(self._btn_primary())
        self.btn_save.clicked.connect(self._on_save)
        buttons.addWidget(self.btn_save)

        v.addLayout(buttons)

    def _label(self, text):
        lbl = QtWidgets.QLabel(text)
        lbl.setStyleSheet(
            f"color: {C_TEXT_DIM}; font-size: {SIZE_SM}px; font-weight: 600;"
        )
        return lbl

    def _section_title(self, text):
        lbl = QtWidgets.QLabel(text)
        lbl.setStyleSheet(
            f"color: {C_TEXT}; font-size: {SIZE_SM}px; font-weight: 700; "
            f"letter-spacing: 1.2px; padding-bottom: 4px;"
        )
        return lbl

    def _divider(self):
        line = QtWidgets.QFrame()
        line.setFixedHeight(1)
        line.setStyleSheet(f"background: {C_BORDER}; border: none;")
        return line

    def _btn_primary(self):
        return f"""
            QPushButton {{
                background-color: {C_ACCENT};
                color: white; border: none; border-radius: 6px;
                padding: 10px 22px;
                font-size: {SIZE_BASE}px; font-weight: 700;
                letter-spacing: 0.5px;
            }}
            QPushButton:hover {{ background-color: #0068d3; }}
        """

    def _btn_secondary(self):
        return f"""
            QPushButton {{
                background-color: {C_PANEL_ALT};
                color: {C_TEXT};
                border: 1px solid {C_BORDER_STRONG};
                border-radius: 6px;
                padding: 10px 22px;
                font-size: {SIZE_BASE}px; font-weight: 700;
            }}
            QPushButton:hover {{ background-color: #3a3a3c; }}
        """

    def _on_save(self):
        self.prefs["mando_port"]         = self.edit_mando_port.text().strip()
        self.prefs["mando_baudrate"]     = int(self.spin_mando_baud.value())
        self.prefs["robot_port"]         = self.edit_robot_port.text().strip()
        self.prefs["pacient_id"]         = self.edit_pacient.text().strip()
        self.prefs["auto_connect_mando"] = bool(self.chk_auto_mando.isChecked())
        save_preferences(self.prefs)

        QtWidgets.QMessageBox.information(
            self, "Preferències",
            "Les preferències s'han guardat.\n\n"
            "Alguns canvis (ports serie) requereixen reiniciar l'app "
            "per aplicar-se."
        )
        self.accept()

