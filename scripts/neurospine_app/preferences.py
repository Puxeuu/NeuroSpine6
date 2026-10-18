"""
Diàleg de preferències de NeuroSpine Navigator.

Permet configurar:
    · Port serie del mando
    · Port serie del robot
    · Baudrate del mando
    · Directori del pacient actiu (per canviar de cas)

Les preferències es guarden a data/preferences.json i es carreguen a
l'arrencada de l'app.
"""

import os                                                                       # importem el mòdul os
import json                                                                     # importem el mòdul json

from qtpy import QtWidgets                                                      # importem QtWidgets des de qtpy
from qtpy.QtCore import Qt                                                      # importem Qt des de qtpy.QtCore

from .theme import (                                                            # importem ( des de .theme
    C_BG_WINDOW, C_PANEL_ALT, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED, C_ACCENT,
    SIZE_XS, SIZE_SM, SIZE_BASE, SIZE_LG, FONT_MONO,
)


# ==============================================================================
# GESTIÓ DE PREFERÈNCIES
# ==============================================================================

DEFAULT_PREFS = {                                                               # assignem DEFAULT_PREFS
    "mando_port":     "COM5",
    "mando_baudrate": 115200,
    "robot_port":     "tcp://10.42.0.2:5000",
    "pacient_id":     "839201",
    "auto_connect_mando": False,
}


def _prefs_path():                                                              # definim la funció _prefs_path()
    root = os.path.dirname(os.path.dirname(os.path.dirname(                     # assignem root
        os.path.abspath(__file__))))
    return os.path.join(root, "data", "preferences.json")                       # retornem: os.path.join(root, "data", "preferenc...


def load_preferences():                                                         # definim la funció load_preferences()
    """Carrega les preferències des del JSON, o retorna els defaults."""        # """Carrega les preferències des del JSON, o retorna els d...
    path = _prefs_path()                                                        # path ← resultat de _prefs_path()
    if not os.path.exists(path):                                                # si not os.path.exists(path)
        return dict(DEFAULT_PREFS)                                              # retornem: dict(DEFAULT_PREFS)

    try:                                                                        # intentem el bloc següent
        with open(path, "r", encoding="utf-8") as f:                            # context: open(path, "r", encoding="utf-8") as f
            saved = json.load(f)                                                # saved ← resultat de load()
        prefs = dict(DEFAULT_PREFS)                                             # prefs ← resultat de dict()
        prefs.update(saved)                                                     # executem update()
        return prefs                                                            # retornem: prefs
    except Exception as e:                                                      # capturem excepció: Exception as e
        print(f"[prefs] Error llegint {path}: {e}")
        return dict(DEFAULT_PREFS)                                              # retornem: dict(DEFAULT_PREFS)


def save_preferences(prefs):                                                    # definim la funció save_preferences()
    """Guarda les preferències al JSON."""                                      # """Guarda les preferències al JSON."""
    path = _prefs_path()                                                        # path ← resultat de _prefs_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)                           # assignem os.makedirs(os.path.dirname(path), exist_ok
    try:                                                                        # intentem el bloc següent
        with open(path, "w", encoding="utf-8") as f:                            # context: open(path, "w", encoding="utf-8") as f
            json.dump(prefs, f, indent=2, ensure_ascii=False)                   # assignem json.dump(prefs, f, indent
    except Exception as e:                                                      # capturem excepció: Exception as e
        print(f"[prefs] Error guardant {path}: {e}")


# ==============================================================================
# DIÀLEG DE PREFERÈNCIES
# ==============================================================================

class PreferencesDialog(QtWidgets.QDialog):                                     # definim la classe PreferencesDialog

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setWindowTitle("Preferències")                                     # executem setWindowTitle()
        self.setMinimumWidth(520)                                               # executem setMinimumWidth()
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

        self.prefs = load_preferences()                                         # self.prefs ← resultat de load_preferences()

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(24, 24, 24, 20)                                    # executem setContentsMargins()
        v.setSpacing(16)                                                        # executem setSpacing()

        # Títol
        title = QtWidgets.QLabel("PREFERÈNCIES")                                # title ← resultat de QLabel()
        title.setStyleSheet(                                                    # executem setStyleSheet()
            f"color: {C_TEXT}; font-size: {SIZE_LG}px; font-weight: 700; "
            f"letter-spacing: 1.5px;"
        )
        v.addWidget(title)                                                      # executem addWidget()

        sub = QtWidgets.QLabel(                                                 # assignem sub
            "Ajusta la configuració de l'aplicació. Els canvis es guarden "
            "automàticament i s'apliquen al reiniciar l'app."
        )
        sub.setWordWrap(True)                                                   # executem setWordWrap()
        sub.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: {SIZE_SM}px;")
        v.addWidget(sub)                                                        # executem addWidget()

        # Separador
        v.addWidget(self._divider())                                            # executem addWidget()

        # === SECCIÓ: CONNEXIONS HARDWARE ===
        v.addWidget(self._section_title("CONNEXIONS HARDWARE"))                 # executem addWidget()

        form = QtWidgets.QFormLayout()                                          # form ← resultat de QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)                                    # executem setLabelAlignment()
        form.setHorizontalSpacing(20)                                           # executem setHorizontalSpacing()
        form.setVerticalSpacing(10)                                             # executem setVerticalSpacing()

        # Port mando
        self.edit_mando_port = QtWidgets.QLineEdit(str(self.prefs["mando_port"]))  # self.edit_mando_port ← resultat de QLineEdit()
        self.edit_mando_port.setPlaceholderText("Ex: COM5")                    # executem setPlaceholderText()
        form.addRow(self._label("Port serie mando:"), self.edit_mando_port)     # executem addRow()

        # Baudrate mando
        self.spin_mando_baud = QtWidgets.QSpinBox()                             # self.spin_mando_baud ← resultat de QSpinBox()
        self.spin_mando_baud.setRange(9600, 921600)                             # executem setRange()
        self.spin_mando_baud.setSingleStep(9600)                                # executem setSingleStep()
        self.spin_mando_baud.setValue(int(self.prefs["mando_baudrate"]))        # executem setValue()
        form.addRow(self._label("Baudrate mando:"), self.spin_mando_baud)       # executem addRow()

        # Port robot
        self.edit_robot_port = QtWidgets.QLineEdit(str(self.prefs["robot_port"]))  # self.edit_robot_port ← resultat de QLineEdit()
        self.edit_robot_port.setPlaceholderText("Ex: tcp://10.42.0.2:5000 o /dev/cu.usbmodem...")                    # executem setPlaceholderText()
        form.addRow(self._label("Port serie robot:"), self.edit_robot_port)     # executem addRow()

        # Auto-connect mando
        self.chk_auto_mando = QtWidgets.QCheckBox("Connectar el mando automàticament a l'arrencada")  # self.chk_auto_mando ← resultat de QCheckBox()
        self.chk_auto_mando.setChecked(bool(self.prefs.get("auto_connect_mando", False)))  # executem setChecked()
        form.addRow("", self.chk_auto_mando)                                    # executem addRow()

        v.addLayout(form)                                                       # executem addLayout()

        v.addWidget(self._divider())                                            # executem addWidget()

        # === SECCIÓ: PACIENT ===
        v.addWidget(self._section_title("PACIENT ACTIU"))                       # executem addWidget()

        form2 = QtWidgets.QFormLayout()                                         # form2 ← resultat de QFormLayout()
        form2.setHorizontalSpacing(20)                                          # executem setHorizontalSpacing()
        form2.setVerticalSpacing(10)                                            # executem setVerticalSpacing()

        self.edit_pacient = QtWidgets.QLineEdit(str(self.prefs["pacient_id"]))  # self.edit_pacient ← resultat de QLineEdit()
        form2.addRow(self._label("ID del pacient:"), self.edit_pacient)         # executem addRow()

        hint = QtWidgets.QLabel(                                                # assignem hint
            "El pacient actiu determina on es busquen els fitxers TAC, "
            "STL de la vèrtebra, punts fiducials i matriu de registre "
            "(a data/pacients/{ID}/)."
        )
        hint.setWordWrap(True)                                                  # executem setWordWrap()
        hint.setStyleSheet(                                                     # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-size: {SIZE_XS}px; "
            f"font-style: italic; padding-top: 6px;"
        )
        form2.addRow("", hint)                                                  # executem addRow()
        v.addLayout(form2)                                                      # executem addLayout()

        v.addWidget(self._divider())                                            # executem addWidget()

        # === INFO DE LA VERSIÓ ===
        info = QtWidgets.QLabel(                                                # assignem info
            "NeuroSpine Navigator · TFG Jordi Puxeu · UdG 2027"
        )
        info.setAlignment(Qt.AlignCenter)                                       # executem setAlignment()
        info.setStyleSheet(                                                     # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-size: {SIZE_XS}px; "
            f"font-style: italic; padding: 6px;"
        )
        v.addWidget(info)                                                       # executem addWidget()

        # === BOTONS ===
        v.addStretch()                                                          # executem addStretch()
        buttons = QtWidgets.QHBoxLayout()                                       # buttons ← resultat de QHBoxLayout()
        buttons.addStretch()                                                    # executem addStretch()

        self.btn_cancel = QtWidgets.QPushButton("Cancel·lar")                   # self.btn_cancel ← resultat de QPushButton()
        self.btn_cancel.setStyleSheet(self._btn_secondary())                    # executem setStyleSheet()
        self.btn_cancel.clicked.connect(self.reject)                            # executem connect()
        buttons.addWidget(self.btn_cancel)                                      # executem addWidget()

        self.btn_save = QtWidgets.QPushButton("Guardar")                        # self.btn_save ← resultat de QPushButton()
        self.btn_save.setStyleSheet(self._btn_primary())                        # executem setStyleSheet()
        self.btn_save.clicked.connect(self._on_save)                            # executem connect()
        buttons.addWidget(self.btn_save)                                        # executem addWidget()

        v.addLayout(buttons)                                                    # executem addLayout()

    # -----------------------------------------------------------
    def _label(self, text):                                                     # definim la funció _label()
        lbl = QtWidgets.QLabel(text)                                            # lbl ← resultat de QLabel()
        lbl.setStyleSheet(                                                      # executem setStyleSheet()
            f"color: {C_TEXT_DIM}; font-size: {SIZE_SM}px; font-weight: 600;"
        )
        return lbl                                                              # retornem: lbl

    def _section_title(self, text):                                             # definim la funció _section_title()
        lbl = QtWidgets.QLabel(text)                                            # lbl ← resultat de QLabel()
        lbl.setStyleSheet(                                                      # executem setStyleSheet()
            f"color: {C_TEXT}; font-size: {SIZE_SM}px; font-weight: 700; "
            f"letter-spacing: 1.2px; padding-bottom: 4px;"
        )
        return lbl                                                              # retornem: lbl

    def _divider(self):                                                         # definim la funció _divider()
        line = QtWidgets.QFrame()                                               # line ← resultat de QFrame()
        line.setFixedHeight(1)                                                  # executem setFixedHeight()
        line.setStyleSheet(f"background: {C_BORDER}; border: none;")
        return line                                                             # retornem: line

    def _btn_primary(self):                                                     # definim la funció _btn_primary()
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

    def _btn_secondary(self):                                                   # definim la funció _btn_secondary()
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

    # -----------------------------------------------------------
    def _on_save(self):                                                         # definim la funció _on_save()
        self.prefs["mando_port"]         = self.edit_mando_port.text().strip()  # self.prefs["mando_port"]         = self.edit_mando_port.t...
        self.prefs["mando_baudrate"]     = int(self.spin_mando_baud.value())    # self.prefs["mando_baudrate"]     = int(self.spin_mando_ba...
        self.prefs["robot_port"]         = self.edit_robot_port.text().strip()  # self.prefs["robot_port"]         = self.edit_robot_port.t...
        self.prefs["pacient_id"]         = self.edit_pacient.text().strip()     # self.prefs["pacient_id"]         = self.edit_pacient.text...
        self.prefs["auto_connect_mando"] = bool(self.chk_auto_mando.isChecked())  # self.prefs["auto_connect_mando"] = bool(self.chk_auto_man...
        save_preferences(self.prefs)                                            # executem save_preferences()

        # Missatge suau
        QtWidgets.QMessageBox.information(                                      # executem information()
            self, "Preferències",
            "Les preferències s'han guardat.\n\n"
            "Alguns canvis (ports serie) requereixen reiniciar l'app "
            "per aplicar-se."
        )
        self.accept()                                                           # executem accept()
