"""
Workspace de Navegació quirúrgica — versió completa.

És la vista OPERATIVA REAL del sistema: el que veurà el cirurgià durant
la col·locació dels cargols. Combina tot el que s'ha construït als
altres workspaces (planning, register, robot) en una UI minimalista
pensada per operar amb seguretat.

Layout (3 columnes):

    ┌─────────────┬──────────────────────────────────┬──────────────┐
    │             │                                  │              │
    │   HUD       │                                  │   PLA        │
    │   TELE-     │       VISTA 3D CENTRAL           │   QUIRÚRGIC  │
    │   METRIA    │       amb "corredor" del         │   Cargols +  │
    │             │       cargol actiu               │   estat +    │
    │   Mode      │                                  │   distància  │
    │   Precisió  │                                  │              │
    │   Estat     │                                  │   AVANÇAR    │
    │             │                                  │   RETROCEDIR │
    └─────────────┴──────────────────────────────────┴──────────────┘

L'estat "cargol actiu", la distància al head, els errors angulars, etc.
són números calculats a temps real a partir de:
    - viewer.poses["tcp_link"]           (pose actual del TCP)
    - robot_workspace._reg_screws_m      (cargols registrats)
    - self._active_idx                   (cargol seleccionat)
"""

import numpy as np                                                              # importem el mòdul numpy as np
from qtpy import QtWidgets, QtCore, QtGui                                       # importem QtWidgets, QtCore, QtGui des de qtpy

from ..theme import (                                                           # importem ( des de ..theme
    C_BG_WINDOW, C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_ACCENT, C_SAFE, C_WARN, C_DANGER,
    SIZE_XS, SIZE_BASE, FONT_MONO,
)


# ==============================================================================
# TOLERÀNCIES DE GUIAT (verd < OK ≤ groc < WARN ≤ vermell)
# ==============================================================================

TOL_OK_MM    = 1.0                                                              # desviació lateral de la punta (mm)
TOL_WARN_MM  = 2.0                                                              # assignem TOL_WARN_MM
TOL_OK_DEG   = 1.0                                                              # error angular de l'eix de la broca (°)
TOL_WARN_DEG = 2.0                                                              # assignem TOL_WARN_DEG
TAIL_MM      = 40.0                                                             # la "cua" de la mira: 4 cm enrere sobre l'eix de la broca
APPROACH_MM  = 10.0                                                             # l'approach és a 1 cm de l'entrada (robot.py)
DEPTH_STEP_MM  = 0.1                                                            # ④ Avanç: mida de cada pas (abans 0.5 → 5× més fi)
DEPTH_EXTRA_MM = 5.0                                                            # ④ Avanç: recorregut més enllà del final del cargol
_DEPTH_STEPS_PER_MM = round(1.0 / DEPTH_STEP_MM)                                # l'slider treballa en passos enters


def _tol_color(value, ok, warn):                                                # definim la funció _tol_color()
    """Color d'estat segons una tolerància."""
    if value < ok:                                                              # si value < ok
        return C_SAFE                                                           # retornem: C_SAFE
    if value < warn:                                                            # si value < warn
        return C_WARN                                                           # retornem: C_WARN
    return C_DANGER                                                             # retornem: C_DANGER


# ==============================================================================
# ESTIL BASE DELS "SEGMENTS" DEL HUD
# ==============================================================================

def _segment_frame(bg="#0e0e10", border_alpha=0.10):                            # definim la funció _segment_frame()
    """Frame base d'una secció del HUD (fons molt fosc, borde subtil)."""       # """Frame base d'una secció del HUD (fons molt fosc, borde...
    f = QtWidgets.QFrame()                                                      # f ← resultat de QFrame()
    f.setStyleSheet(f"""
        QFrame {{
            background-color: {bg};
            border: 1px solid rgba(255, 255, 255, {border_alpha});
            border-radius: 14px;
        }}
    """)
    return f                                                                    # retornem: f


def _mono_label(text, size=28, color="#ffffff", weight=700, letter=1):          # definim la funció _mono_label()
    lbl = QtWidgets.QLabel(text)                                                # lbl ← resultat de QLabel()
    lbl.setAlignment(QtCore.Qt.AlignCenter)                                     # executem setAlignment()
    lbl.setStyleSheet(                                                          # executem setStyleSheet()
        f"color: {color}; font-family: {FONT_MONO}; "
        f"font-size: {size}px; font-weight: {weight}; "
        f"letter-spacing: {letter}px; background: transparent; border: none;"
    )
    return lbl                                                                  # retornem: lbl


def _eyebrow(text, color=None):                                                 # definim la funció _eyebrow()
    if color is None:                                                           # si color is None
        color = C_TEXT_MUTED                                                    # assignem color
    lbl = QtWidgets.QLabel(text.upper())                                        # lbl ← resultat de QLabel()
    lbl.setAlignment(QtCore.Qt.AlignCenter)                                     # executem setAlignment()
    lbl.setStyleSheet(                                                          # executem setStyleSheet()
        f"color: {color}; font-size: 10px; font-weight: 700; "
        f"letter-spacing: 3px; background: transparent; border: none;"
    )
    return lbl                                                                  # retornem: lbl


# ==============================================================================
# HUD LATERAL ESQUERRA — TELEMETRIA
# ==============================================================================

class TelemetryHUD(QtWidgets.QWidget):                                          # definim la classe TelemetryHUD
    """
    Columna esquerra amb 4 segments cinemàtics:

        [ MODE OPERATIU ]
        [ PRECISIÓ DE POSICIÓ ]
        [ PRECISIÓ ANGULAR ]
        [ ESTAT DEL SISTEMA ]
    """

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setFixedWidth(280)                                                 # executem setFixedWidth()
        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(16, 16, 8, 16)                                     # executem setContentsMargins()
        v.setSpacing(14)                                                        # executem setSpacing()

        # ---- Segment 1: Mode operatiu ------------------------------------
        seg_mode = _segment_frame()                                             # seg_mode ← resultat de _segment_frame()
        seg_mode.setFixedHeight(110)                                            # executem setFixedHeight()
        m = QtWidgets.QVBoxLayout(seg_mode)                                     # m ← resultat de QVBoxLayout()
        m.setContentsMargins(20, 18, 20, 18)                                    # executem setContentsMargins()
        m.setSpacing(6)                                                         # executem setSpacing()
        m.addWidget(_eyebrow("MODE", color=C_ACCENT))                           # executem addWidget()
        m.addSpacing(6)                                                         # executem addSpacing()
        self.lbl_mode = _mono_label("STANDBY", size=22, letter=3)               # self.lbl_mode ← resultat de _mono_label()
        m.addWidget(self.lbl_mode)                                              # executem addWidget()
        self.lbl_mode_hint = QtWidgets.QLabel("Esperant selecció de cargol")    # self.lbl_mode_hint ← resultat de QLabel()
        self.lbl_mode_hint.setAlignment(QtCore.Qt.AlignCenter)                  # executem setAlignment()
        self.lbl_mode_hint.setStyleSheet(                                       # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-size: 10px; "
            f"letter-spacing: 0.5px; background: transparent; border: none;"
        )
        m.addWidget(self.lbl_mode_hint)                                         # executem addWidget()
        v.addWidget(seg_mode)                                                   # executem addWidget()

        # ---- Segment 2: Precisió posició --------------------------------
        seg_pos = _segment_frame()                                              # seg_pos ← resultat de _segment_frame()
        seg_pos.setFixedHeight(160)                                             # executem setFixedHeight()
        p = QtWidgets.QVBoxLayout(seg_pos)                                      # p ← resultat de QVBoxLayout()
        p.setContentsMargins(20, 18, 20, 18)                                    # executem setContentsMargins()
        p.setSpacing(4)                                                         # executem setSpacing()
        p.addWidget(_eyebrow("DESVIACIÓ LATERAL"))                              # punta ↔ trajectòria planificada
        p.addSpacing(8)                                                         # executem addSpacing()

        # Nombre gran
        row_num = QtWidgets.QHBoxLayout()                                       # row_num ← resultat de QHBoxLayout()
        row_num.setSpacing(4)                                                   # executem setSpacing()
        row_num.setAlignment(QtCore.Qt.AlignCenter)                             # executem setAlignment()
        self.lbl_dist_num = _mono_label("—", size=44, letter=-1)                # self.lbl_dist_num ← resultat de _mono_label()
        row_num.addWidget(self.lbl_dist_num)                                    # executem addWidget()
        self.lbl_dist_unit = QtWidgets.QLabel("mm")                             # self.lbl_dist_unit ← resultat de QLabel()
        self.lbl_dist_unit.setStyleSheet(                                       # executem setStyleSheet()
            f"color: {C_TEXT_DIM}; font-size: 14px; font-weight: 500; "
            f"letter-spacing: 1px; background: transparent; border: none; "
            f"padding-top: 22px;"
        )
        row_num.addWidget(self.lbl_dist_unit)                                   # executem addWidget()
        p.addLayout(row_num)                                                    # executem addLayout()

        # Progressbar visual
        self.pb_dist = QtWidgets.QProgressBar()                                 # self.pb_dist ← resultat de QProgressBar()
        self.pb_dist.setRange(0, 100)                                           # executem setRange()
        self.pb_dist.setValue(0)                                                # executem setValue()
        self.pb_dist.setTextVisible(False)                                      # executem setTextVisible()
        self.pb_dist.setFixedHeight(4)                                          # executem setFixedHeight()
        self.pb_dist.setStyleSheet(f"""
            QProgressBar {{
                background-color: rgba(255, 255, 255, 0.08);
                border: none;
                border-radius: 2px;
            }}
            QProgressBar::chunk {{
                background-color: {C_ACCENT};
                border-radius: 2px;
            }}
        """)
        p.addWidget(self.pb_dist)                                               # executem addWidget()
        p.addSpacing(4)                                                         # executem addSpacing()
        self.lbl_dist_hint = QtWidgets.QLabel("Sense cargol seleccionat")       # self.lbl_dist_hint ← resultat de QLabel()
        self.lbl_dist_hint.setAlignment(QtCore.Qt.AlignCenter)                  # executem setAlignment()
        self.lbl_dist_hint.setStyleSheet(                                       # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-size: 10px; "
            f"letter-spacing: 0.5px; background: transparent; border: none;"
        )
        p.addWidget(self.lbl_dist_hint)                                         # executem addWidget()

        v.addWidget(seg_pos)                                                    # executem addWidget()

        # ---- Segment 3: Precisió angular --------------------------------
        seg_ang = _segment_frame()                                              # seg_ang ← resultat de _segment_frame()
        seg_ang.setFixedHeight(160)                                             # executem setFixedHeight()
        a = QtWidgets.QVBoxLayout(seg_ang)                                      # a ← resultat de QVBoxLayout()
        a.setContentsMargins(20, 18, 20, 18)                                    # executem setContentsMargins()
        a.setSpacing(4)                                                         # executem setSpacing()
        a.addWidget(_eyebrow("DESVIACIÓ ANGULAR"))                              # executem addWidget()
        a.addSpacing(8)                                                         # executem addSpacing()

        row_ang = QtWidgets.QHBoxLayout()                                       # row_ang ← resultat de QHBoxLayout()
        row_ang.setSpacing(4)                                                   # executem setSpacing()
        row_ang.setAlignment(QtCore.Qt.AlignCenter)                             # executem setAlignment()
        self.lbl_ang_num = _mono_label("—", size=44, letter=-1)                 # self.lbl_ang_num ← resultat de _mono_label()
        row_ang.addWidget(self.lbl_ang_num)                                     # executem addWidget()
        self.lbl_ang_unit = QtWidgets.QLabel("°")                               # self.lbl_ang_unit ← resultat de QLabel()
        self.lbl_ang_unit.setStyleSheet(                                        # executem setStyleSheet()
            f"color: {C_TEXT_DIM}; font-size: 20px; font-weight: 500; "
            f"background: transparent; border: none; padding-top: 18px;"
        )
        row_ang.addWidget(self.lbl_ang_unit)                                    # executem addWidget()
        a.addLayout(row_ang)                                                    # executem addLayout()

        self.pb_ang = QtWidgets.QProgressBar()                                  # self.pb_ang ← resultat de QProgressBar()
        self.pb_ang.setRange(0, 100)                                            # executem setRange()
        self.pb_ang.setValue(0)                                                 # executem setValue()
        self.pb_ang.setTextVisible(False)                                       # executem setTextVisible()
        self.pb_ang.setFixedHeight(4)                                           # executem setFixedHeight()
        self.pb_ang.setStyleSheet(self.pb_dist.styleSheet())                    # executem setStyleSheet()
        a.addWidget(self.pb_ang)                                                # executem addWidget()
        a.addSpacing(4)                                                         # executem addSpacing()
        self.lbl_ang_hint = QtWidgets.QLabel("Sense referència")                # self.lbl_ang_hint ← resultat de QLabel()
        self.lbl_ang_hint.setAlignment(QtCore.Qt.AlignCenter)                   # executem setAlignment()
        self.lbl_ang_hint.setStyleSheet(                                        # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-size: 10px; "
            f"letter-spacing: 0.5px; background: transparent; border: none;"
        )
        a.addWidget(self.lbl_ang_hint)                                          # executem addWidget()

        v.addWidget(seg_ang)                                                    # executem addWidget()

        # ---- Segment 4: Estat del sistema -------------------------------
        seg_sys = _segment_frame()                                              # seg_sys ← resultat de _segment_frame()
        s = QtWidgets.QVBoxLayout(seg_sys)                                      # s ← resultat de QVBoxLayout()
        s.setContentsMargins(20, 18, 20, 18)                                    # executem setContentsMargins()
        s.setSpacing(10)                                                        # executem setSpacing()
        s.addWidget(_eyebrow("SISTEMA"))                                        # executem addWidget()
        s.addSpacing(4)                                                         # executem addSpacing()

        self.status_lines = []                                                  # assignem self.status_lines
        for label in ("Robot", "Registre", "Pla quirúrgic"):                    # iterem: label in ("Robot", "Registre", "Pla q...
            row = QtWidgets.QHBoxLayout()                                       # row ← resultat de QHBoxLayout()
            row.setSpacing(10)                                                  # executem setSpacing()
            dot = QtWidgets.QLabel("●")                                         # dot ← resultat de QLabel()
            dot.setStyleSheet(                                                  # executem setStyleSheet()
                f"color: {C_TEXT_MUTED}; font-size: 11px; "
                f"background: transparent; border: none;"
            )
            row.addWidget(dot)                                                  # executem addWidget()
            lbl = QtWidgets.QLabel(label)                                       # lbl ← resultat de QLabel()
            lbl.setStyleSheet(                                                  # executem setStyleSheet()
                f"color: {C_TEXT_DIM}; font-size: 12px; font-weight: 500; "
                f"letter-spacing: 0.3px; background: transparent; border: none;"
            )
            row.addWidget(lbl)                                                  # executem addWidget()
            row.addStretch()                                                    # executem addStretch()
            val = QtWidgets.QLabel("—")                                         # val ← resultat de QLabel()
            val.setStyleSheet(                                                  # executem setStyleSheet()
                f"color: {C_TEXT_MUTED}; font-family: {FONT_MONO}; "
                f"font-size: 10px; letter-spacing: 0.5px; "
                f"background: transparent; border: none;"
            )
            row.addWidget(val)                                                  # executem addWidget()
            s.addLayout(row)                                                    # executem addLayout()
            self.status_lines.append((dot, lbl, val))                           # executem append()

        s.addStretch()                                                          # executem addStretch()
        v.addWidget(seg_sys, 1)                                                 # executem addWidget()

    # ------------------------------------------------------------------
    # SETTERS
    # ------------------------------------------------------------------
    def set_mode(self, mode_text, hint_text=""):                                # definim la funció set_mode()
        self.lbl_mode.setText(mode_text)                                        # executem setText()
        self.lbl_mode_hint.setText(hint_text)                                   # executem setText()

    def set_distance(self, dist_mm, hint_text=""):                              # definim la funció set_distance()
        """dist_mm: None si no aplica. Colors segons proximitat."""             # """dist_mm: None si no aplica. Colors segons proximitat."""
        if dist_mm is None:                                                     # si dist_mm is None
            self.lbl_dist_num.setText("—")                                      # executem setText()
            self.lbl_dist_num.setStyleSheet(                                    # executem setStyleSheet()
                f"color: {C_TEXT_DIM}; font-family: {FONT_MONO}; "
                f"font-size: 44px; font-weight: 700; letter-spacing: -1px; "
                f"background: transparent; border: none;"
            )
            self.pb_dist.setValue(0)                                            # executem setValue()
        else:                                                                   # en cas contrari
            self.lbl_dist_num.setText(f"{dist_mm:.1f}")
            col = _tol_color(dist_mm, TOL_OK_MM, TOL_WARN_MM)                   # verd / groc / vermell
            self.lbl_dist_num.setStyleSheet(                                    # executem setStyleSheet()
                f"color: {col}; font-family: {FONT_MONO}; "
                f"font-size: 44px; font-weight: 700; letter-spacing: -1px; "
                f"background: transparent; border: none;"
            )
            # Progress: 5 mm → 0 %, 0 mm → 100 %
            pct = int(np.clip(100.0 - (abs(dist_mm) / 5.0) * 100.0, 0, 100))    # pct ← resultat de int()
            self.pb_dist.setValue(pct)                                          # executem setValue()
            # Barra color
            self.pb_dist.setStyleSheet(f"""
                QProgressBar {{
                    background-color: rgba(255, 255, 255, 0.08);
                    border: none; border-radius: 2px;
                }}
                QProgressBar::chunk {{
                    background-color: {col};
                    border-radius: 2px;
                }}
            """)
        self.lbl_dist_hint.setText(hint_text)                                   # executem setText()

    def set_angular(self, deg, hint_text=""):                                   # definim la funció set_angular()
        """deg: None si no aplica. Colors segons desviació angular."""          # """deg: None si no aplica. Colors segons desviació angula...
        if deg is None:                                                         # si deg is None
            self.lbl_ang_num.setText("—")                                       # executem setText()
            self.lbl_ang_num.setStyleSheet(                                     # executem setStyleSheet()
                f"color: {C_TEXT_DIM}; font-family: {FONT_MONO}; "
                f"font-size: 44px; font-weight: 700; letter-spacing: -1px; "
                f"background: transparent; border: none;"
            )
            self.pb_ang.setValue(0)                                             # executem setValue()
        else:                                                                   # en cas contrari
            self.lbl_ang_num.setText(f"{deg:.1f}")
            col = _tol_color(deg, TOL_OK_DEG, TOL_WARN_DEG)                     # verd / groc / vermell
            self.lbl_ang_num.setStyleSheet(                                     # executem setStyleSheet()
                f"color: {col}; font-family: {FONT_MONO}; "
                f"font-size: 44px; font-weight: 700; letter-spacing: -1px; "
                f"background: transparent; border: none;"
            )
            pct = int(np.clip(100.0 - (deg / 5.0) * 100.0, 0, 100))             # pct ← resultat de int()
            self.pb_ang.setValue(pct)                                           # executem setValue()
            self.pb_ang.setStyleSheet(f"""
                QProgressBar {{
                    background-color: rgba(255, 255, 255, 0.08);
                    border: none; border-radius: 2px;
                }}
                QProgressBar::chunk {{
                    background-color: {col};
                    border-radius: 2px;
                }}
            """)
        self.lbl_ang_hint.setText(hint_text)                                    # executem setText()

    def set_status(self, row_idx, ok, detail=""):                               # definim la funció set_status()
        """
        row_idx 0: Robot, 1: Registre, 2: Pla quirúrgic.
        """
        if row_idx < 0 or row_idx >= len(self.status_lines):                    # si row_idx < 0 or row_idx >= len(self.st...
            return                                                              # retornem sense valor
        dot, lbl, val = self.status_lines[row_idx]                              # assignem dot, lbl, val
        if ok:                                                                  # si ok
            dot.setStyleSheet(                                                  # executem setStyleSheet()
                f"color: {C_SAFE}; font-size: 11px; "
                f"background: transparent; border: none;"
            )
        else:                                                                   # en cas contrari
            dot.setStyleSheet(                                                  # executem setStyleSheet()
                f"color: {C_TEXT_MUTED}; font-size: 11px; "
                f"background: transparent; border: none;"
            )
        val.setText(detail)                                                     # executem setText()


# ==============================================================================
# HEADER DE NAVEGACIÓ (títol + pill de precisió)
# ==============================================================================

class NavigationHeader(QtWidgets.QFrame):                                       # definim la classe NavigationHeader
    """Barra fina superior amb el nom del cargol actiu + estat de precisió."""  # """Barra fina superior amb el nom del cargol actiu + esta...

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setFixedHeight(58)                                                 # executem setFixedHeight()
        self.setStyleSheet("""
            QFrame {
                background-color: #0a0a0c;
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 14px;
            }
        """)

        h = QtWidgets.QHBoxLayout(self)                                         # h ← resultat de QHBoxLayout()
        h.setContentsMargins(22, 0, 22, 0)                                      # executem setContentsMargins()
        h.setSpacing(20)                                                        # executem setSpacing()

        # Eyebrow "CARGOL ACTIU"
        eb = QtWidgets.QLabel("CARGOL ACTIU")                                   # eb ← resultat de QLabel()
        eb.setStyleSheet(                                                       # executem setStyleSheet()
            f"color: {C_ACCENT}; font-size: 10px; font-weight: 700; "
            f"letter-spacing: 3px; background: transparent; border: none;"
        )
        h.addWidget(eb)                                                         # executem addWidget()

        # Nom del cargol actiu (gran)
        self.lbl_screw = QtWidgets.QLabel("— cap seleccionat —")                # self.lbl_screw ← resultat de QLabel()
        self.lbl_screw.setStyleSheet(                                           # executem setStyleSheet()
            f"color: {C_TEXT}; font-size: 18px; font-weight: 700; "
            f"letter-spacing: 1px; background: transparent; border: none;"
        )
        h.addWidget(self.lbl_screw)                                             # executem addWidget()
        h.addStretch()                                                          # executem addStretch()

        # Pill de precisió global
        self.pill_precision = QtWidgets.QLabel("●  ESPERANT")                   # self.pill_precision ← resultat de QLabel()
        self.pill_precision.setStyleSheet(                                      # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-size: 11px; font-weight: 700; "
            f"letter-spacing: 2px; background: rgba(255,255,255,0.04); "
            f"border: 1px solid rgba(255,255,255,0.10); "
            f"border-radius: 12px; padding: 6px 14px;"
        )
        h.addWidget(self.pill_precision)                                        # executem addWidget()

    def set_screw_name(self, name):                                             # definim la funció set_screw_name()
        self.lbl_screw.setText(name if name else "— cap seleccionat —")         # executem setText()

    def set_precision(self, level):                                             # definim la funció set_precision()
        """level: 'ok', 'warn', 'danger', 'idle'."""                            # """level: 'ok', 'warn', 'danger', 'idle'."""
        colors = {                                                              # assignem colors
            "ok":     (C_SAFE,      "PRECÍS"),
            "warn":   (C_WARN,      "ATENCIÓ"),
            "danger": (C_DANGER,    "DESVIACIÓ ALTA"),
            "idle":   (C_TEXT_MUTED, "ESPERANT"),
        }
        col, txt = colors.get(level, colors["idle"])                            # col, txt ← resultat de get()
        self.pill_precision.setText(f"●  {txt}")
        self.pill_precision.setStyleSheet(                                      # executem setStyleSheet()
            f"color: {col}; font-size: 11px; font-weight: 700; "
            f"letter-spacing: 2px; background: rgba(255,255,255,0.04); "
            f"border: 1px solid rgba(255,255,255,0.10); "
            f"border-radius: 12px; padding: 6px 14px;"
        )


# ==============================================================================
# PANELL DRET — LLISTA DE CARGOLS + ACCIONS
# ==============================================================================

class ScrewProgressItem(QtWidgets.QFrame):                                      # definim la classe ScrewProgressItem
    """Un ítem de la llista de cargols amb estat: pending / active / done."""   # """Un ítem de la llista de cargols amb estat: pending / a...

    clicked = QtCore.Signal(int)                                                # clicked ← resultat de Signal()

    def __init__(self, idx, name, parent=None):                                 # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.idx = idx                                                          # assignem self.idx
        self.name = name                                                        # assignem self.name
        self.state = "pending"                                                  # assignem self.state
        self.setCursor(QtCore.Qt.PointingHandCursor)                            # executem setCursor()
        self.setFixedHeight(58)                                                 # executem setFixedHeight()

        h = QtWidgets.QHBoxLayout(self)                                         # h ← resultat de QHBoxLayout()
        h.setContentsMargins(14, 10, 14, 10)                                    # executem setContentsMargins()
        h.setSpacing(12)                                                        # executem setSpacing()

        # Nombre gran
        self.lbl_num = QtWidgets.QLabel(f"{idx + 1:02d}")
        self.lbl_num.setFixedWidth(28)                                          # executem setFixedWidth()
        self.lbl_num.setStyleSheet(                                             # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-family: {FONT_MONO}; "
            f"font-size: 15px; font-weight: 700; letter-spacing: 0.5px; "
            f"background: transparent; border: none;"
        )
        h.addWidget(self.lbl_num)                                               # executem addWidget()

        # Nom del cargol
        col_txt = QtWidgets.QVBoxLayout()                                       # col_txt ← resultat de QVBoxLayout()
        col_txt.setSpacing(2)                                                   # executem setSpacing()
        self.lbl_name = QtWidgets.QLabel(name)                                  # self.lbl_name ← resultat de QLabel()
        self.lbl_name.setStyleSheet(                                            # executem setStyleSheet()
            f"color: {C_TEXT}; font-size: 13px; font-weight: 600; "
            f"letter-spacing: 0.3px; background: transparent; border: none;"
        )
        col_txt.addWidget(self.lbl_name)                                        # executem addWidget()

        self.lbl_status = QtWidgets.QLabel("Pendent")                           # self.lbl_status ← resultat de QLabel()
        self.lbl_status.setStyleSheet(                                          # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-size: 10px; font-weight: 500; "
            f"letter-spacing: 0.5px; background: transparent; border: none;"
        )
        col_txt.addWidget(self.lbl_status)                                      # executem addWidget()
        h.addLayout(col_txt, 1)                                                 # executem addLayout()

        # Indicador circular
        self.dot = QtWidgets.QLabel("○")                                        # self.dot ← resultat de QLabel()
        self.dot.setStyleSheet(                                                 # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-size: 16px; "
            f"background: transparent; border: none;"
        )
        h.addWidget(self.dot)                                                   # executem addWidget()

        self._apply_style()                                                     # executem _apply_style()

    def _apply_style(self):                                                     # definim la funció _apply_style()
        if self.state == "active":                                              # si self.state == "active"
            self.setStyleSheet("""
                ScrewProgressItem {
                    background-color: rgba(10, 132, 255, 0.10);
                    border: 1px solid rgba(10, 132, 255, 0.45);
                    border-radius: 10px;
                }
            """)
            self.dot.setText("●")                                               # executem setText()
            self.dot.setStyleSheet(                                             # executem setStyleSheet()
                f"color: {C_ACCENT}; font-size: 16px; "
                f"background: transparent; border: none;"
            )
            self.lbl_status.setText("En curs")                                  # executem setText()
            self.lbl_status.setStyleSheet(                                      # executem setStyleSheet()
                f"color: {C_ACCENT}; font-size: 10px; font-weight: 700; "
                f"letter-spacing: 0.5px; background: transparent; border: none;"
            )
        elif self.state == "done":                                              # altrament si self.state == "done"
            self.setStyleSheet("""
                ScrewProgressItem {
                    background-color: rgba(48, 209, 88, 0.08);
                    border: 1px solid rgba(48, 209, 88, 0.30);
                    border-radius: 10px;
                }
            """)
            self.dot.setText("✓")                                               # executem setText()
            self.dot.setStyleSheet(                                             # executem setStyleSheet()
                f"color: {C_SAFE}; font-size: 16px; font-weight: 700; "
                f"background: transparent; border: none;"
            )
            self.lbl_status.setText("Completat")                                # executem setText()
            self.lbl_status.setStyleSheet(                                      # executem setStyleSheet()
                f"color: {C_SAFE}; font-size: 10px; font-weight: 700; "
                f"letter-spacing: 0.5px; background: transparent; border: none;"
            )
        else:  # pending
            self.setStyleSheet("""
                ScrewProgressItem {
                    background-color: #0e0e10;
                    border: 1px solid rgba(255, 255, 255, 0.08);
                    border-radius: 10px;
                }
                ScrewProgressItem:hover {
                    background-color: #1a1a1c;
                    border: 1px solid rgba(255, 255, 255, 0.20);
                }
            """)
            self.dot.setText("○")                                               # executem setText()
            self.dot.setStyleSheet(                                             # executem setStyleSheet()
                f"color: {C_TEXT_MUTED}; font-size: 16px; "
                f"background: transparent; border: none;"
            )
            self.lbl_status.setText("Pendent")                                  # executem setText()
            self.lbl_status.setStyleSheet(                                      # executem setStyleSheet()
                f"color: {C_TEXT_MUTED}; font-size: 10px; font-weight: 500; "
                f"letter-spacing: 0.5px; background: transparent; border: none;"
            )

    def set_state(self, state):                                                 # definim la funció set_state()
        """state: 'pending', 'active', 'done'."""                               # """state: 'pending', 'active', 'done'."""
        self.state = state                                                      # assignem self.state
        self._apply_style()                                                     # executem _apply_style()

    def mousePressEvent(self, event):                                           # definim la funció mousePressEvent()
        if event.button() == QtCore.Qt.LeftButton:                              # si event.button() == QtCore.Qt.LeftButton
            self.clicked.emit(self.idx)                                         # executem emit()
        super().mousePressEvent(event)                                          # executem super()


class SurgicalPlanPanel(QtWidgets.QWidget):                                     # definim la classe SurgicalPlanPanel
    """Columna dreta: llista de cargols + accions."""                           # """Columna dreta: llista de cargols + accions."""

    screwSelected = QtCore.Signal(int)                                          # screwSelected ← resultat de Signal()
    approachRequested = QtCore.Signal(int)                                      # approachRequested ← resultat de Signal()
    completeRequested = QtCore.Signal(int)                                      # completeRequested ← resultat de Signal()

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setFixedWidth(300)                                                 # executem setFixedWidth()
        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(8, 16, 16, 16)                                     # executem setContentsMargins()
        v.setSpacing(14)                                                        # executem setSpacing()

        # ---- Card contenidor de la llista ------------------------------
        list_card = _segment_frame()                                            # list_card ← resultat de _segment_frame()
        lc = QtWidgets.QVBoxLayout(list_card)                                   # lc ← resultat de QVBoxLayout()
        lc.setContentsMargins(16, 18, 16, 16)                                   # executem setContentsMargins()
        lc.setSpacing(10)                                                       # executem setSpacing()

        header_row = QtWidgets.QHBoxLayout()                                    # header_row ← resultat de QHBoxLayout()
        header_row.setSpacing(8)                                                # executem setSpacing()
        eb = _eyebrow("PLA QUIRÚRGIC")                                          # eb ← resultat de _eyebrow()
        eb.setAlignment(QtCore.Qt.AlignLeft)                                    # executem setAlignment()
        header_row.addWidget(eb)                                                # executem addWidget()
        header_row.addStretch()                                                 # executem addStretch()
        self.lbl_counter = QtWidgets.QLabel("0 / 0")                            # self.lbl_counter ← resultat de QLabel()
        self.lbl_counter.setStyleSheet(                                         # executem setStyleSheet()
            f"color: {C_TEXT_MUTED}; font-family: {FONT_MONO}; "
            f"font-size: 11px; font-weight: 700; letter-spacing: 1px; "
            f"background: transparent; border: none;"
        )
        header_row.addWidget(self.lbl_counter)                                  # executem addWidget()
        lc.addLayout(header_row)                                                # executem addLayout()
        lc.addSpacing(4)                                                        # executem addSpacing()

        # Contenidor scroll dels ítems
        scroll = QtWidgets.QScrollArea()                                        # scroll ← resultat de QScrollArea()
        scroll.setWidgetResizable(True)                                         # executem setWidgetResizable()
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)                          # executem setFrameShape()
        scroll.setStyleSheet(                                                   # executem setStyleSheet()
            "QScrollArea { background: transparent; border: none; } "
            "QScrollBar:vertical { background: transparent; width: 6px; } "
            "QScrollBar::handle:vertical { "
            "  background: rgba(255,255,255,0.15); border-radius: 3px; } "
            "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }"
        )
        self.items_container = QtWidgets.QWidget()                              # self.items_container ← resultat de QWidget()
        self.items_container.setStyleSheet("background: transparent;")          # executem setStyleSheet()
        self.items_layout = QtWidgets.QVBoxLayout(self.items_container)         # self.items_layout ← resultat de QVBoxLayout()
        self.items_layout.setContentsMargins(0, 0, 0, 0)                        # executem setContentsMargins()
        self.items_layout.setSpacing(6)                                         # executem setSpacing()
        self.items_layout.addStretch()                                          # executem addStretch()
        scroll.setWidget(self.items_container)                                  # executem setWidget()
        lc.addWidget(scroll, 1)                                                 # executem addWidget()

        v.addWidget(list_card, 1)                                               # executem addWidget()

        # ---- Accions --------------------------------------------------
        actions = QtWidgets.QVBoxLayout()                                       # actions ← resultat de QVBoxLayout()
        actions.setSpacing(8)                                                   # executem setSpacing()

        self.btn_approach = self._make_action_button(                           # assignem self.btn_approach
            "ANAR A APPROACH", accent=True)
        self.btn_approach.clicked.connect(self._on_approach)                    # executem connect()
        self.btn_approach.setEnabled(False)                                     # executem setEnabled()
        actions.addWidget(self.btn_approach)                                    # executem addWidget()

        self.btn_complete = self._make_action_button(                           # assignem self.btn_complete
            "MARCAR COMPLETAT", accent=False)
        self.btn_complete.clicked.connect(self._on_complete)                    # executem connect()
        self.btn_complete.setEnabled(False)                                     # executem setEnabled()
        actions.addWidget(self.btn_complete)                                    # executem addWidget()

        v.addLayout(actions)                                                    # executem addLayout()

        # ---- Estat intern ----
        self.items = []                                                         # assignem self.items
        self._active_idx = None                                                 # assignem self._active_idx

    def _make_action_button(self, text, accent=False):                          # definim la funció _make_action_button()
        btn = QtWidgets.QPushButton(text)                                       # btn ← resultat de QPushButton()
        btn.setFixedHeight(46)                                                  # executem setFixedHeight()
        btn.setCursor(QtCore.Qt.PointingHandCursor)                             # executem setCursor()
        if accent:                                                              # si accent
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {C_ACCENT};
                    color: white;
                    border: none;
                    border-radius: 12px;
                    font-size: 12px;
                    font-weight: 700;
                    letter-spacing: 2px;
                }}
                QPushButton:hover {{
                    background-color: #1a94ff;
                }}
                QPushButton:disabled {{
                    background-color: rgba(255,255,255,0.06);
                    color: {C_TEXT_MUTED};
                }}
            """)
        else:                                                                   # en cas contrari
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: rgba(255,255,255,0.05);
                    color: {C_TEXT};
                    border: 1px solid rgba(255,255,255,0.15);
                    border-radius: 12px;
                    font-size: 12px;
                    font-weight: 700;
                    letter-spacing: 2px;
                }}
                QPushButton:hover {{
                    background-color: rgba(255,255,255,0.10);
                    border: 1px solid rgba(255,255,255,0.30);
                }}
                QPushButton:disabled {{
                    background-color: transparent;
                    color: {C_TEXT_MUTED};
                    border: 1px solid rgba(255,255,255,0.05);
                }}
            """)
        return btn                                                              # retornem: btn

    # -----------------------------------------------------------------
    def set_screws(self, screws_list):                                          # definim la funció set_screws()
        """Rep la llista de cargols del RobotWorkspace._reg_screws_m."""        # """Rep la llista de cargols del RobotWorkspace._reg_screw...
        # Netegem
        for item in self.items:                                                 # iterem: item in self.items
            self.items_layout.removeWidget(item)                                # executem removeWidget()
            item.deleteLater()                                                  # executem deleteLater()
        self.items = []                                                         # assignem self.items

        # Traiem lo stretch temporalment
        while self.items_layout.count():                                        # mentre self.items_layout.count()
            item = self.items_layout.takeAt(0)                                  # item ← resultat de takeAt()
            w = item.widget()                                                   # w ← resultat de widget()
            if w is not None:                                                   # si w is not None
                w.deleteLater()                                                 # executem deleteLater()

        # Reconstruïm
        for i, sc in enumerate(screws_list):                                    # iterem: i, sc in enumerate(screws_list)
            name = sc.get("name", f"Cargol {i+1}")
            it = ScrewProgressItem(i, name)                                     # it ← resultat de ScrewProgressItem()
            it.clicked.connect(self._on_item_clicked)                           # executem connect()
            self.items_layout.addWidget(it)                                     # executem addWidget()
            self.items.append(it)                                               # executem append()
        self.items_layout.addStretch()                                          # executem addStretch()

        self._active_idx = None                                                 # assignem self._active_idx
        self._update_counter()                                                  # executem _update_counter()
        self.btn_approach.setEnabled(False)                                     # executem setEnabled()
        self.btn_complete.setEnabled(False)                                     # executem setEnabled()

    def _on_item_clicked(self, idx):                                            # definim la funció _on_item_clicked()
        self.set_active(idx)                                                    # executem set_active()
        self.screwSelected.emit(idx)                                            # executem emit()

    def set_active(self, idx):                                                  # definim la funció set_active()
        for i, it in enumerate(self.items):                                     # iterem: i, it in enumerate(self.items)
            if i == idx:                                                        # si i == idx
                if it.state != "done":                                          # si it.state != "done"
                    it.set_state("active")                                      # executem set_state()
            elif it.state == "active":                                          # altrament si it.state == "active"
                it.set_state("pending")                                         # executem set_state()
        self._active_idx = idx                                                  # assignem self._active_idx
        self.btn_approach.setEnabled(idx is not None)                           # executem setEnabled()
        self.btn_complete.setEnabled(                                           # executem setEnabled()
            idx is not None and self.items[idx].state == "active"
        )

    def _on_approach(self):                                                     # definim la funció _on_approach()
        if self._active_idx is not None:                                        # si self._active_idx is not None
            self.approachRequested.emit(self._active_idx)                       # executem emit()

    def _on_complete(self):                                                     # definim la funció _on_complete()
        if self._active_idx is not None:                                        # si self._active_idx is not None
            self.items[self._active_idx].set_state("done")                      # self.items[self._active_idx].set_state("done")
            self.completeRequested.emit(self._active_idx)                       # executem emit()
            self._update_counter()                                              # executem _update_counter()
            self.btn_complete.setEnabled(False)                                 # executem setEnabled()

    def _update_counter(self):                                                  # definim la funció _update_counter()
        done = sum(1 for it in self.items if it.state == "done")                # done ← resultat de sum()
        total = len(self.items)                                                 # total ← resultat de len()
        self.lbl_counter.setText(f"{done:02d} / {total:02d}")


# ==============================================================================
# GEOMETRIA DEL GUIAT (pura, sense Qt)
# ==============================================================================

def screw_frame(axis_unit):                                                     # definim la funció screw_frame()
    """
    Base (u, v) del pla perpendicular a l'eix del cargol, per dibuixar la
    mira "mirant al llarg de l'eix" (cap a dins de l'os):
        v = el "amunt" del robot (Z) projectat al pla  → amunt a la pantalla
        u = d × v                                      → dreta a la pantalla
    Si l'eix és gairebé vertical, l'"amunt" passa a ser X del robot.
    """
    d = np.asarray(axis_unit, dtype=float)                                      # d ← resultat de asarray()
    ref = np.array([0.0, 0.0, 1.0])                                             # ref ← resultat de array()
    if abs(np.dot(ref, d)) > 0.9:                                               # eix gairebé vertical
        ref = np.array([1.0, 0.0, 0.0])                                         # ref ← resultat de array()
    v = ref - np.dot(ref, d) * d                                                # v ← resultat de dot()
    v /= np.linalg.norm(v)                                                      # dividim v
    u = np.cross(d, v)                                                          # u ← resultat de cross()
    return u, v                                                                 # retornem: u, v


def guidance_metrics(T_tcp, screw):                                             # definim la funció guidance_metrics()
    """
    Mètriques de guiat de la broca respecte del cargol planificat (mm, °).

        depth_mm    posició de la punta al llarg de l'eix, des de l'ENTRADA
                    (p0): < 0 abans de l'os, 0 a l'entrada, L al final
        lateral_mm  distància de la punta a la línia de la trajectòria
        angle_deg   angle entre l'eix de la broca (X del TCP) i el del cargol
        tip_uv      desplaçament lateral de la punta al pla de la mira (mm)
        tail_uv     el mateix per a un punt TAIL_MM enrere sobre la broca
        length_mm   llargada planificada del cargol
    """
    p0 = np.asarray(screw["p0_m"], dtype=float)                                 # p0 ← entrada (cap del cargol)
    d = np.asarray(screw["axis_unit"], dtype=float)                             # d ← eix del cargol
    tip = np.asarray(T_tcp[:3, 3], dtype=float)                                 # tip ← punta de la broca
    x_tool = np.asarray(T_tcp[:3, 0], dtype=float)                              # la broca surt per l'eix X del TCP
    u, v = screw_frame(d)                                                       # u, v ← resultat de screw_frame()

    rel = tip - p0                                                              # assignem rel
    depth = float(np.dot(rel, d))                                               # depth ← resultat de float()
    lat = rel - depth * d                                                       # component perpendicular a l'eix
    tail = tip - x_tool * (TAIL_MM * 1e-3)                                      # un punt enrere sobre la broca
    rel_t = tail - p0                                                           # assignem rel_t
    lat_t = rel_t - np.dot(rel_t, d) * d                                        # assignem lat_t
    cos_a = float(np.clip(np.dot(x_tool, d), -1.0, 1.0))                        # cos_a ← resultat de float()
    return {                                                                    # retornem: les mètriques
        "depth_mm":   depth * 1000.0,
        "lateral_mm": float(np.linalg.norm(lat)) * 1000.0,
        "angle_deg":  float(np.degrees(np.arccos(cos_a))),
        "tip_uv":     (float(np.dot(lat, u)) * 1000.0, float(np.dot(lat, v)) * 1000.0),
        "tail_uv":    (float(np.dot(lat_t, u)) * 1000.0, float(np.dot(lat_t, v)) * 1000.0),
        "length_mm":  float(screw["length_m"]) * 1000.0,
    }


# ==============================================================================
# VISOR DE PUNTERIA (mira al llarg de l'eix del cargol)
# ==============================================================================

class TargetingView(QtWidgets.QWidget):                                         # definim la classe TargetingView
    """
    Mira "bull's-eye": mirem al llarg de l'eix planificat del cargol.

        ●  punta de la broca (desviació lateral)
        ○  cua de la broca, TAIL_MM enrere (desviació angular)

    Tots dos al centre = broca sobre la trajectòria i alineada.
    Anells d'1, 2 i 5 mm (i més si cal); l'escala s'adapta sola.
    """

    SCALES_MM = (3.0, 6.0, 12.0, 25.0, 50.0)                                    # radi visible possible (mm)

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setMinimumSize(320, 320)                                           # executem setMinimumSize()
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, True)                  # pintem tot el fons
        self.m = None                                                           # mètriques (None = sense cargol)
        self.scale_mm = self.SCALES_MM[-1]                                      # assignem self.scale_mm
        self.message = "Selecciona un cargol del pla quirúrgic"                 # assignem self.message

    def set_metrics(self, m, message=None):                                     # definim la funció set_metrics()
        self.m = m                                                              # assignem self.m
        if message is not None:                                                 # si message is not None
            self.message = message                                              # assignem self.message
        if m is not None:                                                       # escala amb histèresi
            need = 1.25 * max(np.hypot(*m["tip_uv"]), np.hypot(*m["tail_uv"]), 0.5)
            fit = next((s for s in self.SCALES_MM if s >= need), self.SCALES_MM[-1])
            if fit > self.scale_mm or need < 0.55 * self.scale_mm:              # ampliem de seguida; reduïm amb marge
                self.scale_mm = fit                                             # assignem self.scale_mm
        self.update()                                                           # executem update()

    def paintEvent(self, ev):                                                   # definim la funció paintEvent()
        p = QtGui.QPainter(self)                                                # p ← resultat de QPainter()
        p.setRenderHint(QtGui.QPainter.Antialiasing)                            # executem setRenderHint()
        p.fillRect(self.rect(), QtGui.QColor("#050506"))                        # executem fillRect()
        w, h = self.width(), self.height()                                      # assignem w, h
        cx, cy = w / 2.0, h / 2.0                                               # assignem cx, cy
        R = min(w, h) / 2.0 - 34                                                # radi útil (px)

        f_small = QtGui.QFont(self.font()); f_small.setPixelSize(SIZE_XS)       # f_small ← resultat de QFont()
        f_tag = QtGui.QFont(self.font()); f_tag.setPixelSize(SIZE_XS); f_tag.setBold(True)
        f_tag.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 3)                  # executem setLetterSpacing()

        # Títol
        p.setFont(f_tag)                                                        # executem setFont()
        p.setPen(QtGui.QColor(C_ACCENT))                                        # executem setPen()
        p.drawText(QtCore.QRectF(18, 14, w - 36, 16), QtCore.Qt.AlignLeft, "PUNTERIA")
        p.setFont(f_small)                                                      # executem setFont()
        p.setPen(QtGui.QColor(C_TEXT_MUTED))                                    # executem setPen()
        p.drawText(QtCore.QRectF(18, 30, w - 36, 16), QtCore.Qt.AlignLeft,
                   "mirant al llarg de l'eix del cargol")

        if self.m is None:                                                      # sense cargol actiu
            p.setPen(QtGui.QColor(C_TEXT_DIM))                                  # executem setPen()
            f = QtGui.QFont(self.font()); f.setPixelSize(SIZE_BASE)             # f ← resultat de QFont()
            p.setFont(f)                                                        # executem setFont()
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, self.message)        # executem drawText()
            p.end()                                                             # executem end()
            return                                                              # retornem sense valor

        k = R / self.scale_mm                                                   # px per mm

        # Creu de referència
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 22), 1))                # executem setPen()
        p.drawLine(QtCore.QPointF(cx - R, cy), QtCore.QPointF(cx + R, cy))      # executem drawLine()
        p.drawLine(QtCore.QPointF(cx, cy - R), QtCore.QPointF(cx, cy + R))      # executem drawLine()

        # Anells (mm) + zona de tolerància "OK" ombrejada
        p.setPen(QtCore.Qt.NoPen)                                               # executem setPen()
        ok_col = QtGui.QColor(C_SAFE); ok_col.setAlpha(28)                      # ok_col ← resultat de QColor()
        p.setBrush(ok_col)                                                      # executem setBrush()
        p.drawEllipse(QtCore.QPointF(cx, cy), TOL_OK_MM * k, TOL_OK_MM * k)     # executem drawEllipse()
        p.setBrush(QtCore.Qt.NoBrush)                                           # executem setBrush()
        p.setFont(f_small)                                                      # executem setFont()
        for ring in (1, 2, 5, 10, 20, 50):                                      # iterem: ring in (1, 2, 5, 10, 20, 50)
            if ring > self.scale_mm + 1e-6:                                     # fora de l'escala
                break                                                           # sortim del bucle
            r = ring * k                                                        # assignem r
            alpha = 90 if ring in (1, 2) else 45                                # assignem alpha
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, alpha), 1))         # executem setPen()
            p.drawEllipse(QtCore.QPointF(cx, cy), r, r)                         # executem drawEllipse()
            p.setPen(QtGui.QColor(C_TEXT_MUTED))                                # executem setPen()
            p.drawText(QtCore.QPointF(cx + r * 0.7071 + 4, cy - r * 0.7071 - 4), f"{ring} mm")

        # Posicions (clamp a la vora si surten de l'escala)
        def to_px(uv):                                                          # definim la funció to_px()
            x, y = uv[0] * k, -uv[1] * k                                        # Y de pantalla cap avall
            n = np.hypot(x, y)                                                  # n ← resultat de hypot()
            out = n > R                                                         # out ← resultat de >()
            if out:                                                             # si out
                x, y = x * R / n, y * R / n                                     # assignem x, y
            return QtCore.QPointF(cx + x, cy + y), out                          # retornem: punt, fora d'escala

        tip_pt, tip_out = to_px(self.m["tip_uv"])                               # assignem tip_pt, tip_out
        tail_pt, tail_out = to_px(self.m["tail_uv"])                            # assignem tail_pt, tail_out
        col_tip = QtGui.QColor(_tol_color(self.m["lateral_mm"], TOL_OK_MM, TOL_WARN_MM))
        col_tail = QtGui.QColor(_tol_color(self.m["angle_deg"], TOL_OK_DEG, TOL_WARN_DEG))

        # Projecció de la broca (de la cua a la punta)
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 110), 2))               # executem setPen()
        p.drawLine(tail_pt, tip_pt)                                             # executem drawLine()

        # Cua: anell
        p.setPen(QtGui.QPen(col_tail, 2.5))                                     # executem setPen()
        p.setBrush(QtCore.Qt.NoBrush)                                           # executem setBrush()
        p.drawEllipse(tail_pt, 13, 13)                                          # executem drawEllipse()

        # Punta: punt amb anell del color de fons
        p.setPen(QtGui.QPen(QtGui.QColor("#050506"), 3))                        # executem setPen()
        p.setBrush(col_tip)                                                     # executem setBrush()
        p.drawEllipse(tip_pt, 8, 8)                                             # executem drawEllipse()

        # Marca central
        p.setPen(QtGui.QPen(QtGui.QColor(C_TEXT), 1.5))                         # executem setPen()
        p.drawLine(QtCore.QPointF(cx - 6, cy), QtCore.QPointF(cx + 6, cy))      # executem drawLine()
        p.drawLine(QtCore.QPointF(cx, cy - 6), QtCore.QPointF(cx, cy + 6))      # executem drawLine()

        # Peu: llegenda + avís si algun marcador és fora d'escala
        p.setFont(f_small)                                                      # executem setFont()
        p.setPen(QtGui.QColor(C_TEXT_MUTED))                                    # executem setPen()
        legend = f"●  punta     ○  cua ({TAIL_MM:.0f} mm enrere)     escala ±{self.scale_mm:.0f} mm"
        if tip_out or tail_out:                                                 # si tip_out or tail_out
            legend += "     ▲ fora d'escala"                                    # assignem legend
        p.drawText(QtCore.QRectF(18, h - 26, w - 36, 16), QtCore.Qt.AlignLeft, legend)
        p.end()                                                                 # executem end()


# ==============================================================================
# INDICADOR DE PROFUNDITAT
# ==============================================================================

class DepthGauge(QtWidgets.QWidget):                                            # definim la classe DepthGauge
    """
    Barra vertical al llarg de l'eix del cargol:
        APPROACH (-10 mm) · ENTRADA (0) · cargol planificat (0..L) · perill > L
    La punta de la broca hi és en directe, amb la profunditat i el que queda.
    """

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setFixedWidth(190)                                                 # executem setFixedWidth()
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, True)                  # pintem tot el fons
        self.depth = None                                                       # mm des de l'entrada (None = sense dada)
        self.length = 40.0                                                      # mm planificats

    def set_depth(self, depth_mm, length_mm):                                   # definim la funció set_depth()
        self.depth, self.length = depth_mm, max(float(length_mm), 1.0)          # assignem self.depth, self.length
        self.update()                                                           # executem update()

    def paintEvent(self, ev):                                                   # definim la funció paintEvent()
        p = QtGui.QPainter(self)                                                # p ← resultat de QPainter()
        p.setRenderHint(QtGui.QPainter.Antialiasing)                            # executem setRenderHint()
        p.fillRect(self.rect(), QtGui.QColor("#050506"))                        # executem fillRect()
        w, h = self.width(), self.height()                                      # assignem w, h
        f_small = QtGui.QFont(self.font()); f_small.setPixelSize(SIZE_XS)       # f_small ← resultat de QFont()
        f_tag = QtGui.QFont(self.font()); f_tag.setPixelSize(SIZE_XS); f_tag.setBold(True)
        f_tag.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 3)                  # executem setLetterSpacing()
        f_big = QtGui.QFont("Consolas")                                         # f_big ← número gran (monoespai)
        f_big.setStyleHint(QtGui.QFont.Monospace)                               # executem setStyleHint()
        f_big.setPixelSize(30); f_big.setBold(True)                             # executem setPixelSize()

        p.setFont(f_tag)                                                        # executem setFont()
        p.setPen(QtGui.QColor(C_ACCENT))                                        # executem setPen()
        p.drawText(QtCore.QRectF(14, 14, w - 28, 16), QtCore.Qt.AlignLeft, "PROFUNDITAT")

        if self.depth is None:                                                  # sense cargol
            p.setPen(QtGui.QColor(C_TEXT_MUTED))                                # executem setPen()
            p.setFont(f_small)                                                  # executem setFont()
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, "—")                 # executem drawText()
            p.end()                                                             # executem end()
            return                                                              # retornem sense valor

        L, d = self.length, self.depth                                          # assignem L, d
        # Número gran: profunditat des de l'entrada, i el que queda
        p.setFont(f_big)                                                        # executem setFont()
        over = d > L + 0.2                                                      # passat del final planificat
        p.setPen(QtGui.QColor(C_DANGER if over else C_TEXT))                    # executem setPen()
        p.drawText(QtCore.QRectF(14, 34, w - 28, 36), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
                   f"{d:+.1f}")
        p.setFont(f_small)                                                      # executem setFont()
        p.setPen(QtGui.QColor(C_TEXT_DIM))                                      # executem setPen()
        rest = L - d                                                            # assignem rest
        txt = ("abans de l'entrada" if d < 0 else
               (f"queden {rest:.1f} mm" if rest > 0.2 else
                ("final assolit" if not over else f"PASSAT {-rest:.1f} mm")))
        p.drawText(QtCore.QRectF(14, 70, w - 28, 16), QtCore.Qt.AlignLeft, f"mm · {txt}")

        # Escala vertical: de -APPROACH-3 mm (dalt) fins a L+6 mm (baix)
        top_mm, bot_mm = -APPROACH_MM - 3.0, L + 6.0                            # assignem top_mm, bot_mm
        y0, y1 = 104.0, h - 24.0                                                # assignem y0, y1
        def y(mm):                                                              # definim la funció y()
            return y0 + (mm - top_mm) / (bot_mm - top_mm) * (y1 - y0)           # retornem: píxel
        bx, bw = 34.0, 30.0                                                     # columna del cargol

        # Zona de perill més enllà del final
        danger = QtGui.QColor(C_DANGER); danger.setAlpha(45)                    # danger ← resultat de QColor()
        p.fillRect(QtCore.QRectF(bx - 10, y(L), bw + 20, y(bot_mm) - y(L)), danger)
        # Aire (abans de l'os)
        p.fillRect(QtCore.QRectF(bx + bw / 2 - 1, y(top_mm), 2, y(0) - y(top_mm)), QtGui.QColor(255, 255, 255, 30))
        # Cargol planificat
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 140), 1))               # executem setPen()
        p.setBrush(QtGui.QColor(255, 255, 255, 18))                             # executem setBrush()
        p.drawRoundedRect(QtCore.QRectF(bx, y(0), bw, y(L) - y(0)), 4, 4)       # executem drawRoundedRect()
        # Progrés dins l'os
        if d > 0:                                                               # si d > 0
            prog = QtGui.QColor(C_DANGER if over else C_ACCENT); prog.setAlpha(160)
            p.setPen(QtCore.Qt.NoPen)                                           # executem setPen()
            p.setBrush(prog)                                                    # executem setBrush()
            p.drawRoundedRect(QtCore.QRectF(bx + 3, y(0) + 1, bw - 6, y(min(d, bot_mm)) - y(0) - 1), 3, 3)

        # Marques: approach, entrada, final
        p.setFont(f_small)                                                      # executem setFont()
        for mm, label, col in ((-APPROACH_MM, "APPROACH", C_TEXT_MUTED),
                               (0.0, "ENTRADA", C_TEXT_DIM),
                               (L, f"FINAL · {L:.0f}", C_TEXT_DIM)):
            yy = y(mm)                                                          # yy ← resultat de y()
            p.setPen(QtGui.QPen(QtGui.QColor(col), 1, QtCore.Qt.DashLine))      # executem setPen()
            p.drawLine(QtCore.QPointF(bx - 10, yy), QtCore.QPointF(bx + bw + 10, yy))
            p.setPen(QtGui.QColor(col))                                         # executem setPen()
            p.drawText(QtCore.QRectF(bx + bw + 14, yy - 8, w - bx - bw - 16, 16),
                       QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, label)

        # Punta de la broca (triangle + línia)
        yd = y(float(np.clip(d, top_mm, bot_mm)))                               # yd ← resultat de y()
        tip_col = QtGui.QColor(C_DANGER if over else C_TEXT)                    # tip_col ← resultat de QColor()
        p.setPen(QtGui.QPen(tip_col, 2))                                        # executem setPen()
        p.drawLine(QtCore.QPointF(bx - 12, yd), QtCore.QPointF(bx + bw + 4, yd))  # executem drawLine()
        p.setPen(QtCore.Qt.NoPen)                                               # executem setPen()
        p.setBrush(tip_col)                                                     # executem setBrush()
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(bx - 12, yd - 6), QtCore.QPointF(bx - 4, yd),
                                       QtCore.QPointF(bx - 12, yd + 6)]))
        p.end()                                                                 # executem end()


# ==============================================================================
# BARRA DE CONTROL (flux quirúrgic, delega al RobotWorkspace)
# ==============================================================================

def _ctl_button(text, accent=False):                                            # definim la funció _ctl_button()
    b = QtWidgets.QPushButton(text)                                             # b ← resultat de QPushButton()
    b.setFixedHeight(40)                                                        # executem setFixedHeight()
    b.setCursor(QtCore.Qt.PointingHandCursor)                                   # executem setCursor()
    bg = C_ACCENT if accent else "rgba(255,255,255,0.05)"                       # assignem bg
    b.setStyleSheet(f"""
        QPushButton {{
            background-color: {bg}; color: {C_TEXT};
            border: 1px solid {'transparent' if accent else 'rgba(255,255,255,0.15)'};
            border-radius: 10px; padding: 0 14px;
            font-size: 11px; font-weight: 700; letter-spacing: 1.5px;
        }}
        QPushButton:hover {{ border: 1px solid rgba(255,255,255,0.35); }}
        QPushButton:checked {{ background-color: {C_DANGER}; border: 1px solid {C_DANGER}; }}
        QPushButton:disabled {{ background-color: transparent; color: {C_TEXT_MUTED};
                                border: 1px solid rgba(255,255,255,0.05); }}
    """)
    return b                                                                    # retornem: b


def _ctl_caption(text):                                                         # definim la funció _ctl_caption()
    lbl = QtWidgets.QLabel(text)                                                # lbl ← resultat de QLabel()
    lbl.setStyleSheet(f"color: {C_TEXT_MUTED}; font-size: 10px; font-weight: 700; "
                      f"letter-spacing: 2px; background: transparent; border: none;")
    return lbl                                                                  # retornem: lbl


class NavControls(QtWidgets.QFrame):                                            # definim la classe NavControls
    """
    ① APPROACH  ② MODE BROCA  ③ BROCA start/stop + corrent  ④ PROFUNDITAT
    Cada botó crida la lògica que ja existeix al RobotWorkspace.
    """

    approachClicked = QtCore.Signal()                                           # approachClicked ← resultat de Signal()
    drillModeClicked = QtCore.Signal()                                          # drillModeClicked ← resultat de Signal()
    drillToggled = QtCore.Signal(bool)                                          # drillToggled ← resultat de Signal()
    currentChanged = QtCore.Signal(int)                                         # currentChanged ← resultat de Signal()
    depthChanged = QtCore.Signal(float)                                         # mm des de l'approach

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setObjectName("NavControls")                                       # executem setObjectName()
        self.setStyleSheet("""
            QFrame#NavControls { background-color: #0e0e10; border: 1px solid rgba(255,255,255,0.10);
                                  border-radius: 14px; }
            QLabel { background: transparent; border: none; }
        """)
        h = QtWidgets.QHBoxLayout(self)                                         # h ← resultat de QHBoxLayout()
        h.setContentsMargins(16, 12, 16, 12)                                    # executem setContentsMargins()
        h.setSpacing(12)                                                        # executem setSpacing()

        self.btn_approach = _ctl_button("①  APPROACH", accent=True)             # self.btn_approach ← resultat de _ctl_button()
        self.btn_approach.setToolTip("Mode IK6 i el robot va a 1 cm de l'entrada, alineat amb el cargol")
        self.btn_approach.clicked.connect(self.approachClicked.emit)            # executem connect()
        h.addWidget(self.btn_approach)                                          # executem addWidget()

        self.btn_mode = _ctl_button("②  MODE BROCA")                            # self.btn_mode ← resultat de _ctl_button()
        self.btn_mode.setToolTip("Mode IK5: J1-J5 aguanten la trajectòria i el J6 queda per a la broca")
        self.btn_mode.clicked.connect(self.drillModeClicked.emit)               # executem connect()
        h.addWidget(self.btn_mode)                                              # executem addWidget()

        # ③ Broca
        drill = QtWidgets.QVBoxLayout(); drill.setSpacing(4)                    # drill ← resultat de QVBoxLayout()
        self.lbl_current = _ctl_caption("BROCA · — mA")                         # self.lbl_current ← resultat de _ctl_caption()
        drill.addWidget(self.lbl_current)                                       # executem addWidget()
        row = QtWidgets.QHBoxLayout(); row.setSpacing(8)                        # row ← resultat de QHBoxLayout()
        self.btn_drill = _ctl_button("③  START")                                # self.btn_drill ← resultat de _ctl_button()
        self.btn_drill.setCheckable(True)                                       # executem setCheckable()
        self.btn_drill.setFixedWidth(120)                                       # executem setFixedWidth()
        self.btn_drill.toggled.connect(self._on_drill_toggled)                  # executem connect()
        row.addWidget(self.btn_drill)                                           # executem addWidget()
        self.sl_current = QtWidgets.QSlider(QtCore.Qt.Horizontal)               # self.sl_current ← resultat de QSlider()
        self.sl_current.setMinimumWidth(110)                                    # executem setMinimumWidth()
        self.sl_current.valueChanged.connect(self._on_current)                  # executem connect()
        row.addWidget(self.sl_current)                                          # executem addWidget()
        drill.addLayout(row)                                                    # executem addLayout()
        h.addLayout(drill, 2)                                                   # executem addLayout()

        # ④ Profunditat (DEPTH_STEP_MM per pas, des de l'approach)
        depth = QtWidgets.QVBoxLayout(); depth.setSpacing(4)                    # depth ← resultat de QVBoxLayout()
        self.lbl_depth = _ctl_caption("④  AVANÇ · 0.0 mm")                      # self.lbl_depth ← resultat de _ctl_caption()
        depth.addWidget(self.lbl_depth)                                         # executem addWidget()
        self.sl_depth = QtWidgets.QSlider(QtCore.Qt.Horizontal)                 # self.sl_depth ← resultat de QSlider()
        self.sl_depth.setRange(0, int(_DEPTH_STEPS_PER_MM * (APPROACH_MM + 40 + DEPTH_EXTRA_MM)))
        self.sl_depth.setSingleStep(1)                                          # fletxes: 1 pas (0.1 mm)
        self.sl_depth.setPageStep(_DEPTH_STEPS_PER_MM)                          # RePàg/AvPàg: 1 mm
        self.sl_depth.setFixedHeight(40)                                        # executem setFixedHeight()
        self.sl_depth.valueChanged.connect(self._on_depth)                      # executem connect()
        depth.addWidget(self.sl_depth)                                          # executem addWidget()
        h.addLayout(depth, 3)                                                   # executem addLayout()

    # ---- configuració des del workspace --------------------------------------

    def configure_current(self, lo, hi, value):                                 # definim la funció configure_current()
        self.sl_current.blockSignals(True)                                      # executem blockSignals()
        self.sl_current.setRange(int(lo), int(hi))                              # executem setRange()
        self.sl_current.setValue(int(value))                                    # executem setValue()
        self.sl_current.blockSignals(False)                                     # executem blockSignals()
        self.lbl_current.setText(f"BROCA · {int(value)} mA")                    # executem setText()

    def set_screw_length(self, length_mm):                                      # definim la funció set_screw_length()
        """L'avanç va de l'approach (0) fins a DEPTH_EXTRA_MM més enllà del final (10 + L + extra)."""
        self.sl_depth.blockSignals(True)                                        # executem blockSignals()
        self.sl_depth.setRange(0, int(round(_DEPTH_STEPS_PER_MM * (APPROACH_MM + length_mm + DEPTH_EXTRA_MM))))
        self.sl_depth.blockSignals(False)                                       # executem blockSignals()

    def reset_depth(self):                                                      # definim la funció reset_depth()
        self.sl_depth.blockSignals(True)                                        # executem blockSignals()
        self.sl_depth.setValue(0)                                               # executem setValue()
        self.sl_depth.blockSignals(False)                                       # executem blockSignals()
        self.lbl_depth.setText("④  AVANÇ · 0.0 mm")                             # executem setText()

    def set_drilling(self, on):                                                 # definim la funció set_drilling()
        if self.btn_drill.isChecked() != on:                                    # sense reemetre
            self.btn_drill.blockSignals(True)                                   # executem blockSignals()
            self.btn_drill.setChecked(on)                                       # executem setChecked()
            self.btn_drill.blockSignals(False)                                  # executem blockSignals()
        self.btn_drill.setText("■  STOP" if on else "③  START")                 # executem setText()

    # ---- interns -------------------------------------------------------------

    def _on_drill_toggled(self, on):                                            # definim la funció _on_drill_toggled()
        self.btn_drill.setText("■  STOP" if on else "③  START")                 # executem setText()
        self.drillToggled.emit(on)                                              # executem emit()

    def _on_current(self, v):                                                   # definim la funció _on_current()
        self.lbl_current.setText(f"BROCA · {v} mA")                             # executem setText()
        self.currentChanged.emit(int(v))                                        # executem emit()

    def _on_depth(self, v):                                                     # definim la funció _on_depth()
        mm = v / _DEPTH_STEPS_PER_MM                                            # passos → mm
        self.lbl_depth.setText(f"④  AVANÇ · {mm:.1f} mm  (entrada a {APPROACH_MM:.0f})")
        self.depthChanged.emit(mm)                                              # executem emit()


# ==============================================================================
# WORKSPACE PRINCIPAL DE NAVEGACIÓ
# ==============================================================================

class NavigationWorkspace(QtWidgets.QWidget):                                   # definim la classe NavigationWorkspace
    """
    Pantalla de guiat quirúrgic: el que mira el cirurgià mentre posa un cargol.

        ┌────────── HEADER ─ cargol actiu · precisió ───────────────────┐
        ├──────┬──────────────────────────────────────┬─────────────────┤
        │ HUD  │   PUNTERIA (mira)      │ PROFUNDITAT │  PLA QUIRÚRGIC  │
        │      │                        │             │                 │
        │      ├────────────────────────┴─────────────┤                 │
        │      │ ① APPROACH ② MODE BROCA ③ BROCA ④ AVANÇ               │
        └──────┴──────────────────────────────────────┴─────────────────┘

    No té lògica de robot pròpia: tot (modes, approach, broca, avanç) crida
    el RobotWorkspace, que és l'únic que parla amb els motors.
    """

    REFRESH_HZ = 20                                                             # refresc de la mira i les mètriques

    def __init__(self, mando_service=None, planning_workspace=None,             # definim la funció __init__()
                 robot_workspace=None, parent=None):
        super().__init__(parent)                                                # executem super()
        self.mando_service = mando_service                                      # assignem self.mando_service
        self.planning_workspace = planning_workspace                            # assignem self.planning_workspace
        self.robot_workspace = robot_workspace                                  # assignem self.robot_workspace

        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        root = QtWidgets.QVBoxLayout(self)                                      # root ← resultat de QVBoxLayout()
        root.setContentsMargins(16, 16, 16, 16)                                 # executem setContentsMargins()
        root.setSpacing(12)                                                     # executem setSpacing()

        self.header = NavigationHeader()                                        # self.header ← resultat de NavigationHeader()
        root.addWidget(self.header)                                             # executem addWidget()

        body = QtWidgets.QHBoxLayout()                                          # body ← resultat de QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)                                     # executem setContentsMargins()
        body.setSpacing(12)                                                     # executem setSpacing()

        self.hud = TelemetryHUD()                                               # self.hud ← resultat de TelemetryHUD()
        body.addWidget(self.hud)                                                # executem addWidget()

        # Centre: mira + profunditat a dalt, controls a baix
        center = QtWidgets.QVBoxLayout()                                        # center ← resultat de QVBoxLayout()
        center.setSpacing(12)                                                   # executem setSpacing()
        view_wrap = _segment_frame(bg="#050506", border_alpha=0.10)             # view_wrap ← resultat de _segment_frame()
        vw = QtWidgets.QHBoxLayout(view_wrap)                                   # vw ← resultat de QHBoxLayout()
        vw.setContentsMargins(2, 2, 2, 2)                                       # executem setContentsMargins()
        vw.setSpacing(0)                                                        # executem setSpacing()
        self.target_view = TargetingView()                                      # self.target_view ← resultat de TargetingView()
        vw.addWidget(self.target_view, 1)                                       # executem addWidget()
        sep = QtWidgets.QFrame(); sep.setFixedWidth(1)                          # sep ← resultat de QFrame()
        sep.setStyleSheet("background-color: rgba(255,255,255,0.08); border: none;")
        vw.addWidget(sep)                                                       # executem addWidget()
        self.depth_gauge = DepthGauge()                                         # self.depth_gauge ← resultat de DepthGauge()
        vw.addWidget(self.depth_gauge)                                          # executem addWidget()
        center.addWidget(view_wrap, 1)                                          # executem addWidget()

        self.controls = NavControls()                                           # self.controls ← resultat de NavControls()
        self.controls.approachClicked.connect(self._on_approach_clicked)        # executem connect()
        self.controls.drillModeClicked.connect(self._on_drill_mode_clicked)     # executem connect()
        self.controls.drillToggled.connect(self._on_drill_toggled)              # executem connect()
        self.controls.currentChanged.connect(self._on_current_changed)          # executem connect()
        self.controls.depthChanged.connect(self._on_depth_changed)              # executem connect()
        center.addWidget(self.controls)                                         # executem addWidget()
        body.addLayout(center, 1)                                               # executem addLayout()

        self.plan = SurgicalPlanPanel()                                         # self.plan ← resultat de SurgicalPlanPanel()
        self.plan.screwSelected.connect(self._on_screw_selected)                # executem connect()
        self.plan.btn_approach.hide()                                           # l'approach és a la barra de control (①)
        body.addWidget(self.plan)                                               # executem addWidget()

        root.addLayout(body, 1)                                                 # executem addLayout()

        # ---- Estat intern ----
        self._active_screw_idx = None                                           # assignem self._active_screw_idx
        self._screws_ref = None                                                 # llista de cargols que mostra el pla
        self._tick = 0                                                          # comptador de refrescos
        self._current_cfg = False                                               # slider de corrent ja configurat

        self._refresh_timer = QtCore.QTimer(self)                               # self._refresh_timer ← resultat de QTimer()
        self._refresh_timer.timeout.connect(self._refresh_telemetry)            # executem connect()
        self._refresh_timer.start(int(1000 / self.REFRESH_HZ))                  # executem start()

        QtCore.QTimer.singleShot(200, self._refresh_from_robot_workspace)       # executem singleShot()

    # ==================================================================
    # ACCÉS AL ROBOT
    # ==================================================================

    def _viewer(self):                                                          # definim la funció _viewer()
        return getattr(self.robot_workspace, "_inner", None) if self.robot_workspace else None

    def _robot_mode(self):                                                      # definim la funció _robot_mode()
        return getattr(self.robot_workspace, "_current_mode", None)             # retornem: mode actual del robot

    def _screws(self):                                                          # definim la funció _screws()
        return getattr(self.robot_workspace, "_reg_screws_m", []) if self.robot_workspace else []

    def _active_screw(self):                                                    # definim la funció _active_screw()
        s = self._screws()                                                      # s ← resultat de _screws()
        if self._active_screw_idx is None or not (0 <= self._active_screw_idx < len(s)):
            return None                                                         # retornem: None
        return s[self._active_screw_idx]                                        # retornem: el cargol actiu

    def _set_robot_mode(self, mode_id):                                         # definim la funció _set_robot_mode()
        """Canvia de mode exactament com si es cliqués el selector del Robot."""
        sel = getattr(self.robot_workspace, "_mode_selector", None)             # sel ← resultat de getattr()
        if sel is not None and self._robot_mode() != mode_id:                   # si cal canviar
            sel.set_mode(mode_id)                                               # emet modeChanged → _activate_mode
        return self._robot_mode() == mode_id                                    # retornem: s'hi ha arribat?

    # ==================================================================
    # PLA I ESTAT DEL SISTEMA
    # ==================================================================

    def _refresh_from_robot_workspace(self):                                    # definim la funció _refresh_from_robot_workspace()
        """
        Estat del sistema + llista de cargols. El pla NOMÉS es reconstrueix
        quan la llista de cargols canvia (abans es reconstruïa 10 cops per
        segon i esborrava la selecció: el workspace no es podia fer servir).
        """
        if self.robot_workspace is None:                                        # si self.robot_workspace is None
            return                                                              # retornem sense valor
        screws = self._screws()                                                 # screws ← resultat de _screws()
        if screws is not self._screws_ref:                                      # llista nova (registre nou)
            self._screws_ref = screws                                           # assignem self._screws_ref
            self.plan.set_screws(screws)                                        # executem set_screws()
            self._active_screw_idx = None                                       # assignem self._active_screw_idx
            self.header.set_screw_name("—")                                     # executem set_screw_name()
        self.hud.set_status(2, bool(screws), f"{len(screws)} cargols" if screws else "sense cargols")

        has_reg = bool(getattr(self.robot_workspace, "_reg_actors", []))        # has_reg ← resultat de bool()
        self.hud.set_status(1, has_reg, "OK" if has_reg else "pendent")         # executem set_status()

        connected = False                                                       # assignem connected
        try:                                                                    # intentem el bloc següent
            connected = self.robot_workspace.is_robot_connected()               # connected ← resultat de is_robot_connected()
        except Exception:                                                       # capturem excepció: Exception
            pass                                                                # no fem res
        self.hud.set_status(0, connected, "CONN" if connected else "OFF")       # executem set_status()

    def _on_screw_selected(self, idx):                                          # definim la funció _on_screw_selected()
        self._active_screw_idx = idx                                            # assignem self._active_screw_idx
        sc = self._active_screw()                                               # sc ← resultat de _active_screw()
        if sc is not None:                                                      # si sc is not None
            self.header.set_screw_name(sc.get("name", f"Cargol {idx + 1}"))
            self.controls.set_screw_length(sc["length_m"] * 1000.0)             # executem set_screw_length()
            self.controls.reset_depth()                                         # executem reset_depth()

    # ==================================================================
    # FLUX QUIRÚRGIC (tot delegat al RobotWorkspace)
    # ==================================================================

    def _on_approach_clicked(self):                                             # definim la funció _on_approach_clicked()
        """① IK6 i anar a 1 cm de l'entrada, alineat amb el cargol."""
        if self._active_screw() is None or self.robot_workspace is None:        # cal un cargol
            return                                                              # retornem sense valor
        if not self._set_robot_mode("ik6"):                                     # l'approach es fa en IK6
            print("[nav] no s'ha pogut passar a IK6")                           # executem print()
            return                                                              # retornem sense valor
        self.controls.reset_depth()                                             # executem reset_depth()
        self.robot_workspace._on_go_approach(self._active_screw_idx)            # executem _on_go_approach()

    def _on_drill_mode_clicked(self):                                           # definim la funció _on_drill_mode_clicked()
        """② IK5: J1-J5 aguanten la trajectòria, el J6 queda per a la broca."""
        if self._set_robot_mode("ik5"):                                         # si hi som
            self.controls.reset_depth()                                         # l'avanç parteix d'on som

    def _j6_panel(self):                                                        # definim la funció _j6_panel()
        return getattr(self.robot_workspace, "_j6_panel", None) if self.robot_workspace else None

    def _on_drill_toggled(self, on):                                            # definim la funció _on_drill_toggled()
        """③ Start/Stop: el mateix que els botons del panell J6 del Robot."""
        panel = self._j6_panel()                                                # panel ← resultat de _j6_panel()
        if panel is None:                                                       # si panel is None
            self.controls.set_drilling(False)                                   # executem set_drilling()
            return                                                              # retornem sense valor
        if on:                                                                  # Start
            if self._robot_mode() != "ik5":                                     # la broca només va en IK5
                QtWidgets.QMessageBox.information(
                    self, "Broca", "Passa primer a ② MODE BROCA (IK5).")
                self.controls.set_drilling(False)                               # executem set_drilling()
                return                                                          # retornem sense valor
            panel._on_start()                                                   # executem _on_start()
        else:                                                                   # Stop
            if panel.is_running():                                              # si panel.is_running()
                panel._on_stop()                                                # executem _on_stop()

    def _on_current_changed(self, mA):                                          # definim la funció _on_current_changed()
        """Corrent de la broca: mou l'slider del panell J6 (font de veritat)."""
        panel = self._j6_panel()                                                # panel ← resultat de _j6_panel()
        if panel is not None:                                                   # si panel is not None
            panel.slider_tor.setValue(int(mA))                                  # executem setValue()

    def _on_depth_changed(self, depth_mm):                                      # definim la funció _on_depth_changed()
        """④ Avanç al llarg de l'eix des de l'approach (IK5 o IK6)."""
        if self._active_screw() is None or self._robot_mode() not in ("ik5", "ik6"):
            return                                                              # retornem sense valor
        self.robot_workspace._on_trajectory_depth(self._active_screw_idx, depth_mm)

    # ==================================================================
    # REFRESC EN DIRECTE
    # ==================================================================

    def _refresh_telemetry(self):                                               # definim la funció _refresh_telemetry()
        if self.robot_workspace is None or not self.isVisible():                # amagat: res a fer
            return                                                              # retornem sense valor
        viewer = self._viewer()                                                 # viewer ← resultat de _viewer()
        if viewer is None:                                                      # si viewer is None
            return                                                              # retornem sense valor
        self._tick += 1                                                         # assignem self._tick
        if self._tick % 5 == 1:                                                 # estat del sistema a ~4 Hz
            self._refresh_from_robot_workspace()                                # executem _refresh_from_robot_workspace()

        mode = self._robot_mode()                                               # mode ← resultat de _robot_mode()
        sc = self._active_screw()                                               # sc ← resultat de _active_screw()
        panel = self._j6_panel()                                                # panel ← resultat de _j6_panel()
        drill = getattr(self.robot_workspace, "_drill", None)                   # drill ← resultat de getattr()
        drilling = bool(drill is not None and drill.state == "running")         # drilling ← resultat de bool()

        # Controls: què es pot fer ara
        if panel is not None and not self._current_cfg:                         # rang del corrent = el del panell J6
            s = panel.slider_tor                                                # assignem s
            self.controls.configure_current(max(s.minimum(), 0), s.maximum(), s.value())
            self._current_cfg = True                                            # assignem self._current_cfg
        self.controls.btn_approach.setEnabled(sc is not None)                   # executem setEnabled()
        self.controls.btn_mode.setEnabled(sc is not None and mode != "ik5")     # executem setEnabled()
        self.controls.btn_drill.setEnabled(mode == "ik5")                       # executem setEnabled()
        self.controls.sl_depth.setEnabled(sc is not None and mode in ("ik5", "ik6"))
        self.controls.set_drilling(bool(panel is not None and panel.is_running()))

        if sc is None:                                                          # sense cargol actiu
            self.target_view.set_metrics(None, "Selecciona un cargol del pla quirúrgic")
            self.depth_gauge.set_depth(None, 40.0)                              # executem set_depth()
            self.hud.set_distance(None, "Sense cargol seleccionat")             # executem set_distance()
            self.hud.set_angular(None, "Sense referència")                      # executem set_angular()
            self.hud.set_mode("STANDBY", "Selecciona un cargol")                # executem set_mode()
            self.header.set_precision("idle")                                   # executem set_precision()
            return                                                              # retornem sense valor

        try:                                                                    # intentem el bloc següent
            m = guidance_metrics(viewer.poses["tcp_link"], sc)                  # m ← resultat de guidance_metrics()
        except Exception:                                                       # capturem excepció: Exception
            return                                                              # retornem sense valor

        self.target_view.set_metrics(m)                                         # executem set_metrics()
        self.depth_gauge.set_depth(m["depth_mm"], m["length_mm"])               # executem set_depth()
        if self._tick % 2 == 0:                                                 # números grans a ~10 Hz (llegibles)
            self.hud.set_distance(m["lateral_mm"], "Punta ↔ trajectòria planificada")
            self.hud.set_angular(m["angle_deg"], "Eix de la broca ↔ eix del cargol")

        aligned = m["lateral_mm"] < TOL_OK_MM and m["angle_deg"] < TOL_OK_DEG   # aligned ← resultat de <()
        warn = m["lateral_mm"] < TOL_WARN_MM and m["angle_deg"] < TOL_WARN_DEG  # warn ← resultat de <()
        self.header.set_precision("ok" if aligned else ("warn" if warn else "danger"))

        # Fase del procediment (MODE del HUD)
        L, d = m["length_mm"], m["depth_mm"]                                    # assignem L, d
        if d > L + 0.2:                                                         # si d > L + 0.2
            self.hud.set_mode("PASSAT", f"{d - L:.1f} mm més enllà del final")
        elif d >= L - 0.2:                                                      # altrament si d >= L - 0.2
            self.hud.set_mode("FINAL", "Profunditat planificada assolida")      # executem set_mode()
        elif drilling:                                                          # altrament si drilling
            self.hud.set_mode("TREPANT", f"{d:.1f} / {L:.0f} mm")
        elif mode == "ik5":                                                     # altrament si mode == "ik5"
            self.hud.set_mode("BROCA LLESTA", "③ Start · ④ Avança")             # executem set_mode()
        elif aligned:                                                           # altrament si aligned
            self.hud.set_mode("ALINEAT", "② Mode broca")                        # executem set_mode()
        else:                                                                   # en cas contrari
            self.hud.set_mode("APROXIMACIÓ", "① Approach")                      # executem set_mode()

    # ==================================================================
    def cleanup(self):                                                          # definim la funció cleanup()
        self._refresh_timer.stop()                                              # executem stop()
