import numpy as np
from qtpy import QtWidgets, QtCore, QtGui

from ..theme import (
    C_BG_WINDOW, C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_ACCENT, C_SAFE, C_WARN, C_DANGER,
    SIZE_XS, SIZE_BASE, FONT_MONO,
)


TOL_OK_MM    = 1.0
TOL_WARN_MM  = 2.0
TOL_OK_DEG   = 1.0
TOL_WARN_DEG = 2.0
TAIL_MM      = 40.0
APPROACH_MM  = 10.0
DEPTH_STEP_MM  = 0.1
DEPTH_EXTRA_MM = 5.0
_DEPTH_STEPS_PER_MM = round(1.0 / DEPTH_STEP_MM)


def _tol_color(value, ok, warn):
    if value < ok:
        return C_SAFE
    if value < warn:
        return C_WARN
    return C_DANGER


def _segment_frame(bg="#0e0e10", border_alpha=0.10):
    f = QtWidgets.QFrame()
    f.setStyleSheet(f"""
        QFrame {{
            background-color: {bg};
            border: 1px solid rgba(255, 255, 255, {border_alpha});
            border-radius: 14px;
        }}
    """)
    return f


def _mono_label(text, size=28, color="#ffffff", weight=700, letter=1):
    lbl = QtWidgets.QLabel(text)
    lbl.setAlignment(QtCore.Qt.AlignCenter)
    lbl.setStyleSheet(
        f"color: {color}; font-family: {FONT_MONO}; "
        f"font-size: {size}px; font-weight: {weight}; "
        f"letter-spacing: {letter}px; background: transparent; border: none;"
    )
    return lbl


def _eyebrow(text, color=None):
    if color is None:
        color = C_TEXT_MUTED
    lbl = QtWidgets.QLabel(text.upper())
    lbl.setAlignment(QtCore.Qt.AlignCenter)
    lbl.setStyleSheet(
        f"color: {color}; font-size: 10px; font-weight: 700; "
        f"letter-spacing: 3px; background: transparent; border: none;"
    )
    return lbl


class TelemetryHUD(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(280)
        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(16, 16, 8, 16)
        v.setSpacing(14)

        seg_mode = _segment_frame()
        seg_mode.setFixedHeight(110)
        m = QtWidgets.QVBoxLayout(seg_mode)
        m.setContentsMargins(20, 18, 20, 18)
        m.setSpacing(6)
        m.addWidget(_eyebrow("MODE", color=C_ACCENT))
        m.addSpacing(6)
        self.lbl_mode = _mono_label("STANDBY", size=22, letter=3)
        m.addWidget(self.lbl_mode)
        self.lbl_mode_hint = QtWidgets.QLabel("Esperant selecció de cargol")
        self.lbl_mode_hint.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_mode_hint.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-size: 10px; "
            f"letter-spacing: 0.5px; background: transparent; border: none;"
        )
        m.addWidget(self.lbl_mode_hint)
        v.addWidget(seg_mode)

        seg_pos = _segment_frame()
        seg_pos.setFixedHeight(160)
        p = QtWidgets.QVBoxLayout(seg_pos)
        p.setContentsMargins(20, 18, 20, 18)
        p.setSpacing(4)
        p.addWidget(_eyebrow("DESVIACIÓ LATERAL"))
        p.addSpacing(8)

        row_num = QtWidgets.QHBoxLayout()
        row_num.setSpacing(4)
        row_num.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_dist_num = _mono_label("—", size=44, letter=-1)
        row_num.addWidget(self.lbl_dist_num)
        self.lbl_dist_unit = QtWidgets.QLabel("mm")
        self.lbl_dist_unit.setStyleSheet(
            f"color: {C_TEXT_DIM}; font-size: 14px; font-weight: 500; "
            f"letter-spacing: 1px; background: transparent; border: none; "
            f"padding-top: 22px;"
        )
        row_num.addWidget(self.lbl_dist_unit)
        p.addLayout(row_num)

        self.pb_dist = QtWidgets.QProgressBar()
        self.pb_dist.setRange(0, 100)
        self.pb_dist.setValue(0)
        self.pb_dist.setTextVisible(False)
        self.pb_dist.setFixedHeight(4)
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
        p.addWidget(self.pb_dist)
        p.addSpacing(4)
        self.lbl_dist_hint = QtWidgets.QLabel("Sense cargol seleccionat")
        self.lbl_dist_hint.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_dist_hint.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-size: 10px; "
            f"letter-spacing: 0.5px; background: transparent; border: none;"
        )
        p.addWidget(self.lbl_dist_hint)

        v.addWidget(seg_pos)

        seg_ang = _segment_frame()
        seg_ang.setFixedHeight(160)
        a = QtWidgets.QVBoxLayout(seg_ang)
        a.setContentsMargins(20, 18, 20, 18)
        a.setSpacing(4)
        a.addWidget(_eyebrow("DESVIACIÓ ANGULAR"))
        a.addSpacing(8)

        row_ang = QtWidgets.QHBoxLayout()
        row_ang.setSpacing(4)
        row_ang.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_ang_num = _mono_label("—", size=44, letter=-1)
        row_ang.addWidget(self.lbl_ang_num)
        self.lbl_ang_unit = QtWidgets.QLabel("°")
        self.lbl_ang_unit.setStyleSheet(
            f"color: {C_TEXT_DIM}; font-size: 20px; font-weight: 500; "
            f"background: transparent; border: none; padding-top: 18px;"
        )
        row_ang.addWidget(self.lbl_ang_unit)
        a.addLayout(row_ang)

        self.pb_ang = QtWidgets.QProgressBar()
        self.pb_ang.setRange(0, 100)
        self.pb_ang.setValue(0)
        self.pb_ang.setTextVisible(False)
        self.pb_ang.setFixedHeight(4)
        self.pb_ang.setStyleSheet(self.pb_dist.styleSheet())
        a.addWidget(self.pb_ang)
        a.addSpacing(4)
        self.lbl_ang_hint = QtWidgets.QLabel("Sense referència")
        self.lbl_ang_hint.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_ang_hint.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-size: 10px; "
            f"letter-spacing: 0.5px; background: transparent; border: none;"
        )
        a.addWidget(self.lbl_ang_hint)

        v.addWidget(seg_ang)

        seg_sys = _segment_frame()
        s = QtWidgets.QVBoxLayout(seg_sys)
        s.setContentsMargins(20, 18, 20, 18)
        s.setSpacing(10)
        s.addWidget(_eyebrow("SISTEMA"))
        s.addSpacing(4)

        self.status_lines = []
        for label in ("Robot", "Registre", "Pla quirúrgic"):
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(10)
            dot = QtWidgets.QLabel("●")
            dot.setStyleSheet(
                f"color: {C_TEXT_MUTED}; font-size: 11px; "
                f"background: transparent; border: none;"
            )
            row.addWidget(dot)
            lbl = QtWidgets.QLabel(label)
            lbl.setStyleSheet(
                f"color: {C_TEXT_DIM}; font-size: 12px; font-weight: 500; "
                f"letter-spacing: 0.3px; background: transparent; border: none;"
            )
            row.addWidget(lbl)
            row.addStretch()
            val = QtWidgets.QLabel("—")
            val.setStyleSheet(
                f"color: {C_TEXT_MUTED}; font-family: {FONT_MONO}; "
                f"font-size: 10px; letter-spacing: 0.5px; "
                f"background: transparent; border: none;"
            )
            row.addWidget(val)
            s.addLayout(row)
            self.status_lines.append((dot, lbl, val))

        s.addStretch()
        v.addWidget(seg_sys, 1)

    def set_mode(self, mode_text, hint_text=""):
        self.lbl_mode.setText(mode_text)
        self.lbl_mode_hint.setText(hint_text)

    def set_distance(self, dist_mm, hint_text=""):
        if dist_mm is None:
            self.lbl_dist_num.setText("—")
            self.lbl_dist_num.setStyleSheet(
                f"color: {C_TEXT_DIM}; font-family: {FONT_MONO}; "
                f"font-size: 44px; font-weight: 700; letter-spacing: -1px; "
                f"background: transparent; border: none;"
            )
            self.pb_dist.setValue(0)
        else:
            self.lbl_dist_num.setText(f"{dist_mm:.1f}")
            col = _tol_color(dist_mm, TOL_OK_MM, TOL_WARN_MM)
            self.lbl_dist_num.setStyleSheet(
                f"color: {col}; font-family: {FONT_MONO}; "
                f"font-size: 44px; font-weight: 700; letter-spacing: -1px; "
                f"background: transparent; border: none;"
            )
            pct = int(np.clip(100.0 - (abs(dist_mm) / 5.0) * 100.0, 0, 100))
            self.pb_dist.setValue(pct)
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
        self.lbl_dist_hint.setText(hint_text)

    def set_angular(self, deg, hint_text=""):
        if deg is None:
            self.lbl_ang_num.setText("—")
            self.lbl_ang_num.setStyleSheet(
                f"color: {C_TEXT_DIM}; font-family: {FONT_MONO}; "
                f"font-size: 44px; font-weight: 700; letter-spacing: -1px; "
                f"background: transparent; border: none;"
            )
            self.pb_ang.setValue(0)
        else:
            self.lbl_ang_num.setText(f"{deg:.1f}")
            col = _tol_color(deg, TOL_OK_DEG, TOL_WARN_DEG)
            self.lbl_ang_num.setStyleSheet(
                f"color: {col}; font-family: {FONT_MONO}; "
                f"font-size: 44px; font-weight: 700; letter-spacing: -1px; "
                f"background: transparent; border: none;"
            )
            pct = int(np.clip(100.0 - (deg / 5.0) * 100.0, 0, 100))
            self.pb_ang.setValue(pct)
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
        self.lbl_ang_hint.setText(hint_text)

    def set_status(self, row_idx, ok, detail=""):
        if row_idx < 0 or row_idx >= len(self.status_lines):
            return
        dot, lbl, val = self.status_lines[row_idx]
        if ok:
            dot.setStyleSheet(
                f"color: {C_SAFE}; font-size: 11px; "
                f"background: transparent; border: none;"
            )
        else:
            dot.setStyleSheet(
                f"color: {C_TEXT_MUTED}; font-size: 11px; "
                f"background: transparent; border: none;"
            )
        val.setText(detail)


class NavigationHeader(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(58)
        self.setStyleSheet("""
            QFrame {
                background-color: #0a0a0c;
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 14px;
            }
        """)

        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(22, 0, 22, 0)
        h.setSpacing(20)

        eb = QtWidgets.QLabel("CARGOL ACTIU")
        eb.setStyleSheet(
            f"color: {C_ACCENT}; font-size: 10px; font-weight: 700; "
            f"letter-spacing: 3px; background: transparent; border: none;"
        )
        h.addWidget(eb)

        self.lbl_screw = QtWidgets.QLabel("— cap seleccionat —")
        self.lbl_screw.setStyleSheet(
            f"color: {C_TEXT}; font-size: 18px; font-weight: 700; "
            f"letter-spacing: 1px; background: transparent; border: none;"
        )
        h.addWidget(self.lbl_screw)
        h.addStretch()

        self.pill_precision = QtWidgets.QLabel("●  ESPERANT")
        self.pill_precision.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-size: 11px; font-weight: 700; "
            f"letter-spacing: 2px; background: rgba(255,255,255,0.04); "
            f"border: 1px solid rgba(255,255,255,0.10); "
            f"border-radius: 12px; padding: 6px 14px;"
        )
        h.addWidget(self.pill_precision)

    def set_screw_name(self, name):
        self.lbl_screw.setText(name if name else "— cap seleccionat —")

    def set_precision(self, level):
        colors = {
            "ok":     (C_SAFE,      "PRECÍS"),
            "warn":   (C_WARN,      "ATENCIÓ"),
            "danger": (C_DANGER,    "DESVIACIÓ ALTA"),
            "idle":   (C_TEXT_MUTED, "ESPERANT"),
        }
        col, txt = colors.get(level, colors["idle"])
        self.pill_precision.setText(f"●  {txt}")
        self.pill_precision.setStyleSheet(
            f"color: {col}; font-size: 11px; font-weight: 700; "
            f"letter-spacing: 2px; background: rgba(255,255,255,0.04); "
            f"border: 1px solid rgba(255,255,255,0.10); "
            f"border-radius: 12px; padding: 6px 14px;"
        )


class ScrewProgressItem(QtWidgets.QFrame):
    clicked = QtCore.Signal(int)

    def __init__(self, idx, name, parent=None):
        super().__init__(parent)
        self.idx = idx
        self.name = name
        self.state = "pending"
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setFixedHeight(58)

        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(14, 10, 14, 10)
        h.setSpacing(12)

        self.lbl_num = QtWidgets.QLabel(f"{idx + 1:02d}")
        self.lbl_num.setFixedWidth(28)
        self.lbl_num.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-family: {FONT_MONO}; "
            f"font-size: 15px; font-weight: 700; letter-spacing: 0.5px; "
            f"background: transparent; border: none;"
        )
        h.addWidget(self.lbl_num)

        col_txt = QtWidgets.QVBoxLayout()
        col_txt.setSpacing(2)
        self.lbl_name = QtWidgets.QLabel(name)
        self.lbl_name.setStyleSheet(
            f"color: {C_TEXT}; font-size: 13px; font-weight: 600; "
            f"letter-spacing: 0.3px; background: transparent; border: none;"
        )
        col_txt.addWidget(self.lbl_name)

        self.lbl_status = QtWidgets.QLabel("Pendent")
        self.lbl_status.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-size: 10px; font-weight: 500; "
            f"letter-spacing: 0.5px; background: transparent; border: none;"
        )
        col_txt.addWidget(self.lbl_status)
        h.addLayout(col_txt, 1)

        self.dot = QtWidgets.QLabel("○")
        self.dot.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-size: 16px; "
            f"background: transparent; border: none;"
        )
        h.addWidget(self.dot)

        self._apply_style()

    def _apply_style(self):
        if self.state == "active":
            self.setStyleSheet("""
                ScrewProgressItem {
                    background-color: rgba(10, 132, 255, 0.10);
                    border: 1px solid rgba(10, 132, 255, 0.45);
                    border-radius: 10px;
                }
            """)
            self.dot.setText("●")
            self.dot.setStyleSheet(
                f"color: {C_ACCENT}; font-size: 16px; "
                f"background: transparent; border: none;"
            )
            self.lbl_status.setText("En curs")
            self.lbl_status.setStyleSheet(
                f"color: {C_ACCENT}; font-size: 10px; font-weight: 700; "
                f"letter-spacing: 0.5px; background: transparent; border: none;"
            )
        elif self.state == "done":
            self.setStyleSheet("""
                ScrewProgressItem {
                    background-color: rgba(48, 209, 88, 0.08);
                    border: 1px solid rgba(48, 209, 88, 0.30);
                    border-radius: 10px;
                }
            """)
            self.dot.setText("✓")
            self.dot.setStyleSheet(
                f"color: {C_SAFE}; font-size: 16px; font-weight: 700; "
                f"background: transparent; border: none;"
            )
            self.lbl_status.setText("Completat")
            self.lbl_status.setStyleSheet(
                f"color: {C_SAFE}; font-size: 10px; font-weight: 700; "
                f"letter-spacing: 0.5px; background: transparent; border: none;"
            )
        else:
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
            self.dot.setText("○")
            self.dot.setStyleSheet(
                f"color: {C_TEXT_MUTED}; font-size: 16px; "
                f"background: transparent; border: none;"
            )
            self.lbl_status.setText("Pendent")
            self.lbl_status.setStyleSheet(
                f"color: {C_TEXT_MUTED}; font-size: 10px; font-weight: 500; "
                f"letter-spacing: 0.5px; background: transparent; border: none;"
            )

    def set_state(self, state):
        self.state = state
        self._apply_style()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.clicked.emit(self.idx)
        super().mousePressEvent(event)


class SurgicalPlanPanel(QtWidgets.QWidget):
    screwSelected = QtCore.Signal(int)
    approachRequested = QtCore.Signal(int)
    completeRequested = QtCore.Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(300)
        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 16, 16, 16)
        v.setSpacing(14)

        list_card = _segment_frame()
        lc = QtWidgets.QVBoxLayout(list_card)
        lc.setContentsMargins(16, 18, 16, 16)
        lc.setSpacing(10)

        header_row = QtWidgets.QHBoxLayout()
        header_row.setSpacing(8)
        eb = _eyebrow("PLA QUIRÚRGIC")
        eb.setAlignment(QtCore.Qt.AlignLeft)
        header_row.addWidget(eb)
        header_row.addStretch()
        self.lbl_counter = QtWidgets.QLabel("0 / 0")
        self.lbl_counter.setStyleSheet(
            f"color: {C_TEXT_MUTED}; font-family: {FONT_MONO}; "
            f"font-size: 11px; font-weight: 700; letter-spacing: 1px; "
            f"background: transparent; border: none;"
        )
        header_row.addWidget(self.lbl_counter)
        lc.addLayout(header_row)
        lc.addSpacing(4)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setStyleSheet(
            "QScrollArea { background: transparent; border: none; } "
            "QScrollBar:vertical { background: transparent; width: 6px; } "
            "QScrollBar::handle:vertical { "
            "  background: rgba(255,255,255,0.15); border-radius: 3px; } "
            "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }"
        )
        self.items_container = QtWidgets.QWidget()
        self.items_container.setStyleSheet("background: transparent;")
        self.items_layout = QtWidgets.QVBoxLayout(self.items_container)
        self.items_layout.setContentsMargins(0, 0, 0, 0)
        self.items_layout.setSpacing(6)
        self.items_layout.addStretch()
        scroll.setWidget(self.items_container)
        lc.addWidget(scroll, 1)

        v.addWidget(list_card, 1)

        actions = QtWidgets.QVBoxLayout()
        actions.setSpacing(8)

        self.btn_approach = self._make_action_button(
            "ANAR A APPROACH", accent=True)
        self.btn_approach.clicked.connect(self._on_approach)
        self.btn_approach.setEnabled(False)
        actions.addWidget(self.btn_approach)

        self.btn_complete = self._make_action_button(
            "MARCAR COMPLETAT", accent=False)
        self.btn_complete.clicked.connect(self._on_complete)
        self.btn_complete.setEnabled(False)
        actions.addWidget(self.btn_complete)

        v.addLayout(actions)

        self.items = []
        self._active_idx = None

    def _make_action_button(self, text, accent=False):
        btn = QtWidgets.QPushButton(text)
        btn.setFixedHeight(46)
        btn.setCursor(QtCore.Qt.PointingHandCursor)
        if accent:
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
        else:
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
        return btn

    def set_screws(self, screws_list):
        for item in self.items:
            self.items_layout.removeWidget(item)
            item.deleteLater()
        self.items = []

        while self.items_layout.count():
            item = self.items_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

        for i, sc in enumerate(screws_list):
            name = sc.get("name", f"Cargol {i+1}")
            it = ScrewProgressItem(i, name)
            it.clicked.connect(self._on_item_clicked)
            self.items_layout.addWidget(it)
            self.items.append(it)
        self.items_layout.addStretch()

        self._active_idx = None
        self._update_counter()
        self.btn_approach.setEnabled(False)
        self.btn_complete.setEnabled(False)

    def _on_item_clicked(self, idx):
        self.set_active(idx)
        self.screwSelected.emit(idx)

    def set_active(self, idx):
        for i, it in enumerate(self.items):
            if i == idx:
                if it.state != "done":
                    it.set_state("active")
            elif it.state == "active":
                it.set_state("pending")
        self._active_idx = idx
        self.btn_approach.setEnabled(idx is not None)
        self.btn_complete.setEnabled(
            idx is not None and self.items[idx].state == "active"
        )

    def _on_approach(self):
        if self._active_idx is not None:
            self.approachRequested.emit(self._active_idx)

    def _on_complete(self):
        if self._active_idx is not None:
            self.items[self._active_idx].set_state("done")
            self.completeRequested.emit(self._active_idx)
            self._update_counter()
            self.btn_complete.setEnabled(False)

    def _update_counter(self):
        done = sum(1 for it in self.items if it.state == "done")
        total = len(self.items)
        self.lbl_counter.setText(f"{done:02d} / {total:02d}")


def screw_frame(axis_unit):
    d = np.asarray(axis_unit, dtype=float)
    ref = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(ref, d)) > 0.9:
        ref = np.array([1.0, 0.0, 0.0])
    v = ref - np.dot(ref, d) * d
    v /= np.linalg.norm(v)
    u = np.cross(d, v)
    return u, v


def guidance_metrics(T_tcp, screw):
    p0 = np.asarray(screw["p0_m"], dtype=float)
    d = np.asarray(screw["axis_unit"], dtype=float)
    tip = np.asarray(T_tcp[:3, 3], dtype=float)
    x_tool = np.asarray(T_tcp[:3, 0], dtype=float)
    u, v = screw_frame(d)

    rel = tip - p0
    depth = float(np.dot(rel, d))
    lat = rel - depth * d
    tail = tip - x_tool * (TAIL_MM * 1e-3)
    rel_t = tail - p0
    lat_t = rel_t - np.dot(rel_t, d) * d
    cos_a = float(np.clip(np.dot(x_tool, d), -1.0, 1.0))
    return {
        "depth_mm":   depth * 1000.0,
        "lateral_mm": float(np.linalg.norm(lat)) * 1000.0,
        "angle_deg":  float(np.degrees(np.arccos(cos_a))),
        "tip_uv":     (float(np.dot(lat, u)) * 1000.0, float(np.dot(lat, v)) * 1000.0),
        "tail_uv":    (float(np.dot(lat_t, u)) * 1000.0, float(np.dot(lat_t, v)) * 1000.0),
        "length_mm":  float(screw["length_m"]) * 1000.0,
    }


class TargetingView(QtWidgets.QWidget):
    SCALES_MM = (3.0, 6.0, 12.0, 25.0, 50.0)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 320)
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, True)
        self.m = None
        self.scale_mm = self.SCALES_MM[-1]
        self.message = "Selecciona un cargol del pla quirúrgic"

    def set_metrics(self, m, message=None):
        self.m = m
        if message is not None:
            self.message = message
        if m is not None:
            need = 1.25 * max(np.hypot(*m["tip_uv"]), np.hypot(*m["tail_uv"]), 0.5)
            fit = next((s for s in self.SCALES_MM if s >= need), self.SCALES_MM[-1])
            if fit > self.scale_mm or need < 0.55 * self.scale_mm:
                self.scale_mm = fit
        self.update()

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#050506"))
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        R = min(w, h) / 2.0 - 34

        f_small = QtGui.QFont(self.font()); f_small.setPixelSize(SIZE_XS)
        f_tag = QtGui.QFont(self.font()); f_tag.setPixelSize(SIZE_XS); f_tag.setBold(True)
        f_tag.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 3)

        p.setFont(f_tag)
        p.setPen(QtGui.QColor(C_ACCENT))
        p.drawText(QtCore.QRectF(18, 14, w - 36, 16), QtCore.Qt.AlignLeft, "PUNTERIA")
        p.setFont(f_small)
        p.setPen(QtGui.QColor(C_TEXT_MUTED))
        p.drawText(QtCore.QRectF(18, 30, w - 36, 16), QtCore.Qt.AlignLeft,
                   "mirant al llarg de l'eix del cargol")

        if self.m is None:
            p.setPen(QtGui.QColor(C_TEXT_DIM))
            f = QtGui.QFont(self.font()); f.setPixelSize(SIZE_BASE)
            p.setFont(f)
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, self.message)
            p.end()
            return

        k = R / self.scale_mm

        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 22), 1))
        p.drawLine(QtCore.QPointF(cx - R, cy), QtCore.QPointF(cx + R, cy))
        p.drawLine(QtCore.QPointF(cx, cy - R), QtCore.QPointF(cx, cy + R))

        p.setPen(QtCore.Qt.NoPen)
        ok_col = QtGui.QColor(C_SAFE); ok_col.setAlpha(28)
        p.setBrush(ok_col)
        p.drawEllipse(QtCore.QPointF(cx, cy), TOL_OK_MM * k, TOL_OK_MM * k)
        p.setBrush(QtCore.Qt.NoBrush)
        p.setFont(f_small)
        for ring in (1, 2, 5, 10, 20, 50):
            if ring > self.scale_mm + 1e-6:
                break
            r = ring * k
            alpha = 90 if ring in (1, 2) else 45
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, alpha), 1))
            p.drawEllipse(QtCore.QPointF(cx, cy), r, r)
            p.setPen(QtGui.QColor(C_TEXT_MUTED))
            p.drawText(QtCore.QPointF(cx + r * 0.7071 + 4, cy - r * 0.7071 - 4), f"{ring} mm")

        def to_px(uv):
            x, y = uv[0] * k, -uv[1] * k
            n = np.hypot(x, y)
            out = n > R
            if out:
                x, y = x * R / n, y * R / n
            return QtCore.QPointF(cx + x, cy + y), out

        tip_pt, tip_out = to_px(self.m["tip_uv"])
        tail_pt, tail_out = to_px(self.m["tail_uv"])
        col_tip = QtGui.QColor(_tol_color(self.m["lateral_mm"], TOL_OK_MM, TOL_WARN_MM))
        col_tail = QtGui.QColor(_tol_color(self.m["angle_deg"], TOL_OK_DEG, TOL_WARN_DEG))

        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 110), 2))
        p.drawLine(tail_pt, tip_pt)

        p.setPen(QtGui.QPen(col_tail, 2.5))
        p.setBrush(QtCore.Qt.NoBrush)
        p.drawEllipse(tail_pt, 13, 13)

        p.setPen(QtGui.QPen(QtGui.QColor("#050506"), 3))
        p.setBrush(col_tip)
        p.drawEllipse(tip_pt, 8, 8)

        p.setPen(QtGui.QPen(QtGui.QColor(C_TEXT), 1.5))
        p.drawLine(QtCore.QPointF(cx - 6, cy), QtCore.QPointF(cx + 6, cy))
        p.drawLine(QtCore.QPointF(cx, cy - 6), QtCore.QPointF(cx, cy + 6))

        p.setFont(f_small)
        p.setPen(QtGui.QColor(C_TEXT_MUTED))
        legend = f"●  punta     ○  cua ({TAIL_MM:.0f} mm enrere)     escala ±{self.scale_mm:.0f} mm"
        if tip_out or tail_out:
            legend += "     ▲ fora d'escala"
        p.drawText(QtCore.QRectF(18, h - 26, w - 36, 16), QtCore.Qt.AlignLeft, legend)
        p.end()


class DepthGauge(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(190)
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, True)
        self.depth = None
        self.length = 40.0

    def set_depth(self, depth_mm, length_mm):
        self.depth, self.length = depth_mm, max(float(length_mm), 1.0)
        self.update()

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#050506"))
        w, h = self.width(), self.height()
        f_small = QtGui.QFont(self.font()); f_small.setPixelSize(SIZE_XS)
        f_tag = QtGui.QFont(self.font()); f_tag.setPixelSize(SIZE_XS); f_tag.setBold(True)
        f_tag.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 3)
        f_big = QtGui.QFont("Consolas")
        f_big.setStyleHint(QtGui.QFont.Monospace)
        f_big.setPixelSize(30); f_big.setBold(True)

        p.setFont(f_tag)
        p.setPen(QtGui.QColor(C_ACCENT))
        p.drawText(QtCore.QRectF(14, 14, w - 28, 16), QtCore.Qt.AlignLeft, "PROFUNDITAT")

        if self.depth is None:
            p.setPen(QtGui.QColor(C_TEXT_MUTED))
            p.setFont(f_small)
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, "—")
            p.end()
            return

        L, d = self.length, self.depth
        p.setFont(f_big)
        over = d > L + 0.2
        p.setPen(QtGui.QColor(C_DANGER if over else C_TEXT))
        p.drawText(QtCore.QRectF(14, 34, w - 28, 36), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
                   f"{d:+.1f}")
        p.setFont(f_small)
        p.setPen(QtGui.QColor(C_TEXT_DIM))
        rest = L - d
        txt = ("abans de l'entrada" if d < 0 else
               (f"queden {rest:.1f} mm" if rest > 0.2 else
                ("final assolit" if not over else f"PASSAT {-rest:.1f} mm")))
        p.drawText(QtCore.QRectF(14, 70, w - 28, 16), QtCore.Qt.AlignLeft, f"mm · {txt}")

        top_mm, bot_mm = -APPROACH_MM - 3.0, L + 6.0
        y0, y1 = 104.0, h - 24.0
        def y(mm):
            return y0 + (mm - top_mm) / (bot_mm - top_mm) * (y1 - y0)
        bx, bw = 34.0, 30.0

        danger = QtGui.QColor(C_DANGER); danger.setAlpha(45)
        p.fillRect(QtCore.QRectF(bx - 10, y(L), bw + 20, y(bot_mm) - y(L)), danger)
        p.fillRect(QtCore.QRectF(bx + bw / 2 - 1, y(top_mm), 2, y(0) - y(top_mm)), QtGui.QColor(255, 255, 255, 30))
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 140), 1))
        p.setBrush(QtGui.QColor(255, 255, 255, 18))
        p.drawRoundedRect(QtCore.QRectF(bx, y(0), bw, y(L) - y(0)), 4, 4)
        if d > 0:
            prog = QtGui.QColor(C_DANGER if over else C_ACCENT); prog.setAlpha(160)
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(prog)
            p.drawRoundedRect(QtCore.QRectF(bx + 3, y(0) + 1, bw - 6, y(min(d, bot_mm)) - y(0) - 1), 3, 3)

        p.setFont(f_small)
        for mm, label, col in ((-APPROACH_MM, "APPROACH", C_TEXT_MUTED),
                               (0.0, "ENTRADA", C_TEXT_DIM),
                               (L, f"FINAL · {L:.0f}", C_TEXT_DIM)):
            yy = y(mm)
            p.setPen(QtGui.QPen(QtGui.QColor(col), 1, QtCore.Qt.DashLine))
            p.drawLine(QtCore.QPointF(bx - 10, yy), QtCore.QPointF(bx + bw + 10, yy))
            p.setPen(QtGui.QColor(col))
            p.drawText(QtCore.QRectF(bx + bw + 14, yy - 8, w - bx - bw - 16, 16),
                       QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, label)

        yd = y(float(np.clip(d, top_mm, bot_mm)))
        tip_col = QtGui.QColor(C_DANGER if over else C_TEXT)
        p.setPen(QtGui.QPen(tip_col, 2))
        p.drawLine(QtCore.QPointF(bx - 12, yd), QtCore.QPointF(bx + bw + 4, yd))
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(tip_col)
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(bx - 12, yd - 6), QtCore.QPointF(bx - 4, yd),
                                       QtCore.QPointF(bx - 12, yd + 6)]))
        p.end()


def _ctl_button(text, accent=False):
    b = QtWidgets.QPushButton(text)
    b.setFixedHeight(40)
    b.setCursor(QtCore.Qt.PointingHandCursor)
    bg = C_ACCENT if accent else "rgba(255,255,255,0.05)"
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
    return b


def _ctl_caption(text):
    lbl = QtWidgets.QLabel(text)
    lbl.setStyleSheet(f"color: {C_TEXT_MUTED}; font-size: 10px; font-weight: 700; "
                      f"letter-spacing: 2px; background: transparent; border: none;")
    return lbl


class NavControls(QtWidgets.QFrame):
    approachClicked = QtCore.Signal()
    drillModeClicked = QtCore.Signal()
    drillToggled = QtCore.Signal(bool)
    currentChanged = QtCore.Signal(int)
    depthChanged = QtCore.Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("NavControls")
        self.setStyleSheet("""
            QFrame#NavControls { background-color: #0e0e10; border: 1px solid rgba(255,255,255,0.10);
                                  border-radius: 14px; }
            QLabel { background: transparent; border: none; }
        """)
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(16, 12, 16, 12)
        h.setSpacing(12)

        self.btn_approach = _ctl_button("①  APPROACH", accent=True)
        self.btn_approach.setToolTip("Mode IK6 i el robot va a 1 cm de l'entrada, alineat amb el cargol")
        self.btn_approach.clicked.connect(self.approachClicked.emit)
        h.addWidget(self.btn_approach)

        self.btn_mode = _ctl_button("②  MODE BROCA")
        self.btn_mode.setToolTip("Mode IK5: J1-J5 aguanten la trajectòria i el J6 queda per a la broca")
        self.btn_mode.clicked.connect(self.drillModeClicked.emit)
        h.addWidget(self.btn_mode)

        drill = QtWidgets.QVBoxLayout(); drill.setSpacing(4)
        self.lbl_current = _ctl_caption("BROCA · — mA")
        drill.addWidget(self.lbl_current)
        row = QtWidgets.QHBoxLayout(); row.setSpacing(8)
        self.btn_drill = _ctl_button("③  START")
        self.btn_drill.setCheckable(True)
        self.btn_drill.setFixedWidth(120)
        self.btn_drill.toggled.connect(self._on_drill_toggled)
        row.addWidget(self.btn_drill)
        self.sl_current = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_current.setMinimumWidth(110)
        self.sl_current.valueChanged.connect(self._on_current)
        row.addWidget(self.sl_current)
        drill.addLayout(row)
        h.addLayout(drill, 2)

        depth = QtWidgets.QVBoxLayout(); depth.setSpacing(4)
        self.lbl_depth = _ctl_caption("④  AVANÇ · 0.0 mm")
        depth.addWidget(self.lbl_depth)
        self.sl_depth = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_depth.setRange(0, int(_DEPTH_STEPS_PER_MM * (APPROACH_MM + 40 + DEPTH_EXTRA_MM)))
        self.sl_depth.setSingleStep(1)
        self.sl_depth.setPageStep(_DEPTH_STEPS_PER_MM)
        self.sl_depth.setFixedHeight(40)
        self.sl_depth.valueChanged.connect(self._on_depth)
        depth.addWidget(self.sl_depth)
        h.addLayout(depth, 3)

    def configure_current(self, lo, hi, value):
        self.sl_current.blockSignals(True)
        self.sl_current.setRange(int(lo), int(hi))
        self.sl_current.setValue(int(value))
        self.sl_current.blockSignals(False)
        self.lbl_current.setText(f"BROCA · {int(value)} mA")

    def set_screw_length(self, length_mm):
        self.sl_depth.blockSignals(True)
        self.sl_depth.setRange(0, int(round(_DEPTH_STEPS_PER_MM * (APPROACH_MM + length_mm + DEPTH_EXTRA_MM))))
        self.sl_depth.blockSignals(False)

    def reset_depth(self):
        self.sl_depth.blockSignals(True)
        self.sl_depth.setValue(0)
        self.sl_depth.blockSignals(False)
        self.lbl_depth.setText("④  AVANÇ · 0.0 mm")

    def set_drilling(self, on):
        if self.btn_drill.isChecked() != on:
            self.btn_drill.blockSignals(True)
            self.btn_drill.setChecked(on)
            self.btn_drill.blockSignals(False)
        self.btn_drill.setText("■  STOP" if on else "③  START")

    def _on_drill_toggled(self, on):
        self.btn_drill.setText("■  STOP" if on else "③  START")
        self.drillToggled.emit(on)

    def _on_current(self, v):
        self.lbl_current.setText(f"BROCA · {v} mA")
        self.currentChanged.emit(int(v))

    def _on_depth(self, v):
        mm = v / _DEPTH_STEPS_PER_MM
        self.lbl_depth.setText(f"④  AVANÇ · {mm:.1f} mm  (entrada a {APPROACH_MM:.0f})")
        self.depthChanged.emit(mm)


class NavigationWorkspace(QtWidgets.QWidget):
    REFRESH_HZ = 20

    def __init__(self, mando_service=None, planning_workspace=None,
                 robot_workspace=None, parent=None):
        super().__init__(parent)
        self.mando_service = mando_service
        self.planning_workspace = planning_workspace
        self.robot_workspace = robot_workspace

        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        self.header = NavigationHeader()
        root.addWidget(self.header)

        body = QtWidgets.QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(12)

        self.hud = TelemetryHUD()
        body.addWidget(self.hud)

        center = QtWidgets.QVBoxLayout()
        center.setSpacing(12)
        view_wrap = _segment_frame(bg="#050506", border_alpha=0.10)
        vw = QtWidgets.QHBoxLayout(view_wrap)
        vw.setContentsMargins(2, 2, 2, 2)
        vw.setSpacing(0)
        self.target_view = TargetingView()
        vw.addWidget(self.target_view, 1)
        sep = QtWidgets.QFrame(); sep.setFixedWidth(1)
        sep.setStyleSheet("background-color: rgba(255,255,255,0.08); border: none;")
        vw.addWidget(sep)
        self.depth_gauge = DepthGauge()
        vw.addWidget(self.depth_gauge)
        center.addWidget(view_wrap, 1)

        self.controls = NavControls()
        self.controls.approachClicked.connect(self._on_approach_clicked)
        self.controls.drillModeClicked.connect(self._on_drill_mode_clicked)
        self.controls.drillToggled.connect(self._on_drill_toggled)
        self.controls.currentChanged.connect(self._on_current_changed)
        self.controls.depthChanged.connect(self._on_depth_changed)
        center.addWidget(self.controls)
        body.addLayout(center, 1)

        self.plan = SurgicalPlanPanel()
        self.plan.screwSelected.connect(self._on_screw_selected)
        self.plan.btn_approach.hide()
        body.addWidget(self.plan)

        root.addLayout(body, 1)

        self._active_screw_idx = None
        self._screws_ref = None
        self._tick = 0
        self._current_cfg = False

        self._refresh_timer = QtCore.QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_telemetry)
        self._refresh_timer.start(int(1000 / self.REFRESH_HZ))

        QtCore.QTimer.singleShot(200, self._refresh_from_robot_workspace)

    def _viewer(self):
        return getattr(self.robot_workspace, "_inner", None) if self.robot_workspace else None

    def _robot_mode(self):
        return getattr(self.robot_workspace, "_current_mode", None)

    def _screws(self):
        return getattr(self.robot_workspace, "_reg_screws_m", []) if self.robot_workspace else []

    def _active_screw(self):
        s = self._screws()
        if self._active_screw_idx is None or not (0 <= self._active_screw_idx < len(s)):
            return None
        return s[self._active_screw_idx]

    def _set_robot_mode(self, mode_id):
        sel = getattr(self.robot_workspace, "_mode_selector", None)
        if sel is not None and self._robot_mode() != mode_id:
            sel.set_mode(mode_id)
        return self._robot_mode() == mode_id

    def _refresh_from_robot_workspace(self):
        if self.robot_workspace is None:
            return
        screws = self._screws()
        if screws is not self._screws_ref:
            self._screws_ref = screws
            self.plan.set_screws(screws)
            self._active_screw_idx = None
            self.header.set_screw_name("—")
        self.hud.set_status(2, bool(screws), f"{len(screws)} cargols" if screws else "sense cargols")

        has_reg = bool(getattr(self.robot_workspace, "_reg_actors", []))
        self.hud.set_status(1, has_reg, "OK" if has_reg else "pendent")

        connected = False
        try:
            connected = self.robot_workspace.is_robot_connected()
        except Exception:
            pass
        self.hud.set_status(0, connected, "CONN" if connected else "OFF")

    def _on_screw_selected(self, idx):
        self._active_screw_idx = idx
        sc = self._active_screw()
        if sc is not None:
            self.header.set_screw_name(sc.get("name", f"Cargol {idx + 1}"))
            self.controls.set_screw_length(sc["length_m"] * 1000.0)
            self.controls.reset_depth()

    def _on_approach_clicked(self):
        if self._active_screw() is None or self.robot_workspace is None:
            return
        if not self._set_robot_mode("ik6"):
            print("[nav] no s'ha pogut passar a IK6")
            return
        self.controls.reset_depth()
        self.robot_workspace._on_go_approach(self._active_screw_idx)

    def _on_drill_mode_clicked(self):
        if self._set_robot_mode("ik5"):
            self.controls.reset_depth()

    def _j6_panel(self):
        return getattr(self.robot_workspace, "_j6_panel", None) if self.robot_workspace else None

    def _on_drill_toggled(self, on):
        panel = self._j6_panel()
        if panel is None:
            self.controls.set_drilling(False)
            return
        if on:
            if self._robot_mode() != "ik5":
                QtWidgets.QMessageBox.information(
                    self, "Broca", "Passa primer a ② MODE BROCA (IK5).")
                self.controls.set_drilling(False)
                return
            panel._on_start()
        else:
            if panel.is_running():
                panel._on_stop()

    def _on_current_changed(self, mA):
        panel = self._j6_panel()
        if panel is not None:
            panel.slider_tor.setValue(int(mA))

    def _on_depth_changed(self, depth_mm):
        if self._active_screw() is None or self._robot_mode() not in ("ik5", "ik6"):
            return
        self.robot_workspace._on_trajectory_depth(self._active_screw_idx, depth_mm)

    def _refresh_telemetry(self):
        if self.robot_workspace is None or not self.isVisible():
            return
        viewer = self._viewer()
        if viewer is None:
            return
        self._tick += 1
        if self._tick % 5 == 1:
            self._refresh_from_robot_workspace()

        mode = self._robot_mode()
        sc = self._active_screw()
        panel = self._j6_panel()
        drill = getattr(self.robot_workspace, "_drill", None)
        drilling = bool(drill is not None and drill.state == "running")

        if panel is not None and not self._current_cfg:
            s = panel.slider_tor
            self.controls.configure_current(max(s.minimum(), 0), s.maximum(), s.value())
            self._current_cfg = True
        self.controls.btn_approach.setEnabled(sc is not None)
        self.controls.btn_mode.setEnabled(sc is not None and mode != "ik5")
        self.controls.btn_drill.setEnabled(mode == "ik5")
        self.controls.sl_depth.setEnabled(sc is not None and mode in ("ik5", "ik6"))
        self.controls.set_drilling(bool(panel is not None and panel.is_running()))

        if sc is None:
            self.target_view.set_metrics(None, "Selecciona un cargol del pla quirúrgic")
            self.depth_gauge.set_depth(None, 40.0)
            self.hud.set_distance(None, "Sense cargol seleccionat")
            self.hud.set_angular(None, "Sense referència")
            self.hud.set_mode("STANDBY", "Selecciona un cargol")
            self.header.set_precision("idle")
            return

        try:
            m = guidance_metrics(viewer.poses["tcp_link"], sc)
        except Exception:
            return

        self.target_view.set_metrics(m)
        self.depth_gauge.set_depth(m["depth_mm"], m["length_mm"])
        if self._tick % 2 == 0:
            self.hud.set_distance(m["lateral_mm"], "Punta ↔ trajectòria planificada")
            self.hud.set_angular(m["angle_deg"], "Eix de la broca ↔ eix del cargol")

        aligned = m["lateral_mm"] < TOL_OK_MM and m["angle_deg"] < TOL_OK_DEG
        warn = m["lateral_mm"] < TOL_WARN_MM and m["angle_deg"] < TOL_WARN_DEG
        self.header.set_precision("ok" if aligned else ("warn" if warn else "danger"))

        L, d = m["length_mm"], m["depth_mm"]
        if d > L + 0.2:
            self.hud.set_mode("PASSAT", f"{d - L:.1f} mm més enllà del final")
        elif d >= L - 0.2:
            self.hud.set_mode("FINAL", "Profunditat planificada assolida")
        elif drilling:
            self.hud.set_mode("TREPANT", f"{d:.1f} / {L:.0f} mm")
        elif mode == "ik5":
            self.hud.set_mode("BROCA LLESTA", "③ Start · ④ Avança")
        elif aligned:
            self.hud.set_mode("ALINEAT", "② Mode broca")
        else:
            self.hud.set_mode("APROXIMACIÓ", "① Approach")

    def cleanup(self):
        self._refresh_timer.stop()

