import csv
import math
import os
import time

import numpy as np
from pyqtgraph.functions import arrayToQPath

from qtpy import QtWidgets, QtCore, QtGui
from qtpy.QtCore import Qt, QPointF, QRectF

from ..theme import (
    C_PANEL, C_PANEL_ALT, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    FONT_MONO, SIZE_XS, SIZE_SM,
    style_pill,
)


SAMPLE_HZ        = 25
DRAW_EVERY_N     = 2
HISTORY_S        = 60
STALE_S          = 0.6
WINDOWS_S        = (5, 10, 30, 60)
DEFAULT_WINDOW_S = 10
LABEL_EVERY_N    = 3
MIN_HALF_RANGE   = 300.0
GRID_COLS        = 3

LOAD_WARN = 0.70
LOAD_CRIT = 0.90

C_CELL      = "#161618"
C_LINE      = "#d1d1d6"
C_LINE_FILL = QtGui.QColor(255, 255, 255, 16)
C_GRID      = "#222224"
C_ZERO      = "#3a3a3c"

X_TICK_STEP = {5: 1, 10: 2, 30: 5, 60: 10}


def _fmt_mA(v):
    s = f"{abs(v):,.0f}".replace(",", " ")
    return ("−" + s) if v < -0.5 else s


def _nice_step(half_range):
    limit = max(half_range * 0.75, 1e-9)
    mag = 10 ** math.floor(math.log10(limit))
    for m in (5, 2, 1):
        if m * mag <= limit:
            return m * mag
    return mag


def _envelope(px, y):
    cols = np.floor(px).astype(np.int64)
    if cols.size < 2 or cols.size <= 2 * (cols[-1] - cols[0] + 1):
        return px, y, px, y
    starts = np.flatnonzero(np.r_[True, cols[1:] != cols[:-1]])
    with np.errstate(all="ignore"):
        lo = np.fmin.reduceat(y, starts)
        hi = np.fmax.reduceat(y, starts)
        far = np.where(np.abs(hi) >= np.abs(lo), hi, lo)
    fx = px[starts]
    xs = np.repeat(fx, 2)
    ys = np.empty(xs.size)
    ys[0::2], ys[1::2] = lo, hi
    return xs, ys, fx, far


class _MotorHeader(QtWidgets.QWidget):
    H = 30

    def __init__(self, name, limit_mA, parent=None):
        super().__init__(parent)
        self.name = name
        self.limit_mA = float(limit_mA)
        self.value = None
        self.peak = 0.0
        self.hover_value = None
        self.hovering = False
        self.setFixedHeight(self.H)

        self._f_name = QtGui.QFont(self.font())
        self._f_name.setPixelSize(SIZE_SM)
        self._f_name.setBold(True)
        self._f_small = QtGui.QFont("Consolas")
        self._f_small.setStyleHint(QtGui.QFont.Monospace)
        self._f_small.setPixelSize(SIZE_XS)
        self._f_unit = QtGui.QFont(self.font())
        self._f_unit.setPixelSize(SIZE_XS)
        self._f_value = QtGui.QFont("Consolas")
        self._f_value.setStyleHint(QtGui.QFont.Monospace)
        self._f_value.setPixelSize(SIZE_SM + 2)
        self._f_value.setBold(True)

    def set_state(self, value, peak):
        if value == self.value and peak == self.peak:
            return
        self.value, self.peak = value, peak
        self.update()

    def set_hover(self, hovering, value=None):
        if hovering == self.hovering and value == self.hover_value:
            return
        self.hovering, self.hover_value = hovering, value
        self.update()

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        w = self.width()
        top_h = 18
        shown = self.hover_value if self.hovering else self.value

        p.setFont(self._f_name)
        p.setPen(QtGui.QColor(C_TEXT))
        p.drawText(QRectF(0, 0, 30, top_h), Qt.AlignVCenter | Qt.AlignLeft, self.name)

        p.setFont(self._f_small)
        p.setPen(QtGui.QColor(C_TEXT_MUTED))
        if self.peak > 0:
            p.drawText(QRectF(30, 0, 90, top_h), Qt.AlignVCenter | Qt.AlignLeft,
                       f"pic {_fmt_mA(self.peak)}")

        p.setFont(self._f_unit)
        unit_w = 20
        p.drawText(QRectF(w - unit_w, 0, unit_w, top_h), Qt.AlignVCenter | Qt.AlignRight, "mA")

        p.setFont(self._f_value)
        p.setPen(QtGui.QColor(C_TEXT if shown is not None else C_TEXT_MUTED))
        p.drawText(QRectF(120, 0, w - unit_w - 4 - 120, top_h), Qt.AlignVCenter | Qt.AlignRight,
                   _fmt_mA(shown) if shown is not None else "—")

        p.setRenderHint(QtGui.QPainter.Antialiasing)
        bx, by, bw, bh = 0.0, top_h + 5.0, float(w), 3.0
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QColor(C_BORDER_STRONG))
        p.drawRoundedRect(QRectF(bx, by, bw, bh), 1.5, 1.5)
        if self.value is not None and self.limit_mA > 0:
            frac = min(abs(self.value) / self.limit_mA, 1.0)
            fill = "#ffffff" if frac >= LOAD_CRIT else ("#c7c7cc" if frac >= LOAD_WARN else "#8e8e93")
            p.setBrush(QtGui.QColor(fill))
            p.drawRoundedRect(QRectF(bx, by, max(bw * frac, bh), bh), 1.5, 1.5)
            pk = min(self.peak / self.limit_mA, 1.0)
            if pk > frac + 0.01:
                p.setBrush(QtGui.QColor(C_TEXT_DIM))
                p.drawRect(QRectF(bx + bw * pk - 1, by - 1, 2, bh + 2))
        p.end()


class _Scope(QtWidgets.QWidget):
    hoverMoved = QtCore.Signal(object)

    PAD_L, PAD_R, PAD_T = 44, 6, 6

    def __init__(self, limit_mA, show_time_axis, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)
        self.setMouseTracking(True)
        self.setMinimumHeight(40)
        self.limit_mA = float(limit_mA)
        self.show_time_axis = show_time_axis
        self.pad_b = 18 if show_time_axis else 4
        self.window_s = float(DEFAULT_WINDOW_S)
        self.half_range = 1000.0
        self.x = np.empty(0)
        self.y = np.empty(0)
        self.hover_x = None
        self.hover_y = None

        self._bg = QtGui.QColor(C_CELL)
        self._pen_grid = QtGui.QPen(QtGui.QColor(C_GRID), 1)
        self._pen_axis = QtGui.QPen(QtGui.QColor(C_BORDER_STRONG), 1)
        self._pen_zero = QtGui.QPen(QtGui.QColor(C_ZERO), 1)
        self._pen_lim = QtGui.QPen(QtGui.QColor(C_TEXT_MUTED), 1, Qt.DashLine)
        self._pen_text = QtGui.QPen(QtGui.QColor(C_TEXT_MUTED))
        self._pen_line = QtGui.QPen(QtGui.QColor(C_LINE), 1.0)
        self._pen_vline = QtGui.QPen(QtGui.QColor(255, 255, 255, 70), 1)
        self._pen_ring = QtGui.QPen(self._bg, 2)
        self._brush_dot = QtGui.QBrush(QtGui.QColor(C_TEXT))
        self._font = QtGui.QFont(self.font())
        self._font.setPixelSize(SIZE_XS - 1)
        self._font_lim = QtGui.QFont(self.font())
        self._font_lim.setPixelSize(SIZE_XS - 2)
        self._font_lim.setBold(True)

    def set_data(self, x, y, window_s, half_range):
        self.x, self.y = x, y
        self.window_s, self.half_range = float(window_s), float(half_range)
        self.update()

    def set_hover(self, x, y):
        self.hover_x, self.hover_y = x, y
        self.update()

    def _plot_rect(self):
        return QRectF(self.PAD_L, self.PAD_T,
                      self.width() - self.PAD_L - self.PAD_R,
                      self.height() - self.PAD_T - self.pad_b)

    def _px(self, x, r):
        return r.left() + (x + self.window_s) / self.window_s * r.width()

    def _py(self, y, r):
        return r.center().y() - y / self.half_range * (r.height() / 2.0)

    def mouseMoveEvent(self, ev):
        r = self._plot_rect()
        pos = ev.pos()
        if r.contains(QPointF(pos)):
            x = (pos.x() - r.left()) / r.width() * self.window_s - self.window_s
            self.hoverMoved.emit(x)
        else:
            self.hoverMoved.emit(None)

    def leaveEvent(self, ev):
        self.hoverMoved.emit(None)

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), self._bg)
        r = self._plot_rect()
        if r.width() < 10 or r.height() < 10:
            p.end()
            return
        p.setFont(self._font)

        step = _nice_step(self.half_range)
        k = int(self.half_range // step)
        for i in range(-k, k + 1):
            v = i * step
            py = self._py(v, r)
            if i != 0:
                p.setPen(self._pen_grid)
                p.drawLine(QPointF(r.left(), py), QPointF(r.right(), py))
            p.setPen(self._pen_text)
            p.drawText(QRectF(0, py - 7, self.PAD_L - 7, 14), Qt.AlignRight | Qt.AlignVCenter, _fmt_mA(v))

        xstep = X_TICK_STEP.get(int(self.window_s), max(1, int(self.window_s // 5)))
        n_x = int(self.window_s // xstep)
        for i in range(1, n_x + 1):
            xv = -i * xstep
            px = self._px(xv, r)
            if px <= r.left() + 0.5:
                continue
            p.setPen(self._pen_grid)
            p.drawLine(QPointF(px, r.top()), QPointF(px, r.bottom()))
            if self.show_time_axis:
                p.setPen(self._pen_text)
                p.drawText(QRectF(px - 30, r.bottom() + 3, 60, 13), Qt.AlignCenter, f"−{abs(xv):.0f} s")
        if self.show_time_axis:
            p.setPen(self._pen_text)
            p.drawText(QRectF(r.right() - 40, r.bottom() + 3, 40, 13), Qt.AlignRight | Qt.AlignVCenter, "ara")

        p.setPen(self._pen_axis)
        p.drawLine(QPointF(r.left(), r.top()), QPointF(r.left(), r.bottom()))
        if self.show_time_axis:
            p.drawLine(QPointF(r.left(), r.bottom()), QPointF(r.right(), r.bottom()))

        finite = np.isfinite(self.y) if self.y.size else np.zeros(0, bool)
        has_data = bool(finite.any())

        if not has_data:
            p.setPen(self._pen_text)
            p.drawText(r, Qt.AlignCenter, "sense dades")
            p.end()
            return

        zy = self._py(0.0, r)
        p.setPen(self._pen_zero)
        p.drawLine(QPointF(r.left(), zy), QPointF(r.right(), zy))
        if self.half_range >= self.limit_mA * 0.98:
            p.setFont(self._font_lim)
            for sign in (+1, -1):
                ly = self._py(sign * self.limit_mA, r)
                p.setPen(self._pen_lim)
                p.drawLine(QPointF(r.left(), ly), QPointF(r.right(), ly))
                p.setPen(self._pen_text)
                tr = QRectF(r.right() - 60, ly + 2 if sign > 0 else ly - 14, 56, 12)
                p.drawText(tr, Qt.AlignRight | Qt.AlignVCenter, "LÍMIT")

        p.setClipRect(r)
        px = self._px(self.x, r)
        xs, ys, fx, fv = _envelope(px, self.y)
        k = r.height() / 2.0 / self.half_range
        py = r.center().y() - ys * k

        ok = np.isfinite(fv)
        fv = np.where(ok, fv, 0.0)
        gx = np.linspace(max(fx[0], r.left()), fx[-1], max(int(r.width() / 3), 2))
        gv = np.interp(gx, fx, fv)
        gv[np.interp(gx, fx, ok.astype(float)) < 0.999] = 0.0
        gy = r.center().y() - gv * k
        fill = arrayToQPath(gx, gy, connect="all")
        fill.lineTo(gx[-1], zy)
        fill.lineTo(gx[0], zy)
        fill.closeSubpath()
        p.fillPath(fill, C_LINE_FILL)

        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.setPen(self._pen_line)
        p.setBrush(Qt.NoBrush)
        p.drawPath(arrayToQPath(xs, py, connect="finite"))

        p.setPen(self._pen_ring)
        p.setBrush(self._brush_dot)
        if finite[-1]:
            p.drawEllipse(QPointF(px[-1], self._py(self.y[-1], r)), 3.5, 3.5)

        if self.hover_x is not None:
            hx = self._px(self.hover_x, r)
            p.setPen(self._pen_vline)
            p.drawLine(QPointF(hx, r.top()), QPointF(hx, r.bottom()))
            if self.hover_y is not None and np.isfinite(self.hover_y):
                p.setPen(self._pen_ring)
                p.drawEllipse(QPointF(hx, self._py(self.hover_y, r)), 3.5, 3.5)
        p.end()


class _MotorCell(QtWidgets.QFrame):
    def __init__(self, name, limit_mA, show_time_axis, parent=None):
        super().__init__(parent)
        self.setObjectName("MotorCell")
        self.setStyleSheet(f"""
            QFrame#MotorCell {{
                background-color: {C_CELL};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)
        self.limit_mA = float(limit_mA)
        self.half_range = 1000.0

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(12, 8, 10, 6)
        v.setSpacing(2)

        self.header = _MotorHeader(name, limit_mA)
        v.addWidget(self.header)

        self.scope = _Scope(limit_mA, show_time_axis)
        v.addWidget(self.scope, 1)

    def set_series(self, x, y, window_s, auto_scale):
        if not auto_scale:
            self.half_range = self.limit_mA * 1.10
        else:
            finite = y[np.isfinite(y)] if y.size else y
            peak = float(np.abs(finite).max()) if finite.size else 0.0
            target = max(peak * 1.18, MIN_HALF_RANGE)
            if target > self.half_range:
                self.half_range = target
            else:
                self.half_range += 0.08 * (target - self.half_range)
        self.scope.set_data(x, y, window_s, self.half_range)

    def set_hover(self, xs, y):
        if xs is None:
            self.scope.set_hover(None, None)
            self.header.set_hover(False)
            return
        ok = y is not None and np.isfinite(y)
        self.scope.set_hover(xs, float(y) if ok else None)
        self.header.set_hover(True, float(y) if ok else None)


class _Segmented(QtWidgets.QFrame):
    changed = QtCore.Signal(object)

    def __init__(self, options, current, parent=None):
        super().__init__(parent)
        self.setObjectName("Seg")
        self.setFixedHeight(26)
        self.setStyleSheet(f"""
            QFrame#Seg {{
                background-color: {C_PANEL_ALT};
                border: 1px solid {C_BORDER_STRONG};
                border-radius: 7px;
            }}
            QPushButton {{
                background: transparent; color: {C_TEXT_MUTED};
                border: none; border-radius: 5px;
                padding: 0 9px; font-size: {SIZE_XS}px; font-weight: 700;
            }}
            QPushButton:hover   {{ color: {C_TEXT}; background: {C_BORDER}; }}
            QPushButton:checked {{ color: {C_TEXT}; background: {C_BORDER_STRONG}; }}
        """)
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(2, 2, 2, 2)
        h.setSpacing(1)
        self._group = QtWidgets.QButtonGroup(self)
        self._group.setExclusive(True)
        for value, label in options:
            b = QtWidgets.QPushButton(label)
            b.setCheckable(True)
            b.setFixedHeight(20)
            b.setCursor(Qt.PointingHandCursor)
            b.setChecked(value == current)
            b.clicked.connect(lambda _=False, v=value: self.changed.emit(v))
            self._group.addButton(b)
            h.addWidget(b)


def _tool_button(text, tooltip, checkable=False):
    b = QtWidgets.QPushButton(text)
    b.setCheckable(checkable)
    b.setFixedHeight(26)
    b.setCursor(Qt.PointingHandCursor)
    b.setToolTip(tooltip)
    b.setStyleSheet(f"""
        QPushButton {{
            background-color: {C_PANEL_ALT}; color: {C_TEXT_DIM};
            border: 1px solid {C_BORDER_STRONG}; border-radius: 7px;
            padding: 0 10px; font-size: {SIZE_XS}px; font-weight: 700;
        }}
        QPushButton:hover   {{ background-color: {C_BORDER}; color: {C_TEXT}; }}
        QPushButton:checked {{ background-color: {C_BORDER_STRONG}; color: {C_TEXT};
                               border: 1px solid #5a5d62; }}
    """)
    return b


class CurrentsPanel(QtWidgets.QFrame):
    def __init__(self, viewer, parent=None):
        super().__init__(parent)
        self.viewer = viewer
        self.setObjectName("CurrentsPanel")
        self.setStyleSheet(f"""
            QFrame#CurrentsPanel {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 12px;
            }}
            QFrame#CurrentsPanel QLabel, QFrame#CurrentsPanel QWidget#Grid {{
                background: transparent;
            }}
        """)
        self.setMinimumHeight(300)

        joints = []
        cfg = getattr(viewer, "motor_config", None)
        if cfg is not None:
            joints = list(getattr(cfg, "joints", []))
        self._motor_ids = [j.motor_id for j in joints] or [1, 2, 3, 4, 5, 6]
        self._names = [j.joint_name.upper() for j in joints] or [f"J{i}" for i in range(1, 7)]
        self._limits = [float(getattr(j, "current_limit_mA", 5000)) for j in joints] or [5000.0] * 6
        n = len(self._motor_ids)
        self._n = n

        self._cap = HISTORY_S * SAMPLE_HZ + SAMPLE_HZ
        self._t = np.full(self._cap, np.nan)
        self._v = np.full((n, self._cap), np.nan)
        self._head = 0
        self._count = 0
        self._peaks = np.zeros(n)
        self._tick = 0
        self._t0_wall = time.time() - time.monotonic()

        self._window_s = DEFAULT_WINDOW_S
        self._auto_scale = True
        self._paused = False
        self._paused_now = None
        self._view_x = np.array([])
        self._view_v = np.zeros((n, 0))
        self._mouse_x = None
        self._source = "off"

        self._build_ui()

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(int(1000 / SAMPLE_HZ))

    def _build_ui(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(14, 10, 12, 10)
        v.setSpacing(8)

        head = QtWidgets.QHBoxLayout()
        head.setSpacing(8)

        title = QtWidgets.QLabel("INTENSITATS")
        title.setStyleSheet(f"color: {C_TEXT}; font-size: {SIZE_SM}px; font-weight: 700;"
                            f" letter-spacing: 1.2px; border: none;")
        head.addWidget(title)

        self._sub = QtWidgets.QLabel("corrent mesurat · per motor")
        self._sub.setStyleSheet(f"color: {C_TEXT_MUTED}; font-size: {SIZE_XS}px; border: none;")
        head.addWidget(self._sub)

        head.addSpacing(4)
        self._pill = QtWidgets.QLabel()
        head.addWidget(self._pill)

        self._total = QtWidgets.QLabel()
        self._total.setToolTip("Suma de |I| de tots els motors")
        self._total.setStyleSheet(f"color: {C_TEXT_DIM}; font-family: {FONT_MONO};"
                                  f" font-size: {SIZE_SM}px; font-weight: 700; border: none;")
        head.addWidget(self._total)

        self._when = QtWidgets.QLabel()
        self._when.setMinimumWidth(90)
        self._when.setStyleSheet(f"color: {C_TEXT}; font-family: {FONT_MONO};"
                                 f" font-size: {SIZE_SM}px; font-weight: 700; border: none;")
        head.addWidget(self._when)

        head.addStretch(1)

        seg_win = _Segmented([(w, f"{w}s") for w in WINDOWS_S], self._window_s)
        seg_win.setToolTip("Finestra de temps visible")
        seg_win.changed.connect(self._set_window)
        head.addWidget(seg_win)

        seg_scale = _Segmented([("auto", "Auto"), ("lim", "Límit")], "auto")
        seg_scale.setToolTip("Auto: cada motor amb la seva escala · Límit: tots a ±límit (comparables)")
        seg_scale.changed.connect(lambda m: self._set_auto_scale(m == "auto"))
        head.addWidget(seg_scale)

        self._btn_pause = _tool_button("❚❚  Pausa", "Congela la vista (es continua gravant)", True)
        self._btn_pause.toggled.connect(self._set_paused)
        head.addWidget(self._btn_pause)

        btn_peaks = _tool_button("↺  Pics", "Reinicia els valors de pic")
        btn_peaks.clicked.connect(self._reset_peaks)
        head.addWidget(btn_peaks)

        btn_csv = _tool_button("CSV", f"Exporta els últims {HISTORY_S} s a CSV")
        btn_csv.clicked.connect(self._export_csv)
        head.addWidget(btn_csv)

        v.addLayout(head)

        grid_w = QtWidgets.QWidget()
        grid_w.setObjectName("Grid")
        grid_w.setAttribute(Qt.WA_StyledBackground, True)
        grid = QtWidgets.QGridLayout(grid_w)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)

        rows = (self._n + GRID_COLS - 1) // GRID_COLS
        self._cells = []
        for i in range(self._n):
            r, c = divmod(i, GRID_COLS)
            cell = _MotorCell(self._names[i], self._limits[i], show_time_axis=(r == rows - 1))
            cell.scope.hoverMoved.connect(self._on_hover_x)
            grid.addWidget(cell, r, c)
            self._cells.append(cell)
        for r in range(rows):
            grid.setRowStretch(r, 1)
        for c in range(GRID_COLS):
            grid.setColumnStretch(c, 1)

        v.addWidget(grid_w, 1)

        self._apply_source_pill()
        self._total.setText("Σ —")

    def _read_currents(self):
        mc = getattr(self.viewer, "motor_controller", None)
        bus = getattr(mc, "bus", None) if mc is not None else None
        if mc is None or bus is None or not getattr(mc, "is_connected", False):
            return "off", None

        try:
            daemon = getattr(bus, "_daemon", None)
            if daemon is not None:
                latest = daemon.latest_status()
                now = time.monotonic()
                vals = []
                for mid in self._motor_ids:
                    d = latest.get(mid)
                    last = float(d.get("last_b1", 0.0)) if d else 0.0
                    if d is None or last <= 0.0 or (now - last) > STALE_S:
                        vals.append(None)
                    else:
                        vals.append(float(d.get("curr", 0)))
                fresh = any(x is not None for x in vals)
                return ("live" if fresh else "stale"), vals
            st = bus.poll_feedback()
            return "sim", [float(st[m].current_mA) if m in st else None for m in self._motor_ids]
        except Exception:
            return "stale", None

    def _on_tick(self):
        source, vals = self._read_currents()
        if source != self._source:
            self._source = source
            self._apply_source_pill()

        now = time.monotonic()
        row = np.full(self._n, np.nan)
        if vals is not None:
            for i, x in enumerate(vals):
                if x is not None:
                    row[i] = x
                    if abs(x) > self._peaks[i]:
                        self._peaks[i] = abs(x)

        if source != "off" or self._count == 0 or not np.all(np.isnan(self._v[:, self._head - 1])):
            self._t[self._head] = now
            self._v[:, self._head] = row
            self._head = (self._head + 1) % self._cap
            self._count = min(self._count + 1, self._cap)

        self._tick += 1
        if not self.isVisible():
            return
        if not self._paused and self._tick % DRAW_EVERY_N == 0:
            self._redraw(now)
        if self._tick % LABEL_EVERY_N == 0:
            self._refresh_labels()

    def _ordered(self):
        if self._count < self._cap:
            return self._t[:self._count], self._v[:, :self._count]
        idx = np.r_[self._head:self._cap, 0:self._head]
        return self._t[idx], self._v[:, idx]

    def _redraw(self, now):
        t, vv = self._ordered()
        x = t - now
        keep = x >= -(self._window_s + 1.0)
        x = x[keep]
        vv = vv[:, keep]
        self._view_x, self._view_v = x, vv

        for i, cell in enumerate(self._cells):
            cell.set_series(x, vv[i] if x.size else np.empty(0), self._window_s, self._auto_scale)
        if self._mouse_x is not None:
            self._update_hover()

    def _refresh_labels(self):
        if self._count == 0:
            vv = np.zeros((self._n, 0))
        else:
            idx = [(self._head - 1 - k) % self._cap for k in range(min(LABEL_EVERY_N, self._count))]
            vv = self._v[:, idx]
        total = 0.0
        any_val = False
        for i, cell in enumerate(self._cells):
            col = vv[i][np.isfinite(vv[i])] if vv.size else np.empty(0)
            val = float(col.mean()) if col.size and self._source != "off" else None
            if val is not None:
                total += abs(val)
                any_val = True
            cell.header.set_state(val, float(self._peaks[i]))
        self._total.setText(f"Σ {total/1000:.2f} A" if any_val else "Σ —")

    def _apply_source_pill(self):
        text, color = {
            "live":  ("●  EN DIRECTE", "#e5e5ea"),
            "sim":   ("●  SIMULACIÓ", "#aeaeb2"),
            "stale": ("▲  SENSE FEEDBACK", "#aeaeb2"),
            "off":   ("○  DESCONNECTAT", C_TEXT_MUTED),
        }[self._source]
        self._pill.setText(text)
        self._pill.setStyleSheet(style_pill(color))

    def _on_hover_x(self, x):
        if x is None:
            self._hide_hover()
            return
        self._mouse_x = float(x)
        self._update_hover()

    def _update_hover(self):
        x = self._view_x
        if x.size == 0:
            self._hide_hover()
            return
        k = int(np.clip(np.searchsorted(x, self._mouse_x), 0, x.size - 1))
        if k > 0 and abs(x[k - 1] - self._mouse_x) < abs(x[k] - self._mouse_x):
            k -= 1
        xs = float(x[k])
        for i, cell in enumerate(self._cells):
            cell.set_hover(xs, self._view_v[i, k])
        self._when.setText("@ ara" if abs(xs) < 0.05 else f"@ −{abs(xs):.2f} s")

    def _hide_hover(self):
        if self._mouse_x is None:
            return
        self._mouse_x = None
        for cell in self._cells:
            cell.set_hover(None, None)
        self._when.setText("")

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._sub.setVisible(self.width() > 1080)

    def _set_window(self, seconds):
        self._window_s = int(seconds)
        self._redraw_now()

    def _set_auto_scale(self, auto):
        self._auto_scale = bool(auto)
        self._redraw_now()

    def _set_paused(self, paused):
        self._paused = bool(paused)
        self._paused_now = time.monotonic() if paused else None
        self._btn_pause.setText("▶  Reprendre" if paused else "❚❚  Pausa")
        if not paused:
            self._redraw(time.monotonic())

    def _redraw_now(self):
        self._redraw(self._paused_now if self._paused else time.monotonic())

    def _reset_peaks(self):
        self._peaks[:] = 0.0
        self._refresh_labels()

    def _export_csv(self):
        t, vv = self._ordered()
        if t.size == 0:
            QtWidgets.QMessageBox.information(self, "Intensitats", "Encara no hi ha dades per exportar.")
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        root = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
        default = os.path.join(root, "data", f"intensitats_{stamp}.csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Exporta intensitats", default, "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["timestamp", "t_rel_s"] + [f"{n}_mA" for n in self._names])
                t_end = t[-1]
                for k in range(t.size):
                    wall = time.strftime("%H:%M:%S", time.localtime(t[k] + self._t0_wall))
                    ms = int(((t[k] + self._t0_wall) % 1) * 1000)
                    w.writerow([f"{wall}.{ms:03d}", f"{t[k] - t_end:.3f}"]
                               + ["" if not np.isfinite(x) else int(round(x)) for x in vv[:, k]])
            print(f"[intensitats] exportat {t.size} mostres a {path}")
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Intensitats", f"No s'ha pogut desar:\n{e}")

    def stop(self):
        self._timer.stop()

