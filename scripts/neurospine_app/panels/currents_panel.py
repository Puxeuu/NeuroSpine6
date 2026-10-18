"""
Panell d'INTENSITATS: corrent en temps real dels 6 motors, un gràfic per motor.

Font de dades:
    · Daemon real  → bus._daemon.latest_status()  (camp "curr" del paquet B1,
                     i "last_b1" per saber si la dada és fresca).
    · Simulador    → bus.poll_feedback()  (current_mA de cada MotorStatus).
    El panell NOMÉS LLEGEIX: no envia cap comanda al bus.

Disseny (monocrom, estil fosc de l'app):
    ┌──────────────────────────────────────────────────────────────────────┐
    │ INTENSITATS  ● EN DIRECTE  Σ 3.42 A      [5s|10s|30s|60s] [Auto|Fix] │
    │ ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐       │
    │ │ J1  pic 2 310 1 234│ J2 ...           │ │ J3 ...           │       │
    │ │ ▬▬▬▬▬──────────  │ │                  │ │                  │       │
    │ │ ~~~~ corba ~~~~  │ │ ~~~~ corba ~~~~  │ │ ~~~~ corba ~~~~  │       │
    │ └──────────────────┘ └──────────────────┘ └──────────────────┘       │
    │ ┌── J4 ──┐ ┌── J5 ──┐ ┌── J6 ──┐   (eix de temps compartit)          │
    └──────────────────────────────────────────────────────────────────────┘

RENDIMENT:
    Tot això corre al fil principal de Qt, el mateix que el bucle de control
    a 100 Hz i la vista 3D. Per això els gràfics NO fan servir pyqtgraph
    (6 QGraphicsView amb eixos i graella costaven 15-90 ms per fotograma):
    cada gràfic és un QWidget pintat amb QPainter (_Scope), i la corba es
    redueix a mínim/màxim per columna de píxel abans de pintar-la. Així el
    cost no depèn de la finestra de temps i no es perd cap pic.

    · Passar el ratolí per qualsevol gràfic → creu sincronitzada als 6, i
      cada capçalera mostra el valor del seu motor en aquell instant.
    · Escala "Auto": cada motor amb la seva escala.  "Límit": tots a ±límit.
    · Pausa  → congela la vista (es continua gravant).
    · CSV    → exporta els últims 60 s.
"""

import csv                                                                      # importem el mòdul csv
import math                                                                     # importem el mòdul math
import os                                                                       # importem el mòdul os
import time                                                                     # importem el mòdul time

import numpy as np                                                              # importem el mòdul numpy as np
from pyqtgraph.functions import arrayToQPath                                    # numpy → QPainterPath (ràpid, suporta forats NaN)

from qtpy import QtWidgets, QtCore, QtGui                                       # importem QtWidgets, QtCore, QtGui des de qtpy
from qtpy.QtCore import Qt, QPointF, QRectF                                     # importem Qt, QPointF, QRectF des de qtpy.QtCore

from ..theme import (                                                           # importem ( des de ..theme
    C_PANEL, C_PANEL_ALT, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    FONT_MONO, SIZE_XS, SIZE_SM,
    style_pill,
)


# ==============================================================================
# CONSTANTS
# ==============================================================================

SAMPLE_HZ        = 25                                                           # el daemon publica l'estat a 25 Hz → mostregem igual
DRAW_EVERY_N     = 2                                                            # redibuixem cada 2 mostres (12.5 fps): fluid i barat
HISTORY_S        = 60                                                           # segons de memòria (finestra màxima + export CSV)
STALE_S          = 0.6                                                          # si un motor no envia B1 en aquest temps, la dada no és fresca
WINDOWS_S        = (5, 10, 30, 60)                                              # finestres seleccionables
DEFAULT_WINDOW_S = 10                                                           # finestra inicial
LABEL_EVERY_N    = 3                                                            # refresquem els números cada N mostres (~8 Hz, llegible)
MIN_HALF_RANGE   = 300.0                                                        # mA: l'escala auto mai fa zoom per sota de ±300 mA (no amplifiquem soroll)
GRID_COLS        = 3                                                            # graella 3 × 2

LOAD_WARN = 0.70                                                                # fracció del límit a partir de la qual la barra s'il·lumina
LOAD_CRIT = 0.90                                                                # fracció del límit a partir de la qual la barra és blanca

# Paleta monocroma (només grisos)
C_CELL      = "#161618"                                                         # fons de cada gràfic (una mica més fosc que el panell)
C_LINE      = "#d1d1d6"                                                         # corba
C_LINE_FILL = QtGui.QColor(255, 255, 255, 16)                                   # ombra sota la corba
C_GRID      = "#222224"                                                         # graella
C_ZERO      = "#3a3a3c"                                                         # línia de zero

X_TICK_STEP = {5: 1, 10: 2, 30: 5, 60: 10}                                      # separació de la graella vertical (s) per finestra


def _fmt_mA(v):                                                                 # definim la funció _fmt_mA()
    """1234.0 → '1 234', -50 → '−50'. Separador de milers fi i signe tipogràfic."""
    s = f"{abs(v):,.0f}".replace(",", " ")                                 # espai fi com a separador de milers
    return ("−" + s) if v < -0.5 else s                                    # signe menys de veritat (no guionet)


def _nice_step(half_range):                                                     # definim la funció _nice_step()
    """
    El pas 'rodó' (1, 2, 5 × 10^k) més gran que deixa almenys una marca per
    banda amb aire: ±4 700 → 2 000 (±2 000, ±4 000) · ±1 400 → 1 000 · ±300 → 200.
    """
    limit = max(half_range * 0.75, 1e-9)                                        # la marca més alta ha de quedar per sota del 75 %
    mag = 10 ** math.floor(math.log10(limit))                                   # ordre de magnitud
    for m in (5, 2, 1):                                                         # iterem: del més gran al més petit
        if m * mag <= limit:                                                    # si m * mag <= limit
            return m * mag                                                      # retornem: m * mag
    return mag                                                                  # retornem: mag


def _envelope(px, y):                                                           # definim la funció _envelope()
    """
    Redueix la corba a (mín, màx) per columna de píxel.
    1500 mostres en 350 px → ~700 punts, i els pics es conserven.
    Els forats (columnes només amb NaN) continuen sent NaN.

    Retorna (xs, ys) per a la línia i (fx, fy) per a l'ombra: l'ombra fa servir
    UN punt per columna (el més allunyat de zero), perquè un polígon en
    ziga-zaga és molt car de rasteritzar (era el 80 % del temps de pintat).
    """
    cols = np.floor(px).astype(np.int64)                                        # columna de píxel de cada mostra
    if cols.size < 2 or cols.size <= 2 * (cols[-1] - cols[0] + 1):             # ja hi ha poques mostres per columna
        return px, y, px, y                                                     # retornem: px, y, px, y
    starts = np.flatnonzero(np.r_[True, cols[1:] != cols[:-1]])                 # inici de cada columna
    with np.errstate(all="ignore"):                                             # fmin/fmax ignoren NaN (excepte si tot és NaN)
        lo = np.fmin.reduceat(y, starts)                                        # lo ← resultat de reduceat()
        hi = np.fmax.reduceat(y, starts)                                        # hi ← resultat de reduceat()
        far = np.where(np.abs(hi) >= np.abs(lo), hi, lo)                        # el valor més allunyat de zero
    fx = px[starts]                                                             # fx ← un punt per columna
    xs = np.repeat(fx, 2)                                                       # dos punts per columna
    ys = np.empty(xs.size)                                                      # ys ← resultat de empty()
    ys[0::2], ys[1::2] = lo, hi                                                 # assignem ys
    return xs, ys, fx, far                                                      # retornem: xs, ys, fx, far


# ==============================================================================
# CAPÇALERA D'UN MOTOR (pintada a mà)
# ==============================================================================

class _MotorHeader(QtWidgets.QWidget):                                          # definim la classe _MotorHeader
    """
        J1   pic 2 310                    1 234 mA
        ▬▬▬▬▬▬▬▬▬▬────────────────────────────────   ← càrrega |I| / límit
    """

    H = 30                                                                      # assignem H

    def __init__(self, name, limit_mA, parent=None):                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.name = name                                                        # assignem self.name
        self.limit_mA = float(limit_mA)                                         # assignem self.limit_mA
        self.value = None                                                       # None = sense dada fresca
        self.peak = 0.0                                                         # assignem self.peak
        self.hover_value = None                                                 # valor sota la creu (si n'hi ha)
        self.hovering = False                                                   # assignem self.hovering
        self.setFixedHeight(self.H)                                             # executem setFixedHeight()

        # Fonts creades un sol cop (no a cada paintEvent)
        self._f_name = QtGui.QFont(self.font())                                 # self._f_name ← resultat de QFont()
        self._f_name.setPixelSize(SIZE_SM)                                      # executem setPixelSize()
        self._f_name.setBold(True)                                              # executem setBold()
        self._f_small = QtGui.QFont("Consolas")                                 # self._f_small ← resultat de QFont()
        self._f_small.setStyleHint(QtGui.QFont.Monospace)                       # executem setStyleHint()
        self._f_small.setPixelSize(SIZE_XS)                                     # executem setPixelSize()
        self._f_unit = QtGui.QFont(self.font())                                 # self._f_unit ← resultat de QFont()
        self._f_unit.setPixelSize(SIZE_XS)                                      # executem setPixelSize()
        self._f_value = QtGui.QFont("Consolas")                                 # self._f_value ← resultat de QFont()
        self._f_value.setStyleHint(QtGui.QFont.Monospace)                       # executem setStyleHint()
        self._f_value.setPixelSize(SIZE_SM + 2)                                 # executem setPixelSize()
        self._f_value.setBold(True)                                             # executem setBold()

    def set_state(self, value, peak):                                           # definim la funció set_state()
        if value == self.value and peak == self.peak:                           # res a repintar
            return                                                              # retornem sense valor
        self.value, self.peak = value, peak                                     # assignem self.value, self.peak
        self.update()                                                           # executem update()

    def set_hover(self, hovering, value=None):                                  # definim la funció set_hover()
        if hovering == self.hovering and value == self.hover_value:             # res a repintar
            return                                                              # retornem sense valor
        self.hovering, self.hover_value = hovering, value                       # assignem self.hovering, self.hover_value
        self.update()                                                           # executem update()

    def paintEvent(self, ev):                                                   # definim la funció paintEvent()
        p = QtGui.QPainter(self)                                                # p ← resultat de QPainter()
        w = self.width()                                                        # w ← resultat de width()
        top_h = 18                                                              # alçada de la línia de text
        shown = self.hover_value if self.hovering else self.value               # en hover: valor de l'instant apuntat

        p.setFont(self._f_name)                                                 # nom del joint
        p.setPen(QtGui.QColor(C_TEXT))                                          # executem setPen()
        p.drawText(QRectF(0, 0, 30, top_h), Qt.AlignVCenter | Qt.AlignLeft, self.name)

        p.setFont(self._f_small)                                                # pic
        p.setPen(QtGui.QColor(C_TEXT_MUTED))                                    # executem setPen()
        if self.peak > 0:                                                       # si self.peak > 0
            p.drawText(QRectF(30, 0, 90, top_h), Qt.AlignVCenter | Qt.AlignLeft,
                       f"pic {_fmt_mA(self.peak)}")

        p.setFont(self._f_unit)                                                 # unitat
        unit_w = 20                                                             # assignem unit_w
        p.drawText(QRectF(w - unit_w, 0, unit_w, top_h), Qt.AlignVCenter | Qt.AlignRight, "mA")

        p.setFont(self._f_value)                                                # valor
        p.setPen(QtGui.QColor(C_TEXT if shown is not None else C_TEXT_MUTED))   # executem setPen()
        p.drawText(QRectF(120, 0, w - unit_w - 4 - 120, top_h), Qt.AlignVCenter | Qt.AlignRight,
                   _fmt_mA(shown) if shown is not None else "—")

        # Barra de càrrega |I| / límit (grisos: més clara com més a prop del límit)
        p.setRenderHint(QtGui.QPainter.Antialiasing)                            # executem setRenderHint()
        bx, by, bw, bh = 0.0, top_h + 5.0, float(w), 3.0                        # assignem bx, by, bw, bh
        p.setPen(Qt.NoPen)                                                      # executem setPen()
        p.setBrush(QtGui.QColor(C_BORDER_STRONG))                               # executem setBrush()
        p.drawRoundedRect(QRectF(bx, by, bw, bh), 1.5, 1.5)                     # executem drawRoundedRect()
        if self.value is not None and self.limit_mA > 0:                        # si tenim dada
            frac = min(abs(self.value) / self.limit_mA, 1.0)                    # frac ← resultat de min()
            fill = "#ffffff" if frac >= LOAD_CRIT else ("#c7c7cc" if frac >= LOAD_WARN else "#8e8e93")
            p.setBrush(QtGui.QColor(fill))                                      # executem setBrush()
            p.drawRoundedRect(QRectF(bx, by, max(bw * frac, bh), bh), 1.5, 1.5)
            pk = min(self.peak / self.limit_mA, 1.0)                            # marca del pic
            if pk > frac + 0.01:                                                # si pk > frac + 0.01
                p.setBrush(QtGui.QColor(C_TEXT_DIM))                            # executem setBrush()
                p.drawRect(QRectF(bx + bw * pk - 1, by - 1, 2, bh + 2))         # executem drawRect()
        p.end()                                                                 # executem end()


# ==============================================================================
# GRÀFIC LLEUGER (QPainter pur)
# ==============================================================================

class _Scope(QtWidgets.QWidget):                                                # definim la classe _Scope
    """Oscil·loscopi d'un motor: graella, zero, límits, corba, creu."""

    hoverMoved = QtCore.Signal(object)                                          # x en segons relatius, o None si surt

    PAD_L, PAD_R, PAD_T = 44, 6, 6                                              # marges de l'àrea de dades (px)

    def __init__(self, limit_mA, show_time_axis, parent=None):                  # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)                         # pintem tot el fons: Qt no cal que l'esborri
        self.setMouseTracking(True)                                             # moviment sense clicar
        self.setMinimumHeight(40)                                               # executem setMinimumHeight()
        self.limit_mA = float(limit_mA)                                         # assignem self.limit_mA
        self.show_time_axis = show_time_axis                                    # assignem self.show_time_axis
        self.pad_b = 18 if show_time_axis else 4                                # espai per a les etiquetes de temps
        self.window_s = float(DEFAULT_WINDOW_S)                                 # assignem self.window_s
        self.half_range = 1000.0                                                # assignem self.half_range
        self.x = np.empty(0)                                                    # temps relatiu (s, negatius)
        self.y = np.empty(0)                                                    # mA
        self.hover_x = None                                                     # assignem self.hover_x
        self.hover_y = None                                                     # assignem self.hover_y

        # Recursos de pintat creats un sol cop
        self._bg = QtGui.QColor(C_CELL)                                         # self._bg ← resultat de QColor()
        self._pen_grid = QtGui.QPen(QtGui.QColor(C_GRID), 1)                    # self._pen_grid ← resultat de QPen()
        self._pen_axis = QtGui.QPen(QtGui.QColor(C_BORDER_STRONG), 1)           # self._pen_axis ← resultat de QPen()
        self._pen_zero = QtGui.QPen(QtGui.QColor(C_ZERO), 1)                    # self._pen_zero ← resultat de QPen()
        self._pen_lim = QtGui.QPen(QtGui.QColor(C_TEXT_MUTED), 1, Qt.DashLine)  # self._pen_lim ← resultat de QPen()
        self._pen_text = QtGui.QPen(QtGui.QColor(C_TEXT_MUTED))                 # self._pen_text ← resultat de QPen()
        self._pen_line = QtGui.QPen(QtGui.QColor(C_LINE), 1.0)                  # 1 px: Qt té una via ràpida (≈40× més ràpid que 1.6 px)
        self._pen_vline = QtGui.QPen(QtGui.QColor(255, 255, 255, 70), 1)        # self._pen_vline ← resultat de QPen()
        self._pen_ring = QtGui.QPen(self._bg, 2)                                # anell del color de fons al voltant dels punts
        self._brush_dot = QtGui.QBrush(QtGui.QColor(C_TEXT))                    # self._brush_dot ← resultat de QBrush()
        self._font = QtGui.QFont(self.font())                                   # self._font ← resultat de QFont()
        self._font.setPixelSize(SIZE_XS - 1)                                    # executem setPixelSize()
        self._font_lim = QtGui.QFont(self.font())                               # self._font_lim ← resultat de QFont()
        self._font_lim.setPixelSize(SIZE_XS - 2)                                # executem setPixelSize()
        self._font_lim.setBold(True)                                            # executem setBold()

    # ---- API -----------------------------------------------------------------

    def set_data(self, x, y, window_s, half_range):                             # definim la funció set_data()
        self.x, self.y = x, y                                                   # assignem self.x, self.y
        self.window_s, self.half_range = float(window_s), float(half_range)     # assignem self.window_s, self.half_range
        self.update()                                                           # executem update()

    def set_hover(self, x, y):                                                  # definim la funció set_hover()
        self.hover_x, self.hover_y = x, y                                       # assignem self.hover_x, self.hover_y
        self.update()                                                           # executem update()

    # ---- geometria -----------------------------------------------------------

    def _plot_rect(self):                                                       # definim la funció _plot_rect()
        return QRectF(self.PAD_L, self.PAD_T,                                   # retornem: l'àrea de dades
                      self.width() - self.PAD_L - self.PAD_R,
                      self.height() - self.PAD_T - self.pad_b)

    def _px(self, x, r):                                                        # temps → píxel X
        return r.left() + (x + self.window_s) / self.window_s * r.width()       # retornem: r.left() + (x + self.window_s) / sel...

    def _py(self, y, r):                                                        # mA → píxel Y
        return r.center().y() - y / self.half_range * (r.height() / 2.0)       # retornem: r.center().y() - y / self.half_rang...

    # ---- ratolí --------------------------------------------------------------

    def mouseMoveEvent(self, ev):                                               # definim la funció mouseMoveEvent()
        r = self._plot_rect()                                                   # r ← resultat de _plot_rect()
        pos = ev.pos()                                                          # pos ← resultat de pos()
        if r.contains(QPointF(pos)):                                            # si r.contains(QPointF(pos))
            x = (pos.x() - r.left()) / r.width() * self.window_s - self.window_s
            self.hoverMoved.emit(x)                                             # executem emit()
        else:                                                                   # en cas contrari
            self.hoverMoved.emit(None)                                          # executem emit()

    def leaveEvent(self, ev):                                                   # definim la funció leaveEvent()
        self.hoverMoved.emit(None)                                              # executem emit()

    # ---- pintat --------------------------------------------------------------

    def paintEvent(self, ev):                                                   # definim la funció paintEvent()
        p = QtGui.QPainter(self)                                                # p ← resultat de QPainter()
        p.fillRect(self.rect(), self._bg)                                       # executem fillRect()
        r = self._plot_rect()                                                   # r ← resultat de _plot_rect()
        if r.width() < 10 or r.height() < 10:                                   # massa petit per pintar res
            p.end()                                                             # executem end()
            return                                                              # retornem sense valor
        p.setFont(self._font)                                                   # executem setFont()

        # Graella horitzontal + etiquetes de mA
        step = _nice_step(self.half_range)                                      # step ← resultat de _nice_step()
        k = int(self.half_range // step)                                        # k ← resultat de int()
        for i in range(-k, k + 1):                                              # iterem: i in range(-k, k + 1)
            v = i * step                                                        # assignem v
            py = self._py(v, r)                                                 # py ← resultat de _py()
            if i != 0:                                                          # si i != 0
                p.setPen(self._pen_grid)                                        # executem setPen()
                p.drawLine(QPointF(r.left(), py), QPointF(r.right(), py))       # executem drawLine()
            p.setPen(self._pen_text)                                            # executem setPen()
            p.drawText(QRectF(0, py - 7, self.PAD_L - 7, 14), Qt.AlignRight | Qt.AlignVCenter, _fmt_mA(v))

        # Graella vertical + etiquetes de temps
        xstep = X_TICK_STEP.get(int(self.window_s), max(1, int(self.window_s // 5)))
        n_x = int(self.window_s // xstep)                                       # n_x ← resultat de int()
        for i in range(1, n_x + 1):                                             # iterem: i in range(1, n_x + 1)
            xv = -i * xstep                                                     # assignem xv
            px = self._px(xv, r)                                                # px ← resultat de _px()
            if px <= r.left() + 0.5:                                            # la vora esquerra ja és l'eix
                continue                                                        # passem a la següent iteració
            p.setPen(self._pen_grid)                                            # executem setPen()
            p.drawLine(QPointF(px, r.top()), QPointF(px, r.bottom()))           # executem drawLine()
            if self.show_time_axis:                                             # si self.show_time_axis
                p.setPen(self._pen_text)                                        # executem setPen()
                p.drawText(QRectF(px - 30, r.bottom() + 3, 60, 13), Qt.AlignCenter, f"−{abs(xv):.0f} s")
        if self.show_time_axis:                                                 # "ara" a la vora dreta
            p.setPen(self._pen_text)                                            # executem setPen()
            p.drawText(QRectF(r.right() - 40, r.bottom() + 3, 40, 13), Qt.AlignRight | Qt.AlignVCenter, "ara")

        # Eixos
        p.setPen(self._pen_axis)                                                # executem setPen()
        p.drawLine(QPointF(r.left(), r.top()), QPointF(r.left(), r.bottom()))   # executem drawLine()
        if self.show_time_axis:                                                 # si self.show_time_axis
            p.drawLine(QPointF(r.left(), r.bottom()), QPointF(r.right(), r.bottom()))

        finite = np.isfinite(self.y) if self.y.size else np.zeros(0, bool)      # finite ← resultat de isfinite()
        has_data = bool(finite.any())                                           # has_data ← resultat de bool()

        if not has_data:                                                        # estat buit
            p.setPen(self._pen_text)                                            # executem setPen()
            p.drawText(r, Qt.AlignCenter, "sense dades")                        # executem drawText()
            p.end()                                                             # executem end()
            return                                                              # retornem sense valor

        # Zero i límits
        zy = self._py(0.0, r)                                                   # zy ← resultat de _py()
        p.setPen(self._pen_zero)                                                # executem setPen()
        p.drawLine(QPointF(r.left(), zy), QPointF(r.right(), zy))               # executem drawLine()
        if self.half_range >= self.limit_mA * 0.98:                             # només si caben a la vista
            p.setFont(self._font_lim)                                           # executem setFont()
            for sign in (+1, -1):                                               # iterem: sign in (+1, -1)
                ly = self._py(sign * self.limit_mA, r)                          # ly ← resultat de _py()
                p.setPen(self._pen_lim)                                         # executem setPen()
                p.drawLine(QPointF(r.left(), ly), QPointF(r.right(), ly))       # executem drawLine()
                p.setPen(self._pen_text)                                        # etiqueta per DINS de la línia
                tr = QRectF(r.right() - 60, ly + 2 if sign > 0 else ly - 14, 56, 12)
                p.drawText(tr, Qt.AlignRight | Qt.AlignVCenter, "LÍMIT")        # executem drawText()

        # Corba (retallada a l'àrea de dades)
        p.setClipRect(r)                                                        # executem setClipRect()
        px = self._px(self.x, r)                                                # px ← resultat de _px()
        xs, ys, fx, fv = _envelope(px, self.y)                                  # mín/màx per columna de píxel
        k = r.height() / 2.0 / self.half_range                                  # mA → píxel
        py = r.center().y() - ys * k                                            # py ← mA → píxel (vectoritzat)

        # Ombra fins a zero, sense AA i a 1 punt cada 3 px (és gairebé transparent:
        # no es nota, i un polígon simple és molt més barat de rasteritzar)
        ok = np.isfinite(fv)                                                    # ok ← resultat de isfinite()
        fv = np.where(ok, fv, 0.0)                                              # els forats queden a zero → sense ombra
        gx = np.linspace(max(fx[0], r.left()), fx[-1], max(int(r.width() / 3), 2))
        gv = np.interp(gx, fx, fv)                                              # gv ← resultat de interp()
        gv[np.interp(gx, fx, ok.astype(float)) < 0.999] = 0.0                   # l'ombra no s'escapa per les vores dels forats
        gy = r.center().y() - gv * k                                            # gy ← mA → píxel
        fill = arrayToQPath(gx, gy, connect="all")                              # fill ← resultat de arrayToQPath()
        fill.lineTo(gx[-1], zy)                                                 # executem lineTo()
        fill.lineTo(gx[0], zy)                                                  # executem lineTo()
        fill.closeSubpath()                                                     # executem closeSubpath()
        p.fillPath(fill, C_LINE_FILL)                                           # executem fillPath()

        p.setRenderHint(QtGui.QPainter.Antialiasing, True)                      # només la corba i els punts amb AA
        p.setPen(self._pen_line)                                              # executem setPen()
        p.setBrush(Qt.NoBrush)                                                  # executem setBrush()
        p.drawPath(arrayToQPath(xs, py, connect="finite"))                      # els NaN tallen la línia (forats reals)

        # Punt al valor actual
        p.setPen(self._pen_ring)                                                # executem setPen()
        p.setBrush(self._brush_dot)                                             # executem setBrush()
        if finite[-1]:                                                          # només si l'última dada és vàlida
            p.drawEllipse(QPointF(px[-1], self._py(self.y[-1], r)), 3.5, 3.5)   # executem drawEllipse()

        # Creu sincronitzada
        if self.hover_x is not None:                                            # si self.hover_x is not None
            hx = self._px(self.hover_x, r)                                      # hx ← resultat de _px()
            p.setPen(self._pen_vline)                                           # executem setPen()
            p.drawLine(QPointF(hx, r.top()), QPointF(hx, r.bottom()))           # executem drawLine()
            if self.hover_y is not None and np.isfinite(self.hover_y):          # si hi ha valor en aquell instant
                p.setPen(self._pen_ring)                                        # executem setPen()
                p.drawEllipse(QPointF(hx, self._py(self.hover_y, r)), 3.5, 3.5)  # executem drawEllipse()
        p.end()                                                                 # executem end()


# ==============================================================================
# CEL·LA D'UN MOTOR: capçalera + gràfic propi
# ==============================================================================

class _MotorCell(QtWidgets.QFrame):                                             # definim la classe _MotorCell
    def __init__(self, name, limit_mA, show_time_axis, parent=None):            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setObjectName("MotorCell")                                         # executem setObjectName()
        self.setStyleSheet(f"""
            QFrame#MotorCell {{
                background-color: {C_CELL};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)
        self.limit_mA = float(limit_mA)                                         # assignem self.limit_mA
        self.half_range = 1000.0                                                # semi-rang Y actual (suavitzat)

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(12, 8, 10, 6)                                      # executem setContentsMargins()
        v.setSpacing(2)                                                         # executem setSpacing()

        self.header = _MotorHeader(name, limit_mA)                              # self.header ← resultat de _MotorHeader()
        v.addWidget(self.header)                                                # executem addWidget()

        self.scope = _Scope(limit_mA, show_time_axis)                           # self.scope ← resultat de _Scope()
        v.addWidget(self.scope, 1)                                              # executem addWidget()

    def set_series(self, x, y, window_s, auto_scale):                           # definim la funció set_series()
        """Ajusta l'escala Y d'aquest motor i passa les dades al gràfic."""
        if not auto_scale:                                                      # escala fixa al límit
            self.half_range = self.limit_mA * 1.10                              # assignem self.half_range
        else:                                                                   # en cas contrari
            finite = y[np.isfinite(y)] if y.size else y                         # finite ← y sense NaN
            peak = float(np.abs(finite).max()) if finite.size else 0.0          # peak ← resultat de float()
            target = max(peak * 1.18, MIN_HALF_RANGE)                           # 18 % d'aire per sobre
            if target > self.half_range:                                        # créixer: immediat (mai retallem un pic)
                self.half_range = target                                        # assignem self.half_range
            else:                                                               # decréixer: suau (sense salts)
                self.half_range += 0.08 * (target - self.half_range)            # assignem self.half_range
        self.scope.set_data(x, y, window_s, self.half_range)                    # executem set_data()

    def set_hover(self, xs, y):                                                 # definim la funció set_hover()
        """xs=None amaga la creu; si no, la posa a xs amb el valor y (pot ser NaN)."""
        if xs is None:                                                          # si xs is None
            self.scope.set_hover(None, None)                                    # executem set_hover()
            self.header.set_hover(False)                                        # executem set_hover()
            return                                                              # retornem sense valor
        ok = y is not None and np.isfinite(y)                                   # ok ← resultat de isfinite()
        self.scope.set_hover(xs, float(y) if ok else None)                      # executem set_hover()
        self.header.set_hover(True, float(y) if ok else None)                   # executem set_hover()


# ==============================================================================
# CONTROLS DE LA CAPÇALERA
# ==============================================================================

class _Segmented(QtWidgets.QFrame):                                             # definim la classe _Segmented
    changed = QtCore.Signal(object)                                             # changed ← resultat de Signal()

    def __init__(self, options, current, parent=None):                          # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setObjectName("Seg")                                               # executem setObjectName()
        self.setFixedHeight(26)                                                 # executem setFixedHeight()
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
        h = QtWidgets.QHBoxLayout(self)                                         # h ← resultat de QHBoxLayout()
        h.setContentsMargins(2, 2, 2, 2)                                        # executem setContentsMargins()
        h.setSpacing(1)                                                         # executem setSpacing()
        self._group = QtWidgets.QButtonGroup(self)                              # self._group ← resultat de QButtonGroup()
        self._group.setExclusive(True)                                          # executem setExclusive()
        for value, label in options:                                            # iterem: value, label in options
            b = QtWidgets.QPushButton(label)                                    # b ← resultat de QPushButton()
            b.setCheckable(True)                                                # executem setCheckable()
            b.setFixedHeight(20)                                                # executem setFixedHeight()
            b.setCursor(Qt.PointingHandCursor)                                  # executem setCursor()
            b.setChecked(value == current)                                      # executem setChecked()
            b.clicked.connect(lambda _=False, v=value: self.changed.emit(v))    # executem connect()
            self._group.addButton(b)                                            # executem addButton()
            h.addWidget(b)                                                      # executem addWidget()


def _tool_button(text, tooltip, checkable=False):                               # definim la funció _tool_button()
    b = QtWidgets.QPushButton(text)                                             # b ← resultat de QPushButton()
    b.setCheckable(checkable)                                                   # executem setCheckable()
    b.setFixedHeight(26)                                                        # executem setFixedHeight()
    b.setCursor(Qt.PointingHandCursor)                                          # executem setCursor()
    b.setToolTip(tooltip)                                                       # executem setToolTip()
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
    return b                                                                    # retornem: b


# ==============================================================================
# PANELL PRINCIPAL
# ==============================================================================

class CurrentsPanel(QtWidgets.QFrame):                                          # definim la classe CurrentsPanel
    """Intensitats dels 6 motors, un gràfic per motor amb eix de temps compartit."""

    def __init__(self, viewer, parent=None):                                    # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.viewer = viewer                                                    # assignem self.viewer
        self.setObjectName("CurrentsPanel")                                     # executem setObjectName()
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
        self.setMinimumHeight(300)                                              # executem setMinimumHeight()

        # ---- Configuració dels motors (noms + límits del motor_config.yaml) ----
        joints = []                                                             # assignem joints
        cfg = getattr(viewer, "motor_config", None)                             # cfg ← resultat de getattr()
        if cfg is not None:                                                     # si cfg is not None
            joints = list(getattr(cfg, "joints", []))                           # joints ← resultat de list()
        self._motor_ids = [j.motor_id for j in joints] or [1, 2, 3, 4, 5, 6]    # assignem self._motor_ids
        self._names = [j.joint_name.upper() for j in joints] or [f"J{i}" for i in range(1, 7)]
        self._limits = [float(getattr(j, "current_limit_mA", 5000)) for j in joints] or [5000.0] * 6
        n = len(self._motor_ids)                                                # n ← resultat de len()
        self._n = n                                                             # assignem self._n

        # ---- Buffer circular -----------------------------------------------
        self._cap = HISTORY_S * SAMPLE_HZ + SAMPLE_HZ                           # capacitat (una mica de marge)
        self._t = np.full(self._cap, np.nan)                                    # temps (monotonic) de cada mostra
        self._v = np.full((n, self._cap), np.nan)                               # mA per motor; NaN = sense dada
        self._head = 0                                                          # següent posició a escriure
        self._count = 0                                                         # mostres vàlides al buffer
        self._peaks = np.zeros(n)                                               # |I| màxim per motor
        self._tick = 0                                                          # comptador de mostres
        self._t0_wall = time.time() - time.monotonic()                          # per convertir a hora real al CSV

        # ---- Estat de la vista ---------------------------------------------
        self._window_s = DEFAULT_WINDOW_S                                       # assignem self._window_s
        self._auto_scale = True                                                 # assignem self._auto_scale
        self._paused = False                                                    # assignem self._paused
        self._paused_now = None                                                 # instant congelat en pausa
        self._view_x = np.array([])                                             # últim X dibuixat (per a la creu)
        self._view_v = np.zeros((n, 0))                                         # últims Y dibuixats
        self._mouse_x = None                                                    # X de dades sota el cursor (None = fora)
        self._source = "off"                                                    # off | live | stale | sim

        self._build_ui()                                                        # executem _build_ui()

        self._timer = QtCore.QTimer(self)                                       # self._timer ← resultat de QTimer()
        self._timer.timeout.connect(self._on_tick)                              # executem connect()
        self._timer.start(int(1000 / SAMPLE_HZ))                                # mostregem sempre (així en obrir el panell ja hi ha història)

    # ==========================================================================
    # CONSTRUCCIÓ DE LA UI
    # ==========================================================================

    def _build_ui(self):                                                        # definim la funció _build_ui()
        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(14, 10, 12, 10)                                    # executem setContentsMargins()
        v.setSpacing(8)                                                         # executem setSpacing()

        # ---------------- Capçalera ----------------
        head = QtWidgets.QHBoxLayout()                                          # head ← resultat de QHBoxLayout()
        head.setSpacing(8)                                                      # executem setSpacing()

        title = QtWidgets.QLabel("INTENSITATS")                                 # title ← resultat de QLabel()
        title.setStyleSheet(f"color: {C_TEXT}; font-size: {SIZE_SM}px; font-weight: 700;"
                            f" letter-spacing: 1.2px; border: none;")
        head.addWidget(title)                                                   # executem addWidget()

        self._sub = QtWidgets.QLabel("corrent mesurat · per motor")             # self._sub ← resultat de QLabel()
        self._sub.setStyleSheet(f"color: {C_TEXT_MUTED}; font-size: {SIZE_XS}px; border: none;")
        head.addWidget(self._sub)                                               # executem addWidget()

        head.addSpacing(4)                                                      # executem addSpacing()
        self._pill = QtWidgets.QLabel()                                         # self._pill ← resultat de QLabel()
        head.addWidget(self._pill)                                              # executem addWidget()

        self._total = QtWidgets.QLabel()                                        # Σ|I| de tots els motors
        self._total.setToolTip("Suma de |I| de tots els motors")                # executem setToolTip()
        self._total.setStyleSheet(f"color: {C_TEXT_DIM}; font-family: {FONT_MONO};"
                                  f" font-size: {SIZE_SM}px; font-weight: 700; border: none;")
        head.addWidget(self._total)                                             # executem addWidget()

        self._when = QtWidgets.QLabel()                                         # instant apuntat per la creu
        self._when.setMinimumWidth(90)                                          # amplada fixa: no fa ballar la capçalera
        self._when.setStyleSheet(f"color: {C_TEXT}; font-family: {FONT_MONO};"
                                 f" font-size: {SIZE_SM}px; font-weight: 700; border: none;")
        head.addWidget(self._when)                                              # executem addWidget()

        head.addStretch(1)                                                      # executem addStretch()

        seg_win = _Segmented([(w, f"{w}s") for w in WINDOWS_S], self._window_s)  # seg_win ← resultat de _Segmented()
        seg_win.setToolTip("Finestra de temps visible")                         # executem setToolTip()
        seg_win.changed.connect(self._set_window)                               # executem connect()
        head.addWidget(seg_win)                                                 # executem addWidget()

        seg_scale = _Segmented([("auto", "Auto"), ("lim", "Límit")], "auto")    # seg_scale ← resultat de _Segmented()
        seg_scale.setToolTip("Auto: cada motor amb la seva escala · Límit: tots a ±límit (comparables)")
        seg_scale.changed.connect(lambda m: self._set_auto_scale(m == "auto"))  # executem connect()
        head.addWidget(seg_scale)                                               # executem addWidget()

        self._btn_pause = _tool_button("❚❚  Pausa", "Congela la vista (es continua gravant)", True)
        self._btn_pause.toggled.connect(self._set_paused)                       # executem connect()
        head.addWidget(self._btn_pause)                                         # executem addWidget()

        btn_peaks = _tool_button("↺  Pics", "Reinicia els valors de pic")   # btn_peaks ← resultat de _tool_button()
        btn_peaks.clicked.connect(self._reset_peaks)                            # executem connect()
        head.addWidget(btn_peaks)                                               # executem addWidget()

        btn_csv = _tool_button("CSV", f"Exporta els últims {HISTORY_S} s a CSV")  # btn_csv ← resultat de _tool_button()
        btn_csv.clicked.connect(self._export_csv)                               # executem connect()
        head.addWidget(btn_csv)                                                 # executem addWidget()

        v.addLayout(head)                                                       # executem addLayout()

        # ---------------- Graella de gràfics ----------------
        grid_w = QtWidgets.QWidget()                                            # grid_w ← resultat de QWidget()
        grid_w.setObjectName("Grid")                                            # executem setObjectName()
        grid_w.setAttribute(Qt.WA_StyledBackground, True)                       # executem setAttribute()
        grid = QtWidgets.QGridLayout(grid_w)                                    # grid ← resultat de QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)                                     # executem setContentsMargins()
        grid.setHorizontalSpacing(8)                                            # executem setHorizontalSpacing()
        grid.setVerticalSpacing(8)                                              # executem setVerticalSpacing()

        rows = (self._n + GRID_COLS - 1) // GRID_COLS                           # rows ← nombre de files
        self._cells = []                                                        # assignem self._cells
        for i in range(self._n):                                                # iterem: i in range(self._n)
            r, c = divmod(i, GRID_COLS)                                         # r, c ← resultat de divmod()
            cell = _MotorCell(self._names[i], self._limits[i], show_time_axis=(r == rows - 1))
            cell.scope.hoverMoved.connect(self._on_hover_x)                     # creu sincronitzada des de qualsevol gràfic
            grid.addWidget(cell, r, c)                                          # executem addWidget()
            self._cells.append(cell)                                            # executem append()
        for r in range(rows):                                                   # iterem: r in range(rows)
            grid.setRowStretch(r, 1)                                            # executem setRowStretch()
        for c in range(GRID_COLS):                                              # iterem: c in range(GRID_COLS)
            grid.setColumnStretch(c, 1)                                         # executem setColumnStretch()

        v.addWidget(grid_w, 1)                                                  # executem addWidget()

        self._apply_source_pill()                                               # executem _apply_source_pill()
        self._total.setText("Σ —")                                    # executem setText()

    # ==========================================================================
    # ADQUISICIÓ
    # ==========================================================================

    def _read_currents(self):                                                   # definim la funció _read_currents()
        """
        Retorna (font, llista de mA o None per motor).
        No llança mai excepcions: el bucle de control no ha de patir per un gràfic.
        """
        mc = getattr(self.viewer, "motor_controller", None)                     # mc ← resultat de getattr()
        bus = getattr(mc, "bus", None) if mc is not None else None              # bus ← resultat de getattr()
        if mc is None or bus is None or not getattr(mc, "is_connected", False):  # si no hi ha bus
            return "off", None                                                  # retornem: "off", None

        try:                                                                    # intentem el bloc següent
            daemon = getattr(bus, "_daemon", None)                              # daemon ← resultat de getattr()
            if daemon is not None:                                              # ---- daemon real ----
                latest = daemon.latest_status()                                 # còpia sota lock
                now = time.monotonic()                                          # now ← resultat de monotonic()
                vals = []                                                       # assignem vals
                for mid in self._motor_ids:                                     # iterem: mid in self._motor_ids
                    d = latest.get(mid)                                         # d ← resultat de get()
                    last = float(d.get("last_b1", 0.0)) if d else 0.0           # instant de l'últim paquet B1
                    if d is None or last <= 0.0 or (now - last) > STALE_S:      # mai rebut o massa vell
                        vals.append(None)                                       # executem append()
                    else:                                                       # en cas contrari
                        vals.append(float(d.get("curr", 0)))                    # executem append()
                fresh = any(x is not None for x in vals)                        # fresh ← resultat de any()
                return ("live" if fresh else "stale"), vals                     # retornem: ("live" if fresh else "stale"), vals
            st = bus.poll_feedback()                                            # ---- simulador ----
            return "sim", [float(st[m].current_mA) if m in st else None for m in self._motor_ids]
        except Exception:                                                       # capturem excepció: Exception
            return "stale", None                                                # retornem: "stale", None

    def _on_tick(self):                                                         # definim la funció _on_tick()
        source, vals = self._read_currents()                                    # source, vals ← resultat de _read_currents()
        if source != self._source:                                              # si source != self._source
            self._source = source                                               # assignem self._source
            self._apply_source_pill()                                           # executem _apply_source_pill()

        now = time.monotonic()                                                  # now ← resultat de monotonic()
        row = np.full(self._n, np.nan)                                          # NaN = forat a la corba
        if vals is not None:                                                    # si vals is not None
            for i, x in enumerate(vals):                                        # iterem: i, x in enumerate(vals)
                if x is not None:                                               # si x is not None
                    row[i] = x                                                  # assignem row[i]
                    if abs(x) > self._peaks[i]:                                 # si abs(x) > self._peaks[i]
                        self._peaks[i] = abs(x)                                 # assignem self._peaks[i]

        # Si estem desconnectats i el buffer ja acaba en forat, no cal omplir-lo de NaN
        if source != "off" or self._count == 0 or not np.all(np.isnan(self._v[:, self._head - 1])):
            self._t[self._head] = now                                           # assignem self._t[self._head]
            self._v[:, self._head] = row                                        # assignem self._v[:, self._head]
            self._head = (self._head + 1) % self._cap                           # assignem self._head
            self._count = min(self._count + 1, self._cap)                       # self._count ← resultat de min()

        self._tick += 1                                                         # assignem self._tick
        if not self.isVisible():                                                # amagat → només gravem
            return                                                              # retornem sense valor
        if not self._paused and self._tick % DRAW_EVERY_N == 0:                 # si toca redibuixar
            self._redraw(now)                                                   # executem _redraw()
        if self._tick % LABEL_EVERY_N == 0:                                     # si self._tick % LABEL_EVERY_N == 0
            self._refresh_labels()                                              # executem _refresh_labels()

    def _ordered(self):                                                         # definim la funció _ordered()
        """Buffer en ordre cronològic (antic → nou)."""
        if self._count < self._cap:                                             # si self._count < self._cap
            return self._t[:self._count], self._v[:, :self._count]              # retornem: self._t[:self._count], self._v[:, :s...
        idx = np.r_[self._head:self._cap, 0:self._head]                         # assignem idx
        return self._t[idx], self._v[:, idx]                                    # retornem: self._t[idx], self._v[:, idx]

    # ==========================================================================
    # DIBUIX
    # ==========================================================================

    def _redraw(self, now):                                                     # definim la funció _redraw()
        t, vv = self._ordered()                                                 # t, vv ← resultat de _ordered()
        x = t - now                                                             # segons relatius a "ara" (negatius)
        keep = x >= -(self._window_s + 1.0)                                     # una mica més enllà de la vora → la corba no "neix" a dins
        x = x[keep]                                                             # assignem x
        vv = vv[:, keep]                                                        # assignem vv
        self._view_x, self._view_v = x, vv                                      # assignem self._view_x, self._view_v

        for i, cell in enumerate(self._cells):                                  # iterem: i, cell in enumerate(self._cells)
            cell.set_series(x, vv[i] if x.size else np.empty(0), self._window_s, self._auto_scale)
        if self._mouse_x is not None:                                           # la creu segueix les dades que es mouen
            self._update_hover()                                                # executem _update_hover()

    def _refresh_labels(self):                                                  # definim la funció _refresh_labels()
        if self._count == 0:                                                    # si self._count == 0
            vv = np.zeros((self._n, 0))                                         # vv ← resultat de zeros()
        else:                                                                   # últimes N mostres (mitjana → número estable)
            idx = [(self._head - 1 - k) % self._cap for k in range(min(LABEL_EVERY_N, self._count))]
            vv = self._v[:, idx]                                                # assignem vv
        total = 0.0                                                             # assignem total
        any_val = False                                                         # assignem any_val
        for i, cell in enumerate(self._cells):                                  # iterem: i, cell in enumerate(self._cells)
            col = vv[i][np.isfinite(vv[i])] if vv.size else np.empty(0)         # assignem col
            val = float(col.mean()) if col.size and self._source != "off" else None
            if val is not None:                                                 # si val is not None
                total += abs(val)                                               # assignem total
                any_val = True                                                  # assignem any_val
            cell.header.set_state(val, float(self._peaks[i]))                   # executem set_state()
        self._total.setText(f"Σ {total/1000:.2f} A" if any_val else "Σ —")

    def _apply_source_pill(self):                                               # definim la funció _apply_source_pill()
        text, color = {                                                         # tot en grisos: la intensitat del gris diu l'estat
            "live":  ("●  EN DIRECTE", "#e5e5ea"),
            "sim":   ("●  SIMULACIÓ", "#aeaeb2"),
            "stale": ("▲  SENSE FEEDBACK", "#aeaeb2"),
            "off":   ("○  DESCONNECTAT", C_TEXT_MUTED),
        }[self._source]
        self._pill.setText(text)                                                # executem setText()
        self._pill.setStyleSheet(style_pill(color))                             # executem setStyleSheet()

    # ==========================================================================
    # INTERACCIÓ
    # ==========================================================================

    def _on_hover_x(self, x):                                                   # definim la funció _on_hover_x()
        if x is None:                                                           # el ratolí ha sortit
            self._hide_hover()                                                  # executem _hide_hover()
            return                                                              # retornem sense valor
        self._mouse_x = float(x)                                                # assignem self._mouse_x
        self._update_hover()                                                    # executem _update_hover()

    def _update_hover(self):                                                    # definim la funció _update_hover()
        x = self._view_x                                                        # assignem x
        if x.size == 0:                                                         # si x.size == 0
            self._hide_hover()                                                  # executem _hide_hover()
            return                                                              # retornem sense valor
        k = int(np.clip(np.searchsorted(x, self._mouse_x), 0, x.size - 1))      # mostra més propera
        if k > 0 and abs(x[k - 1] - self._mouse_x) < abs(x[k] - self._mouse_x):  # si la de l'esquerra és més a prop
            k -= 1                                                              # assignem k
        xs = float(x[k])                                                        # xs ← resultat de float()
        for i, cell in enumerate(self._cells):                                  # iterem: i, cell in enumerate(self._cells)
            cell.set_hover(xs, self._view_v[i, k])                              # executem set_hover()
        self._when.setText("@ ara" if abs(xs) < 0.05 else f"@ −{abs(xs):.2f} s")

    def _hide_hover(self):                                                      # definim la funció _hide_hover()
        if self._mouse_x is None:                                               # ja amagada
            return                                                              # retornem sense valor
        self._mouse_x = None                                                    # assignem self._mouse_x
        for cell in self._cells:                                                # iterem: cell in self._cells
            cell.set_hover(None, None)                                          # executem set_hover()
        self._when.setText("")                                                  # executem setText()

    def resizeEvent(self, ev):                                                  # definim la funció resizeEvent()
        super().resizeEvent(ev)                                                 # executem super()
        self._sub.setVisible(self.width() > 1080)                               # si anem justos d'espai, fora el subtítol

    def _set_window(self, seconds):                                             # definim la funció _set_window()
        self._window_s = int(seconds)                                           # assignem self._window_s
        self._redraw_now()                                                      # executem _redraw_now()

    def _set_auto_scale(self, auto):                                            # definim la funció _set_auto_scale()
        self._auto_scale = bool(auto)                                           # assignem self._auto_scale
        self._redraw_now()                                                      # executem _redraw_now()

    def _set_paused(self, paused):                                              # definim la funció _set_paused()
        self._paused = bool(paused)                                             # assignem self._paused
        self._paused_now = time.monotonic() if paused else None                 # assignem self._paused_now
        self._btn_pause.setText("▶  Reprendre" if paused else "❚❚  Pausa")
        if not paused:                                                          # si not paused
            self._redraw(time.monotonic())                                      # executem _redraw()

    def _redraw_now(self):                                                      # definim la funció _redraw_now()
        """Redibuixa respectant la pausa (en pausa, l'instant de referència no es mou)."""
        self._redraw(self._paused_now if self._paused else time.monotonic())    # executem _redraw()

    def _reset_peaks(self):                                                     # definim la funció _reset_peaks()
        self._peaks[:] = 0.0                                                    # assignem self._peaks[:]
        self._refresh_labels()                                                  # executem _refresh_labels()

    def _export_csv(self):                                                      # definim la funció _export_csv()
        t, vv = self._ordered()                                                 # t, vv ← resultat de _ordered()
        if t.size == 0:                                                         # si t.size == 0
            QtWidgets.QMessageBox.information(self, "Intensitats", "Encara no hi ha dades per exportar.")
            return                                                              # retornem sense valor
        stamp = time.strftime("%Y%m%d_%H%M%S")                                  # stamp ← resultat de strftime()
        root = os.path.dirname(os.path.dirname(os.path.dirname(                 # panels → neurospine_app → scripts → arrel
            os.path.dirname(os.path.abspath(__file__)))))
        default = os.path.join(root, "data", f"intensitats_{stamp}.csv")        # default ← resultat de join()
        path, _ = QtWidgets.QFileDialog.getSaveFileName(                        # path, _ ← resultat de getSaveFileName()
            self, "Exporta intensitats", default, "CSV (*.csv)")
        if not path:                                                            # si not path
            return                                                              # retornem sense valor
        try:                                                                    # intentem el bloc següent
            with open(path, "w", newline="", encoding="utf-8") as f:            # obrim el fitxer
                w = csv.writer(f)                                               # w ← resultat de writer()
                w.writerow(["timestamp", "t_rel_s"] + [f"{n}_mA" for n in self._names])
                t_end = t[-1]                                                   # assignem t_end
                for k in range(t.size):                                         # iterem: k in range(t.size)
                    wall = time.strftime("%H:%M:%S", time.localtime(t[k] + self._t0_wall))
                    ms = int(((t[k] + self._t0_wall) % 1) * 1000)               # ms ← resultat de int()
                    w.writerow([f"{wall}.{ms:03d}", f"{t[k] - t_end:.3f}"]
                               + ["" if not np.isfinite(x) else int(round(x)) for x in vv[:, k]])
            print(f"[intensitats] exportat {t.size} mostres a {path}")
        except Exception as e:                                                  # capturem excepció: Exception as e
            QtWidgets.QMessageBox.warning(self, "Intensitats", f"No s'ha pogut desar:\n{e}")

    def stop(self):                                                             # definim la funció stop()
        """Atura el mostreig (per al cleanup del workspace)."""
        self._timer.stop()                                                      # executem stop()
