"""
Workspace de Planificació de Cargols — versió PyQtGraph.

BASAT EN LA VERSIÓ ANTIGA QUE FUNCIONAVA VISUALMENT, amb aquestes millores:

    1. DISTÀNCIA A LA MEDUL·LA — PRECISA
       Fem servir vtkCellLocator (distància al triangle, no al vèrtex),
       i APLIQUEM LA MATEIXA TRANSFORMACIÓ que la medul·la (rotate_z(180))
       als endpoints del cargol abans de fer la query. Així el càlcul
       és coherent amb la geometria de la medul·la rotada.

    2. MARCADOR DEL PUNT CRÍTIC
       A la vista axial es mostra una X vermella al punt de l'eix del
       cargol més proper a la medul·la, amb la distància al panell dret.

    3. BOTÓ "CREAR TRAJECTÒRIA" AMB FEEDBACK CLAR
       Quan s'activa el mode creació, el botó canvia a blanc perquè es
       vegi molt clar que està actiu. Torna al color normal quan es
       fixa el cargol o es cancel·la.

    4. VISTA DE DENSITAT ÒSSIA (X-RAY)
       Toggle nou que baixa l'opacitat de l'os al 3D per veure-hi a través
       (medul·la, cargols, canals dels pedicles).

    5. ESTIL MONOCROMÀTIC
       Crosshair gris (no blau), mando gris, preview blanc.
       Els cargols mantenen els seus colors distintius per poder-los
       identificar a la llista.

    6. RECONSTRUCCIÓ 3D OPACA
       L'os al 3D és opac per defecte (aspecte metàl·lic professional),
       i el toggle X-ray el fa translúcid quan es necessita.

TOT EL SISTEMA DE COORDENADES ES MANTÉ IGUAL QUE LA VERSIÓ ANTIGA
per garantir que la superposició visual continua funcionant correctament.
"""

import os                                                                       # importem el mòdul os
import math                                                                     # importem el mòdul math
import numpy as np                                                              # importem el mòdul numpy as np

from qtpy import QtWidgets, QtCore, QtGui                                       # importem QtWidgets, QtCore, QtGui des de qtpy
from qtpy.QtGui import QColor                                                   # importem QColor des de qtpy.QtGui
from qtpy.QtCore import Qt                                                      # importem Qt des de qtpy.QtCore

import SimpleITK as sitk                                                        # importem el mòdul SimpleITK as sitk
import pyvista as pv                                                            # importem el mòdul pyvista as pv
from pyvistaqt import QtInteractor                                              # importem QtInteractor des de pyvistaqt
import pyqtgraph as pg                                                          # importem el mòdul pyqtgraph as pg

import vtk                                                                      # importem el mòdul vtk

from ..theme import (                                                           # importem ( des de ..theme
    C_BG_WINDOW, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_ACCENT,
    C_SAFE, C_WARN, C_DANGER,
    CARGOL_COLORS,
    SIZE_XS, SIZE_LG, FONT_MONO,
)


# ==============================================================================
# CONFIGURACIÓ GLOBAL
# ==============================================================================

vtk.vtkObject.GlobalWarningDisplayOff()                                         # executem GlobalWarningDisplayOff()
os.environ["VTK_SILENCE_GET_WINDOW_INFO_ERRORS"] = "1"                          # os.environ["VTK_SILENCE_GET_WINDOW_INFO_ERRORS"] = "1"

pg.setConfigOption("background", C_BG_WINDOW)                                   # executem setConfigOption()
pg.setConfigOption("foreground", C_TEXT_DIM)                                    # executem setConfigOption()
pg.setConfigOption("useOpenGL", False)                                          # executem setConfigOption()
pg.setConfigOption("imageAxisOrder", "row-major")                               # executem setConfigOption()


# ==============================================================================
# RUTES
# ==============================================================================

PACIENT_ID = "839201"                                                           # assignem PACIENT_ID
_HERE        = os.path.dirname(os.path.abspath(__file__))                       # _HERE ← resultat de dirname()
_APP_DIR     = os.path.dirname(_HERE)                                           # _APP_DIR ← resultat de dirname()
_SCRIPTS     = os.path.dirname(_APP_DIR)                                        # _SCRIPTS ← resultat de dirname()
_ROOT        = os.path.dirname(_SCRIPTS)                                        # _ROOT ← resultat de dirname()
_PACIENT_DIR = os.path.join(_ROOT, "data", "pacients", PACIENT_ID)              # _PACIENT_DIR ← resultat de join()

TAC_PATH    = os.path.join(_PACIENT_DIR, "escoliosis.nrrd")                     # TAC_PATH ← resultat de join()
MEDULA_PATH = os.path.join(_PACIENT_DIR, "medula_rectificada.stl")              # MEDULA_PATH ← resultat de join()

DIAMETRES_STD = [4.5, 5.5, 6.5, 7.5]                                            # assignem DIAMETRES_STD


# ==============================================================================
# BONE LUT
# ==============================================================================
def make_bone_lut():                                                            # definim la funció make_bone_lut()
    n = 256                                                                     # assignem n
    lut = np.zeros((n, 4), dtype=np.uint8)                                      # lut ← resultat de zeros()
    for i in range(n):                                                          # iterem: i in range(n)
        v = i / (n - 1)                                                         # v ← resultat de i /()
        r = int(255 * v)                                                        # r ← resultat de int()
        g = int(255 * v)                                                        # g ← resultat de int()
        b = int(255 * min(1.0, v * 1.05))                                       # b ← resultat de int()
        lut[i] = (r, g, b, 255)                                                 # lut[i] ← resultat de ()
    return lut                                                                  # retornem: lut

BONE_LUT = make_bone_lut()                                                      # BONE_LUT ← resultat de make_bone_lut()


# ==============================================================================
# CT VIEWPORT
# ==============================================================================

class CTViewport(pg.GraphicsLayoutWidget):                                      # definim la classe CTViewport
    """
    Una vista CT (axial/sagital/coronal) amb PyQtGraph.
    """

    wheelScrolled    = QtCore.Signal(int)                                       # wheelScrolled ← resultat de Signal()
    clickedInView    = QtCore.Signal(float, float, int)                         # clickedInView ← resultat de Signal()
    mouseMovedInView = QtCore.Signal(float, float)                              # mouseMovedInView ← resultat de Signal()

    def __init__(self, name, invert_y=False, parent=None):                      # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.name = name                                                        # assignem self.name

        self.vb = self.addViewBox(row=0, col=0, lockAspect=True, invertY=invert_y)  # self.vb ← resultat de addViewBox()
        self.vb.setMenuEnabled(False)                                           # executem setMenuEnabled()
        self.vb.setMouseEnabled(x=True, y=True)                                 # assignem self.vb.setMouseEnabled(x

        # Imatge base
        self.image_item = pg.ImageItem()                                        # self.image_item ← resultat de ImageItem()
        self.image_item.setLookupTable(BONE_LUT)                                # executem setLookupTable()
        self.image_item.setLevels([0, 1500])                                    # executem setLevels()
        self.vb.addItem(self.image_item)                                        # executem addItem()

        # Crosshair gris translúcid (abans era blau accent)
        pen = pg.mkPen(color=(200, 200, 205, 180), width=1, style=Qt.DashLine)  # pen ← resultat de mkPen()
        self.hline = pg.InfiniteLine(angle=0,  pen=pen, movable=False)          # self.hline ← resultat de InfiniteLine()
        self.vline = pg.InfiniteLine(angle=90, pen=pen, movable=False)          # self.vline ← resultat de InfiniteLine()
        self.vb.addItem(self.hline)                                             # executem addItem()
        self.vb.addItem(self.vline)                                             # executem addItem()

        # Medul·la (contorn 2D, vermell suau)
        self.medula_curve = pg.PlotCurveItem(                                   # assignem self.medula_curve
            pen=pg.mkPen(color=(240, 100, 100, 220), width=1.8),
            connect="pairs",
        )
        self.vb.addItem(self.medula_curve)                                      # executem addItem()

        # Mando gris
        self.mando_line = pg.PlotDataItem(                                      # assignem self.mando_line
            pen=pg.mkPen(color=(220, 220, 225), width=2),
        )
        self.mando_pts = pg.ScatterPlotItem(                                    # assignem self.mando_pts
            pen=pg.mkPen(color=(220, 220, 225), width=1),
            brush=pg.mkBrush(color=(220, 220, 225)),
            size=8, symbol="+",
        )
        self.vb.addItem(self.mando_line)                                        # executem addItem()
        self.vb.addItem(self.mando_pts)                                         # executem addItem()

        # Cargols
        self.screw_items = {}                                                   # assignem self.screw_items

        # Preview durant la creació (blanc)
        self.preview_p0 = pg.ScatterPlotItem(                                   # assignem self.preview_p0
            pen=pg.mkPen(color=(255, 255, 255), width=2),
            brush=pg.mkBrush(color=(255, 255, 255, 200)),
            size=12, symbol="o",
        )
        self.preview_line = pg.PlotDataItem(                                    # assignem self.preview_line
            pen=pg.mkPen(color=(255, 255, 255, 220), width=2, style=Qt.DashLine),
        )
        self.vb.addItem(self.preview_p0)                                        # executem addItem()
        self.vb.addItem(self.preview_line)                                      # executem addItem()

        # Overlay de densitat òssia (porositat). ImageItem addicional que
        # es dibuixa sobre la imatge CT quan el toggle "densitat" està actiu.
        # Usa un colormap on:
        #   - transparent → sense dades / fora del rang
        #   - vermell/taronja → HU baix (os POROS, mal ancoratge)
        #   - blanc/verd → HU alt (os DENS, ideal per cargol)
        self.density_item = pg.ImageItem()                                      # self.density_item ← resultat de ImageItem()
        self.density_item.setZValue(5)   # per sobre de la imatge base
        self.density_item.setVisible(False)                                     # executem setVisible()
        self.vb.addItem(self.density_item)                                      # executem addItem()

        # Events
        self.scene().sigMouseClicked.connect(self._on_scene_click)              # executem scene()
        self.scene().sigMouseMoved.connect(self._on_scene_moved)                # executem scene()

    # ------------------------------------------------------------
    def show_preview_p0(self, x, y):                                            # definim la funció show_preview_p0()
        self.preview_p0.setData(x=[x], y=[y])                                   # assignem self.preview_p0.setData(x

    def show_preview_line(self, x0, y0, x1, y1):                                # definim la funció show_preview_line()
        self.preview_line.setData(x=[x0, x1], y=[y0, y1])                       # assignem self.preview_line.setData(x

    def clear_preview(self):                                                    # definim la funció clear_preview()
        self.preview_p0.clear()                                                 # executem clear()
        self.preview_line.clear()                                               # executem clear()

    def set_image(self, arr_2d):                                                # definim la funció set_image()
        self.image_item.setImage(arr_2d, autoLevels=False)                      # assignem self.image_item.setImage(arr_2d, autoLevels

    def set_density(self, rgba_2d):                                             # definim la funció set_density()
        """RGBA (H, W, 4) uint8 dibuixat sobre la imatge base. None per amagar."""  # """RGBA (H, W, 4) uint8 dibuixat sobre la imatge base. No...
        if rgba_2d is None:                                                     # si rgba_2d is None
            self.density_item.setVisible(False)                                 # executem setVisible()
        else:                                                                   # en cas contrari
            self.density_item.setImage(rgba_2d, autoLevels=False)               # assignem self.density_item.setImage(rgba_2d, autoLevels
            self.density_item.setVisible(True)                                  # executem setVisible()

    def clear_density(self):                                                    # definim la funció clear_density()
        self.density_item.setVisible(False)                                     # executem setVisible()

    def set_crosshair(self, x, y):                                              # definim la funció set_crosshair()
        self.vline.setValue(x)                                                  # executem setValue()
        self.hline.setValue(y)                                                  # executem setValue()

    def set_medula(self, xs, ys):                                               # definim la funció set_medula()
        if xs is None or len(xs) == 0:                                          # si xs is None or len(xs) == 0
            self.medula_curve.setData(x=[], y=[])                               # assignem self.medula_curve.setData(x
        else:                                                                   # en cas contrari
            self.medula_curve.setData(x=xs, y=ys)                               # assignem self.medula_curve.setData(x

    def clear_medula(self):                                                     # definim la funció clear_medula()
        self.medula_curve.setData(x=[], y=[])                                   # assignem self.medula_curve.setData(x

    def set_mando(self, x0, y0, x1, y1):                                        # definim la funció set_mando()
        self.mando_line.setData(x=[x0, x1], y=[y0, y1])                         # assignem self.mando_line.setData(x
        self.mando_pts.setData(x=[x0, x1], y=[y0, y1])                          # assignem self.mando_pts.setData(x

    def clear_mando(self):                                                      # definim la funció clear_mando()
        self.mando_line.clear()                                                 # executem clear()
        self.mando_pts.clear()                                                  # executem clear()

    # ---- Cargols (representació realista amb cap, rosques i punta) ----

    def _clear_screw_items(self, idx):                                          # definim la funció _clear_screw_items()
        """Elimina tots els sub-items d'un cargol (poden ser molts: cos,
        cap, rosques, endpoints)."""
        if idx not in self.screw_items:                                         # si idx not in self.screw_items
            return                                                              # retornem sense valor
        for it in self.screw_items[idx]["items"]:                               # iterem: it in self.screw_items[idx]["items"]
            try:                                                                # intentem el bloc següent
                self.vb.removeItem(it)                                          # executem removeItem()
            except Exception:                                                   # capturem excepció: Exception
                pass                                                            # no fem res (placeholder)
        del self.screw_items[idx]                                               # eliminem: self.screw_items[idx]

    def set_screw(self, idx, poly_pts, color, selected=False, p0=None, p1=None):  # definim la funció set_screw()
        """
        Compat: dibuix simple rectangle (mode fallback). Usar
        set_screw_realistic per veure el cargol amb detall.
        """
        border = QColor("white") if selected else QColor(color)                 # border ← resultat de QColor()
        fill = QColor(color)                                                    # fill ← resultat de QColor()
        fill.setAlpha(200 if selected else 110)                                 # executem setAlpha()

        polygon = QtGui.QPolygonF([QtCore.QPointF(x, y) for x, y in poly_pts])  # polygon ← resultat de QPolygonF()

        if p0 is None or p1 is None:                                            # si p0 is None or p1 is None
            p0 = ((poly_pts[0][0] + poly_pts[3][0]) / 2.0,                      # assignem p0
                  (poly_pts[0][1] + poly_pts[3][1]) / 2.0)
            p1 = ((poly_pts[1][0] + poly_pts[2][0]) / 2.0,                      # assignem p1
                  (poly_pts[1][1] + poly_pts[2][1]) / 2.0)

        self._clear_screw_items(idx)                                            # executem _clear_screw_items()

        item = QtWidgets.QGraphicsPolygonItem()                                 # item ← resultat de QGraphicsPolygonItem()
        item.setPolygon(polygon)                                                # executem setPolygon()
        item.setBrush(QtGui.QBrush(fill))                                       # executem setBrush()
        item.setPen(QtGui.QPen(border, 0))                                      # executem setPen()
        self.vb.addItem(item)                                                   # executem addItem()

        pts = pg.ScatterPlotItem(                                               # assignem pts
            x=[p0[0], p1[0]], y=[p0[1], p1[1]],
            symbol=["o", "s"], size=10,
            pen=pg.mkPen(border, width=1.5),
            brush=pg.mkBrush(color),
        )
        self.vb.addItem(pts)                                                    # executem addItem()

        self.screw_items[idx] = {"items": [item, pts], "kind": "rect"}          # assignem self.screw_items[idx]

    def set_screw_realistic(self, idx, geom, color, selected=False):            # definim la funció set_screw_realistic()
        """
        Dibuixa el cargol amb aspecte de cargol pedicular real (perfil
        de perfil). El diccionari geom té:

            p0_view    (x, y)  Cap del cargol a la vista (píxels)
            p1_view    (x, y)  Punta del cargol a la vista (píxels)
            half_width_head  Semi-amplada al cap (píxels)
            half_width_body  Semi-amplada al cos rosca (píxels)
            head_frac  Fracció de la longitud que ocupa el cap (0-1)
            tip_frac   Fracció de la longitud que ocupa la punta cònica (0-1)
            n_threads  Nombre de rosques a dibuixar al cos

        Es dibuixa: cap (més ample, fosc, sense rosques) + cos rosca amb
        línies fines transversals + punta cònica.
        """
        self._clear_screw_items(idx)                                            # executem _clear_screw_items()

        p0 = geom["p0_view"]                                                    # assignem p0
        p1 = geom["p1_view"]                                                    # assignem p1
        hw_head = geom["half_width_head"]                                       # assignem hw_head
        hw_body = geom["half_width_body"]                                       # assignem hw_body
        head_frac = geom.get("head_frac", 0.15)                                 # head_frac ← resultat de get()
        tip_frac  = geom.get("tip_frac",  0.10)                                 # tip_frac ← resultat de get()
        n_threads = geom.get("n_threads", 8)                                    # n_threads ← resultat de get()

        # Vector eix i normal
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]                                   # assignem dx, dy
        L = math.hypot(dx, dy)                                                  # L ← resultat de hypot()
        if L < 1e-6:                                                            # si L < 1e-6
            return                                                              # retornem sense valor
        ux, uy = dx / L, dy / L               # unitari al llarg del cargol
        nx, ny = -uy, ux                       # normal (esquerra)

        # Colors
        body_fill  = QColor(color)                                              # body_fill ← resultat de QColor()
        body_fill.setAlpha(220 if selected else 150)                            # executem setAlpha()
        head_fill  = QColor(color).darker(135)                                  # head_fill ← resultat de QColor()
        head_fill.setAlpha(255 if selected else 200)                            # executem setAlpha()
        border_col = QColor("white") if selected else QColor(color).darker(180)  # border_col ← resultat de QColor()
        border_pen = QtGui.QPen(border_col, 0.8)                                # border_pen ← resultat de QPen()
        border_pen.setCosmetic(True)                                            # executem setCosmetic()
        thread_col = QColor(color).darker(180)                                  # thread_col ← resultat de QColor()
        thread_col.setAlpha(180)                                                # executem setAlpha()
        thread_pen = QtGui.QPen(thread_col, 0.6)                                # thread_pen ← resultat de QPen()
        thread_pen.setCosmetic(True)                                            # executem setCosmetic()

        items = []                                                              # assignem items

        # Punts clau al llarg de l'eix
        pt = lambda t: (p0[0] + t*dx, p0[1] + t*dy)                             # pt ← resultat de lambda t:()
        head_end = pt(head_frac)                                                # head_end ← resultat de pt()
        tip_start = pt(1.0 - tip_frac)                                          # tip_start ← resultat de pt()
        tip_end   = pt(1.0)                                                     # tip_end ← resultat de pt()

        # ---- 1. Cap del cargol (rectangle més ample, fosc) ----
        head_poly = QtGui.QPolygonF([                                           # assignem head_poly
            QtCore.QPointF(p0[0] + nx*hw_head, p0[1] + ny*hw_head),
            QtCore.QPointF(head_end[0] + nx*hw_head, head_end[1] + ny*hw_head),
            QtCore.QPointF(head_end[0] - nx*hw_head, head_end[1] - ny*hw_head),
            QtCore.QPointF(p0[0] - nx*hw_head, p0[1] - ny*hw_head),
        ])
        head_item = QtWidgets.QGraphicsPolygonItem(head_poly)                   # head_item ← resultat de QGraphicsPolygonItem()
        head_item.setBrush(QtGui.QBrush(head_fill))                             # executem setBrush()
        head_item.setPen(border_pen)                                            # executem setPen()
        self.vb.addItem(head_item)                                              # executem addItem()
        items.append(head_item)                                                 # executem append()

        # ---- 2. Cos rosca (rectangle amb ample body) ----
        body_poly = QtGui.QPolygonF([                                           # assignem body_poly
            QtCore.QPointF(head_end[0] + nx*hw_body, head_end[1] + ny*hw_body),
            QtCore.QPointF(tip_start[0] + nx*hw_body, tip_start[1] + ny*hw_body),
            QtCore.QPointF(tip_start[0] - nx*hw_body, tip_start[1] - ny*hw_body),
            QtCore.QPointF(head_end[0] - nx*hw_body, head_end[1] - ny*hw_body),
        ])
        body_item = QtWidgets.QGraphicsPolygonItem(body_poly)                   # body_item ← resultat de QGraphicsPolygonItem()
        body_item.setBrush(QtGui.QBrush(body_fill))                             # executem setBrush()
        body_item.setPen(border_pen)                                            # executem setPen()
        self.vb.addItem(body_item)                                              # executem addItem()
        items.append(body_item)                                                 # executem append()

        # ---- 3. Punta cònica (triangle isòsceles) ----
        tip_poly = QtGui.QPolygonF([                                            # assignem tip_poly
            QtCore.QPointF(tip_start[0] + nx*hw_body, tip_start[1] + ny*hw_body),
            QtCore.QPointF(tip_end[0], tip_end[1]),
            QtCore.QPointF(tip_start[0] - nx*hw_body, tip_start[1] - ny*hw_body),
        ])
        tip_item = QtWidgets.QGraphicsPolygonItem(tip_poly)                     # tip_item ← resultat de QGraphicsPolygonItem()
        tip_item.setBrush(QtGui.QBrush(body_fill))                              # executem setBrush()
        tip_item.setPen(border_pen)                                             # executem setPen()
        self.vb.addItem(tip_item)                                               # executem addItem()
        items.append(tip_item)                                                  # executem append()

        # ---- 4. Rosques al cos (línies fines transversals) ----
        # Densitat: n_threads espaiades uniformement al llarg del cos
        body_len_frac = 1.0 - head_frac - tip_frac                              # assignem body_len_frac
        if n_threads > 0 and body_len_frac > 0:                                 # si n_threads > 0 and body_len_frac > 0
            xs_thread = []                                                      # assignem xs_thread
            ys_thread = []                                                      # assignem ys_thread
            for i in range(n_threads):                                          # iterem: i in range(n_threads)
                t = head_frac + body_len_frac * (i + 0.5) / n_threads           # assignem t
                cx, cy = pt(t)                                                  # cx, cy ← resultat de pt()
                xs_thread.extend([cx + nx*hw_body, cx - nx*hw_body])            # executem extend()
                ys_thread.extend([cy + ny*hw_body, cy - ny*hw_body])            # executem extend()
            threads_item = pg.PlotCurveItem(                                    # assignem threads_item
                x=xs_thread, y=ys_thread,
                pen=thread_pen, connect="pairs")
            self.vb.addItem(threads_item)                                       # executem addItem()
            items.append(threads_item)                                          # executem append()

        # ---- 5. Endpoints (marcadors del cap i la punta) ----
        pts = pg.ScatterPlotItem(                                               # assignem pts
            x=[p0[0], p1[0]], y=[p0[1], p1[1]],
            symbol=["o", "t"],   # cercle al cap, triangle a la punta
            size=[12, 10],
            pen=pg.mkPen(QColor("white") if selected else QColor(color),
                         width=1.5),
            brush=pg.mkBrush(color),
        )
        self.vb.addItem(pts)                                                    # executem addItem()
        items.append(pts)                                                       # executem append()

        self.screw_items[idx] = {"items": items, "kind": "realistic"}           # assignem self.screw_items[idx]

    def set_screw_ellipse(self, idx, cx, cy, semi_major, semi_minor,            # definim la funció set_screw_ellipse()
                          angle_deg, color, selected=False):
        """
        Dibuixa la secció el·líptica del cargol (quan el pla talla el
        cilindre amb un angle). Ara amb un anell interior més fosc per
        donar sensació de "canó" del cargol vist des de dins.
        """
        self._clear_screw_items(idx)                                            # executem _clear_screw_items()

        border = QColor("white") if selected else QColor(color).darker(180)     # border ← resultat de QColor()
        border_pen = QtGui.QPen(border, 1.2)                                    # border_pen ← resultat de QPen()
        border_pen.setCosmetic(True)                                            # executem setCosmetic()

        fill = QColor(color)                                                    # fill ← resultat de QColor()
        fill.setAlpha(180 if selected else 130)                                 # executem setAlpha()

        # Anell exterior (rosques)
        rect_outer = QtCore.QRectF(-semi_major, -semi_minor,                    # assignem rect_outer
                                   2*semi_major, 2*semi_minor)
        ell_outer = QtWidgets.QGraphicsEllipseItem(rect_outer)                  # ell_outer ← resultat de QGraphicsEllipseItem()
        ell_outer.setBrush(QtGui.QBrush(fill))                                  # executem setBrush()
        ell_outer.setPen(border_pen)                                            # executem setPen()
        ell_outer.setPos(cx, cy)                                                # executem setPos()
        ell_outer.setRotation(angle_deg)                                        # executem setRotation()
        self.vb.addItem(ell_outer)                                              # executem addItem()

        # Cor interior (nucli del cargol, més fosc)
        inner_scale = 0.55                                                      # assignem inner_scale
        rect_inner = QtCore.QRectF(-semi_major*inner_scale,                     # assignem rect_inner
                                   -semi_minor*inner_scale,
                                   2*semi_major*inner_scale,
                                   2*semi_minor*inner_scale)
        core_col = QColor(color).darker(160)                                    # core_col ← resultat de QColor()
        core_col.setAlpha(220 if selected else 170)                             # executem setAlpha()
        ell_inner = QtWidgets.QGraphicsEllipseItem(rect_inner)                  # ell_inner ← resultat de QGraphicsEllipseItem()
        ell_inner.setBrush(QtGui.QBrush(core_col))                              # executem setBrush()
        ell_inner.setPen(QtGui.QPen(QColor(0, 0, 0, 0), 0))  # sense stroke
        ell_inner.setPos(cx, cy)                                                # executem setPos()
        ell_inner.setRotation(angle_deg)                                        # executem setRotation()
        self.vb.addItem(ell_inner)                                              # executem addItem()

        # Marcador central petit
        pts = pg.ScatterPlotItem(                                               # assignem pts
            x=[cx], y=[cy], symbol="+", size=8,
            pen=pg.mkPen(border, width=1.5),
            brush=pg.mkBrush(color),
        )
        self.vb.addItem(pts)                                                    # executem addItem()

        self.screw_items[idx] = {                                               # assignem self.screw_items[idx]
            "items": [ell_outer, ell_inner, pts],
            "kind": "ellipse",
        }

    def remove_screw(self, idx):                                                # definim la funció remove_screw()
        self._clear_screw_items(idx)                                            # executem _clear_screw_items()

    def clear_screws(self):                                                     # definim la funció clear_screws()
        for idx in list(self.screw_items.keys()):                               # iterem: idx in list(self.screw_items.keys())
            self._clear_screw_items(idx)                                        # executem _clear_screw_items()

    # ==========================================================
    # EVENTS
    # ==========================================================
    def wheelEvent(self, event):                                                # definim la funció wheelEvent()
        mods = event.modifiers() if hasattr(event, "modifiers") else Qt.NoModifier  # assignem mods
        if mods & Qt.ControlModifier:                                           # si mods & Qt.ControlModifier
            super().wheelEvent(event)                                           # executem super()
        else:                                                                   # en cas contrari
            delta = 1 if event.angleDelta().y() > 0 else -1                     # assignem delta
            self.wheelScrolled.emit(delta)                                      # executem emit()
            event.accept()                                                      # executem accept()

    def _on_scene_click(self, ev):                                              # definim la funció _on_scene_click()
        if not self.vb.sceneBoundingRect().contains(ev.scenePos()):             # si not self.vb.sceneBoundingRect().conta...
            return                                                              # retornem sense valor
        view_pos = self.vb.mapSceneToView(ev.scenePos())                        # view_pos ← resultat de mapSceneToView()
        button = 1 if ev.button() == Qt.LeftButton else \
                 2 if ev.button() == Qt.RightButton else 3                      # assignem 2 if ev.button()
        self.clickedInView.emit(view_pos.x(), view_pos.y(), button)             # executem emit()

    def _on_scene_moved(self, scene_pos):                                       # definim la funció _on_scene_moved()
        if not self.vb.sceneBoundingRect().contains(scene_pos):                 # si not self.vb.sceneBoundingRect().conta...
            return                                                              # retornem sense valor
        view_pos = self.vb.mapSceneToView(scene_pos)                            # view_pos ← resultat de mapSceneToView()
        self.mouseMovedInView.emit(view_pos.x(), view_pos.y())                  # executem emit()


# ==============================================================================
# PERFIL DE DENSITAT AL LLARG DEL CARGOL (anàlisi d'ancoratge)
# ==============================================================================

class DensityProfile(QtWidgets.QWidget):                                        # definim la classe DensityProfile
    """
    HU de l'os que mosseguen les estries, de l'entrada (esquerra) a la punta
    (dreta). Franges vermelles = trams on el cargol surt de l'os. Línies
    discontínues = llindars orientatius HU_LOW / HU_OK. ▼ = istme.
    """

    HU_TOP = 600.0                                                              # sostre de l'eix Y (os trabecular → cortical)

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setFixedHeight(118)                                                # executem setFixedHeight()
        self.res = None                                                         # resultat d'analyze_screw()

    def set_result(self, res):                                                  # definim la funció set_result()
        self.res = res                                                          # assignem self.res
        self.update()                                                           # executem update()

    def paintEvent(self, ev):                                                   # definim la funció paintEvent()
        p = QtGui.QPainter(self)                                                # p ← resultat de QPainter()
        p.setRenderHint(QtGui.QPainter.Antialiasing)                            # executem setRenderHint()
        w, h = self.width(), self.height()                                      # assignem w, h
        p.fillRect(self.rect(), QtGui.QColor("#0e0e10"))                        # executem fillRect()
        f = QtGui.QFont(self.font()); f.setPixelSize(SIZE_XS - 1)               # f ← resultat de QFont()
        p.setFont(f)                                                            # executem setFont()
        r = QtCore.QRectF(30, 8, w - 38, h - 26)                                # àrea de dades
        if self.res is None:                                                    # sense cargol
            p.setPen(QtGui.QColor(C_TEXT_MUTED))                                # executem setPen()
            p.drawText(self.rect(), Qt.AlignCenter, "Selecciona un cargol")     # executem drawText()
            p.end()                                                             # executem end()
            return                                                              # retornem sense valor
        from src.planning.screw_analysis import HU_LOW, HU_OK                   # llindars orientatius
        t, hu = self.res["profile"]                                             # assignem t, hu
        L = max(self.res["length_mm"], 1.0)                                     # assignem L
        X = lambda mm: r.left() + mm / L * r.width()                            # mm → px
        Y = lambda v: r.bottom() - min(max(v, 0.0), self.HU_TOP) / self.HU_TOP * r.height()

        # Trams perforats
        red = QtGui.QColor(C_DANGER); red.setAlpha(70)                          # red ← resultat de QColor()
        for a, b, _canal in self.res["breaches"]:                               # iterem: a, b, _canal
            p.fillRect(QtCore.QRectF(X(a), r.top(), max(X(b + 0.5) - X(a), 2), r.height()), red)
        # Llindars i eixos
        for thr, dy in ((HU_LOW, 6), (HU_OK, -6)):                              # etiqueta sota / sobre la línia
            p.setPen(QtGui.QPen(QtGui.QColor(C_TEXT_MUTED), 1, Qt.DashLine))    # executem setPen()
            p.drawLine(QtCore.QPointF(r.left(), Y(thr)), QtCore.QPointF(r.right(), Y(thr)))
            p.setPen(QtGui.QColor(C_TEXT_MUTED))                                # executem setPen()
            p.drawText(QtCore.QRectF(0, Y(thr) + dy - 7, 27, 14), Qt.AlignRight | Qt.AlignVCenter, f"{thr:.0f}")
        p.drawText(QtCore.QRectF(0, r.top() - 1, 27, 14), Qt.AlignRight | Qt.AlignVCenter, f"{self.HU_TOP:.0f}")
        p.setPen(QtGui.QPen(QtGui.QColor(C_BORDER_STRONG), 1))                  # executem setPen()
        p.drawLine(r.bottomLeft(), r.bottomRight())                             # executem drawLine()
        # Corba
        path = QtGui.QPainterPath()                                             # path ← resultat de QPainterPath()
        for i, (tt, vv) in enumerate(zip(t, hu)):                               # iterem: i, (tt, vv)
            pt = QtCore.QPointF(X(tt), Y(vv))                                   # pt ← resultat de QPointF()
            path.moveTo(pt) if i == 0 else path.lineTo(pt)                      # executem moveTo()
        p.setPen(QtGui.QPen(QtGui.QColor(C_TEXT), 1.4))                         # executem setPen()
        p.drawPath(path)                                                        # executem drawPath()
        # Istme
        if self.res.get("isthmus_mm") is not None:                              # si hi ha istme
            xi = X(self.res["isthmus_mm"])                                      # xi ← resultat de X()
            p.setPen(Qt.NoPen); p.setBrush(QtGui.QColor(C_ACCENT))              # executem setPen()
            p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(xi - 4, r.top()), QtCore.QPointF(xi + 4, r.top()),
                                           QtCore.QPointF(xi, r.top() + 6)]))
        # Eix X
        p.setPen(QtGui.QColor(C_TEXT_MUTED))                                    # executem setPen()
        p.drawText(QtCore.QRectF(r.left(), r.bottom() + 2, 60, 14), Qt.AlignLeft, "entrada")
        p.drawText(QtCore.QRectF(r.right() - 80, r.bottom() + 2, 80, 14), Qt.AlignRight, f"punta · {L:.0f} mm")
        p.drawText(QtCore.QRectF(r.left() + 4, r.top(), 60, 14), Qt.AlignLeft, "HU")
        p.end()                                                                 # executem end()


# ==============================================================================
# WIDGET PRINCIPAL
# ==============================================================================

class PlanningWidget(QtWidgets.QWidget):                                        # definim la classe PlanningWidget

    cargolsChanged = QtCore.Signal(list)                                        # cargolsChanged ← resultat de Signal()

    def __init__(self, mando_service=None, parent=None):                        # definim la funció __init__()
        super().__init__(parent)                                                # executem super()

        # ---- Mando ----
        self.mando_service = mando_service                                      # assignem self.mando_service
        self.mando_x = self.mando_y = self.mando_z = 0.0                        # assignem self.mando_x
        self.dx = self.dy = 0.0                                                 # assignem self.dx
        self.dz = 60.0                                                          # assignem self.dz
        self.offset_x = self.offset_y = self.offset_z = 0.0                     # assignem self.offset_x
        self.primera_lectura = True                                             # assignem self.primera_lectura

        # ---- Planificació ----
        self.cargols = []                                                       # assignem self.cargols
        self.current = None                                                     # assignem self.current
        self.creating = False                                                   # assignem self.creating
        self.selected_idx = None                                                # assignem self.selected_idx
        self.edit_point = "p0"                                                  # assignem self.edit_point
        self.show_medula = False                                                # assignem self.show_medula
        self.show_mando = True                                                  # assignem self.show_mando
        self.show_porosity = False                                              # assignem self.show_porosity
        self.is_3d_loaded = False                                               # assignem self.is_3d_loaded
        self.current_diam = 5.5                                                 # assignem self.current_diam

        # ---- Cache de la medul·la ----
        self._medula_cache = {"ax": {}, "sag": {}, "cor": {}}                   # assignem self._medula_cache
        self._medula_cache_ready = False                                        # assignem self._medula_cache_ready

        # ---- Càrrega de dades ----
        self._load_medical_data()                                               # executem _load_medical_data()
        self.iz, self.ix, self.iy = self.Z // 2, self.X // 2, self.Y // 2       # assignem self.iz, self.ix, self.iy

        # ---- UI ----
        self._build_ui()                                                        # executem _build_ui()

        if self.mando_service is not None:                                      # si self.mando_service is not None
            self.mando_service.reading.connect(self.on_mando_reading)           # executem connect()

        # ---- Primera pintada ----
        self._update_view("ax")                                                 # executem _update_view()
        self._update_view("sag")                                                # executem _update_view()
        self._update_view("cor")                                                # executem _update_view()
        self._refresh_screws_all_views()                                        # executem _refresh_screws_all_views()

        self._medula_cache_ready = True                                         # assignem self._medula_cache_ready

        # Debounce timers
        self._medula_pending_axes = set()                                       # self._medula_pending_axes ← resultat de set()
        self._medula_timer = QtCore.QTimer(self)                                # self._medula_timer ← resultat de QTimer()
        self._medula_timer.setSingleShot(True)                                  # executem setSingleShot()
        self._medula_timer.timeout.connect(self._flush_medula_pending)          # executem connect()

        self._nudge_timer = QtCore.QTimer(self)                                 # self._nudge_timer ← resultat de QTimer()
        self._nudge_timer.setSingleShot(True)                                   # executem setSingleShot()
        self._nudge_timer.timeout.connect(self._flush_nudge_pending)            # executem connect()

    # ==========================================================================
    # CÀRREGA DE DADES
    # ==========================================================================
    def _load_medical_data(self):                                               # definim la funció _load_medical_data()
        """
        Carrega el volum CT i la medul·la STL.

        La medul·la STL es rota 180° al voltant de Z per fer-la coincidir
        visualment amb el volum (convenció del pipeline de segmentació).
        """
        img = sitk.ReadImage(TAC_PATH)                                          # img ← resultat de ReadImage()
        self.vol_original = sitk.GetArrayFromImage(img)                         # self.vol_original ← resultat de GetArrayFromImage()
        self.vol = np.copy(self.vol_original)                                   # self.vol ← resultat de copy()
        self.vol[self.vol < 200] = 0                                            # self.vol[self.vol < 200] = 0
        self.vol_flipped = np.flip(self.vol, axis=0)                            # self.vol_flipped ← resultat de flip()

        # Versió sense filtrar per al mapa de porositat òssia
        # (necessita els HU reals sense el llindar a 200)
        self.vol_orig_flipped = np.flip(self.vol_original, axis=0)              # self.vol_orig_flipped ← resultat de flip()

        self.spacing = img.GetSpacing()                                         # self.spacing ← resultat de GetSpacing()
        self.origin  = img.GetOrigin()                                          # self.origin ← resultat de GetOrigin()
        self.Z, self.Y, self.X = self.vol.shape                                 # assignem self.Z, self.Y, self.X

        # Medul·la: es rota exactament com al codi antic (funciona visualment)
        self.medula = pv.read(MEDULA_PATH).clean().triangulate()                # self.medula ← resultat de read()
        self.medula.rotate_z(180, inplace=True)                                 # assignem self.medula.rotate_z(180, inplace
        self.med_cx, self.med_cy, self.med_cz = self.medula.center              # assignem self.med_cx, self.med_cy, self.med_cz

        # Versió decimada per fer slices ràpides a les vistes 2D
        try:                                                                    # intentem el bloc següent
            reduced = self.medula.decimate(0.98)                                # reduced ← resultat de decimate()
            if reduced.n_points > 50:                                           # si reduced.n_points > 50
                self._medula_fast = reduced                                     # assignem self._medula_fast
                print(f"[medula] decimada per slices: "
                      f"{self.medula.n_points} → {reduced.n_points} punts")
            else:                                                               # en cas contrari
                self._medula_fast = self.medula                                 # assignem self._medula_fast
        except Exception as e:                                                  # capturem excepció: Exception as e
            self._medula_fast = self.medula                                     # assignem self._medula_fast
            print(f"[medula] error decimant: {e}")

        self._med_center = self._medula_fast.center                             # assignem self._med_center

    # ==========================================================================
    # SLICES DE MEDUL·LA
    # ==========================================================================
    def _compute_medula_slice(self, axis, idx):                                 # definim la funció _compute_medula_slice()
        if idx in self._medula_cache[axis]:                                     # si idx in self._medula_cache[axis]
            return self._medula_cache[axis][idx]                                # retornem: self._medula_cache[axis][idx]

        mesh = self._medula_fast                                                # assignem mesh
        cx, cy, cz = self._med_center                                           # assignem cx, cy, cz

        try:                                                                    # intentem el bloc següent
            if axis == "ax":                                                    # si axis == "ax"
                z_world = self.origin[2] + idx * self.spacing[2]                # assignem z_world
                sec = mesh.slice(normal=(0, 0, 1), origin=(cx, cy, z_world))    # sec ← resultat de slice()
                result = self._segments_from_slice_ax(sec)                      # result ← resultat de _segments_from_slice_ax()
                if result is not None:                                          # si result is not None
                    self._medula_cache["ax"][idx] = result                      # self._medula_cache["ax"][idx] = result
                    return result                                               # retornem: result

            elif axis == "sag":                                                 # altrament si axis == "sag"
                x_world = self.origin[0] + idx * self.spacing[0]                # assignem x_world
                sec = mesh.slice(normal=(1, 0, 0), origin=(x_world, cy, cz))    # sec ← resultat de slice()
                result = self._segments_from_slice_sag(sec)                     # result ← resultat de _segments_from_slice_sag()
                if result is not None:                                          # si result is not None
                    self._medula_cache["sag"][idx] = result                     # self._medula_cache["sag"][idx] = result
                    return result                                               # retornem: result

            elif axis == "cor":                                                 # altrament si axis == "cor"
                y_world = self.origin[1] + idx * self.spacing[1]                # assignem y_world
                sec = mesh.slice(normal=(0, 1, 0), origin=(cx, y_world, cz))    # sec ← resultat de slice()
                result = self._segments_from_slice_cor(sec)                     # result ← resultat de _segments_from_slice_cor()
                if result is not None:                                          # si result is not None
                    self._medula_cache["cor"][idx] = result                     # self._medula_cache["cor"][idx] = result
                    return result                                               # retornem: result
        except Exception:                                                       # capturem excepció: Exception
            pass                                                                # no fem res (placeholder)

        self._medula_cache[axis][idx] = None                                    # assignem self._medula_cache[axis][idx]
        return None                                                             # retornem: None

    def _segments_from_slice_ax(self, sec):                                     # definim la funció _segments_from_slice_ax()
        if sec.n_points == 0 or sec.lines is None or len(sec.lines) == 0:       # si sec.n_points == 0 or sec.lines is Non...
            return None                                                         # retornem: None
        pts = sec.points                                                        # assignem pts
        xs_all = (pts[:, 0] - self.origin[0]) / self.spacing[0]                 # assignem xs_all
        ys_all = (pts[:, 1] - self.origin[1]) / self.spacing[1]                 # assignem ys_all
        return self._pairs_from_lines(sec.lines, xs_all, ys_all, self.X, self.Y)  # retornem: self._pairs_from_lines(sec.lines, xs_...

    def _segments_from_slice_sag(self, sec):                                    # definim la funció _segments_from_slice_sag()
        if sec.n_points == 0 or sec.lines is None or len(sec.lines) == 0:       # si sec.n_points == 0 or sec.lines is Non...
            return None                                                         # retornem: None
        pts = sec.points                                                        # assignem pts
        ys_all = (pts[:, 1] - self.origin[1]) / self.spacing[1]                 # assignem ys_all
        zs_all = (pts[:, 2] - self.origin[2]) / self.spacing[2]                 # assignem zs_all
        vs = self.Z - 1 - zs_all                                                # assignem vs
        return self._pairs_from_lines(sec.lines, ys_all, vs, self.Y, self.Z)    # retornem: self._pairs_from_lines(sec.lines, ys_...

    def _segments_from_slice_cor(self, sec):                                    # definim la funció _segments_from_slice_cor()
        if sec.n_points == 0 or sec.lines is None or len(sec.lines) == 0:       # si sec.n_points == 0 or sec.lines is Non...
            return None                                                         # retornem: None
        pts = sec.points                                                        # assignem pts
        xs_all = (pts[:, 0] - self.origin[0]) / self.spacing[0]                 # assignem xs_all
        zs_all = (pts[:, 2] - self.origin[2]) / self.spacing[2]                 # assignem zs_all
        vs = self.Z - 1 - zs_all                                                # assignem vs
        return self._pairs_from_lines(sec.lines, xs_all, vs, self.X, self.Z)    # retornem: self._pairs_from_lines(sec.lines, xs_...

    @staticmethod                                                               # decorador staticmethod
    def _pairs_from_lines(lines, xs_all, ys_all, max_x, max_y):                 # definim la funció _pairs_from_lines()
        try:                                                                    # intentem el bloc següent
            lines = np.asarray(lines, dtype=np.int64)                           # lines ← resultat de asarray()
        except Exception:                                                       # capturem excepció: Exception
            return None                                                         # retornem: None

        xs_out = []                                                             # assignem xs_out
        ys_out = []                                                             # assignem ys_out

        i = 0                                                                   # assignem i
        L = len(lines)                                                          # L ← resultat de len()
        while i < L:                                                            # mentre i < L
            n = int(lines[i]); i += 1                                           # assignem n
            if n < 2 or i + n > L:                                              # si n < 2 or i + n > L
                i += n                                                          # sumem a i
                continue                                                        # saltem a la següent iteració
            idxs = lines[i:i+n]; i += n                                         # assignem idxs

            for k in range(n - 1):                                              # iterem: k in range(n - 1)
                a, b = int(idxs[k]), int(idxs[k+1])                             # a, b ← resultat de int()
                ax, ay = xs_all[a], ys_all[a]                                   # assignem ax, ay
                bx, by = xs_all[b], ys_all[b]                                   # assignem bx, by
                if (0 <= ax < max_x and 0 <= ay < max_y and                     # si (0 <= ax < max_x and 0 <= ay < max_y and
                    0 <= bx < max_x and 0 <= by < max_y):
                    xs_out.append(ax); xs_out.append(bx)                        # executem append()
                    ys_out.append(ay); ys_out.append(by)                        # executem append()

        if not xs_out:                                                          # si not xs_out
            return None                                                         # retornem: None
        return (np.asarray(xs_out, dtype=np.float32),                           # retornem: (np.asarray(xs_out, dtype=np.float32),
                np.asarray(ys_out, dtype=np.float32))

    def _flush_medula_pending(self):                                            # definim la funció _flush_medula_pending()
        if not self.show_medula:                                                # si not self.show_medula
            self._medula_pending_axes.clear()                                   # executem clear()
            return                                                              # retornem sense valor
        for axis in list(self._medula_pending_axes):                            # iterem: axis in list(self._medula_pending_axes)
            view = {"ax": self.view_ax, "sag": self.view_sag,                   # assignem view
                    "cor": self.view_cor}[axis]
            idx = {"ax": self.iz, "sag": self.ix, "cor": self.iy}[axis]         # assignem idx
            result = self._compute_medula_slice(axis, idx)                      # result ← resultat de _compute_medula_slice()
            if result is None:                                                  # si result is None
                view.clear_medula()                                             # executem clear_medula()
            else:                                                               # en cas contrari
                view.set_medula(result[0], result[1])                           # executem set_medula()
        self._medula_pending_axes.clear()                                       # executem clear()

    # ==========================================================================
    # UI
    # ==========================================================================
    def _build_ui(self):                                                        # definim la funció _build_ui()
        main_layout = QtWidgets.QHBoxLayout(self)                               # main_layout ← resultat de QHBoxLayout()
        main_layout.setContentsMargins(12, 12, 12, 12)                          # executem setContentsMargins()
        main_layout.setSpacing(12)                                              # executem setSpacing()

        main_layout.addWidget(self._build_left_panel())                         # executem addWidget()
        main_layout.addWidget(self._build_viewports(), 1)                       # executem addWidget()
        main_layout.addWidget(self._build_right_panel())                        # executem addWidget()

    def _build_left_panel(self):                                                # definim la funció _build_left_panel()
        panel = QtWidgets.QFrame()                                              # panel ← resultat de QFrame()
        panel.setObjectName("Panel")                                            # executem setObjectName()
        panel.setFixedWidth(260)                                                # executem setFixedWidth()
        v = QtWidgets.QVBoxLayout(panel)                                        # v ← resultat de QVBoxLayout()
        v.setContentsMargins(16, 16, 16, 16)                                    # executem setContentsMargins()
        v.setSpacing(12)                                                        # executem setSpacing()
        v.setAlignment(Qt.AlignTop)                                             # executem setAlignment()

        t1 = QtWidgets.QLabel("PLANIFICACIÓ DE CARGOLS")                        # t1 ← resultat de QLabel()
        t1.setObjectName("SectionTitle")                                        # executem setObjectName()
        v.addWidget(t1)                                                         # executem addWidget()

        # --- Botó "Crear trajectòria" (amb feedback visual clar) ---
        self.btn_new = QtWidgets.QPushButton("Crear trajectòria manual")        # self.btn_new ← resultat de QPushButton()
        self.btn_new.setCheckable(True)                                         # executem setCheckable()
        self._btn_new_style_idle = """
            QPushButton {
                background-color: #2c2c2e;
                color: white;
                border: 1px solid #3a3a3c;
                border-radius: 8px;
                padding: 10px 14px;
                font-weight: 600;
            }
            QPushButton:hover { background-color: #3a3a3c; }
        """
        self._btn_new_style_active = """
            QPushButton {
                background-color: #ffffff;
                color: #000000;
                border: 2px solid #ffffff;
                border-radius: 8px;
                padding: 9px 13px;
                font-weight: 700;
            }
        """
        self.btn_new.setStyleSheet(self._btn_new_style_idle)                    # executem setStyleSheet()
        v.addWidget(self.btn_new)                                               # executem addWidget()

        diam_row = QtWidgets.QHBoxLayout()                                      # diam_row ← resultat de QHBoxLayout()
        diam_row.addWidget(QtWidgets.QLabel("Diàmetre:"))                       # executem addWidget()
        self.combo_diam = QtWidgets.QComboBox()                                 # self.combo_diam ← resultat de QComboBox()
        for d in DIAMETRES_STD:                                                 # iterem: d in DIAMETRES_STD
            self.combo_diam.addItem(f"{d} mm")
        self.combo_diam.setCurrentIndex(1)                                      # executem setCurrentIndex()
        diam_row.addWidget(self.combo_diam, 1)                                  # executem addWidget()
        v.addLayout(diam_row)                                                   # executem addLayout()

        t2 = QtWidgets.QLabel("VISUALITZACIÓ")                                  # t2 ← resultat de QLabel()
        t2.setObjectName("SectionTitle")                                        # executem setObjectName()
        t2.setStyleSheet(t2.styleSheet() + "margin-top: 8px;")                  # executem setStyleSheet()
        v.addWidget(t2)                                                         # executem addWidget()

        self.btn_load_3d = QtWidgets.QPushButton("Generar model 3D")            # self.btn_load_3d ← resultat de QPushButton()
        self.btn_medula  = QtWidgets.QPushButton("Mostrar canal medul·lar")     # self.btn_medula ← resultat de QPushButton()
        self.btn_medula.setCheckable(True)                                      # executem setCheckable()
        self.btn_xray    = QtWidgets.QPushButton("Vista densitat òssia (X-ray)")  # self.btn_xray ← resultat de QPushButton()
        self.btn_xray.setCheckable(True)                                        # executem setCheckable()
        self.btn_xray.setEnabled(False)                                         # executem setEnabled()
        self.btn_porosity = QtWidgets.QPushButton("Mapa de porositat òssia")    # self.btn_porosity ← resultat de QPushButton()
        self.btn_porosity.setCheckable(True)                                    # executem setCheckable()
        v.addWidget(self.btn_load_3d)                                           # executem addWidget()
        v.addWidget(self.btn_medula)                                            # executem addWidget()
        v.addWidget(self.btn_xray)                                              # executem addWidget()
        v.addWidget(self.btn_porosity)                                          # executem addWidget()

        t3 = QtWidgets.QLabel("NAVEGACIÓ RÀPIDA")                               # t3 ← resultat de QLabel()
        t3.setObjectName("SectionTitle")                                        # executem setObjectName()
        t3.setStyleSheet(t3.styleSheet() + "margin-top: 8px;")                  # executem setStyleSheet()
        v.addWidget(t3)                                                         # executem addWidget()

        for label, attr, maxval in [                                            # iterem: label, attr, maxval in [
            ("Axial (Z)",   "spin_iz", self.Z),
            ("Sagital (X)", "spin_ix", self.X),
            ("Coronal (Y)", "spin_iy", self.Y),
        ]:
            row = QtWidgets.QHBoxLayout()                                       # row ← resultat de QHBoxLayout()
            lbl = QtWidgets.QLabel(label)                                       # lbl ← resultat de QLabel()
            lbl.setFixedWidth(90)                                               # executem setFixedWidth()
            row.addWidget(lbl)                                                  # executem addWidget()
            spin = QtWidgets.QSpinBox()                                         # spin ← resultat de QSpinBox()
            spin.setRange(1, maxval)                                            # executem setRange()
            spin.setValue(1)                                                    # executem setValue()
            spin.setSingleStep(1)                                               # executem setSingleStep()
            spin.setKeyboardTracking(False)                                     # executem setKeyboardTracking()
            row.addWidget(spin, 1)                                              # executem addWidget()
            v.addLayout(row)                                                    # executem addLayout()
            setattr(self, attr, spin)                                           # executem setattr()

        self.spin_iz.valueChanged.connect(self._on_spin_iz)                     # executem connect()
        self.spin_ix.valueChanged.connect(self._on_spin_ix)                     # executem connect()
        self.spin_iy.valueChanged.connect(self._on_spin_iy)                     # executem connect()

        t4 = QtWidgets.QLabel("AJUST FI DEL TRAJECTE")                          # t4 ← resultat de QLabel()
        t4.setObjectName("SectionTitle")                                        # executem setObjectName()
        t4.setStyleSheet(t4.styleSheet() + "margin-top: 8px;")                  # executem setStyleSheet()
        v.addWidget(t4)                                                         # executem addWidget()

        mode_row = QtWidgets.QHBoxLayout()                                      # mode_row ← resultat de QHBoxLayout()
        self.btn_p0 = QtWidgets.QPushButton("Base (P0)")                        # self.btn_p0 ← resultat de QPushButton()
        self.btn_p0.setCheckable(True); self.btn_p0.setChecked(True)            # executem setCheckable()
        self.btn_p1 = QtWidgets.QPushButton("Punta (P1)")                       # self.btn_p1 ← resultat de QPushButton()
        self.btn_p1.setCheckable(True)                                          # executem setCheckable()
        point_group = QtWidgets.QButtonGroup(self)                              # point_group ← resultat de QButtonGroup()
        point_group.addButton(self.btn_p0)                                      # executem addButton()
        point_group.addButton(self.btn_p1)                                      # executem addButton()
        mode_row.addWidget(self.btn_p0)                                         # executem addWidget()
        mode_row.addWidget(self.btn_p1)                                         # executem addWidget()
        v.addLayout(mode_row)                                                   # executem addLayout()

        dpad = QtWidgets.QGridLayout()                                          # dpad ← resultat de QGridLayout()
        dpad.setSpacing(4)                                                      # executem setSpacing()
        self.btn_up   = QtWidgets.QPushButton("↑")                              # self.btn_up ← resultat de QPushButton()
        self.btn_dn   = QtWidgets.QPushButton("↓")                              # self.btn_dn ← resultat de QPushButton()
        self.btn_lt   = QtWidgets.QPushButton("←")                              # self.btn_lt ← resultat de QPushButton()
        self.btn_rt   = QtWidgets.QPushButton("→")                              # self.btn_rt ← resultat de QPushButton()
        self.btn_z_up = QtWidgets.QPushButton("+Z")                             # self.btn_z_up ← resultat de QPushButton()
        self.btn_z_dn = QtWidgets.QPushButton("-Z")                             # self.btn_z_dn ← resultat de QPushButton()
        for b in (self.btn_up, self.btn_dn, self.btn_lt, self.btn_rt,           # iterem: b in (self.btn_up, self.btn_dn, self....
                  self.btn_z_up, self.btn_z_dn):
            b.setFixedHeight(32)                                                # executem setFixedHeight()
        dpad.addWidget(self.btn_up, 0, 1)                                       # executem addWidget()
        dpad.addWidget(self.btn_lt, 1, 0)                                       # executem addWidget()
        dpad.addWidget(self.btn_rt, 1, 2)                                       # executem addWidget()
        dpad.addWidget(self.btn_dn, 2, 1)                                       # executem addWidget()
        dpad.addWidget(self.btn_z_up, 0, 3)                                     # executem addWidget()
        dpad.addWidget(self.btn_z_dn, 2, 3)                                     # executem addWidget()
        v.addLayout(dpad)                                                       # executem addLayout()

        v.addStretch()                                                          # executem addStretch()

        # Connexions
        self.btn_load_3d.clicked.connect(self._generate_3d)                     # executem connect()
        self.btn_new.clicked.connect(self._new_cargol)                          # executem connect()
        self.btn_medula.clicked.connect(self._toggle_medula)                    # executem connect()
        self.btn_xray.clicked.connect(self._toggle_xray)                        # executem connect()
        self.btn_porosity.clicked.connect(self._toggle_porosity)                # executem connect()
        self.combo_diam.currentIndexChanged.connect(self._change_diameter)      # executem connect()
        self.btn_p0.clicked.connect(lambda: self._set_edit("p0"))               # executem connect()
        self.btn_p1.clicked.connect(lambda: self._set_edit("p1"))               # executem connect()
        self.btn_up.clicked.connect(lambda: self._nudge(0, -1, 0))              # executem connect()
        self.btn_dn.clicked.connect(lambda: self._nudge(0,  1, 0))              # executem connect()
        self.btn_lt.clicked.connect(lambda: self._nudge(-1, 0, 0))              # executem connect()
        self.btn_rt.clicked.connect(lambda: self._nudge( 1, 0, 0))              # executem connect()
        self.btn_z_up.clicked.connect(lambda: self._nudge(0, 0,  1))            # executem connect()
        self.btn_z_dn.clicked.connect(lambda: self._nudge(0, 0, -1))            # executem connect()

        return panel                                                            # retornem: panel

    def _build_viewports(self):                                                 # definim la funció _build_viewports()
        container = QtWidgets.QWidget()                                         # container ← resultat de QWidget()
        grid = QtWidgets.QGridLayout(container)                                 # grid ← resultat de QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)                                     # executem setContentsMargins()
        grid.setSpacing(12)                                                     # executem setSpacing()

        self.view_ax  = CTViewport("ax",  invert_y=True)                        # self.view_ax ← resultat de CTViewport()
        self.view_sag = CTViewport("sag", invert_y=True)                        # self.view_sag ← resultat de CTViewport()
        self.view_cor = CTViewport("cor", invert_y=True)                        # self.view_cor ← resultat de CTViewport()

        self.view_ax .wheelScrolled.connect(lambda d: self._on_scroll("ax",  d))  # self.view_ax .wheelScrolled.connect(lambda d: self._on_sc...
        self.view_sag.wheelScrolled.connect(lambda d: self._on_scroll("sag", d))  # executem connect()
        self.view_cor.wheelScrolled.connect(lambda d: self._on_scroll("cor", d))  # executem connect()

        self.view_ax .clickedInView.connect(lambda x, y, b: self._on_click("ax",  x, y, b))  # self.view_ax .clickedInView.connect(lambda x, y, b: self....
        self.view_sag.clickedInView.connect(lambda x, y, b: self._on_click("sag", x, y, b))  # executem connect()
        self.view_cor.clickedInView.connect(lambda x, y, b: self._on_click("cor", x, y, b))  # executem connect()

        self.view_ax.mouseMovedInView.connect(self._on_preview_motion)          # executem connect()

        self.plotter = QtInteractor(container)                                  # self.plotter ← resultat de QtInteractor()
        self.plotter.set_background(C_BG_WINDOW)                                # executem set_background()

        grid.addWidget(self._wrap_view(self.view_ax,  "AXIAL"),   0, 0)         # executem addWidget()
        grid.addWidget(self._wrap_view(self.view_sag, "SAGITAL"), 0, 1)         # executem addWidget()
        grid.addWidget(self._wrap_view(self.view_cor, "CORONAL"), 1, 0)         # executem addWidget()
        grid.addWidget(self._wrap_view(self.plotter,  "RECONSTRUCCIÓ 3D",       # executem addWidget()
                                        is_plotter=True), 1, 1)

        return container                                                        # retornem: container

    def _wrap_view(self, widget, title, is_plotter=False):                      # definim la funció _wrap_view()
        w = QtWidgets.QFrame()                                                  # w ← resultat de QFrame()
        w.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_WINDOW};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)
        v = QtWidgets.QVBoxLayout(w)                                            # v ← resultat de QVBoxLayout()
        v.setContentsMargins(10, 8, 10, 10)                                     # executem setContentsMargins()
        v.setSpacing(6)                                                         # executem setSpacing()

        header_layout = QtWidgets.QHBoxLayout()                                 # header_layout ← resultat de QHBoxLayout()
        lbl = QtWidgets.QLabel(title)                                           # lbl ← resultat de QLabel()
        lbl.setStyleSheet(                                                      # executem setStyleSheet()
            f"color: {C_TEXT_DIM}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;"
        )
        header_layout.addWidget(lbl)                                            # executem addWidget()
        header_layout.addStretch()                                              # executem addStretch()

        if not is_plotter:                                                      # si not is_plotter
            widget.header_label = lbl                                           # assignem widget.header_label

        v.addLayout(header_layout)                                              # executem addLayout()
        v.addWidget(widget, 1)                                                  # executem addWidget()
        return w                                                                # retornem: w

    def _build_right_panel(self):                                               # definim la funció _build_right_panel()
        panel = QtWidgets.QFrame()                                              # panel ← resultat de QFrame()
        panel.setObjectName("Panel")                                            # executem setObjectName()
        panel.setFixedWidth(280)                                                # executem setFixedWidth()
        v = QtWidgets.QVBoxLayout(panel)                                        # v ← resultat de QVBoxLayout()
        v.setContentsMargins(16, 16, 16, 16)                                    # executem setContentsMargins()
        v.setSpacing(12)                                                        # executem setSpacing()

        t1 = QtWidgets.QLabel("ANÀLISI GEOMÈTRIC")                              # t1 ← resultat de QLabel()
        t1.setObjectName("SectionTitle")                                        # executem setObjectName()
        v.addWidget(t1)                                                         # executem addWidget()

        self.lbl_len = self._hud_block("LONGITUD DEL CARGOL", "──", "mm")       # self.lbl_len ← resultat de _hud_block()
        v.addWidget(self.lbl_len["w"])                                          # executem addWidget()

        t2 = QtWidgets.QLabel("CARGOLS PLANIFICATS")                            # t2 ← resultat de QLabel()
        t2.setObjectName("SectionTitle")                                        # executem setObjectName()
        t2.setStyleSheet(t2.styleSheet() + "margin-top: 12px;")                 # executem setStyleSheet()
        v.addWidget(t2)                                                         # executem addWidget()

        self.list_widget = QtWidgets.QListWidget()                              # self.list_widget ← resultat de QListWidget()
        self.list_widget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)  # executem setSelectionMode()
        self.list_widget.itemSelectionChanged.connect(self._select_from_list)   # executem connect()
        v.addWidget(self.list_widget, 1)                                        # executem addWidget()

        # Eliminar el cargol seleccionat (també amb la tecla Supr)
        self.btn_delete = QtWidgets.QPushButton("Eliminar cargol")              # self.btn_delete ← resultat de QPushButton()
        self.btn_delete.setEnabled(False)                                       # executem setEnabled()
        self.btn_delete.setToolTip("Elimina el cargol seleccionat (Supr)")      # executem setToolTip()
        self.btn_delete.setStyleSheet(f"""
            QPushButton {{ color: {C_DANGER}; border: 1px solid rgba(217, 83, 79, 0.45); }}
            QPushButton:hover {{ background-color: rgba(217, 83, 79, 0.15); }}
            QPushButton:disabled {{ color: {C_TEXT_MUTED}; border: 1px solid {C_BORDER}; }}
        """)
        self.btn_delete.clicked.connect(self._delete_cargol)                    # executem connect()
        v.addWidget(self.btn_delete)                                            # executem addWidget()
        sc_del = QtWidgets.QShortcut(QtGui.QKeySequence(QtGui.QKeySequence.Delete), self.list_widget)
        sc_del.activated.connect(self._delete_cargol)                           # executem connect()

        v.addWidget(self._build_anchor_card())                                  # anàlisi d'ancoratge

        return panel                                                            # retornem: panel

    def _build_anchor_card(self):                                               # definim la funció _build_anchor_card()
        """Targeta ANÀLISI D'ANCORATGE (src/planning/screw_analysis.py)."""
        card = QtWidgets.QWidget()                                              # card ← resultat de QWidget()
        card.setStyleSheet("background: transparent;")                          # executem setStyleSheet()
        v = QtWidgets.QVBoxLayout(card)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(0, 6, 0, 0)                                        # executem setContentsMargins()
        v.setSpacing(6)                                                         # executem setSpacing()

        head = QtWidgets.QHBoxLayout()                                          # head ← resultat de QHBoxLayout()
        t = QtWidgets.QLabel("ANÀLISI D'ANCORATGE")                             # t ← resultat de QLabel()
        t.setObjectName("SectionTitle")                                         # executem setObjectName()
        head.addWidget(t, 1)                                                    # executem addWidget()
        self.lbl_verdict = QtWidgets.QLabel("—")                                # self.lbl_verdict ← resultat de QLabel()
        head.addWidget(self.lbl_verdict)                                        # executem addWidget()
        v.addLayout(head)                                                       # executem addLayout()

        self._anchor_rows = {}                                                  # assignem self._anchor_rows
        grid = QtWidgets.QGridLayout()                                          # grid ← resultat de QGridLayout()
        grid.setHorizontalSpacing(8); grid.setVerticalSpacing(3)                # executem setHorizontalSpacing()
        for i, (key, label) in enumerate([("hu", "Densitat d'ancoratge"),       # iterem: files de mètriques
                                          ("breach", "Perforació"),
                                          ("med", "Medul·la"),
                                          ("isth", "Istme · Ø màx"),
                                          ("diam", "Ø recomanat")]):
            lk = QtWidgets.QLabel(label)                                        # lk ← resultat de QLabel()
            lk.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: {SIZE_XS}px; background: transparent;")
            lv = QtWidgets.QLabel("—")                                          # lv ← resultat de QLabel()
            lv.setAlignment(Qt.AlignRight | Qt.AlignVCenter)                    # executem setAlignment()
            lv.setStyleSheet(f"color: {C_TEXT}; font-family: {FONT_MONO}; font-size: {SIZE_XS}px; "
                             f"font-weight: 700; background: transparent;")
            grid.addWidget(lk, i, 0); grid.addWidget(lv, i, 1)                  # executem addWidget()
            self._anchor_rows[key] = lv                                         # assignem self._anchor_rows[key]
        v.addLayout(grid)                                                       # executem addLayout()

        self.density_profile = DensityProfile()                                 # self.density_profile ← resultat de DensityProfile()
        v.addWidget(self.density_profile)                                       # executem addWidget()

        self.lbl_anchor_notes = QtWidgets.QLabel("")                            # self.lbl_anchor_notes ← resultat de QLabel()
        self.lbl_anchor_notes.setWordWrap(True)                                 # executem setWordWrap()
        self.lbl_anchor_notes.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: {SIZE_XS}px; background: transparent;")
        v.addWidget(self.lbl_anchor_notes)                                      # executem addWidget()

        row = QtWidgets.QHBoxLayout(); row.setSpacing(6)                        # row ← resultat de QHBoxLayout()
        self.btn_optimize = QtWidgets.QPushButton("Optimitzar trajectòria")     # self.btn_optimize ← resultat de QPushButton()
        self.btn_optimize.setToolTip("Entrada fixa: prova inclinacions de fins a ±6° i proposa la de més "
                                     "densitat, sense perforar i lluny de la medul·la")
        self.btn_optimize.clicked.connect(self._optimize_selected)              # executem connect()
        self.btn_apply = QtWidgets.QPushButton("Aplicar")                       # self.btn_apply ← resultat de QPushButton()
        self.btn_apply.setToolTip("Aplica la trajectòria proposada al cargol")  # executem setToolTip()
        self.btn_apply.clicked.connect(self._apply_proposal)                    # executem connect()
        row.addWidget(self.btn_optimize, 2); row.addWidget(self.btn_apply, 1)   # executem addWidget()
        v.addLayout(row)                                                        # executem addLayout()

        foot = QtWidgets.QLabel("Llindars orientatius (no validats clínicament)")  # foot ← resultat de QLabel()
        foot.setStyleSheet(f"color: {C_TEXT_MUTED}; font-size: 9px; background: transparent;")
        v.addWidget(foot)                                                       # executem addWidget()

        self._proposal = None                                                   # (idx, p1_mm) proposta pendent
        self._anchor_sampler = None                                             # es creen a la primera anàlisi
        self._anchor_medulla = None                                             # assignem self._anchor_medulla
        self._show_analysis(None)                                               # executem _show_analysis()
        return card                                                             # retornem: card

    def _hud_block(self, title, value, unit):                                   # definim la funció _hud_block()
        w = QtWidgets.QWidget()                                                 # w ← resultat de QWidget()
        w.setStyleSheet("background: transparent;")                             # executem setStyleSheet()
        l = QtWidgets.QVBoxLayout(w)                                            # l ← resultat de QVBoxLayout()
        l.setContentsMargins(0, 4, 0, 4)                                        # executem setContentsMargins()
        l.setSpacing(4)                                                         # executem setSpacing()

        cap = QtWidgets.QLabel(title)                                           # cap ← resultat de QLabel()
        cap.setStyleSheet(                                                      # executem setStyleSheet()
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_XS}px; font-weight: 600; letter-spacing: 0.8px;"
        )
        val = QtWidgets.QLabel(f"{value} {unit}")
        val.setStyleSheet(                                                      # executem setStyleSheet()
            f"color: {C_TEXT}; background: transparent; "
            f"font-size: {SIZE_LG}px; font-weight: 700; font-family: {FONT_MONO};"
        )
        l.addWidget(cap)                                                        # executem addWidget()
        l.addWidget(val)                                                        # executem addWidget()
        return {"w": w, "val": val, "unit": unit}                               # retornem: {"w": w, "val": val, "unit": unit}

    def _set_hud_value(self, block, value, color=None):                         # definim la funció _set_hud_value()
        if isinstance(value, str):                                              # si isinstance(value, str)
            text = f"{value} {block['unit']}"
        else:                                                                   # en cas contrari
            text = f"{value:.2f} {block['unit']}"
        block["val"].setText(text)                                              # block["val"].setText(text)
        if color:                                                               # si color
            block["val"].setStyleSheet(                                         # block["val"].setStyleSheet(
                f"color: {color}; background: transparent; "
                f"font-size: {SIZE_LG}px; font-weight: 700; "
                f"font-family: {FONT_MONO};"
            )

    # ==========================================================================
    # MAPA DE POROSITAT ÒSSIA
    # ==========================================================================
    def _build_porosity_lut(self):                                              # definim la funció _build_porosity_lut()
        """
        Construeix una LUT (Look-Up Table) que mapa valors HU (Hounsfield
        Units) a colors RGBA, ordenats de menys dens (vermell) a més dens
        (verd/blanc).

        Interpretació clínica per planificació de cargols pediculars:
          · HU < 100        Molt porós (moll/trabecular esponjós)   VERMELL
          · HU 100 – 200    Porós (trabecular clar)                  TARONJA
          · HU 200 – 400    Densitat mitjana (trabecular dens)       GROC
          · HU 400 – 700    Dens (cortical fi)                       VERD CLAR
          · HU > 700        Molt dens (cortical)                     VERD FOSC

        El cargol ha de trobar densitats altes per ancorar-se bé.
        Valors baixos són zones de risc (ancoratge feble, risc de loosening).

        Retorna un array (256, 4) uint8 amb l'alpha calibrat perquè:
          - Els valors extrems (vermell, verd) siguin visibles
          - Els aires (HU molt baix, fora del cos) siguin transparents
          - Els valors mitjans tinguin transparència moderada
        """
        # Rang HU que representem: 0 a 1200 (cobreix os trabecular fins cortical)
        # Valors fora d'aquest rang són transparents o saturats
        HU_MIN = 0                                                              # assignem HU_MIN
        HU_MAX = 1200                                                           # assignem HU_MAX

        lut = np.zeros((256, 4), dtype=np.uint8)                                # lut ← resultat de zeros()

        for i in range(256):                                                    # iterem: i in range(256)
            # HU corresponent a aquest bin
            hu = HU_MIN + (HU_MAX - HU_MIN) * i / 255.0                         # assignem hu

            if hu < 100:                                                        # si hu < 100
                # Aire / greix / múscul → transparent
                # (dins del cos podem tenir valors molt baixos que no ens
                #  interessa colorejar perquè no són os)
                r, g, b, a = 0, 0, 0, 0                                         # assignem r, g, b, a
            elif hu < 200:                                                      # altrament si hu < 200
                # Os molt porós → VERMELL (perill, mal ancoratge)
                t = (hu - 100) / 100.0                                          # assignem t
                r, g, b = 255, int(30 + 60*t), 30                               # assignem r, g, b
                a = 170                                                         # assignem a
            elif hu < 400:                                                      # altrament si hu < 400
                # Os porós → TARONJA
                t = (hu - 200) / 200.0                                          # assignem t
                r, g, b = 255, int(90 + 100*t), 20                              # assignem r, g, b
                a = 160                                                         # assignem a
            elif hu < 700:                                                      # altrament si hu < 700
                # Densitat mitjana / bona → GROC → VERD CLAR
                t = (hu - 400) / 300.0                                          # assignem t
                r = int(255 - 100*t)                                            # r ← resultat de int()
                g = int(210 + 30*t)                                             # g ← resultat de int()
                b = int(20 + 30*t)                                              # b ← resultat de int()
                a = 150                                                         # assignem a
            elif hu < 1000:                                                     # altrament si hu < 1000
                # Densitat alta → VERD
                t = (hu - 700) / 300.0                                          # assignem t
                r = int(155 - 100*t)                                            # r ← resultat de int()
                g = int(240 - 20*t)                                             # g ← resultat de int()
                b = int(50 + 40*t)                                              # b ← resultat de int()
                a = 145                                                         # assignem a
            else:                                                               # en cas contrari
                # Densitat molt alta (cortical dens) → VERD FOSC
                r, g, b = 40, 200, 90                                           # assignem r, g, b
                a = 135                                                         # assignem a

            lut[i] = (r, g, b, a)                                               # lut[i] ← resultat de ()

        return lut                                                              # retornem: lut

    def _hu_slice_to_rgba(self, hu_slice):                                      # definim la funció _hu_slice_to_rgba()
        """
        Converteix un slice 2D de valors HU (numpy array) a una imatge
        RGBA (H, W, 4) uint8 fent servir la LUT de porositat.

        Els valors HU es normalitzen al rang [0, 1200] i es discretitzen
        a 256 bins per indexar la LUT.
        """
        if not hasattr(self, "_porosity_lut"):                                  # si not hasattr(self, "_porosity_lut")
            self._porosity_lut = self._build_porosity_lut()                     # self._porosity_lut ← resultat de _build_porosity_lut()

        HU_MIN, HU_MAX = 0, 1200                                                # assignem HU_MIN, HU_MAX
        # Normalitzem a [0, 255] com a índex de la LUT
        clipped = np.clip(hu_slice, HU_MIN, HU_MAX)                             # clipped ← resultat de clip()
        idx = ((clipped - HU_MIN) / (HU_MAX - HU_MIN) * 255.0).astype(np.uint8)  # idx ← resultat de ()
        # Aplicar la LUT
        return self._porosity_lut[idx]                                          # retornem: self._porosity_lut[idx]

    def _update_porosity(self, name):                                           # definim la funció _update_porosity()
        """Actualitza el mapa de porositat en una vista concreta."""            # """Actualitza el mapa de porositat en una vista concreta."""
        view = {"ax": self.view_ax, "sag": self.view_sag,                       # assignem view
                "cor": self.view_cor}[name]

        if not self.show_porosity:                                              # si not self.show_porosity
            view.clear_density()                                                # executem clear_density()
            return                                                              # retornem sense valor

        # Slice del volum ORIGINAL (sense filtrar) per tenir els valors HU reals
        if name == "ax":                                                        # si name == "ax"
            hu_slice = self.vol_original[self.iz]                               # assignem hu_slice
        elif name == "sag":                                                     # altrament si name == "sag"
            hu_slice = self.vol_orig_flipped[:, :, self.ix]                     # assignem hu_slice
        elif name == "cor":                                                     # altrament si name == "cor"
            hu_slice = self.vol_orig_flipped[:, self.iy, :]                     # assignem hu_slice
        else:                                                                   # en cas contrari
            return                                                              # retornem sense valor

        rgba = self._hu_slice_to_rgba(hu_slice)                                 # rgba ← resultat de _hu_slice_to_rgba()
        view.set_density(rgba)                                                  # executem set_density()

    def _toggle_porosity(self):                                                 # definim la funció _toggle_porosity()
        """Activa o desactiva el mapa de porositat òssia."""                    # """Activa o desactiva el mapa de porositat òssia."""
        self.show_porosity = self.btn_porosity.isChecked()                      # self.show_porosity ← resultat de isChecked()

        if self.show_porosity:                                                  # si self.show_porosity
            self.btn_porosity.setStyleSheet(                                    # executem setStyleSheet()
                f"background-color: {C_WARN}; color: black; font-weight: bold;")
        else:
            self.btn_porosity.setStyleSheet("")

        for name in ("ax", "sag", "cor"):
            self._update_porosity(name)

    # ==========================================================================
    # ACTUALITZACIÓ DE VISTES
    # ==========================================================================
    def _update_view(self, name):
        if name == "ax":
            self.view_ax.set_image(self.vol[self.iz])
            self.view_ax.set_crosshair(self.ix, self.iy)
            self.view_ax.header_label.setText(f"AXIAL   ·   TALL {self.iz+1} / {self.Z}")
            self._update_medula("ax")
            self._update_porosity("ax")

        elif name == "sag":
            self.view_sag.set_image(self.vol_flipped[:, :, self.ix])
            self.view_sag.set_crosshair(self.iy, self.Z - 1 - self.iz)
            self.view_sag.header_label.setText(f"SAGITAL   ·   TALL {self.ix+1} / {self.X}")
            self._update_medula("sag")
            self._update_porosity("sag")

        elif name == "cor":
            self.view_cor.set_image(self.vol_flipped[:, self.iy, :])
            self.view_cor.set_crosshair(self.ix, self.Z - 1 - self.iz)
            self.view_cor.header_label.setText(f"CORONAL   ·   TALL {self.iy+1} / {self.Y}")
            self._update_medula("cor")
            self._update_porosity("cor")

    def _update_medula(self, name):
        view = {"ax": self.view_ax, "sag": self.view_sag,
                "cor": self.view_cor}[name]

        if not self.show_medula:
            view.clear_medula()
            return

        idx = {"ax": self.iz, "sag": self.ix, "cor": self.iy}[name]

        if idx in self._medula_cache[name]:
            cached = self._medula_cache[name][idx]
            if cached is None:
                view.clear_medula()
            else:
                view.set_medula(cached[0], cached[1])
            return

        view.clear_medula()
        self._medula_pending_axes.add(name)
        self._medula_timer.start(150)

    def _refresh_screws_all_views(self):
        self.view_ax .clear_screws()
        self.view_sag.clear_screws()
        self.view_cor.clear_screws()
        for i, cg in enumerate(self.cargols):
            self._draw_screw(i, cg)

    # ==========================================================================
    # DIBUIX DE CARGOLS (idèntic al codi antic)
    # ==========================================================================
    def _draw_screw(self, idx, cg):
        p0 = cg["p0"]
        p1 = cg["p1"]
        color = cg["color"]
        diam = cg.get("diam", 5.5)
        selected = (idx == self.selected_idx)
        radi_mm = diam / 2.0

        p0_mm = np.array([
            p0[0] * self.spacing[0],
            p0[1] * self.spacing[1],
            cg["z_p0"] * self.spacing[2],
        ])
        p1_mm = np.array([
            p1[0] * self.spacing[0],
            p1[1] * self.spacing[1],
            cg["z_p1"] * self.spacing[2],
        ])
        axis = p1_mm - p0_mm
        length_mm = np.linalg.norm(axis)
        axis_unit = axis / length_mm if length_mm >= 1e-6 else np.array([1.,0.,0.])

        # ---- AXIAL ----
        self._draw_screw_in_plane(idx, cg, color, selected, radi_mm,
            plane_normal=np.array([0., 0., 1.]),
            plane_value=self.iz * self.spacing[2],
            axis_unit=axis_unit, p0_mm=p0_mm, p1_mm=p1_mm,
            view=self.view_ax, view_axes=("x", "y"))

        # ---- SAGITAL ----
        self._draw_screw_in_plane(idx, cg, color, selected, radi_mm,
            plane_normal=np.array([1., 0., 0.]),
            plane_value=self.ix * self.spacing[0],
            axis_unit=axis_unit, p0_mm=p0_mm, p1_mm=p1_mm,
            view=self.view_sag, view_axes=("y", "z_flip"))

        # ---- CORONAL ----
        self._draw_screw_in_plane(idx, cg, color, selected, radi_mm,
            plane_normal=np.array([0., 1., 0.]),
            plane_value=self.iy * self.spacing[1],
            axis_unit=axis_unit, p0_mm=p0_mm, p1_mm=p1_mm,
            view=self.view_cor, view_axes=("x", "z_flip"))

    def _draw_screw_in_plane(self, idx, cg, color, selected, radi_mm,
                              plane_normal, plane_value, axis_unit,
                              p0_mm, p1_mm, view, view_axes):
        cos_a = float(abs(np.dot(axis_unit, plane_normal)))
        if cos_a < 0.15:
            self._draw_screw_rect_in_view(idx, cg, color, selected, radi_mm,
                p0_mm, p1_mm, view, view_axes, plane_normal, plane_value)
        else:
            self._draw_screw_ellipse_in_view(idx, cg, color, selected, radi_mm,
                axis_unit, p0_mm, p1_mm, plane_normal, plane_value,
                view, view_axes, cos_a)

    def _map_3d_to_view(self, pt_mm, view_axes):
        def _one(coord_name, pt_mm):
            if coord_name == "x":
                return pt_mm[0] / self.spacing[0]
            elif coord_name == "y":
                return pt_mm[1] / self.spacing[1]
            elif coord_name == "z_flip":
                z_px = pt_mm[2] / self.spacing[2]
                return self.Z - 1 - z_px
            return 0.0
        return (_one(view_axes[0], pt_mm), _one(view_axes[1], pt_mm))

    def _draw_screw_rect_in_view(self, idx, cg, color, selected, radi_mm,
                                   p0_mm, p1_mm, view, view_axes,
                                   plane_normal, plane_value):
        """
        Dibuixa el cargol com un rectangle a la vista quan el pla és
        (aproximadament) paral·lel a l'eix del cargol.

        AMPLADA GEOMÈTRICAMENT CORRECTA:
          - Quan el pla talla el cilindre pel diàmetre, el rectangle té
            amplada 2*radi (com abans, era l'únic cas cobert).
          - Quan el pla és tangent a un costat del cilindre, l'amplada
            visible és sqrt(radi² - d_perp²) * 2, on d_perp és la
            distància perpendicular entre el pla i l'eix del cargol.
          - Quan el pla està a més de radi de l'eix, no hi ha intersecció
            (el rectangle no es dibuixa).
        """
        # Distància perpendicular entre el pla (que passa per plane_value
        # en la direcció plane_normal) i l'eix del cilindre.
        # Prenem un punt qualsevol de l'eix (p0_mm) i calculem la seva
        # distància al pla, projectant sobre la normal.
        axis_dir = p1_mm - p0_mm
        axis_len = float(np.linalg.norm(axis_dir))
        if axis_len < 1e-6:
            view.remove_screw(idx); return
        axis_unit = axis_dir / axis_len

        # Component perpendicular de la normal del pla respecte a l'eix
        # (això és el que "compta" per la distància perpendicular pla ↔ eix)
        n_perp = plane_normal - float(np.dot(plane_normal, axis_unit)) * axis_unit
        n_perp_len = float(np.linalg.norm(n_perp))

        if n_perp_len < 1e-6:
            # Pla perpendicular a l'eix; no hauríem d'entrar aquí (aniria a
            # el·lipse), però per si de cas: retallem al rang complet.
            d_perp = 0.0
        else:
            # Distància signada del punt p0_mm al pla, mesurada al llarg
            # de plane_normal, dividida per |n_perp| ens dona la distància
            # perpendicular pla ↔ eix (des del punt d'una perpendicular real).
            # Equivalent: | (plane_value - normal·p0) / |n_perp| |
            d_perp = abs(
                (plane_value - float(np.dot(plane_normal, p0_mm))) / n_perp_len
            )

        if d_perp >= radi_mm:
            # El pla no talla el cilindre — no dibuixem res
            view.remove_screw(idx); return

        # Semi-amplada visible del cargol en aquesta secció (mm)
        half_width_mm = math.sqrt(radi_mm * radi_mm - d_perp * d_perp)

        # Projectem el cargol SENCER al pla de la vista (no retallem als
        # extrems). Així el cap i la punta apareixen sempre al mateix lloc
        # respecte a la vertebra i les rosques mantenen la seva posició
        # mentre navegues per talls veïns. L'ample visible ja s'ha reduït
        # segons half_width_mm si el pla està a la vora del cilindre.
        p0_view = self._map_3d_to_view(p0_mm, view_axes)
        p1_view = self._map_3d_to_view(p1_mm, view_axes)

        # Conversió mm → píxels (per la dimensió lateral del cargol usem
        # l'spacing horitzontal de la vista)
        if view_axes[0] == "x":   px_per_mm = 1.0 / self.spacing[0]
        elif view_axes[0] == "y": px_per_mm = 1.0 / self.spacing[1]
        else:                     px_per_mm = 1.0 / self.spacing[2]

        # ---- Geometria realista del cargol pedicular ----
        # Un cargol pedicular típic (Medtronic Solera, Stryker Xia) té:
        #   · Cap poliaxial d'uns 1.6 - 1.8 × el diàmetre del cos
        #   · Cap ocupa 12-15% de la longitud total (el "tulip")
        #   · Punta cònica de 8-12% de la longitud (rosca autotarraudant)
        #   · Rosca al llarg d'aproximadament tot el cos
        head_diam_mm = radi_mm * 2.0 * 1.65     # cap 65% més ample que el cos
        hw_head_full_px = (head_diam_mm / 2.0) * px_per_mm

        # AMPLADA DEL COS AL PLA: no és el radi total del cargol, sinó la
        # semi-amplada visible en aquesta secció (calculada dalt segons d_perp).
        # Això fa que quan el pla passa per la vora del cilindre, el cargol
        # es vegi ESTRET, com correspon a la geometria d'un cilindre tallat.
        hw_body_px = half_width_mm * px_per_mm

        # El cap segueix el mateix ratio de visibilitat que el cos
        visibility_ratio = half_width_mm / radi_mm if radi_mm > 0 else 1.0
        hw_head_px = hw_head_full_px * visibility_ratio

        # Nombre de rosques proporcional a la longitud visible del cos
        body_len_px = math.hypot(p1_view[0] - p0_view[0],
                                   p1_view[1] - p0_view[1])
        # ~1 rosca cada 3 mm (pas de rosca real de cargols pediculars)
        rosca_pitch_mm = 3.0
        rosca_pitch_px = rosca_pitch_mm * px_per_mm
        n_threads = max(3, int(body_len_px / max(rosca_pitch_px, 1.0) * 0.75))
        n_threads = min(n_threads, 30)

        geom = {
            "p0_view": p0_view,           # cap del cargol (real, no retallat)
            "p1_view": p1_view,           # punta (real, no retallada)
            "half_width_head": hw_head_px,
            "half_width_body": hw_body_px,
            "head_frac": 0.14,
            "tip_frac":  0.10,
            "n_threads": n_threads,
        }
        view.set_screw_realistic(idx, geom, color, selected)

    def _draw_screw_ellipse_in_view(self, idx, cg, color, selected, radi_mm,
                                     axis_unit, p0_mm, p1_mm,
                                     plane_normal, plane_value,
                                     view, view_axes, cos_a):
        v = p1_mm - p0_mm
        denom = float(np.dot(plane_normal, v))
        if abs(denom) < 1e-9:
            view.remove_screw(idx); return
        t = (plane_value - float(np.dot(plane_normal, p0_mm))) / denom
        if t < 0.0 or t > 1.0:
            view.remove_screw(idx); return

        centre_mm = p0_mm + t * v
        semi_minor_mm = radi_mm
        semi_major_mm = radi_mm / max(cos_a, 0.01)

        v_proj = v - float(np.dot(v, plane_normal)) * plane_normal
        cx_view, cy_view = self._map_3d_to_view(centre_mm, view_axes)
        end_view = self._map_3d_to_view(centre_mm + v_proj, view_axes)
        dx = end_view[0] - cx_view
        dy = end_view[1] - cy_view
        angle_deg = 0.0 if (abs(dx) < 1e-6 and abs(dy) < 1e-6) \
                        else math.degrees(math.atan2(dy, dx))

        if view_axes[0] == "x":   px_x = self.spacing[0]
        elif view_axes[0] == "y": px_x = self.spacing[1]
        else:                     px_x = self.spacing[2]
        if view_axes[1] == "x":   px_y = self.spacing[0]
        elif view_axes[1] == "y": px_y = self.spacing[1]
        else:                     px_y = self.spacing[2]
        px_avg = (px_x + px_y) / 2.0
        semi_major_px = semi_major_mm / px_avg
        semi_minor_px = semi_minor_mm / px_avg

        view.set_screw_ellipse(idx, cx_view, cy_view,
                                 semi_major_px, semi_minor_px,
                                 angle_deg, color, selected)

    @staticmethod
    def _clip_segment_to_plane_range(p0_mm, p1_mm, normal, val_min, val_max):
        v0 = float(np.dot(normal, p0_mm))
        v1 = float(np.dot(normal, p1_mm))
        dv = v1 - v0
        if abs(dv) < 1e-9:
            if val_min <= v0 <= val_max:
                return (p0_mm, p1_mm)
            return None
        t_enter = (val_min - v0) / dv
        t_exit  = (val_max - v0) / dv
        if t_enter > t_exit:
            t_enter, t_exit = t_exit, t_enter
        t_lo = max(0.0, t_enter)
        t_hi = min(1.0, t_exit)
        if t_lo > t_hi:
            return None
        p_lo = p0_mm + t_lo * (p1_mm - p0_mm)
        p_hi = p0_mm + t_hi * (p1_mm - p0_mm)
        return (p_lo, p_hi)

    @staticmethod
    def _screw_polygon_ax(p0, p1, radius):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return [(p0[0], p0[1])] * 4
        nx, ny = -dy / length, dx / length
        return [
            (p0[0] + nx*radius, p0[1] + ny*radius),
            (p1[0] + nx*radius, p1[1] + ny*radius),
            (p1[0] - nx*radius, p1[1] - ny*radius),
            (p0[0] - nx*radius, p0[1] - ny*radius),
        ]

    # ==========================================================================
    # EVENTS
    # ==========================================================================
    def _on_preview_motion(self, x, y):
        if not self.creating or not self.current:
            return
        if len(self.current["pts"]) != 1:
            return
        p0 = self.current["pts"][0]
        self.view_ax.show_preview_line(p0[0], p0[1], x, y)

    def _on_spin_iz(self, val):
        new_iz = int(np.clip(val - 1, 0, self.Z - 1))
        if new_iz == self.iz: return
        self.iz = new_iz
        self._update_view("ax"); self._update_view("sag"); self._update_view("cor")
        self._refresh_screws_all_views()
        if self.selected_idx is not None:
            self._update_telemetry()

    def _on_spin_ix(self, val):
        new_ix = int(np.clip(val - 1, 0, self.X - 1))
        if new_ix == self.ix: return
        self.ix = new_ix
        self._update_view("sag"); self._update_view("ax")
        self._refresh_screws_all_views()

    def _on_spin_iy(self, val):
        new_iy = int(np.clip(val - 1, 0, self.Y - 1))
        if new_iy == self.iy: return
        self.iy = new_iy
        self._update_view("cor"); self._update_view("ax")
        self._refresh_screws_all_views()

    def _sync_spinboxes(self):
        for spin, val in [(self.spin_iz, self.iz + 1),
                          (self.spin_ix, self.ix + 1),
                          (self.spin_iy, self.iy + 1)]:
            spin.blockSignals(True); spin.setValue(val); spin.blockSignals(False)

    def _on_scroll(self, view_name, delta):
        prev_iz = self.iz
        if view_name == "ax":
            self.iz = int(np.clip(self.iz + delta, 0, self.Z - 1))
            self._update_view("ax"); self._update_view("sag"); self._update_view("cor")
        elif view_name == "sag":
            self.ix = int(np.clip(self.ix + delta, 0, self.X - 1))
            self._update_view("sag"); self._update_view("ax")
        elif view_name == "cor":
            self.iy = int(np.clip(self.iy + delta, 0, self.Y - 1))
            self._update_view("cor"); self._update_view("ax")

        self._refresh_screws_all_views()
        self._sync_spinboxes()

        # Si ha canviat Z i hi ha cargol seleccionat, refresc la distància
        if self.iz != prev_iz and self.selected_idx is not None:
            self._update_telemetry()

    def _on_click(self, view_name, x, y, button):
        if button == 2:
            return

        if self.creating:
            if view_name != "ax": return
            if len(self.current["pts"]) < 2:
                self.current["pts"].append((x, y))
                if len(self.current["pts"]) == 2:
                    self._finish_cargol()
                else:
                    self.view_ax.show_preview_p0(x, y)
            return

        if view_name == "ax":
            self.ix = int(np.clip(x, 0, self.X - 1))
            self.iy = int(np.clip(y, 0, self.Y - 1))
        elif view_name == "sag":
            self.iy = int(np.clip(x, 0, self.Y - 1))
            self.iz = self.Z - 1 - int(np.clip(y, 0, self.Z - 1))
        elif view_name == "cor":
            self.ix = int(np.clip(x, 0, self.X - 1))
            self.iz = self.Z - 1 - int(np.clip(y, 0, self.Z - 1))

        self.list_widget.clearSelection()
        self.selected_idx = None
        self._update_view("ax"); self._update_view("sag"); self._update_view("cor")
        self._refresh_screws_all_views()
        self._sync_spinboxes()

    # ==========================================================================
    # MANDO
    # ==========================================================================
    def on_mando_reading(self, data):
        if not self.show_mando: return

        raw_x, raw_y, raw_z = data["raw_x"], data["raw_y"], data["raw_z"]
        rad_x, rad_y, rad_z = data["rad_x"], data["rad_y"], data["rad_z"]

        if self.primera_lectura:
            self.offset_x = (self.origin[0] + self.ix * self.spacing[0]) - raw_x
            self.offset_y = (self.origin[1] + (self.Y - 1) * self.spacing[1]) - raw_y
            self.offset_z = (self.origin[2] + self.iz * self.spacing[2]) - raw_z
            self.primera_lectura = False

        self.mando_x = raw_x + self.offset_x
        self.mando_y = raw_y + self.offset_y
        self.mando_z = raw_z + self.offset_z

        x1, y1, z1 = 0.0, -60.0*math.sin(rad_x), 60.0*math.cos(rad_x)
        x2 = x1*math.cos(rad_y) + z1*math.sin(rad_y)
        y2 = y1
        z2 = -x1*math.sin(rad_y) + z1*math.cos(rad_y)
        self.dx = x2*math.cos(rad_z) - y2*math.sin(rad_z)
        self.dy = x2*math.sin(rad_z) + y2*math.cos(rad_z)
        self.dz = z2

        px0 = (self.mando_x - self.origin[0]) / self.spacing[0]
        py0 = (self.mando_y - self.origin[1]) / self.spacing[1]
        pz0 = (self.mando_z - self.origin[2]) / self.spacing[2]
        px1 = (self.mando_x + self.dx - self.origin[0]) / self.spacing[0]
        py1 = (self.mando_y + self.dy - self.origin[1]) / self.spacing[1]
        pz1 = (self.mando_z + self.dz - self.origin[2]) / self.spacing[2]

        self.view_ax .set_mando(px0, py0, px1, py1)
        self.view_sag.set_mando(py0, self.Z-1-pz0, py1, self.Z-1-pz1)
        self.view_cor.set_mando(px0, self.Z-1-pz0, px1, self.Z-1-pz1)

        if self.is_3d_loaded and hasattr(self, "line_3d"):
            p0_m = (self.mando_x, self.mando_y, self.mando_z)
            p1_m = (self.mando_x + self.dx,
                    self.mando_y + self.dy,
                    self.mando_z + self.dz)
            self.line_3d.points = np.array([p0_m, p1_m])

        if self.selected_idx is None:
            self._update_telemetry()

    # ==========================================================================
    # CÀLCUL DE DISTÀNCIA CARGOL ↔ MEDUL·LA
    # ==========================================================================
    # ==========================================================================
    # RECONSTRUCCIÓ 3D (idèntica al codi antic — funciona visualment)
    # ==========================================================================
    def _generate_3d(self):
        if self.is_3d_loaded:
            return
        self.btn_load_3d.setText("Calculant…")
        self.btn_load_3d.setStyleSheet(f"background-color: {C_WARN}; color: black;")
        self.btn_load_3d.setEnabled(False)
        QtWidgets.QApplication.processEvents()

        step = 2
        vol3d = self.vol[::step, ::step, ::step]
        spac3d = (self.spacing[0]*step, self.spacing[1]*step, self.spacing[2]*step)
        grid_pd = pv.ImageData(dimensions=vol3d.shape[::-1],
                               spacing=spac3d, origin=self.origin)
        grid_pd.point_data["values"] = vol3d.flatten(order="C")

        # Iso-superfície de l'os. Es reflecteix per quadrar visualment amb la
        # medul·la (que està rotada), igual que al codi antic.
        self.bone_3d = (grid_pd.contour([250])
                              .connectivity("largest")
                              .smooth(n_iter=6))

        # Os opac (aspecte metàl·lic professional). Guardem l'actor per poder
        # canviar-li l'opacitat des del toggle X-ray.
        self.actor_bone_3d = self.plotter.add_mesh(
            self.bone_3d,
            color=(225, 225, 228),
            opacity=1.0,
            smooth_shading=True,
            specular=0.20, specular_power=12,
            ambient=0.32, diffuse=0.75,
            show_edges=False,
            name="bone_3d",
        )
        self._bone_opacity_solid = 1.0
        self._bone_opacity_xray  = 0.25

        # Medul·la 3D reflectida (mateixa lògica que el bone)
        self.medula_3d = self.medula
        self.actor_medula_3d = self.plotter.add_mesh(
            self.medula_3d,
            color=(230, 90, 90),
            opacity=0.75,
            smooth_shading=True,
            ambient=0.30, diffuse=0.70,
            name="medula_3d",
        )
        self.actor_medula_3d.SetVisibility(self.show_medula)

        # Línia del mando (blanca)
        self.line_3d = pv.Line((0,0,0), (1,0,0))
        self.actor_3d = self.plotter.add_mesh(
            self.line_3d, color=(240, 240, 245), line_width=4, name="mando_line")

        self.is_3d_loaded = True
        self.btn_xray.setEnabled(True)

        for i in range(len(self.cargols)):
            self.cargols[i]["actor"] = None
            self._update_3d_screw(i)

        cx, cy, cz = self.bone_3d.center
        self.plotter.camera.position = (cx, cy - 300, cz)
        self.plotter.camera.focal_point = (cx, cy, cz)
        self.plotter.camera.up = (0, 0, 1)
        self.plotter.reset_camera_clipping_range()
        self.plotter.update()

        self.btn_load_3d.setText("Model 3D actiu")
        self.btn_load_3d.setStyleSheet("")

    def _update_3d_screw(self, idx):
        if not self.is_3d_loaded:
            return
        cg = self.cargols[idx]
        if cg.get("actor"):
            try:
                self.plotter.remove_actor(cg["actor"])
            except Exception:
                pass
        p0_mm = self.origin + np.array([cg["p0"][0]*self.spacing[0],
                                        cg["p0"][1]*self.spacing[1],
                                        cg["z_p0"]*self.spacing[2]])
        p1_mm = self.origin + np.array([cg["p1"][0]*self.spacing[0],
                                        cg["p1"][1]*self.spacing[1],
                                        cg["z_p1"]*self.spacing[2]])
        # Reflectim el cargol també, igual que el bone i la medul·la
        tube = pv.Line(p0_mm, p1_mm).tube(radius=cg["diam"]/2.0)
        cg["actor"] = self.plotter.add_mesh(
            tube, color=cg["color"], name=f"cg_{idx}",
            smooth_shading=True, specular=0.4, specular_power=15,
            ambient=0.25, diffuse=0.75,
        )
        self.plotter.update()

    # ==========================================================================
    # TOGGLES DE VISUALITZACIÓ
    # ==========================================================================
    def _toggle_medula(self):
        self.show_medula = self.btn_medula.isChecked()

        if self.is_3d_loaded and hasattr(self, "actor_medula_3d"):
            self.actor_medula_3d.SetVisibility(self.show_medula)
            self.plotter.update()

        if not self.show_medula:
            self._medula_pending_axes.clear()
            self._medula_timer.stop()

        for name, view in [("ax", self.view_ax),
                           ("sag", self.view_sag),
                           ("cor", self.view_cor)]:
            if not self.show_medula:
                view.clear_medula()
            else:
                self._update_medula(name)
            view.repaint()

    def _toggle_xray(self):
        """
        Vista de densitat òssia: baixa l'opacitat de l'os per veure-hi a través.

        Quan un mesh opac passa a translúcid, VTK necessita re-configurar el
        pipeline de rendering (translúcid requereix sort per profunditat).
        Fem els canvis via SetOpacity + Modified() + Render() perquè s'apliqui.
        """
        if not self.is_3d_loaded or not hasattr(self, "actor_bone_3d"):
            return
        xray_on = self.btn_xray.isChecked()
        opacity = self._bone_opacity_xray if xray_on else self._bone_opacity_solid
        try:
            prop = self.actor_bone_3d.GetProperty()
            prop.SetOpacity(opacity)
            prop.Modified()
            self.actor_bone_3d.Modified()

            # Feedback visual al botó
            if xray_on:
                self.btn_xray.setStyleSheet(
                    f"background-color: {C_WARN}; color: black; font-weight: bold;")
            else:
                self.btn_xray.setStyleSheet("")

            # Força un render immediat (no només update, que és massa lax)
            self.plotter.render()
        except Exception as e:
            print(f"[xray] error: {e}")

    # ==========================================================================
    # ACCIONS SOBRE CARGOLS
    # ==========================================================================
    def _change_diameter(self):
        self.current_diam = float(self.combo_diam.currentText().replace(" mm", ""))
        if self.selected_idx is not None:
            self.cargols[self.selected_idx]["diam"] = self.current_diam
            self._update_3d_screw(self.selected_idx)
            self._update_telemetry()
            self._draw_screw(self.selected_idx, self.cargols[self.selected_idx])
            item = self.list_widget.item(self.selected_idx)                     # l'etiqueta mostra el Ø nou
            if item is not None:                                                # si item is not None
                item.setText(self._cargol_label(self.selected_idx, self.cargols[self.selected_idx]))
            self._proposal = None                                               # assignem self._proposal
            self._analyze_selected()                                            # executem _analyze_selected()

    def _new_cargol(self):
        if self.creating:
            self._cancel_creation()
            return
        self.creating = True
        self.selected_idx = None
        self.list_widget.clearSelection()
        color = CARGOL_COLORS[len(self.cargols) % len(CARGOL_COLORS)]
        self.current = {"pts": [], "z": self.iz, "color": color}
        self.btn_new.setChecked(True)
        self.btn_new.setStyleSheet(self._btn_new_style_active)
        self.btn_new.setText("Clica P0 i P1 a l'axial   ·   clica per cancel·lar")

    def _cancel_creation(self):
        self.creating = False
        self.current = None
        self.view_ax.clear_preview()
        self.btn_new.setChecked(False)
        self.btn_new.setStyleSheet(self._btn_new_style_idle)
        self.btn_new.setText("Crear trajectòria manual")

    def _finish_cargol(self):
        if self.creating and self.current and len(self.current["pts"]) == 2:
            z = self.current["z"]
            cg = {
                "p0": self.current["pts"][0],
                "p1": self.current["pts"][1],
                "z": z, "z_p0": z, "z_p1": z,
                "color": self.current["color"],
                "actor": None,
                "diam":  self.current_diam,
            }
            self.cargols.append(cg)
            item = QtWidgets.QListWidgetItem(self._cargol_label(len(self.cargols) - 1, cg))
            item.setForeground(QColor(cg["color"]))
            self.list_widget.addItem(item)
            self._draw_screw(len(self.cargols)-1, cg)
            self._update_3d_screw(len(self.cargols)-1)
            self.cargolsChanged.emit(self.cargols)
        self._cancel_creation()

    # ---- Eliminar --------------------------------------------------------------

    def _cargol_label(self, i, cg):                                             # definim la funció _cargol_label()
        return f"Cargol {i + 1}   ·   Ø {cg['diam']:.1f} mm   ·   Z={cg['z'] + 1}"

    def _delete_cargol(self):                                                   # definim la funció _delete_cargol()
        """Elimina el cargol seleccionat (2D, 3D, llista) i renumera la resta."""
        idx = self.selected_idx                                                 # assignem idx
        if idx is None or not (0 <= idx < len(self.cargols)):                   # cap cargol seleccionat
            return                                                              # retornem sense valor
        ans = QtWidgets.QMessageBox.question(                                   # confirmació
            self, "Eliminar cargol", f"Eliminar el Cargol {idx + 1}?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if ans != QtWidgets.QMessageBox.Yes:                                    # si no ho confirma
            return                                                              # retornem sense valor

        if self.is_3d_loaded:                                                   # 3D: fora tots i es tornen a posar
            for cg in self.cargols:                                             # (els noms van per índex)
                if cg.get("actor") is not None:                                 # si té actor
                    try:                                                        # intentem el bloc següent
                        self.plotter.remove_actor(cg["actor"])                  # executem remove_actor()
                    except Exception:                                           # capturem excepció: Exception
                        pass                                                    # no fem res
                    cg["actor"] = None                                          # assignem cg["actor"]
        del self.cargols[idx]                                                   # executem del
        self.selected_idx = None                                                # assignem self.selected_idx
        self._proposal = None                                                   # assignem self._proposal

        self.list_widget.blockSignals(True)                                     # llista renumerada sense disparar selecció
        self.list_widget.clear()                                                # executem clear()
        for i, cg in enumerate(self.cargols):                                   # iterem: i, cg in enumerate(self.cargols)
            item = QtWidgets.QListWidgetItem(self._cargol_label(i, cg))         # item ← resultat de QListWidgetItem()
            item.setForeground(QColor(cg["color"]))                             # executem setForeground()
            self.list_widget.addItem(item)                                      # executem addItem()
        self.list_widget.blockSignals(False)                                    # executem blockSignals()

        for i in range(len(self.cargols)):                                      # 3D de nou amb els índexs nous
            self._update_3d_screw(i)                                            # executem _update_3d_screw()
        if self.is_3d_loaded:                                                   # si is_3d_loaded
            self.plotter.update()                                               # executem update()
        self._refresh_screws_all_views()                                        # 2D de nou
        self.btn_delete.setEnabled(False)                                       # executem setEnabled()
        self._show_analysis(None)                                               # executem _show_analysis()
        self.cargolsChanged.emit(self.cargols)                                  # executem emit()
        print(f"[planning] Cargol {idx + 1} eliminat · en queden {len(self.cargols)}")

    # ---- Anàlisi d'ancoratge -------------------------------------------------

    def _screw_mm(self, cg):                                                    # definim la funció _screw_mm()
        """Extrems del cargol en mm, al marc físic del TAC (el mateix que el 3D)."""
        sp, org = np.asarray(self.spacing), np.asarray(self.origin)             # assignem sp, org
        p0 = org + np.array([cg["p0"][0], cg["p0"][1], cg["z_p0"]]) * sp        # p0 ← entrada
        p1 = org + np.array([cg["p1"][0], cg["p1"][1], cg["z_p1"]]) * sp        # p1 ← punta
        return p0, p1                                                           # retornem: p0, p1

    def _anchor_tools(self):                                                    # definim la funció _anchor_tools()
        """Mostrejador del TAC i localitzador de la medul·la (es creen un cop)."""
        from src.planning.screw_analysis import CTSampler, MedullaDistance      # importem l'anàlisi
        if self._anchor_sampler is None:                                        # si encara no existeix
            self._anchor_sampler = CTSampler(self.vol_original, self.spacing, self.origin)
        if self._anchor_medulla is None:                                        # si encara no existeix
            try:                                                                # intentem el bloc següent
                self._anchor_medulla = MedullaDistance(self._medula_fast)       # medul·la ja rotada (marc del TAC)
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[ancoratge] sense medul·la: {e}")
        return self._anchor_sampler, self._anchor_medulla                       # retornem: sampler, medulla

    def _analyze_selected(self):                                                # definim la funció _analyze_selected()
        if self.selected_idx is None or not (0 <= self.selected_idx < len(self.cargols)):
            self._show_analysis(None)                                           # executem _show_analysis()
            return None                                                         # retornem: None
        from src.planning.screw_analysis import analyze_screw                   # importem analyze_screw
        cg = self.cargols[self.selected_idx]                                    # assignem cg
        p0, p1 = self._screw_mm(cg)                                             # p0, p1 ← resultat de _screw_mm()
        sampler, med = self._anchor_tools()                                     # sampler, med ← resultat de _anchor_tools()
        try:                                                                    # intentem el bloc següent
            res = analyze_screw(sampler, p0, p1, float(cg.get("diam", 5.5)), med)
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[ancoratge] error: {e}")
            res = None                                                          # assignem res
        self._show_analysis(res)                                                # executem _show_analysis()
        return res                                                              # retornem: res

    def _show_analysis(self, res, proposal_text=None):                          # definim la funció _show_analysis()
        from src.planning.screw_analysis import HU_LOW, HU_OK, MED_MIN_MM, MED_OK_MM
        rows = self._anchor_rows                                                # assignem rows
        def setv(key, text, color=C_TEXT):                                      # definim la funció setv()
            rows[key].setText(text)                                             # executem setText()
            rows[key].setStyleSheet(f"color: {color}; font-family: {FONT_MONO}; font-size: {SIZE_XS}px; "
                                    f"font-weight: 700; background: transparent;")
        self.density_profile.set_result(res)                                    # executem set_result()
        has = res is not None                                                   # assignem has
        self.btn_optimize.setEnabled(has)                                       # executem setEnabled()
        self.btn_apply.setVisible(self._proposal is not None)                   # executem setVisible()
        if not has:                                                             # sense cargol
            for k in rows:                                                      # iterem: k in rows
                setv(k, "—", C_TEXT_MUTED)                                      # executem setv()
            self.lbl_verdict.setText("—")                                       # executem setText()
            self.lbl_verdict.setStyleSheet(f"color: {C_TEXT_MUTED}; background: transparent;")
            self.lbl_anchor_notes.setText("")                                   # executem setText()
            return                                                              # retornem sense valor

        hu = res["anchor_hu"]                                                   # assignem hu
        setv("hu", f"{hu:.0f} HU", C_SAFE if hu >= HU_OK else (C_WARN if hu >= HU_LOW else C_DANGER))
        if res["breaches"]:                                                     # si perfora
            a, b = res["breaches"][0][0], res["breaches"][-1][1]                # assignem a, b
            canal = any(x[2] for x in res["breaches"])                          # canal ← resultat de any()
            setv("breach", f"{a:.0f}–{b:.0f} mm" + (" · canal" if canal else ""), C_DANGER)
        else:                                                                   # en cas contrari
            setv("breach", "cap", C_SAFE)                                       # executem setv()
        md = res["med_dist_mm"]                                                 # assignem md
        if md is None:                                                          # si md is None
            setv("med", "—", C_TEXT_MUTED)                                      # executem setv()
        else:                                                                   # en cas contrari
            setv("med", "toca" if md <= 0 else f"{md:.1f} mm",
                 C_DANGER if md < MED_MIN_MM else (C_WARN if md < MED_OK_MM else C_SAFE))
        if res.get("isthmus_mm") is not None:                                   # si hi ha istme
            setv("isth", f"{res['isthmus_mm']:.0f} mm · Ø {res['max_diam_mm']:.1f}")
        else:                                                                   # en cas contrari
            setv("isth", "—", C_TEXT_MUTED)                                     # executem setv()
        rec = res.get("recommended_diam")                                       # rec ← resultat de get()
        cur = float(self.cargols[self.selected_idx].get("diam", 5.5))           # cur ← resultat de float()
        setv("diam", "cap Ø estàndard" if rec is None else f"{rec:.1f} mm",
             C_DANGER if rec is None else (C_SAFE if rec >= cur else C_WARN))
        col = {"ok": C_SAFE, "warn": C_WARN, "danger": C_DANGER}[res["verdict"]]  # col ← resultat de []
        txt = {"ok": "● BO", "warn": "● ATENCIÓ", "danger": "● RISC"}[res["verdict"]]
        self.lbl_verdict.setText(txt)                                           # executem setText()
        self.lbl_verdict.setStyleSheet(f"color: {col}; font-size: {SIZE_XS}px; font-weight: 700; "
                                       f"letter-spacing: 1px; background: transparent;")
        notes = list(res["notes"])                                              # notes ← resultat de list()
        if proposal_text:                                                       # si hi ha una proposta
            notes.insert(0, proposal_text)                                      # executem insert()
        self.lbl_anchor_notes.setText("\n".join("· " + n for n in notes) if notes else "· Sense alertes")

    def _optimize_selected(self):                                               # definim la funció _optimize_selected()
        """Busca una inclinació millor (entrada fixa) i la proposa."""
        if self.selected_idx is None:                                           # si no hi ha cargol
            return                                                              # retornem sense valor
        from src.planning.screw_analysis import optimize_trajectory             # importem optimize_trajectory
        idx = self.selected_idx                                                 # assignem idx
        cg = self.cargols[idx]                                                  # assignem cg
        p0, p1 = self._screw_mm(cg)                                             # p0, p1 ← resultat de _screw_mm()
        sampler, med = self._anchor_tools()                                     # sampler, med ← resultat de _anchor_tools()
        QtWidgets.QApplication.setOverrideCursor(Qt.WaitCursor)                 # executem setOverrideCursor()
        try:                                                                    # intentem el bloc següent
            opt = optimize_trajectory(sampler, p0, p1, float(cg.get("diam", 5.5)), med)
        finally:                                                                # sempre
            QtWidgets.QApplication.restoreOverrideCursor()                      # executem restoreOverrideCursor()
        if opt is None:                                                         # res millor
            self._proposal = None                                               # assignem self._proposal
            res = self._analyze_selected()                                      # res ← resultat de _analyze_selected()
            self._show_analysis(res, "Optimització: cap trajectòria millor dins ±6°")
            return                                                              # retornem sense valor
        p1n, rn, gain, ang = opt                                                # assignem p1n, rn, gain, ang
        self._proposal = (idx, p1n)                                             # assignem self._proposal
        md = rn["med_dist_mm"]                                                  # assignem md
        txt = (f"PROPOSTA: {ang:.1f}° · {rn['anchor_hu']:.0f} HU ({gain:+.0f} %)"
               + (f" · medul·la {md:.1f} mm" if md is not None else "")
               + (" · sense perforar" if not rn["breaches"] else "") + " → Aplicar")
        self._show_analysis(rn, txt)                                            # mostrem com quedaria
        print(f"[ancoratge] {txt}")

    def _apply_proposal(self):                                                  # definim la funció _apply_proposal()
        """Mou la punta (P1) del cargol a la trajectòria proposada."""
        if self._proposal is None:                                              # si no hi ha proposta
            return                                                              # retornem sense valor
        idx, p1n = self._proposal                                               # assignem idx, p1n
        self._proposal = None                                                   # assignem self._proposal
        if idx >= len(self.cargols):                                            # el cargol ja no hi és
            return                                                              # retornem sense valor
        sp, org = np.asarray(self.spacing), np.asarray(self.origin)             # assignem sp, org
        ix, iy, iz = (np.asarray(p1n) - org) / sp                               # mm → índexs del TAC
        cg = self.cargols[idx]                                                  # assignem cg
        cg["p1"] = (float(ix), float(iy))                                       # executem float()
        cg["z_p1"] = int(np.clip(round(iz), 0, self.Z - 1))                     # el tall Z és enter
        self._draw_screw(idx, cg)                                               # executem _draw_screw()
        self._update_3d_screw(idx)                                              # executem _update_3d_screw()
        self._update_telemetry()                                                # executem _update_telemetry()
        self._analyze_selected()                                                # executem _analyze_selected()
        self.cargolsChanged.emit(self.cargols)                                  # executem emit()

    def _select_from_list(self):
        idx = self.list_widget.currentRow()
        if idx < 0:                                                             # selecció buidada
            self.btn_delete.setEnabled(False)                                   # executem setEnabled()
            self._proposal = None                                               # assignem self._proposal
            self._show_analysis(None)                                           # executem _show_analysis()
        if idx >= 0:
            self.btn_delete.setEnabled(True)                                    # executem setEnabled()
            if self._proposal is not None and self._proposal[0] != idx:         # proposta d'un altre cargol
                self._proposal = None                                           # assignem self._proposal
            self.selected_idx = idx
            self.iz = self.cargols[idx]["z"]
            try:
                self.combo_diam.setCurrentIndex(DIAMETRES_STD.index(self.cargols[idx]["diam"]))
            except ValueError:
                pass
            self._update_telemetry()
            self._update_view("ax"); self._update_view("sag"); self._update_view("cor")
            self._refresh_screws_all_views()
            self._analyze_selected()                                            # anàlisi d'ancoratge

    def _set_edit(self, p):
        self.edit_point = p

    def _nudge(self, dx, dy, dz):
        if self.selected_idx is None:
            return
        cg = self.cargols[self.selected_idx]
        pt = self.edit_point
        if dx != 0 or dy != 0:
            x, y = cg[pt]
            cg[pt] = (x + dx, y + dy)
        if dz != 0:
            cg[f"z_{pt}"] = int(np.clip(cg[f"z_{pt}"] + dz, 0, self.Z-1))
        self._draw_screw(self.selected_idx, cg)
        self._nudge_timer.start(300)

    def _flush_nudge_pending(self):
        if self.selected_idx is None:
            return
        self._update_3d_screw(self.selected_idx)
        self._update_telemetry()
        self._proposal = None                                                   # el cargol s'ha mogut a mà
        self._analyze_selected()                                                # executem _analyze_selected()

    # ==========================================================================
    # TELEMETRIA
    # ==========================================================================
    def _update_telemetry(self):
        if self.selected_idx is not None:
            cg = self.cargols[self.selected_idx]
            px0, py0, pz0 = cg["p0"][0], cg["p0"][1], cg["z_p0"]
            px1, py1, pz1 = cg["p1"][0], cg["p1"][1], cg["z_p1"]
            p0_mm = self.origin + np.array([px0*self.spacing[0],
                                            py0*self.spacing[1],
                                            pz0*self.spacing[2]])
            p1_mm = self.origin + np.array([px1*self.spacing[0],
                                            py1*self.spacing[1],
                                            pz1*self.spacing[2]])
            color = cg["color"]
        else:
            if not self.show_mando:
                return
            p0_mm = np.array([self.mando_x, self.mando_y, self.mando_z])
            p1_mm = np.array([self.mando_x+self.dx,
                              self.mando_y+self.dy,
                              self.mando_z+self.dz])
            color = C_TEXT

        longitud = float(np.linalg.norm(p1_mm - p0_mm))
        self._set_hud_value(self.lbl_len, longitud, color=color)

    # ==========================================================================
    def cleanup(self):
        try:
            self.plotter.close()
        except Exception:
            pass