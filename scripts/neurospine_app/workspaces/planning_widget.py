import os
import math
import numpy as np

from qtpy import QtWidgets, QtCore, QtGui
from qtpy.QtGui import QColor
from qtpy.QtCore import Qt

import SimpleITK as sitk
import pyvista as pv
from pyvistaqt import QtInteractor
import pyqtgraph as pg

import vtk

from ..theme import (
    C_BG_WINDOW, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_ACCENT,
    C_SAFE, C_WARN, C_DANGER,
    CARGOL_COLORS,
    SIZE_XS, SIZE_LG, FONT_MONO,
)


vtk.vtkObject.GlobalWarningDisplayOff()
os.environ["VTK_SILENCE_GET_WINDOW_INFO_ERRORS"] = "1"

pg.setConfigOption("background", C_BG_WINDOW)
pg.setConfigOption("foreground", C_TEXT_DIM)
pg.setConfigOption("useOpenGL", False)
pg.setConfigOption("imageAxisOrder", "row-major")


PACIENT_ID = "839201"
_HERE        = os.path.dirname(os.path.abspath(__file__))
_APP_DIR     = os.path.dirname(_HERE)
_SCRIPTS     = os.path.dirname(_APP_DIR)
_ROOT        = os.path.dirname(_SCRIPTS)
_PACIENT_DIR = os.path.join(_ROOT, "data", "pacients", PACIENT_ID)

TAC_PATH    = os.path.join(_PACIENT_DIR, "escoliosis.nrrd")
MEDULA_PATH = os.path.join(_PACIENT_DIR, "medula_rectificada.stl")

DIAMETRES_STD = [4.5, 5.5, 6.5, 7.5]


def make_bone_lut():
    n = 256
    lut = np.zeros((n, 4), dtype=np.uint8)
    for i in range(n):
        v = i / (n - 1)
        r = int(255 * v)
        g = int(255 * v)
        b = int(255 * min(1.0, v * 1.05))
        lut[i] = (r, g, b, 255)
    return lut

BONE_LUT = make_bone_lut()


class CTViewport(pg.GraphicsLayoutWidget):
    wheelScrolled    = QtCore.Signal(int)
    clickedInView    = QtCore.Signal(float, float, int)
    mouseMovedInView = QtCore.Signal(float, float)

    def __init__(self, name, invert_y=False, parent=None):
        super().__init__(parent)
        self.name = name

        self.vb = self.addViewBox(row=0, col=0, lockAspect=True, invertY=invert_y)
        self.vb.setMenuEnabled(False)
        self.vb.setMouseEnabled(x=True, y=True)

        self.image_item = pg.ImageItem()
        self.image_item.setLookupTable(BONE_LUT)
        self.image_item.setLevels([0, 1500])
        self.vb.addItem(self.image_item)

        pen = pg.mkPen(color=(200, 200, 205, 180), width=1, style=Qt.DashLine)
        self.hline = pg.InfiniteLine(angle=0,  pen=pen, movable=False)
        self.vline = pg.InfiniteLine(angle=90, pen=pen, movable=False)
        self.vb.addItem(self.hline)
        self.vb.addItem(self.vline)

        self.medula_curve = pg.PlotCurveItem(
            pen=pg.mkPen(color=(240, 100, 100, 220), width=1.8),
            connect="pairs",
        )
        self.vb.addItem(self.medula_curve)

        self.mando_line = pg.PlotDataItem(
            pen=pg.mkPen(color=(220, 220, 225), width=2),
        )
        self.mando_pts = pg.ScatterPlotItem(
            pen=pg.mkPen(color=(220, 220, 225), width=1),
            brush=pg.mkBrush(color=(220, 220, 225)),
            size=8, symbol="+",
        )
        self.vb.addItem(self.mando_line)
        self.vb.addItem(self.mando_pts)

        self.screw_items = {}

        self.preview_p0 = pg.ScatterPlotItem(
            pen=pg.mkPen(color=(255, 255, 255), width=2),
            brush=pg.mkBrush(color=(255, 255, 255, 200)),
            size=12, symbol="o",
        )
        self.preview_line = pg.PlotDataItem(
            pen=pg.mkPen(color=(255, 255, 255, 220), width=2, style=Qt.DashLine),
        )
        self.vb.addItem(self.preview_p0)
        self.vb.addItem(self.preview_line)

        self.density_item = pg.ImageItem()
        self.density_item.setZValue(5)
        self.density_item.setVisible(False)
        self.vb.addItem(self.density_item)

        self.scene().sigMouseClicked.connect(self._on_scene_click)
        self.scene().sigMouseMoved.connect(self._on_scene_moved)

    def show_preview_p0(self, x, y):
        self.preview_p0.setData(x=[x], y=[y])

    def show_preview_line(self, x0, y0, x1, y1):
        self.preview_line.setData(x=[x0, x1], y=[y0, y1])

    def clear_preview(self):
        self.preview_p0.clear()
        self.preview_line.clear()

    def set_image(self, arr_2d):
        self.image_item.setImage(arr_2d, autoLevels=False)

    def set_density(self, rgba_2d):
        if rgba_2d is None:
            self.density_item.setVisible(False)
        else:
            self.density_item.setImage(rgba_2d, autoLevels=False)
            self.density_item.setVisible(True)

    def clear_density(self):
        self.density_item.setVisible(False)

    def set_crosshair(self, x, y):
        self.vline.setValue(x)
        self.hline.setValue(y)

    def set_medula(self, xs, ys):
        if xs is None or len(xs) == 0:
            self.medula_curve.setData(x=[], y=[])
        else:
            self.medula_curve.setData(x=xs, y=ys)

    def clear_medula(self):
        self.medula_curve.setData(x=[], y=[])

    def set_mando(self, x0, y0, x1, y1):
        self.mando_line.setData(x=[x0, x1], y=[y0, y1])
        self.mando_pts.setData(x=[x0, x1], y=[y0, y1])

    def clear_mando(self):
        self.mando_line.clear()
        self.mando_pts.clear()

    def _clear_screw_items(self, idx):
        if idx not in self.screw_items:
            return
        for it in self.screw_items[idx]["items"]:
            try:
                self.vb.removeItem(it)
            except Exception:
                pass
        del self.screw_items[idx]

    def set_screw(self, idx, poly_pts, color, selected=False, p0=None, p1=None):
        border = QColor("white") if selected else QColor(color)
        fill = QColor(color)
        fill.setAlpha(200 if selected else 110)

        polygon = QtGui.QPolygonF([QtCore.QPointF(x, y) for x, y in poly_pts])

        if p0 is None or p1 is None:
            p0 = ((poly_pts[0][0] + poly_pts[3][0]) / 2.0,
                  (poly_pts[0][1] + poly_pts[3][1]) / 2.0)
            p1 = ((poly_pts[1][0] + poly_pts[2][0]) / 2.0,
                  (poly_pts[1][1] + poly_pts[2][1]) / 2.0)

        self._clear_screw_items(idx)

        item = QtWidgets.QGraphicsPolygonItem()
        item.setPolygon(polygon)
        item.setBrush(QtGui.QBrush(fill))
        item.setPen(QtGui.QPen(border, 0))
        self.vb.addItem(item)

        pts = pg.ScatterPlotItem(
            x=[p0[0], p1[0]], y=[p0[1], p1[1]],
            symbol=["o", "s"], size=10,
            pen=pg.mkPen(border, width=1.5),
            brush=pg.mkBrush(color),
        )
        self.vb.addItem(pts)

        self.screw_items[idx] = {"items": [item, pts], "kind": "rect"}

    def set_screw_realistic(self, idx, geom, color, selected=False):
        self._clear_screw_items(idx)

        p0 = geom["p0_view"]
        p1 = geom["p1_view"]
        hw_head = geom["half_width_head"]
        hw_body = geom["half_width_body"]
        head_frac = geom.get("head_frac", 0.15)
        tip_frac  = geom.get("tip_frac",  0.10)
        n_threads = geom.get("n_threads", 8)

        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return
        ux, uy = dx / L, dy / L
        nx, ny = -uy, ux

        body_fill  = QColor(color)
        body_fill.setAlpha(220 if selected else 150)
        head_fill  = QColor(color).darker(135)
        head_fill.setAlpha(255 if selected else 200)
        border_col = QColor("white") if selected else QColor(color).darker(180)
        border_pen = QtGui.QPen(border_col, 0.8)
        border_pen.setCosmetic(True)
        thread_col = QColor(color).darker(180)
        thread_col.setAlpha(180)
        thread_pen = QtGui.QPen(thread_col, 0.6)
        thread_pen.setCosmetic(True)

        items = []

        pt = lambda t: (p0[0] + t*dx, p0[1] + t*dy)
        head_end = pt(head_frac)
        tip_start = pt(1.0 - tip_frac)
        tip_end   = pt(1.0)

        head_poly = QtGui.QPolygonF([
            QtCore.QPointF(p0[0] + nx*hw_head, p0[1] + ny*hw_head),
            QtCore.QPointF(head_end[0] + nx*hw_head, head_end[1] + ny*hw_head),
            QtCore.QPointF(head_end[0] - nx*hw_head, head_end[1] - ny*hw_head),
            QtCore.QPointF(p0[0] - nx*hw_head, p0[1] - ny*hw_head),
        ])
        head_item = QtWidgets.QGraphicsPolygonItem(head_poly)
        head_item.setBrush(QtGui.QBrush(head_fill))
        head_item.setPen(border_pen)
        self.vb.addItem(head_item)
        items.append(head_item)

        body_poly = QtGui.QPolygonF([
            QtCore.QPointF(head_end[0] + nx*hw_body, head_end[1] + ny*hw_body),
            QtCore.QPointF(tip_start[0] + nx*hw_body, tip_start[1] + ny*hw_body),
            QtCore.QPointF(tip_start[0] - nx*hw_body, tip_start[1] - ny*hw_body),
            QtCore.QPointF(head_end[0] - nx*hw_body, head_end[1] - ny*hw_body),
        ])
        body_item = QtWidgets.QGraphicsPolygonItem(body_poly)
        body_item.setBrush(QtGui.QBrush(body_fill))
        body_item.setPen(border_pen)
        self.vb.addItem(body_item)
        items.append(body_item)

        tip_poly = QtGui.QPolygonF([
            QtCore.QPointF(tip_start[0] + nx*hw_body, tip_start[1] + ny*hw_body),
            QtCore.QPointF(tip_end[0], tip_end[1]),
            QtCore.QPointF(tip_start[0] - nx*hw_body, tip_start[1] - ny*hw_body),
        ])
        tip_item = QtWidgets.QGraphicsPolygonItem(tip_poly)
        tip_item.setBrush(QtGui.QBrush(body_fill))
        tip_item.setPen(border_pen)
        self.vb.addItem(tip_item)
        items.append(tip_item)

        body_len_frac = 1.0 - head_frac - tip_frac
        if n_threads > 0 and body_len_frac > 0:
            xs_thread = []
            ys_thread = []
            for i in range(n_threads):
                t = head_frac + body_len_frac * (i + 0.5) / n_threads
                cx, cy = pt(t)
                xs_thread.extend([cx + nx*hw_body, cx - nx*hw_body])
                ys_thread.extend([cy + ny*hw_body, cy - ny*hw_body])
            threads_item = pg.PlotCurveItem(
                x=xs_thread, y=ys_thread,
                pen=thread_pen, connect="pairs")
            self.vb.addItem(threads_item)
            items.append(threads_item)

        pts = pg.ScatterPlotItem(
            x=[p0[0], p1[0]], y=[p0[1], p1[1]],
            symbol=["o", "t"],
            size=[12, 10],
            pen=pg.mkPen(QColor("white") if selected else QColor(color),
                         width=1.5),
            brush=pg.mkBrush(color),
        )
        self.vb.addItem(pts)
        items.append(pts)

        self.screw_items[idx] = {"items": items, "kind": "realistic"}

    def set_screw_ellipse(self, idx, cx, cy, semi_major, semi_minor,
                          angle_deg, color, selected=False):
        self._clear_screw_items(idx)

        border = QColor("white") if selected else QColor(color).darker(180)
        border_pen = QtGui.QPen(border, 1.2)
        border_pen.setCosmetic(True)

        fill = QColor(color)
        fill.setAlpha(180 if selected else 130)

        rect_outer = QtCore.QRectF(-semi_major, -semi_minor,
                                   2*semi_major, 2*semi_minor)
        ell_outer = QtWidgets.QGraphicsEllipseItem(rect_outer)
        ell_outer.setBrush(QtGui.QBrush(fill))
        ell_outer.setPen(border_pen)
        ell_outer.setPos(cx, cy)
        ell_outer.setRotation(angle_deg)
        self.vb.addItem(ell_outer)

        inner_scale = 0.55
        rect_inner = QtCore.QRectF(-semi_major*inner_scale,
                                   -semi_minor*inner_scale,
                                   2*semi_major*inner_scale,
                                   2*semi_minor*inner_scale)
        core_col = QColor(color).darker(160)
        core_col.setAlpha(220 if selected else 170)
        ell_inner = QtWidgets.QGraphicsEllipseItem(rect_inner)
        ell_inner.setBrush(QtGui.QBrush(core_col))
        ell_inner.setPen(QtGui.QPen(QColor(0, 0, 0, 0), 0))
        ell_inner.setPos(cx, cy)
        ell_inner.setRotation(angle_deg)
        self.vb.addItem(ell_inner)

        pts = pg.ScatterPlotItem(
            x=[cx], y=[cy], symbol="+", size=8,
            pen=pg.mkPen(border, width=1.5),
            brush=pg.mkBrush(color),
        )
        self.vb.addItem(pts)

        self.screw_items[idx] = {
            "items": [ell_outer, ell_inner, pts],
            "kind": "ellipse",
        }

    def remove_screw(self, idx):
        self._clear_screw_items(idx)

    def clear_screws(self):
        for idx in list(self.screw_items.keys()):
            self._clear_screw_items(idx)

    def wheelEvent(self, event):
        mods = event.modifiers() if hasattr(event, "modifiers") else Qt.NoModifier
        if mods & Qt.ControlModifier:
            super().wheelEvent(event)
        else:
            delta = 1 if event.angleDelta().y() > 0 else -1
            self.wheelScrolled.emit(delta)
            event.accept()

    def _on_scene_click(self, ev):
        if not self.vb.sceneBoundingRect().contains(ev.scenePos()):
            return
        view_pos = self.vb.mapSceneToView(ev.scenePos())
        button = 1 if ev.button() == Qt.LeftButton else \
                 2 if ev.button() == Qt.RightButton else 3
        self.clickedInView.emit(view_pos.x(), view_pos.y(), button)

    def _on_scene_moved(self, scene_pos):
        if not self.vb.sceneBoundingRect().contains(scene_pos):
            return
        view_pos = self.vb.mapSceneToView(scene_pos)
        self.mouseMovedInView.emit(view_pos.x(), view_pos.y())


class DensityProfile(QtWidgets.QWidget):
    HU_TOP = 600.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(118)
        self.res = None

    def set_result(self, res):
        self.res = res
        self.update()

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QtGui.QColor("#0e0e10"))
        f = QtGui.QFont(self.font()); f.setPixelSize(SIZE_XS - 1)
        p.setFont(f)
        r = QtCore.QRectF(30, 8, w - 38, h - 26)
        if self.res is None:
            p.setPen(QtGui.QColor(C_TEXT_MUTED))
            p.drawText(self.rect(), Qt.AlignCenter, "Selecciona un cargol")
            p.end()
            return
        from src.planning.screw_analysis import HU_LOW, HU_OK
        t, hu = self.res["profile"]
        L = max(self.res["length_mm"], 1.0)
        X = lambda mm: r.left() + mm / L * r.width()
        Y = lambda v: r.bottom() - min(max(v, 0.0), self.HU_TOP) / self.HU_TOP * r.height()

        red = QtGui.QColor(C_DANGER); red.setAlpha(70)
        for a, b, _canal in self.res["breaches"]:
            p.fillRect(QtCore.QRectF(X(a), r.top(), max(X(b + 0.5) - X(a), 2), r.height()), red)
        for thr, dy in ((HU_LOW, 6), (HU_OK, -6)):
            p.setPen(QtGui.QPen(QtGui.QColor(C_TEXT_MUTED), 1, Qt.DashLine))
            p.drawLine(QtCore.QPointF(r.left(), Y(thr)), QtCore.QPointF(r.right(), Y(thr)))
            p.setPen(QtGui.QColor(C_TEXT_MUTED))
            p.drawText(QtCore.QRectF(0, Y(thr) + dy - 7, 27, 14), Qt.AlignRight | Qt.AlignVCenter, f"{thr:.0f}")
        p.drawText(QtCore.QRectF(0, r.top() - 1, 27, 14), Qt.AlignRight | Qt.AlignVCenter, f"{self.HU_TOP:.0f}")
        p.setPen(QtGui.QPen(QtGui.QColor(C_BORDER_STRONG), 1))
        p.drawLine(r.bottomLeft(), r.bottomRight())
        path = QtGui.QPainterPath()
        for i, (tt, vv) in enumerate(zip(t, hu)):
            pt = QtCore.QPointF(X(tt), Y(vv))
            path.moveTo(pt) if i == 0 else path.lineTo(pt)
        p.setPen(QtGui.QPen(QtGui.QColor(C_TEXT), 1.4))
        p.drawPath(path)
        if self.res.get("isthmus_mm") is not None:
            xi = X(self.res["isthmus_mm"])
            p.setPen(Qt.NoPen); p.setBrush(QtGui.QColor(C_ACCENT))
            p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(xi - 4, r.top()), QtCore.QPointF(xi + 4, r.top()),
                                           QtCore.QPointF(xi, r.top() + 6)]))
        p.setPen(QtGui.QColor(C_TEXT_MUTED))
        p.drawText(QtCore.QRectF(r.left(), r.bottom() + 2, 60, 14), Qt.AlignLeft, "entrada")
        p.drawText(QtCore.QRectF(r.right() - 80, r.bottom() + 2, 80, 14), Qt.AlignRight, f"punta · {L:.0f} mm")
        p.drawText(QtCore.QRectF(r.left() + 4, r.top(), 60, 14), Qt.AlignLeft, "HU")
        p.end()


class PlanningWidget(QtWidgets.QWidget):
    cargolsChanged = QtCore.Signal(list)

    def __init__(self, mando_service=None, parent=None):
        super().__init__(parent)

        self.mando_service = mando_service
        self.mando_x = self.mando_y = self.mando_z = 0.0
        self.dx = self.dy = 0.0
        self.dz = 60.0
        self.offset_x = self.offset_y = self.offset_z = 0.0
        self.primera_lectura = True

        self.cargols = []
        self.current = None
        self.creating = False
        self.selected_idx = None
        self.edit_point = "p0"
        self.show_medula = False
        self.show_mando = True
        self.show_porosity = False
        self.is_3d_loaded = False
        self.current_diam = 5.5

        self._medula_cache = {"ax": {}, "sag": {}, "cor": {}}
        self._medula_cache_ready = False

        self._load_medical_data()
        self.iz, self.ix, self.iy = self.Z // 2, self.X // 2, self.Y // 2

        self._build_ui()

        if self.mando_service is not None:
            self.mando_service.reading.connect(self.on_mando_reading)

        self._update_view("ax")
        self._update_view("sag")
        self._update_view("cor")
        self._refresh_screws_all_views()

        self._medula_cache_ready = True

        self._medula_pending_axes = set()
        self._medula_timer = QtCore.QTimer(self)
        self._medula_timer.setSingleShot(True)
        self._medula_timer.timeout.connect(self._flush_medula_pending)

        self._nudge_timer = QtCore.QTimer(self)
        self._nudge_timer.setSingleShot(True)
        self._nudge_timer.timeout.connect(self._flush_nudge_pending)

    def _load_medical_data(self):
        img = sitk.ReadImage(TAC_PATH)
        self.vol_original = sitk.GetArrayFromImage(img)
        self.vol = np.copy(self.vol_original)
        self.vol[self.vol < 200] = 0
        self.vol_flipped = np.flip(self.vol, axis=0)

        self.vol_orig_flipped = np.flip(self.vol_original, axis=0)

        self.spacing = img.GetSpacing()
        self.origin  = img.GetOrigin()
        self.Z, self.Y, self.X = self.vol.shape

        self.medula = pv.read(MEDULA_PATH).clean().triangulate()
        self.medula.rotate_z(180, inplace=True)
        self.med_cx, self.med_cy, self.med_cz = self.medula.center

        try:
            reduced = self.medula.decimate(0.98)
            if reduced.n_points > 50:
                self._medula_fast = reduced
                print(f"[medula] decimada per slices: "
                      f"{self.medula.n_points} → {reduced.n_points} punts")
            else:
                self._medula_fast = self.medula
        except Exception as e:
            self._medula_fast = self.medula
            print(f"[medula] error decimant: {e}")

        self._med_center = self._medula_fast.center

    def _compute_medula_slice(self, axis, idx):
        if idx in self._medula_cache[axis]:
            return self._medula_cache[axis][idx]

        mesh = self._medula_fast
        cx, cy, cz = self._med_center

        try:
            if axis == "ax":
                z_world = self.origin[2] + idx * self.spacing[2]
                sec = mesh.slice(normal=(0, 0, 1), origin=(cx, cy, z_world))
                result = self._segments_from_slice_ax(sec)
                if result is not None:
                    self._medula_cache["ax"][idx] = result
                    return result

            elif axis == "sag":
                x_world = self.origin[0] + idx * self.spacing[0]
                sec = mesh.slice(normal=(1, 0, 0), origin=(x_world, cy, cz))
                result = self._segments_from_slice_sag(sec)
                if result is not None:
                    self._medula_cache["sag"][idx] = result
                    return result

            elif axis == "cor":
                y_world = self.origin[1] + idx * self.spacing[1]
                sec = mesh.slice(normal=(0, 1, 0), origin=(cx, y_world, cz))
                result = self._segments_from_slice_cor(sec)
                if result is not None:
                    self._medula_cache["cor"][idx] = result
                    return result
        except Exception:
            pass

        self._medula_cache[axis][idx] = None
        return None

    def _segments_from_slice_ax(self, sec):
        if sec.n_points == 0 or sec.lines is None or len(sec.lines) == 0:
            return None
        pts = sec.points
        xs_all = (pts[:, 0] - self.origin[0]) / self.spacing[0]
        ys_all = (pts[:, 1] - self.origin[1]) / self.spacing[1]
        return self._pairs_from_lines(sec.lines, xs_all, ys_all, self.X, self.Y)

    def _segments_from_slice_sag(self, sec):
        if sec.n_points == 0 or sec.lines is None or len(sec.lines) == 0:
            return None
        pts = sec.points
        ys_all = (pts[:, 1] - self.origin[1]) / self.spacing[1]
        zs_all = (pts[:, 2] - self.origin[2]) / self.spacing[2]
        vs = self.Z - 1 - zs_all
        return self._pairs_from_lines(sec.lines, ys_all, vs, self.Y, self.Z)

    def _segments_from_slice_cor(self, sec):
        if sec.n_points == 0 or sec.lines is None or len(sec.lines) == 0:
            return None
        pts = sec.points
        xs_all = (pts[:, 0] - self.origin[0]) / self.spacing[0]
        zs_all = (pts[:, 2] - self.origin[2]) / self.spacing[2]
        vs = self.Z - 1 - zs_all
        return self._pairs_from_lines(sec.lines, xs_all, vs, self.X, self.Z)

    @staticmethod
    def _pairs_from_lines(lines, xs_all, ys_all, max_x, max_y):
        try:
            lines = np.asarray(lines, dtype=np.int64)
        except Exception:
            return None

        xs_out = []
        ys_out = []

        i = 0
        L = len(lines)
        while i < L:
            n = int(lines[i]); i += 1
            if n < 2 or i + n > L:
                i += n
                continue
            idxs = lines[i:i+n]; i += n

            for k in range(n - 1):
                a, b = int(idxs[k]), int(idxs[k+1])
                ax, ay = xs_all[a], ys_all[a]
                bx, by = xs_all[b], ys_all[b]
                if (0 <= ax < max_x and 0 <= ay < max_y and
                    0 <= bx < max_x and 0 <= by < max_y):
                    xs_out.append(ax); xs_out.append(bx)
                    ys_out.append(ay); ys_out.append(by)

        if not xs_out:
            return None
        return (np.asarray(xs_out, dtype=np.float32),
                np.asarray(ys_out, dtype=np.float32))

    def _flush_medula_pending(self):
        if not self.show_medula:
            self._medula_pending_axes.clear()
            return
        for axis in list(self._medula_pending_axes):
            view = {"ax": self.view_ax, "sag": self.view_sag,
                    "cor": self.view_cor}[axis]
            idx = {"ax": self.iz, "sag": self.ix, "cor": self.iy}[axis]
            result = self._compute_medula_slice(axis, idx)
            if result is None:
                view.clear_medula()
            else:
                view.set_medula(result[0], result[1])
        self._medula_pending_axes.clear()

    def _build_ui(self):
        main_layout = QtWidgets.QHBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        main_layout.addWidget(self._build_left_panel())
        main_layout.addWidget(self._build_viewports(), 1)
        main_layout.addWidget(self._build_right_panel())

    def _build_left_panel(self):
        panel = QtWidgets.QFrame()
        panel.setObjectName("Panel")
        panel.setFixedWidth(260)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(16, 16, 16, 16)
        v.setSpacing(12)
        v.setAlignment(Qt.AlignTop)

        t1 = QtWidgets.QLabel("PLANIFICACIÓ DE CARGOLS")
        t1.setObjectName("SectionTitle")
        v.addWidget(t1)

        self.btn_new = QtWidgets.QPushButton("Crear trajectòria manual")
        self.btn_new.setCheckable(True)
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
        self.btn_new.setStyleSheet(self._btn_new_style_idle)
        v.addWidget(self.btn_new)

        diam_row = QtWidgets.QHBoxLayout()
        diam_row.addWidget(QtWidgets.QLabel("Diàmetre:"))
        self.combo_diam = QtWidgets.QComboBox()
        for d in DIAMETRES_STD:
            self.combo_diam.addItem(f"{d} mm")
        self.combo_diam.setCurrentIndex(1)
        diam_row.addWidget(self.combo_diam, 1)
        v.addLayout(diam_row)

        t2 = QtWidgets.QLabel("VISUALITZACIÓ")
        t2.setObjectName("SectionTitle")
        t2.setStyleSheet(t2.styleSheet() + "margin-top: 8px;")
        v.addWidget(t2)

        self.btn_load_3d = QtWidgets.QPushButton("Generar model 3D")
        self.btn_medula  = QtWidgets.QPushButton("Mostrar canal medul·lar")
        self.btn_medula.setCheckable(True)
        self.btn_xray    = QtWidgets.QPushButton("Vista densitat òssia (X-ray)")
        self.btn_xray.setCheckable(True)
        self.btn_xray.setEnabled(False)
        self.btn_porosity = QtWidgets.QPushButton("Mapa de porositat òssia")
        self.btn_porosity.setCheckable(True)
        v.addWidget(self.btn_load_3d)
        v.addWidget(self.btn_medula)
        v.addWidget(self.btn_xray)
        v.addWidget(self.btn_porosity)

        t3 = QtWidgets.QLabel("NAVEGACIÓ RÀPIDA")
        t3.setObjectName("SectionTitle")
        t3.setStyleSheet(t3.styleSheet() + "margin-top: 8px;")
        v.addWidget(t3)

        for label, attr, maxval in [
            ("Axial (Z)",   "spin_iz", self.Z),
            ("Sagital (X)", "spin_ix", self.X),
            ("Coronal (Y)", "spin_iy", self.Y),
        ]:
            row = QtWidgets.QHBoxLayout()
            lbl = QtWidgets.QLabel(label)
            lbl.setFixedWidth(90)
            row.addWidget(lbl)
            spin = QtWidgets.QSpinBox()
            spin.setRange(1, maxval)
            spin.setValue(1)
            spin.setSingleStep(1)
            spin.setKeyboardTracking(False)
            row.addWidget(spin, 1)
            v.addLayout(row)
            setattr(self, attr, spin)

        self.spin_iz.valueChanged.connect(self._on_spin_iz)
        self.spin_ix.valueChanged.connect(self._on_spin_ix)
        self.spin_iy.valueChanged.connect(self._on_spin_iy)

        t4 = QtWidgets.QLabel("AJUST FI DEL TRAJECTE")
        t4.setObjectName("SectionTitle")
        t4.setStyleSheet(t4.styleSheet() + "margin-top: 8px;")
        v.addWidget(t4)

        mode_row = QtWidgets.QHBoxLayout()
        self.btn_p0 = QtWidgets.QPushButton("Base (P0)")
        self.btn_p0.setCheckable(True); self.btn_p0.setChecked(True)
        self.btn_p1 = QtWidgets.QPushButton("Punta (P1)")
        self.btn_p1.setCheckable(True)
        point_group = QtWidgets.QButtonGroup(self)
        point_group.addButton(self.btn_p0)
        point_group.addButton(self.btn_p1)
        mode_row.addWidget(self.btn_p0)
        mode_row.addWidget(self.btn_p1)
        v.addLayout(mode_row)

        dpad = QtWidgets.QGridLayout()
        dpad.setSpacing(4)
        self.btn_up   = QtWidgets.QPushButton("↑")
        self.btn_dn   = QtWidgets.QPushButton("↓")
        self.btn_lt   = QtWidgets.QPushButton("←")
        self.btn_rt   = QtWidgets.QPushButton("→")
        self.btn_z_up = QtWidgets.QPushButton("+Z")
        self.btn_z_dn = QtWidgets.QPushButton("-Z")
        for b in (self.btn_up, self.btn_dn, self.btn_lt, self.btn_rt,
                  self.btn_z_up, self.btn_z_dn):
            b.setFixedHeight(32)
        dpad.addWidget(self.btn_up, 0, 1)
        dpad.addWidget(self.btn_lt, 1, 0)
        dpad.addWidget(self.btn_rt, 1, 2)
        dpad.addWidget(self.btn_dn, 2, 1)
        dpad.addWidget(self.btn_z_up, 0, 3)
        dpad.addWidget(self.btn_z_dn, 2, 3)
        v.addLayout(dpad)

        v.addStretch()

        self.btn_load_3d.clicked.connect(self._generate_3d)
        self.btn_new.clicked.connect(self._new_cargol)
        self.btn_medula.clicked.connect(self._toggle_medula)
        self.btn_xray.clicked.connect(self._toggle_xray)
        self.btn_porosity.clicked.connect(self._toggle_porosity)
        self.combo_diam.currentIndexChanged.connect(self._change_diameter)
        self.btn_p0.clicked.connect(lambda: self._set_edit("p0"))
        self.btn_p1.clicked.connect(lambda: self._set_edit("p1"))
        self.btn_up.clicked.connect(lambda: self._nudge(0, -1, 0))
        self.btn_dn.clicked.connect(lambda: self._nudge(0,  1, 0))
        self.btn_lt.clicked.connect(lambda: self._nudge(-1, 0, 0))
        self.btn_rt.clicked.connect(lambda: self._nudge( 1, 0, 0))
        self.btn_z_up.clicked.connect(lambda: self._nudge(0, 0,  1))
        self.btn_z_dn.clicked.connect(lambda: self._nudge(0, 0, -1))

        return panel

    def _build_viewports(self):
        container = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(container)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(12)

        self.view_ax  = CTViewport("ax",  invert_y=True)
        self.view_sag = CTViewport("sag", invert_y=True)
        self.view_cor = CTViewport("cor", invert_y=True)

        self.view_ax .wheelScrolled.connect(lambda d: self._on_scroll("ax",  d))
        self.view_sag.wheelScrolled.connect(lambda d: self._on_scroll("sag", d))
        self.view_cor.wheelScrolled.connect(lambda d: self._on_scroll("cor", d))

        self.view_ax .clickedInView.connect(lambda x, y, b: self._on_click("ax",  x, y, b))
        self.view_sag.clickedInView.connect(lambda x, y, b: self._on_click("sag", x, y, b))
        self.view_cor.clickedInView.connect(lambda x, y, b: self._on_click("cor", x, y, b))

        self.view_ax.mouseMovedInView.connect(self._on_preview_motion)

        self.plotter = QtInteractor(container)
        self.plotter.set_background(C_BG_WINDOW)

        grid.addWidget(self._wrap_view(self.view_ax,  "AXIAL"),   0, 0)
        grid.addWidget(self._wrap_view(self.view_sag, "SAGITAL"), 0, 1)
        grid.addWidget(self._wrap_view(self.view_cor, "CORONAL"), 1, 0)
        grid.addWidget(self._wrap_view(self.plotter,  "RECONSTRUCCIÓ 3D",
                                        is_plotter=True), 1, 1)

        return container

    def _wrap_view(self, widget, title, is_plotter=False):
        w = QtWidgets.QFrame()
        w.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_WINDOW};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(10, 8, 10, 10)
        v.setSpacing(6)

        header_layout = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel(title)
        lbl.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;"
        )
        header_layout.addWidget(lbl)
        header_layout.addStretch()

        if not is_plotter:
            widget.header_label = lbl

        v.addLayout(header_layout)
        v.addWidget(widget, 1)
        return w

    def _build_right_panel(self):
        panel = QtWidgets.QFrame()
        panel.setObjectName("Panel")
        panel.setFixedWidth(280)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(16, 16, 16, 16)
        v.setSpacing(12)

        t1 = QtWidgets.QLabel("ANÀLISI GEOMÈTRIC")
        t1.setObjectName("SectionTitle")
        v.addWidget(t1)

        self.lbl_len = self._hud_block("LONGITUD DEL CARGOL", "──", "mm")
        v.addWidget(self.lbl_len["w"])

        t2 = QtWidgets.QLabel("CARGOLS PLANIFICATS")
        t2.setObjectName("SectionTitle")
        t2.setStyleSheet(t2.styleSheet() + "margin-top: 12px;")
        v.addWidget(t2)

        self.list_widget = QtWidgets.QListWidget()
        self.list_widget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.list_widget.itemSelectionChanged.connect(self._select_from_list)
        v.addWidget(self.list_widget, 1)

        self.btn_delete = QtWidgets.QPushButton("Eliminar cargol")
        self.btn_delete.setEnabled(False)
        self.btn_delete.setToolTip("Elimina el cargol seleccionat (Supr)")
        self.btn_delete.setStyleSheet(f"""
            QPushButton {{ color: {C_DANGER}; border: 1px solid rgba(217, 83, 79, 0.45); }}
            QPushButton:hover {{ background-color: rgba(217, 83, 79, 0.15); }}
            QPushButton:disabled {{ color: {C_TEXT_MUTED}; border: 1px solid {C_BORDER}; }}
        """)
        self.btn_delete.clicked.connect(self._delete_cargol)
        v.addWidget(self.btn_delete)
        sc_del = QtWidgets.QShortcut(QtGui.QKeySequence(QtGui.QKeySequence.Delete), self.list_widget)
        sc_del.activated.connect(self._delete_cargol)

        v.addWidget(self._build_anchor_card())

        return panel

    def _build_anchor_card(self):
        card = QtWidgets.QWidget()
        card.setStyleSheet("background: transparent;")
        v = QtWidgets.QVBoxLayout(card)
        v.setContentsMargins(0, 6, 0, 0)
        v.setSpacing(6)

        head = QtWidgets.QHBoxLayout()
        t = QtWidgets.QLabel("ANÀLISI D'ANCORATGE")
        t.setObjectName("SectionTitle")
        head.addWidget(t, 1)
        self.lbl_verdict = QtWidgets.QLabel("—")
        head.addWidget(self.lbl_verdict)
        v.addLayout(head)

        self._anchor_rows = {}
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(8); grid.setVerticalSpacing(3)
        for i, (key, label) in enumerate([("hu", "Densitat d'ancoratge"),
                                          ("breach", "Perforació"),
                                          ("med", "Medul·la"),
                                          ("isth", "Istme · Ø màx"),
                                          ("diam", "Ø recomanat")]):
            lk = QtWidgets.QLabel(label)
            lk.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: {SIZE_XS}px; background: transparent;")
            lv = QtWidgets.QLabel("—")
            lv.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lv.setStyleSheet(f"color: {C_TEXT}; font-family: {FONT_MONO}; font-size: {SIZE_XS}px; "
                             f"font-weight: 700; background: transparent;")
            grid.addWidget(lk, i, 0); grid.addWidget(lv, i, 1)
            self._anchor_rows[key] = lv
        v.addLayout(grid)

        self.density_profile = DensityProfile()
        v.addWidget(self.density_profile)

        self.lbl_anchor_notes = QtWidgets.QLabel("")
        self.lbl_anchor_notes.setWordWrap(True)
        self.lbl_anchor_notes.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: {SIZE_XS}px; background: transparent;")
        v.addWidget(self.lbl_anchor_notes)

        row = QtWidgets.QHBoxLayout(); row.setSpacing(6)
        self.btn_optimize = QtWidgets.QPushButton("Optimitzar trajectòria")
        self.btn_optimize.setToolTip("Entrada fixa: prova inclinacions de fins a ±6° i proposa la de més "
                                     "densitat, sense perforar i lluny de la medul·la")
        self.btn_optimize.clicked.connect(self._optimize_selected)
        self.btn_apply = QtWidgets.QPushButton("Aplicar")
        self.btn_apply.setToolTip("Aplica la trajectòria proposada al cargol")
        self.btn_apply.clicked.connect(self._apply_proposal)
        row.addWidget(self.btn_optimize, 2); row.addWidget(self.btn_apply, 1)
        v.addLayout(row)

        foot = QtWidgets.QLabel("Llindars orientatius (no validats clínicament)")
        foot.setStyleSheet(f"color: {C_TEXT_MUTED}; font-size: 9px; background: transparent;")
        v.addWidget(foot)

        self._proposal = None
        self._anchor_sampler = None
        self._anchor_medulla = None
        self._show_analysis(None)
        return card

    def _hud_block(self, title, value, unit):
        w = QtWidgets.QWidget()
        w.setStyleSheet("background: transparent;")
        l = QtWidgets.QVBoxLayout(w)
        l.setContentsMargins(0, 4, 0, 4)
        l.setSpacing(4)

        cap = QtWidgets.QLabel(title)
        cap.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_XS}px; font-weight: 600; letter-spacing: 0.8px;"
        )
        val = QtWidgets.QLabel(f"{value} {unit}")
        val.setStyleSheet(
            f"color: {C_TEXT}; background: transparent; "
            f"font-size: {SIZE_LG}px; font-weight: 700; font-family: {FONT_MONO};"
        )
        l.addWidget(cap)
        l.addWidget(val)
        return {"w": w, "val": val, "unit": unit}

    def _set_hud_value(self, block, value, color=None):
        if isinstance(value, str):
            text = f"{value} {block['unit']}"
        else:
            text = f"{value:.2f} {block['unit']}"
        block["val"].setText(text)
        if color:
            block["val"].setStyleSheet(
                f"color: {color}; background: transparent; "
                f"font-size: {SIZE_LG}px; font-weight: 700; "
                f"font-family: {FONT_MONO};"
            )

    def _build_porosity_lut(self):
        HU_MIN = 0
        HU_MAX = 1200

        lut = np.zeros((256, 4), dtype=np.uint8)

        for i in range(256):
            hu = HU_MIN + (HU_MAX - HU_MIN) * i / 255.0

            if hu < 100:
                r, g, b, a = 0, 0, 0, 0
            elif hu < 200:
                t = (hu - 100) / 100.0
                r, g, b = 255, int(30 + 60*t), 30
                a = 170
            elif hu < 400:
                t = (hu - 200) / 200.0
                r, g, b = 255, int(90 + 100*t), 20
                a = 160
            elif hu < 700:
                t = (hu - 400) / 300.0
                r = int(255 - 100*t)
                g = int(210 + 30*t)
                b = int(20 + 30*t)
                a = 150
            elif hu < 1000:
                t = (hu - 700) / 300.0
                r = int(155 - 100*t)
                g = int(240 - 20*t)
                b = int(50 + 40*t)
                a = 145
            else:
                r, g, b = 40, 200, 90
                a = 135

            lut[i] = (r, g, b, a)

        return lut

    def _hu_slice_to_rgba(self, hu_slice):
        if not hasattr(self, "_porosity_lut"):
            self._porosity_lut = self._build_porosity_lut()

        HU_MIN, HU_MAX = 0, 1200
        clipped = np.clip(hu_slice, HU_MIN, HU_MAX)
        idx = ((clipped - HU_MIN) / (HU_MAX - HU_MIN) * 255.0).astype(np.uint8)
        return self._porosity_lut[idx]

    def _update_porosity(self, name):
        view = {"ax": self.view_ax, "sag": self.view_sag,
                "cor": self.view_cor}[name]

        if not self.show_porosity:
            view.clear_density()
            return

        if name == "ax":
            hu_slice = self.vol_original[self.iz]
        elif name == "sag":
            hu_slice = self.vol_orig_flipped[:, :, self.ix]
        elif name == "cor":
            hu_slice = self.vol_orig_flipped[:, self.iy, :]
        else:
            return

        rgba = self._hu_slice_to_rgba(hu_slice)
        view.set_density(rgba)

    def _toggle_porosity(self):
        self.show_porosity = self.btn_porosity.isChecked()

        if self.show_porosity:
            self.btn_porosity.setStyleSheet(
                f"background-color: {C_WARN}; color: black; font-weight: bold;")
        else:
            self.btn_porosity.setStyleSheet("")

        for name in ("ax", "sag", "cor"):
            self._update_porosity(name)

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

        self._draw_screw_in_plane(idx, cg, color, selected, radi_mm,
            plane_normal=np.array([0., 0., 1.]),
            plane_value=self.iz * self.spacing[2],
            axis_unit=axis_unit, p0_mm=p0_mm, p1_mm=p1_mm,
            view=self.view_ax, view_axes=("x", "y"))

        self._draw_screw_in_plane(idx, cg, color, selected, radi_mm,
            plane_normal=np.array([1., 0., 0.]),
            plane_value=self.ix * self.spacing[0],
            axis_unit=axis_unit, p0_mm=p0_mm, p1_mm=p1_mm,
            view=self.view_sag, view_axes=("y", "z_flip"))

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
        axis_dir = p1_mm - p0_mm
        axis_len = float(np.linalg.norm(axis_dir))
        if axis_len < 1e-6:
            view.remove_screw(idx); return
        axis_unit = axis_dir / axis_len

        n_perp = plane_normal - float(np.dot(plane_normal, axis_unit)) * axis_unit
        n_perp_len = float(np.linalg.norm(n_perp))

        if n_perp_len < 1e-6:
            d_perp = 0.0
        else:
            d_perp = abs(
                (plane_value - float(np.dot(plane_normal, p0_mm))) / n_perp_len
            )

        if d_perp >= radi_mm:
            view.remove_screw(idx); return

        half_width_mm = math.sqrt(radi_mm * radi_mm - d_perp * d_perp)

        p0_view = self._map_3d_to_view(p0_mm, view_axes)
        p1_view = self._map_3d_to_view(p1_mm, view_axes)

        if view_axes[0] == "x":   px_per_mm = 1.0 / self.spacing[0]
        elif view_axes[0] == "y": px_per_mm = 1.0 / self.spacing[1]
        else:                     px_per_mm = 1.0 / self.spacing[2]

        head_diam_mm = radi_mm * 2.0 * 1.65
        hw_head_full_px = (head_diam_mm / 2.0) * px_per_mm

        hw_body_px = half_width_mm * px_per_mm

        visibility_ratio = half_width_mm / radi_mm if radi_mm > 0 else 1.0
        hw_head_px = hw_head_full_px * visibility_ratio

        body_len_px = math.hypot(p1_view[0] - p0_view[0],
                                   p1_view[1] - p0_view[1])
        rosca_pitch_mm = 3.0
        rosca_pitch_px = rosca_pitch_mm * px_per_mm
        n_threads = max(3, int(body_len_px / max(rosca_pitch_px, 1.0) * 0.75))
        n_threads = min(n_threads, 30)

        geom = {
            "p0_view": p0_view,
            "p1_view": p1_view,
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

        self.bone_3d = (grid_pd.contour([250])
                              .connectivity("largest")
                              .smooth(n_iter=6))

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
        tube = pv.Line(p0_mm, p1_mm).tube(radius=cg["diam"]/2.0)
        cg["actor"] = self.plotter.add_mesh(
            tube, color=cg["color"], name=f"cg_{idx}",
            smooth_shading=True, specular=0.4, specular_power=15,
            ambient=0.25, diffuse=0.75,
        )
        self.plotter.update()

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
        if not self.is_3d_loaded or not hasattr(self, "actor_bone_3d"):
            return
        xray_on = self.btn_xray.isChecked()
        opacity = self._bone_opacity_xray if xray_on else self._bone_opacity_solid
        try:
            prop = self.actor_bone_3d.GetProperty()
            prop.SetOpacity(opacity)
            prop.Modified()
            self.actor_bone_3d.Modified()

            if xray_on:
                self.btn_xray.setStyleSheet(
                    f"background-color: {C_WARN}; color: black; font-weight: bold;")
            else:
                self.btn_xray.setStyleSheet("")

            self.plotter.render()
        except Exception as e:
            print(f"[xray] error: {e}")

    def _change_diameter(self):
        self.current_diam = float(self.combo_diam.currentText().replace(" mm", ""))
        if self.selected_idx is not None:
            self.cargols[self.selected_idx]["diam"] = self.current_diam
            self._update_3d_screw(self.selected_idx)
            self._update_telemetry()
            self._draw_screw(self.selected_idx, self.cargols[self.selected_idx])
            item = self.list_widget.item(self.selected_idx)
            if item is not None:
                item.setText(self._cargol_label(self.selected_idx, self.cargols[self.selected_idx]))
            self._proposal = None
            self._analyze_selected()

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

    def _cargol_label(self, i, cg):
        return f"Cargol {i + 1}   ·   Ø {cg['diam']:.1f} mm   ·   Z={cg['z'] + 1}"

    def _delete_cargol(self):
        idx = self.selected_idx
        if idx is None or not (0 <= idx < len(self.cargols)):
            return
        ans = QtWidgets.QMessageBox.question(
            self, "Eliminar cargol", f"Eliminar el Cargol {idx + 1}?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if ans != QtWidgets.QMessageBox.Yes:
            return

        if self.is_3d_loaded:
            for cg in self.cargols:
                if cg.get("actor") is not None:
                    try:
                        self.plotter.remove_actor(cg["actor"])
                    except Exception:
                        pass
                    cg["actor"] = None
        del self.cargols[idx]
        self.selected_idx = None
        self._proposal = None

        self.list_widget.blockSignals(True)
        self.list_widget.clear()
        for i, cg in enumerate(self.cargols):
            item = QtWidgets.QListWidgetItem(self._cargol_label(i, cg))
            item.setForeground(QColor(cg["color"]))
            self.list_widget.addItem(item)
        self.list_widget.blockSignals(False)

        for i in range(len(self.cargols)):
            self._update_3d_screw(i)
        if self.is_3d_loaded:
            self.plotter.update()
        self._refresh_screws_all_views()
        self.btn_delete.setEnabled(False)
        self._show_analysis(None)
        self.cargolsChanged.emit(self.cargols)
        print(f"[planning] Cargol {idx + 1} eliminat · en queden {len(self.cargols)}")

    def _screw_mm(self, cg):
        sp, org = np.asarray(self.spacing), np.asarray(self.origin)
        p0 = org + np.array([cg["p0"][0], cg["p0"][1], cg["z_p0"]]) * sp
        p1 = org + np.array([cg["p1"][0], cg["p1"][1], cg["z_p1"]]) * sp
        return p0, p1

    def _anchor_tools(self):
        from src.planning.screw_analysis import CTSampler, MedullaDistance
        if self._anchor_sampler is None:
            self._anchor_sampler = CTSampler(self.vol_original, self.spacing, self.origin)
        if self._anchor_medulla is None:
            try:
                self._anchor_medulla = MedullaDistance(self._medula_fast)
            except Exception as e:
                print(f"[ancoratge] sense medul·la: {e}")
        return self._anchor_sampler, self._anchor_medulla

    def _analyze_selected(self):
        if self.selected_idx is None or not (0 <= self.selected_idx < len(self.cargols)):
            self._show_analysis(None)
            return None
        from src.planning.screw_analysis import analyze_screw
        cg = self.cargols[self.selected_idx]
        p0, p1 = self._screw_mm(cg)
        sampler, med = self._anchor_tools()
        try:
            res = analyze_screw(sampler, p0, p1, float(cg.get("diam", 5.5)), med)
        except Exception as e:
            print(f"[ancoratge] error: {e}")
            res = None
        self._show_analysis(res)
        return res

    def _show_analysis(self, res, proposal_text=None):
        from src.planning.screw_analysis import HU_LOW, HU_OK, MED_MIN_MM, MED_OK_MM
        rows = self._anchor_rows
        def setv(key, text, color=C_TEXT):
            rows[key].setText(text)
            rows[key].setStyleSheet(f"color: {color}; font-family: {FONT_MONO}; font-size: {SIZE_XS}px; "
                                    f"font-weight: 700; background: transparent;")
        self.density_profile.set_result(res)
        has = res is not None
        self.btn_optimize.setEnabled(has)
        self.btn_apply.setVisible(self._proposal is not None)
        if not has:
            for k in rows:
                setv(k, "—", C_TEXT_MUTED)
            self.lbl_verdict.setText("—")
            self.lbl_verdict.setStyleSheet(f"color: {C_TEXT_MUTED}; background: transparent;")
            self.lbl_anchor_notes.setText("")
            return

        hu = res["anchor_hu"]
        setv("hu", f"{hu:.0f} HU", C_SAFE if hu >= HU_OK else (C_WARN if hu >= HU_LOW else C_DANGER))
        if res["breaches"]:
            a, b = res["breaches"][0][0], res["breaches"][-1][1]
            canal = any(x[2] for x in res["breaches"])
            setv("breach", f"{a:.0f}–{b:.0f} mm" + (" · canal" if canal else ""), C_DANGER)
        else:
            setv("breach", "cap", C_SAFE)
        md = res["med_dist_mm"]
        if md is None:
            setv("med", "—", C_TEXT_MUTED)
        else:
            setv("med", "toca" if md <= 0 else f"{md:.1f} mm",
                 C_DANGER if md < MED_MIN_MM else (C_WARN if md < MED_OK_MM else C_SAFE))
        if res.get("isthmus_mm") is not None:
            setv("isth", f"{res['isthmus_mm']:.0f} mm · Ø {res['max_diam_mm']:.1f}")
        else:
            setv("isth", "—", C_TEXT_MUTED)
        rec = res.get("recommended_diam")
        cur = float(self.cargols[self.selected_idx].get("diam", 5.5))
        setv("diam", "cap Ø estàndard" if rec is None else f"{rec:.1f} mm",
             C_DANGER if rec is None else (C_SAFE if rec >= cur else C_WARN))
        col = {"ok": C_SAFE, "warn": C_WARN, "danger": C_DANGER}[res["verdict"]]
        txt = {"ok": "● BO", "warn": "● ATENCIÓ", "danger": "● RISC"}[res["verdict"]]
        self.lbl_verdict.setText(txt)
        self.lbl_verdict.setStyleSheet(f"color: {col}; font-size: {SIZE_XS}px; font-weight: 700; "
                                       f"letter-spacing: 1px; background: transparent;")
        notes = list(res["notes"])
        if proposal_text:
            notes.insert(0, proposal_text)
        self.lbl_anchor_notes.setText("\n".join("· " + n for n in notes) if notes else "· Sense alertes")

    def _optimize_selected(self):
        if self.selected_idx is None:
            return
        from src.planning.screw_analysis import optimize_trajectory
        idx = self.selected_idx
        cg = self.cargols[idx]
        p0, p1 = self._screw_mm(cg)
        sampler, med = self._anchor_tools()
        QtWidgets.QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            opt = optimize_trajectory(sampler, p0, p1, float(cg.get("diam", 5.5)), med)
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        if opt is None:
            self._proposal = None
            res = self._analyze_selected()
            self._show_analysis(res, "Optimització: cap trajectòria millor dins ±6°")
            return
        p1n, rn, gain, ang = opt
        self._proposal = (idx, p1n)
        md = rn["med_dist_mm"]
        txt = (f"PROPOSTA: {ang:.1f}° · {rn['anchor_hu']:.0f} HU ({gain:+.0f} %)"
               + (f" · medul·la {md:.1f} mm" if md is not None else "")
               + (" · sense perforar" if not rn["breaches"] else "") + " → Aplicar")
        self._show_analysis(rn, txt)
        print(f"[ancoratge] {txt}")

    def _apply_proposal(self):
        if self._proposal is None:
            return
        idx, p1n = self._proposal
        self._proposal = None
        if idx >= len(self.cargols):
            return
        sp, org = np.asarray(self.spacing), np.asarray(self.origin)
        ix, iy, iz = (np.asarray(p1n) - org) / sp
        cg = self.cargols[idx]
        cg["p1"] = (float(ix), float(iy))
        cg["z_p1"] = int(np.clip(round(iz), 0, self.Z - 1))
        self._draw_screw(idx, cg)
        self._update_3d_screw(idx)
        self._update_telemetry()
        self._analyze_selected()
        self.cargolsChanged.emit(self.cargols)

    def _select_from_list(self):
        idx = self.list_widget.currentRow()
        if idx < 0:
            self.btn_delete.setEnabled(False)
            self._proposal = None
            self._show_analysis(None)
        if idx >= 0:
            self.btn_delete.setEnabled(True)
            if self._proposal is not None and self._proposal[0] != idx:
                self._proposal = None
            self.selected_idx = idx
            self.iz = self.cargols[idx]["z"]
            try:
                self.combo_diam.setCurrentIndex(DIAMETRES_STD.index(self.cargols[idx]["diam"]))
            except ValueError:
                pass
            self._update_telemetry()
            self._update_view("ax"); self._update_view("sag"); self._update_view("cor")
            self._refresh_screws_all_views()
            self._analyze_selected()

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
        self._proposal = None
        self._analyze_selected()

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

    def cleanup(self):
        try:
            self.plotter.close()
        except Exception:
            pass

