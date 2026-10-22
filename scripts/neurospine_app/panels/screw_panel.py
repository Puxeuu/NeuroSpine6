from qtpy import QtWidgets, QtCore
from qtpy.QtGui import QColor

from ..theme import (
    C_BG_WINDOW, C_PANEL, C_BORDER, C_TEXT, C_TEXT_DIM,
    C_ACCENT, C_SAFE, SIZE_XS, SIZE_SM,
)

TRAJECTORY_MAX_MM  = 60
TRAJECTORY_STEP_MM = 0.2
_STEPS_PER_MM      = round(1.0 / TRAJECTORY_STEP_MM)


class ScrewPanel(QtWidgets.QFrame):
    showVectorRequested    = QtCore.Signal(int)
    goApproachRequested    = QtCore.Signal(int)
    selectionChanged       = QtCore.Signal(int)
    trajectoryDepthChanged = QtCore.Signal(int, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 12px;
            }}
        """)

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(14, 12, 14, 12)
        v.setSpacing(8)

        title = QtWidgets.QLabel("CARGOLS PLANIFICATS")
        title.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;")
        v.addWidget(title)

        self.list_widget = QtWidgets.QListWidget()
        self.list_widget.setStyleSheet(f"""
            QListWidget {{
                background-color: {C_BG_WINDOW};
                border: 1px solid {C_BORDER};
                border-radius: 6px;
                color: {C_TEXT};
                font-size: {SIZE_SM}px;
                padding: 4px;
            }}
            QListWidget::item {{
                padding: 6px 8px;
                border-radius: 4px;
            }}
            QListWidget::item:selected {{
                background-color: {C_ACCENT};
                color: white;
            }}
        """)
        self.list_widget.itemSelectionChanged.connect(self._on_selection)
        v.addWidget(self.list_widget, 1)

        btn_row = QtWidgets.QVBoxLayout()
        btn_row.setSpacing(6)

        self.btn_show = QtWidgets.QPushButton("Mostrar vector")
        self.btn_show.clicked.connect(self._on_show)
        btn_row.addWidget(self.btn_show)

        self.btn_go = QtWidgets.QPushButton("Anar a approach (1 cm)")
        self.btn_go.setStyleSheet(f"""
            QPushButton {{
                background-color: {C_SAFE};
                color: white;
                font-weight: 700;
                border: none;
                border-radius: 6px;
                padding: 8px;
            }}
            QPushButton:hover {{ background-color: #48c774; }}
            QPushButton:pressed {{ background-color: #3aa060; }}
            QPushButton:disabled {{
                background-color: #3a3a3c;
                color: {C_TEXT_DIM};
            }}
        """)
        self.btn_go.clicked.connect(self._on_go)
        btn_row.addWidget(self.btn_go)

        v.addLayout(btn_row)

        self.depth_label = QtWidgets.QLabel("AVANÇ · 0.0 mm")
        self.depth_label.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;")

        self.depth_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.depth_slider.setMinimum(0)
        self.depth_slider.setMaximum(TRAJECTORY_MAX_MM * _STEPS_PER_MM)
        self.depth_slider.setValue(0)
        self.depth_slider.setSingleStep(1)
        self.depth_slider.setPageStep(_STEPS_PER_MM)
        self.depth_slider.setTickInterval(10 * _STEPS_PER_MM)
        self.depth_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.depth_slider.valueChanged.connect(self._on_depth_changed)

        v.addWidget(self.depth_label)
        v.addWidget(self.depth_slider)

        self.depth_label.hide()
        self.depth_slider.hide()
        self._trajectory_visible_wanted = False

        self._screws = []
        self._update_buttons()

    def set_screws(self, screws):
        self._screws = list(screws) if screws else []
        self.list_widget.clear()
        for cg in self._screws:
            L_mm = cg["length_m"] * 1000.0
            D_mm = cg["diam_m"] * 1000.0
            text = f"  {cg['name']}   ·   Ø{D_mm:.1f} × {L_mm:.0f} mm"
            item = QtWidgets.QListWidgetItem(text)
            try:
                item.setForeground(QColor(cg["color"]))
            except Exception:
                pass
            self.list_widget.addItem(item)
        self._update_buttons()

    def _on_selection(self):
        idx = self.list_widget.currentRow()
        self._reset_depth_slider()
        self.selectionChanged.emit(idx)
        self._update_buttons()
        self._apply_trajectory_visibility()

    def _on_show(self):
        idx = self.list_widget.currentRow()
        if idx >= 0:
            self.showVectorRequested.emit(idx)

    def _on_go(self):
        idx = self.list_widget.currentRow()
        if idx >= 0:
            self._reset_depth_slider()
            self.goApproachRequested.emit(idx)

    def _on_depth_changed(self, value):
        mm = value / _STEPS_PER_MM
        self.depth_label.setText(f"AVANÇ · {mm:.1f} mm")
        idx = self.list_widget.currentRow()
        if idx >= 0:
            self.trajectoryDepthChanged.emit(idx, float(mm))

    def _reset_depth_slider(self):
        self.depth_slider.blockSignals(True)
        self.depth_slider.setValue(0)
        self.depth_slider.blockSignals(False)
        self.depth_label.setText("AVANÇ · 0.0 mm")

    def set_trajectory_slider_visible(self, visible: bool):
        self._trajectory_visible_wanted = bool(visible)
        if not visible:
            self._reset_depth_slider()
        self._apply_trajectory_visibility()

    def _apply_trajectory_visibility(self):
        has_sel = self.list_widget.currentRow() >= 0
        show = self._trajectory_visible_wanted and has_sel
        self.depth_label.setVisible(show)
        self.depth_slider.setVisible(show)

    def _update_buttons(self):
        has_sel = self.list_widget.currentRow() >= 0
        self.btn_show.setEnabled(has_sel)
        self.btn_go.setEnabled(has_sel)

