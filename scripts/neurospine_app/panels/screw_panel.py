"""
Panel lateral amb la llista de cargols planificats i els botons
"Mostrar vector" i "Anar a approach". S'utilitza al robot workspace un cop
s'ha aplicat el registre CT ↔ robot.
"""

from qtpy import QtWidgets, QtCore                                              # importem QtWidgets, QtCore des de qtpy
from qtpy.QtGui import QColor                                                   # importem QColor des de qtpy.QtGui

from ..theme import (                                                           # importem ( des de ..theme
    C_BG_WINDOW, C_PANEL, C_BORDER, C_TEXT, C_TEXT_DIM,
    C_ACCENT, C_SAFE, SIZE_XS, SIZE_SM,
)

# Slider de profunditat (avançar la broca al llarg del vector des de l'approach)
TRAJECTORY_MAX_MM  = 60                                                         # recorregut màxim des de l'approach (abans 50)
TRAJECTORY_STEP_MM = 0.2                                                        # mida de cada pas (abans 1 mm → 5× més fi)
_STEPS_PER_MM      = round(1.0 / TRAJECTORY_STEP_MM)                            # l'slider treballa en passos enters


class ScrewPanel(QtWidgets.QFrame):                                             # definim la classe ScrewPanel
    """
    Signals:
        showVectorRequested(int)        → l'usuari vol veure el vector del cargol idx
        goApproachRequested(int)        → l'usuari vol que el robot vagi a l'approach
        selectionChanged(int)           → nou cargol seleccionat (-1 = cap)
        trajectoryDepthChanged(int,float) → (idx, depth_mm) l'usuari mou l'slider
                                          de profunditat (0..TRAJECTORY_MAX_MM,
                                          en passos de TRAJECTORY_STEP_MM)
    """

    showVectorRequested    = QtCore.Signal(int)                                 # showVectorRequested ← resultat de Signal()
    goApproachRequested    = QtCore.Signal(int)                                 # goApproachRequested ← resultat de Signal()
    selectionChanged       = QtCore.Signal(int)                                 # selectionChanged ← resultat de Signal()
    trajectoryDepthChanged = QtCore.Signal(int, float)                          # (idx, depth_mm)

    def __init__(self, parent=None):                                            # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 12px;
            }}
        """)

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(14, 12, 14, 12)                                    # executem setContentsMargins()
        v.setSpacing(8)                                                         # executem setSpacing()

        title = QtWidgets.QLabel("CARGOLS PLANIFICATS")                         # title ← resultat de QLabel()
        title.setStyleSheet(                                                    # executem setStyleSheet()
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

        # Botons d'acció
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

        # ---- Slider de profunditat (avançar cap a la vèrtebra) ----
        # Aquesta secció només és útil en mode IK5: el mode IK6 t'ha portat
        # a l'approach, i llavors amb l'IK5 vas avançant amb l'slider.
        self.depth_label = QtWidgets.QLabel("AVANÇ · 0.0 mm")                   # etiqueta amb el valor actual
        self.depth_label.setStyleSheet(                                         # executem setStyleSheet()
            f"color: {C_TEXT_DIM}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;")

        self.depth_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)             # slider horitzontal
        self.depth_slider.setMinimum(0)                                         # 0 mm (approach)
        self.depth_slider.setMaximum(TRAJECTORY_MAX_MM * _STEPS_PER_MM)         # fins a X mm, en passos fins
        self.depth_slider.setValue(0)                                           # inicialment a 0
        self.depth_slider.setSingleStep(1)                                      # fletxes: 1 pas (0.2 mm)
        self.depth_slider.setPageStep(_STEPS_PER_MM)                            # RePàg/AvPàg: 1 mm
        self.depth_slider.setTickInterval(10 * _STEPS_PER_MM)                   # marca cada 10 mm
        self.depth_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)         # ticks sota
        self.depth_slider.valueChanged.connect(self._on_depth_changed)          # callback

        v.addWidget(self.depth_label)                                           # afegim al layout
        v.addWidget(self.depth_slider)                                          # afegim al layout

        # Per defecte l'slider està amagat: es mostra només en mode IK5
        # amb un cargol seleccionat. El robot workspace crida
        # set_trajectory_slider_visible(True/False) quan canvia de mode.
        self.depth_label.hide()                                                 # amagat per defecte
        self.depth_slider.hide()                                                # amagat per defecte
        self._trajectory_visible_wanted = False                                 # flag intern

        self._screws = []
        self._update_buttons()

    # ------------------------------------------------------------------
    def set_screws(self, screws):
        """Refresca la llista amb els cargols passats (com al pas 2)."""
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
        # Reset de l'slider en canviar de cargol
        self._reset_depth_slider()
        self.selectionChanged.emit(idx)
        self._update_buttons()
        # Refresquem visibilitat de l'slider (només si hi ha selecció)
        self._apply_trajectory_visibility()

    def _on_show(self):
        idx = self.list_widget.currentRow()
        if idx >= 0:
            self.showVectorRequested.emit(idx)

    def _on_go(self):
        idx = self.list_widget.currentRow()
        if idx >= 0:
            # En anar a l'approach, reset de l'slider
            self._reset_depth_slider()
            self.goApproachRequested.emit(idx)

    def _on_depth_changed(self, value):                                         # definim la funció _on_depth_changed()
        """Signal quan es mou l'slider de profunditat."""
        mm = value / _STEPS_PER_MM                                              # passos → mm
        self.depth_label.setText(f"AVANÇ · {mm:.1f} mm")                        # actualitzem etiqueta
        idx = self.list_widget.currentRow()                                     # cargol actiu
        if idx >= 0:                                                            # només si hi ha selecció
            self.trajectoryDepthChanged.emit(idx, float(mm))                    # emetem signal

    def _reset_depth_slider(self):                                              # definim la funció _reset_depth_slider()
        """Torna l'slider a 0 sense emetre signal."""
        self.depth_slider.blockSignals(True)                                    # bloquegem signals
        self.depth_slider.setValue(0)                                           # reset
        self.depth_slider.blockSignals(False)                                   # desbloquegem
        self.depth_label.setText("AVANÇ · 0.0 mm")                              # etiqueta

    def set_trajectory_slider_visible(self, visible: bool):                     # definim la funció set_trajectory_slider_visible()
        """
        Diu al panell si l'slider de trajectòria és visible o no.
        L'ha de cridar el workspace quan canvia de mode:
            IK5 → visible=True
            altres → visible=False
        """
        self._trajectory_visible_wanted = bool(visible)
        # Si s'amaga, resetem l'slider per no deixar un valor pendent
        if not visible:
            self._reset_depth_slider()
        self._apply_trajectory_visibility()

    def _apply_trajectory_visibility(self):                                     # definim la funció _apply_trajectory_visibility()
        """Aplica la visibilitat efectiva: només si hi ha selecció + volem visible."""
        has_sel = self.list_widget.currentRow() >= 0
        show = self._trajectory_visible_wanted and has_sel
        self.depth_label.setVisible(show)                                       # etiqueta
        self.depth_slider.setVisible(show)                                      # slider

    def _update_buttons(self):
        has_sel = self.list_widget.currentRow() >= 0
        self.btn_show.setEnabled(has_sel)
        self.btn_go.setEnabled(has_sel)