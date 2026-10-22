import os
import numpy as np

import pyvista as pv
from pyvistaqt import QtInteractor
import vtk

from qtpy import QtWidgets, QtCore
from qtpy.QtCore import Qt

from ..theme import (
    C_BG_WINDOW, C_PANEL_ALT, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_ACCENT,
    C_SAFE, C_WARN, C_DANGER,
    SIZE_XS, SIZE_SM, FONT_MONO,
    style_pill,
)
from ..registration.state import RegistrationState, MIN_POINTS, MultiVertebraRegistry


_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))

VERTEBRA_PREFIX = "Vertebra_"
VERTEBRA_SUFFIX = ".stl"

vtk.vtkObject.GlobalWarningDisplayOff()
os.environ["VTK_SILENCE_GET_WINDOW_INFO_ERRORS"] = "1"


COLOR_PENDENT   = "#f0ad4e"
COLOR_CAPTURAT  = "#5cb85c"
COLOR_ACTIU     = "#0a84ff"


class RegistrationWorkspace(QtWidgets.QWidget):
    registrationUpdated = QtCore.Signal(bool)

    def __init__(self, pacient_dir=None, robot_workspace=None, parent=None):
        super().__init__(parent)
        self.pacient_dir = pacient_dir
        self.robot_workspace = robot_workspace

        self.vertebres = {}
        self.registry = MultiVertebraRegistry(self._registry_json_path())
        self.registry.load()
        self.state = None

        self.current_phase = 1
        self.active_idx = None

        self.vertebra_mesh = None
        self._landmark_actors = {}
        self._landmark_label_actors = {}
        self._vertebra_actor = None

        self._picker = None

        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        self._discover_vertebres()
        initial_code = self._initial_vertebra_code()
        if initial_code is not None:
            self.state = self.registry.select(initial_code)
            self._load_active_vertebra()
        else:
            self.state = RegistrationState()

        self._build_ui()
        self._show_phase(1)

        self._tcp_timer = QtCore.QTimer(self)
        self._tcp_timer.timeout.connect(self._refresh_tcp_display)
        self._tcp_timer.start(200)

    def _vertebres_dir(self):
        if not self.pacient_dir:
            return None
        return os.path.join(self.pacient_dir, "vertebres_meshes")

    def _registry_json_path(self):
        if not self.pacient_dir:
            return None
        return os.path.join(self.pacient_dir, "registres.json")

    def _discover_vertebres(self):
        self.vertebres = {}
        folder = self._vertebres_dir()
        if not folder or not os.path.isdir(folder):
            print(f"[registration] carpeta de vèrtebres no trobada: {folder}")
            return
        for fname in sorted(os.listdir(folder)):
            if not fname.startswith(VERTEBRA_PREFIX):
                continue
            if not fname.lower().endswith(VERTEBRA_SUFFIX):
                continue
            code = fname[len(VERTEBRA_PREFIX):-len(VERTEBRA_SUFFIX)]
            if not code:
                continue
            self.vertebres[code] = os.path.join(folder, fname)
        print(f"[registration] vèrtebres descobertes: {list(self.vertebres.keys())}")

    def _initial_vertebra_code(self):
        active = self.registry.active_name()
        if active and active in self.vertebres:
            return active
        if self.vertebres:
            return sorted(self.vertebres.keys())[0]
        return None

    def _load_active_vertebra(self):
        name = self.registry.active_name()
        self.vertebra_mesh = None
        if name is None or name not in self.vertebres:
            return
        stl_path = self.vertebres[name]
        try:
            mesh = pv.read(stl_path).clean().triangulate()
            self.vertebra_mesh = mesh
            print(f"[registration] Vèrtebra '{name}' carregada: "
                  f"{mesh.n_points} vèrtexs, {mesh.n_cells} triangles")
        except Exception as e:
            print(f"[registration] Error carregant STL {stl_path}: {e}")

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(10)

        outer.addWidget(self._build_vertebra_selector_bar())

        outer.addWidget(self._build_phase_bar())

        self.stack = QtWidgets.QStackedWidget()
        self.stack.addWidget(self._build_phase1())
        self.stack.addWidget(self._build_phase2())
        self.stack.addWidget(self._build_phase3())
        outer.addWidget(self.stack, 1)

    def _build_vertebra_selector_bar(self):
        bar = QtWidgets.QFrame()
        bar.setObjectName("Panel")
        bar.setFixedHeight(60)
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(16, 10, 16, 10)
        h.setSpacing(10)

        lbl = QtWidgets.QLabel("VÈRTEBRA ACTIVA")
        lbl.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;")
        h.addWidget(lbl)

        self.vertebra_combo = QtWidgets.QComboBox()
        self.vertebra_combo.setMinimumWidth(260)
        self._refresh_vertebra_combo()
        self.vertebra_combo.currentIndexChanged.connect(self._on_vertebra_selected)
        h.addWidget(self.vertebra_combo)

        self.vertebra_status_lbl = QtWidgets.QLabel("")
        self.vertebra_status_lbl.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_XS}px; font-family: {FONT_MONO};")
        h.addWidget(self.vertebra_status_lbl)

        h.addStretch()

        self.btn_save_all = QtWidgets.QPushButton("Desa tot al disc")
        self.btn_save_all.clicked.connect(self._on_save_registry)
        h.addWidget(self.btn_save_all)

        self.btn_reset_current = QtWidgets.QPushButton("Reinicia aquesta")
        self.btn_reset_current.clicked.connect(self._on_reset_current_vertebra)
        h.addWidget(self.btn_reset_current)

        self._refresh_vertebra_status_label()
        return bar

    def _refresh_vertebra_combo(self):
        self.vertebra_combo.blockSignals(True)
        self.vertebra_combo.clear()
        if not self.vertebres:
            self.vertebra_combo.addItem("(cap vèrtebra a vertebres_meshes/)")
            self.vertebra_combo.setEnabled(False)
        else:
            self.vertebra_combo.setEnabled(True)
            active = self.registry.active_name()
            for i, code in enumerate(sorted(self.vertebres.keys())):
                mark = self._vertebra_status_icon(code)
                self.vertebra_combo.addItem(f"{mark}  {code}", code)
                if code == active:
                    self.vertebra_combo.setCurrentIndex(i)
        self.vertebra_combo.blockSignals(False)

    def _vertebra_status_icon(self, code):
        if code not in self.registry:
            return "·"
        st = self.registry[code]
        if st.has_valid_matrix():
            return "✓"
        if st.n_landmarks() > 0 or st.n_captured() > 0:
            return "○"
        return "·"

    def _refresh_vertebra_status_label(self):
        total = len(self.vertebres)
        done = len([c for c in self.vertebres if c in self.registry
                    and self.registry[c].has_valid_matrix()])
        if total == 0:
            self.vertebra_status_lbl.setText("")
        else:
            self.vertebra_status_lbl.setText(f"{done} / {total} registrades")

    def _on_vertebra_selected(self, idx):
        code = self.vertebra_combo.itemData(idx)
        if not code:
            return
        if code == self.registry.active_name():
            return
        self.state = self.registry.select(code)
        self._load_active_vertebra()
        self.active_idx = None
        self._rebuild_vertebra_scene()
        self._show_phase(1)
        self._refresh_vertebra_status_label()

    def _rebuild_vertebra_scene(self):
        for actor_dict in (self._landmark_actors, self._landmark_label_actors):
            for idx in list(actor_dict.keys()):
                try:
                    self.plotter.remove_actor(actor_dict[idx])
                except Exception:
                    pass
            actor_dict.clear()
        if self._vertebra_actor is not None:
            try:
                self.plotter.remove_actor(self._vertebra_actor)
            except Exception:
                pass
            self._vertebra_actor = None
        try:
            self.plotter.disable_picking()
        except Exception:
            pass
        self._setup_3d_scene()
        if self.state is not None:
            for idx in range(self.state.n_landmarks()):
                self._draw_landmark_3d(idx)
        try:
            self.plotter.render()
        except Exception:
            pass

        if hasattr(self, "plotter_p2"):
            self._rebuild_p2_scene()

    def _rebuild_p2_scene(self):
        if not hasattr(self, "plotter_p2"):
            return
        if self._p2_actors.get("vertebra") is not None:
            try:
                self.plotter_p2.remove_actor(self._p2_actors["vertebra"])
            except Exception:
                pass
            self._p2_actors["vertebra"] = None
        for d in ("landmarks", "labels"):
            for idx, actor in list(self._p2_actors[d].items()):
                try:
                    self.plotter_p2.remove_actor(actor)
                except Exception:
                    pass
            self._p2_actors[d].clear()
        try:
            self.plotter_p2.remove_all_lights()
        except Exception:
            pass
        self._setup_p2_scene()
        if self.state is not None and self.state.n_landmarks() > 0:
            self._p2_redraw_landmarks()
        try:
            self.plotter_p2.render()
        except Exception:
            pass

    def _on_save_registry(self):
        try:
            path = self.registry.save()
            QtWidgets.QMessageBox.information(
                self, "Registres desats",
                f"S'han desat {len(self.registry.names())} registres a:\n{path}"
            )
        except Exception as e:
            QtWidgets.QMessageBox.warning(
                self, "Error desant", f"No s'ha pogut desar: {e}"
            )

    def _on_reset_current_vertebra(self):
        if self.state is None:
            return
        name = self.registry.active_name()
        reply = QtWidgets.QMessageBox.question(
            self, "Reiniciar registre",
            f"Esborrar tots els landmarks, captures i matriu T de '{name}'?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self.registry._states[name] = RegistrationState()
        self.state = self.registry._states[name]
        self.active_idx = None
        self._rebuild_vertebra_scene()
        self._show_phase(1)
        self._refresh_vertebra_combo()
        self._refresh_vertebra_status_label()

    def _build_phase_bar(self):
        bar = QtWidgets.QFrame()
        bar.setObjectName("Panel")
        bar.setFixedHeight(70)
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(16, 10, 16, 10)
        h.setSpacing(10)

        self.phase_pills = []
        for i, label in enumerate(
                ("1. MARCAR SOBRE STL 3D",
                 "2. PALPAR AMB ROBOT",
                 "3. CALIBRAR (KABSCH + ICP)")):
            pill = QtWidgets.QLabel(label)
            pill.setAlignment(Qt.AlignCenter)
            pill.setMinimumWidth(200)
            pill.setStyleSheet(style_pill(C_TEXT_DIM))
            h.addWidget(pill)
            self.phase_pills.append(pill)

        h.addStretch()

        self.btn_prev = QtWidgets.QPushButton("← Anterior")
        self.btn_next = QtWidgets.QPushButton("Següent →")
        self.btn_prev.clicked.connect(self._on_prev_phase)
        self.btn_next.clicked.connect(self._on_next_phase)
        h.addWidget(self.btn_prev)
        h.addWidget(self.btn_next)

        return bar

    def _build_phase1(self):
        wrap = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(wrap)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(10)

        left = QtWidgets.QFrame()
        left.setObjectName("Panel")
        left.setFixedWidth(300)
        lv = QtWidgets.QVBoxLayout(left)
        lv.setContentsMargins(16, 16, 16, 16)
        lv.setSpacing(10)

        title = QtWidgets.QLabel(f"LANDMARKS (mínim {MIN_POINTS})")
        title.setObjectName("SectionTitle")
        lv.addWidget(title)

        self.p1_progress = QtWidgets.QLabel(f"0 / {MIN_POINTS} marcats")
        self.p1_progress.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 0.6px;")
        lv.addWidget(self.p1_progress)

        self.p1_list = QtWidgets.QListWidget()
        self.p1_list.itemSelectionChanged.connect(self._p1_on_list_selection)
        lv.addWidget(self.p1_list, 1)

        instr = QtWidgets.QLabel(
            "Clica sobre la superfície de la vèrtebra per marcar cada "
            "landmark. Ha de ser una zona òssia accessible amb la punta del "
            "TCP (apòfisis, làmines, cortical dorsal).\n\n"
            "Distribueix els punts al voltant de la vèrtebra: eviteu-ne 8 "
            "en línia recta. Rota amb el ratolí per veure totes les cares."
        )
        instr.setWordWrap(True)
        instr.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_XS}px; padding: 6px 4px; line-height: 145%;")
        lv.addWidget(instr)

        self.p1_btn_delete = QtWidgets.QPushButton("Esborrar seleccionat")
        self.p1_btn_delete.clicked.connect(self._p1_on_delete)
        lv.addWidget(self.p1_btn_delete)

        self.p1_btn_reset = QtWidgets.QPushButton("Reiniciar tots")
        self.p1_btn_reset.clicked.connect(self._p1_on_reset)
        lv.addWidget(self.p1_btn_reset)

        h.addWidget(left)

        center = QtWidgets.QFrame()
        center.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_WINDOW};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)
        cv = QtWidgets.QVBoxLayout(center)
        cv.setContentsMargins(10, 8, 10, 10)
        cv.setSpacing(6)

        header = QtWidgets.QLabel("VÈRTEBRA VIRTUAL — clic per marcar landmark")
        header.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;")
        cv.addWidget(header)

        self.plotter = QtInteractor(center)
        self.plotter.set_background(C_BG_WINDOW)
        cv.addWidget(self.plotter.interactor, 1)

        h.addWidget(center, 1)

        self._setup_3d_scene()

        return wrap

    def _setup_3d_scene(self):
        if self.vertebra_mesh is None:
            self.plotter.add_text(
                "ERROR: no s'ha trobat la vèrtebra",
                position="upper_edge", font_size=14, color="red",
            )
            return

        try:
            self.plotter.remove_all_lights()
        except Exception:
            pass

        self.plotter.add_light(pv.Light(light_type="headlight",
                                         color="white", intensity=0.55))

        self._vertebra_actor = self.plotter.add_mesh(
            self.vertebra_mesh,
            color=(207, 200, 188),
            opacity=1.0,
            smooth_shading=True,
            specular=0.0,
            specular_power=1,
            ambient=0.65,
            diffuse=0.45,
            show_edges=False,
            pickable=True,
        )

        self.plotter.reset_camera()

        self.plotter.enable_point_picking(
            callback=self._on_surface_click,
            picker="cell",
            show_message=False,
            show_point=False,
            left_clicking=False,
            tolerance=0.01,
            font_size=12,
        )

    def _on_surface_click(self, point):
        if point is None:
            return

        x, y, z = float(point[0]), float(point[1]), float(point[2])
        idx = self.state.add_virtual_landmark(x, y, z)
        self.active_idx = idx

        self._draw_landmark_3d(idx)
        self._p1_refresh_list()
        self._update_next_button()
        self.plotter.render()

    def _draw_landmark_3d(self, idx):
        if idx >= len(self.state.landmarks):
            return
        lm = self.state.landmarks[idx]

        if idx in self._landmark_actors:
            try:
                self.plotter.remove_actor(self._landmark_actors[idx])
            except Exception:
                pass
        if idx in self._landmark_label_actors:
            try:
                self.plotter.remove_actor(self._landmark_label_actors[idx])
            except Exception:
                pass

        if idx == self.active_idx:
            color = COLOR_ACTIU
            radius_scale = 1.4
        elif lm.is_captured:
            color = COLOR_CAPTURAT
            radius_scale = 1.0
        else:
            color = COLOR_PENDENT
            radius_scale = 1.0

        radius_mm = 1.2 * radius_scale
        sphere = pv.Sphere(radius=radius_mm, center=lm.virtual)
        actor = self.plotter.add_mesh(
            sphere, color=color, opacity=0.95,
            smooth_shading=True, specular=0.3,
            name=f"lm_sphere_{idx}",
            pickable=False,
        )
        self._landmark_actors[idx] = actor

        label_actor = self.plotter.add_point_labels(
            np.array([lm.virtual]),
            [lm.name],
            font_size=14,
            point_size=1,
            point_color=color,
            text_color="white",
            shape_color=color,
            shape_opacity=0.85,
            always_visible=True,
            name=f"lm_label_{idx}",
        )
        self._landmark_label_actors[idx] = label_actor

    def _redraw_all_landmarks(self):
        for actor_dict in (self._landmark_actors, self._landmark_label_actors):
            for idx in list(actor_dict.keys()):
                try:
                    self.plotter.remove_actor(actor_dict[idx])
                except Exception:
                    pass
            actor_dict.clear()

        for idx in range(len(self.state.landmarks)):
            self._draw_landmark_3d(idx)

        self.plotter.render()

    def _p1_refresh_list(self):
        self.p1_list.blockSignals(True)
        self.p1_list.clear()
        for lm in self.state.landmarks:
            v = lm.virtual
            item = QtWidgets.QListWidgetItem(
                f"  {lm.name}    ({v[0]:+7.1f}, {v[1]:+7.1f}, {v[2]:+7.1f})")
            self.p1_list.addItem(item)
            if lm.idx == self.active_idx:
                self.p1_list.setCurrentItem(item)
        self.p1_list.blockSignals(False)

        n = self.state.n_landmarks()
        color = C_SAFE if n >= MIN_POINTS else C_TEXT_DIM
        self.p1_progress.setStyleSheet(
            f"color: {color}; background: transparent; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 0.6px;")
        self.p1_progress.setText(f"{n} / {MIN_POINTS} marcats" +
                                  ("  OK" if n >= MIN_POINTS else ""))

    def _p1_on_list_selection(self):
        row = self.p1_list.currentRow()
        if row < 0 or row >= len(self.state.landmarks):
            return
        self.active_idx = self.state.landmarks[row].idx
        self._redraw_all_landmarks()

    def _p1_on_delete(self):
        row = self.p1_list.currentRow()
        if row < 0:
            return
        self.state.remove_landmark(row)
        n = self.state.n_landmarks()
        if n == 0:
            self.active_idx = None
        else:
            self.active_idx = min(row, n - 1)
        self._redraw_all_landmarks()
        self._p1_refresh_list()
        self._update_next_button()

    def _p1_on_reset(self):
        resp = QtWidgets.QMessageBox.question(
            self, "Reiniciar",
            "Esborrar TOTS els landmarks?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        if resp != QtWidgets.QMessageBox.Yes:
            return
        self.state.clear_all_landmarks()
        self.active_idx = None
        self._redraw_all_landmarks()
        self._p1_refresh_list()
        self._update_next_button()

    def _build_phase2(self):
        wrap = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(wrap)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(10)

        left = QtWidgets.QFrame()
        left.setObjectName("Panel")
        left.setFixedWidth(280)
        lv = QtWidgets.QVBoxLayout(left)
        lv.setContentsMargins(16, 16, 16, 16)
        lv.setSpacing(10)

        title = QtWidgets.QLabel("PALPAR LANDMARKS")
        title.setObjectName("SectionTitle")
        lv.addWidget(title)

        self.p2_progress = QtWidgets.QLabel("0 / 0 capturats")
        self.p2_progress.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 0.6px;")
        lv.addWidget(self.p2_progress)

        self.p2_list = QtWidgets.QListWidget()
        self.p2_list.itemSelectionChanged.connect(self._p2_on_list_selection)
        lv.addWidget(self.p2_list, 1)

        self.p2_btn_undo = QtWidgets.QPushButton("Desfer captura")
        self.p2_btn_undo.clicked.connect(self._p2_on_undo)
        lv.addWidget(self.p2_btn_undo)

        h.addWidget(left)

        center_v = QtWidgets.QFrame()
        center_v.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_WINDOW};
                border: 1px solid {C_BORDER};
                border-radius: 10px;
            }}
        """)
        cv_lay = QtWidgets.QVBoxLayout(center_v)
        cv_lay.setContentsMargins(10, 8, 10, 10)
        cv_lay.setSpacing(6)

        header = QtWidgets.QLabel("VÈRTEBRA VIRTUAL — landmark actiu ressaltat en blau")
        header.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;")
        cv_lay.addWidget(header)

        self.plotter_p2 = QtInteractor(center_v)
        self.plotter_p2.set_background(C_BG_WINDOW)
        cv_lay.addWidget(self.plotter_p2.interactor, 1)

        h.addWidget(center_v, 1)

        right = QtWidgets.QFrame()
        right.setObjectName("Panel")
        right.setFixedWidth(400)
        rv = QtWidgets.QVBoxLayout(right)
        rv.setContentsMargins(24, 24, 24, 24)
        rv.setSpacing(16)

        self.p2_active_title = QtWidgets.QLabel("Selecciona un landmark")
        self.p2_active_title.setAlignment(Qt.AlignCenter)
        self.p2_active_title.setStyleSheet(
            f"color: {C_TEXT}; background: transparent; "
            f"font-size: 22px; font-weight: 700; letter-spacing: 1px;")
        rv.addWidget(self.p2_active_title)

        instr = QtWidgets.QLabel(
            "Mou el robot amb hand-guide fins que la punta del TCP toqui "
            "el landmark real (±10 mm és suficient — l'ICP compensa).\n\n"
            "Prem «Capturar TCP» quan estiguis segur del contacte."
        )
        instr.setWordWrap(True)
        instr.setAlignment(Qt.AlignCenter)
        instr.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_SM}px; line-height: 150%; padding: 0 10px;")
        rv.addWidget(instr)

        self.p2_tac_box = self._make_coord_box("VIRTUAL (frame CT)", C_TEXT_DIM)
        rv.addWidget(self.p2_tac_box["frame"], alignment=Qt.AlignCenter)

        self.p2_tcp_box = self._make_coord_box("TCP ROBOT (en directe)", C_ACCENT)
        rv.addWidget(self.p2_tcp_box["frame"], alignment=Qt.AlignCenter)

        self.p2_cap_box = self._make_coord_box("CAPTURADA", C_SAFE)
        rv.addWidget(self.p2_cap_box["frame"], alignment=Qt.AlignCenter)

        self.p2_btn_capture = QtWidgets.QPushButton("CAPTURAR TCP")
        self.p2_btn_capture.setFixedHeight(60)
        self.p2_btn_capture.setStyleSheet(f"""
            QPushButton {{
                background-color: {C_ACCENT};
                color: white;
                border: none;
                border-radius: 10px;
                font-size: 18px;
                font-weight: 700;
                letter-spacing: 2px;
            }}
            QPushButton:hover {{ background-color: #2a94ff; }}
            QPushButton:pressed {{ background-color: #086bd4; }}
            QPushButton:disabled {{ background-color: {C_BORDER_STRONG};
                                    color: {C_TEXT_MUTED}; }}
        """)
        self.p2_btn_capture.clicked.connect(self._p2_on_capture)
        rv.addWidget(self.p2_btn_capture)

        self.p2_robot_status = QtWidgets.QLabel()
        self.p2_robot_status.setAlignment(Qt.AlignCenter)
        rv.addWidget(self.p2_robot_status)

        rv.addStretch()
        h.addWidget(right)

        self._p2_actors = {"vertebra": None, "landmarks": {}, "labels": {}}
        self._setup_p2_scene()

        return wrap

    def _setup_p2_scene(self):
        if self.vertebra_mesh is None:
            return

        try:
            self.plotter_p2.remove_all_lights()
        except Exception:
            pass
        self.plotter_p2.add_light(pv.Light(light_type="headlight",
                                            color="white", intensity=0.55))

        self._p2_actors["vertebra"] = self.plotter_p2.add_mesh(
            self.vertebra_mesh,
            color=(207, 200, 188),
            opacity=1.0,
            smooth_shading=True,
            specular=0.0,
            specular_power=1,
            ambient=0.65,
            diffuse=0.45,
            show_edges=False,
            pickable=False,
        )
        self.plotter_p2.reset_camera()

    def _p2_redraw_landmarks(self):
        if self.vertebra_mesh is None:
            return

        for d in ("landmarks", "labels"):
            for idx, actor in list(self._p2_actors[d].items()):
                try:
                    self.plotter_p2.remove_actor(actor)
                except Exception:
                    pass
            self._p2_actors[d].clear()

        for lm in self.state.landmarks:
            if lm.idx == self.active_idx:
                color = COLOR_ACTIU
                radius_mm = 1.8
            elif lm.is_captured:
                color = COLOR_CAPTURAT
                radius_mm = 1.8
            else:
                color = COLOR_PENDENT
                radius_mm = 1.8

            sphere = pv.Sphere(radius=radius_mm, center=lm.virtual)
            actor = self.plotter_p2.add_mesh(
                sphere, color=color, opacity=0.95,
                smooth_shading=True, specular=0.3,
                name=f"p2_lm_{lm.idx}",
                pickable=False,
            )
            self._p2_actors["landmarks"][lm.idx] = actor

            label_actor = self.plotter_p2.add_point_labels(
                np.array([lm.virtual]),
                [lm.name],
                font_size=14,
                point_size=1,
                point_color=color,
                text_color="white",
                shape_color=color,
                shape_opacity=0.85,
                always_visible=True,
                name=f"p2_lb_{lm.idx}",
            )
            self._p2_actors["labels"][lm.idx] = label_actor

        self.plotter_p2.render()

    def _make_coord_box(self, title, color):
        frame = QtWidgets.QFrame()
        frame.setStyleSheet(f"""
            QFrame {{
                background-color: {C_PANEL_ALT};
                border: 1px solid {C_BORDER};
                border-radius: 8px;
                padding: 12px;
            }}
        """)
        frame.setFixedWidth(360)
        l = QtWidgets.QVBoxLayout(frame)
        l.setContentsMargins(12, 10, 12, 12)
        l.setSpacing(6)

        cap = QtWidgets.QLabel(title)
        cap.setStyleSheet(
            f"color: {color}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;")
        l.addWidget(cap)

        val = QtWidgets.QLabel("──   ──   ── mm")
        val.setStyleSheet(
            f"color: {C_TEXT}; background: transparent; border: none; "
            f"font-size: 15px; font-weight: 600; "
            f"font-family: {FONT_MONO}; letter-spacing: 1px;")
        l.addWidget(val)

        return {"frame": frame, "value": val, "title": cap, "color": color}

    def _set_coord_box(self, box, coords_mm, dim=False):
        if coords_mm is None:
            box["value"].setText("──   ──   ── mm")
        else:
            x, y, z = coords_mm
            box["value"].setText(f"X {x:+8.2f}   Y {y:+8.2f}   Z {z:+8.2f} mm")
        color = C_TEXT_MUTED if dim else C_TEXT
        box["value"].setStyleSheet(
            f"color: {color}; background: transparent; border: none; "
            f"font-size: 15px; font-weight: 600; "
            f"font-family: {FONT_MONO}; letter-spacing: 1px;")

    def _p2_refresh(self):
        self.p2_list.blockSignals(True)
        self.p2_list.clear()
        for lm in self.state.landmarks:
            if lm.is_captured:
                r = lm.robot
                text = (f"  ●  {lm.name}    "
                        f"({r[0]:+7.1f}, {r[1]:+7.1f}, {r[2]:+7.1f})")
            else:
                text = f"  ○  {lm.name}    pendent"
            item = QtWidgets.QListWidgetItem(text)
            self.p2_list.addItem(item)
            if lm.idx == self.active_idx:
                self.p2_list.setCurrentItem(item)
        self.p2_list.blockSignals(False)

        n_cap = self.state.n_captured()
        n_tot = self.state.n_landmarks()
        color = C_SAFE if n_cap >= MIN_POINTS else C_TEXT_DIM
        self.p2_progress.setStyleSheet(
            f"color: {color}; background: transparent; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 0.6px;")
        self.p2_progress.setText(f"{n_cap} / {n_tot} capturats")

        if self.active_idx is not None and self.active_idx < len(self.state.landmarks):
            lm = self.state.landmarks[self.active_idx]
            self.p2_active_title.setText(f"LANDMARK {lm.name}")
            self._set_coord_box(self.p2_tac_box, lm.virtual, dim=True)
            self._set_coord_box(self.p2_cap_box, lm.robot,
                                 dim=not lm.is_captured)
        else:
            self.p2_active_title.setText("Selecciona un landmark")
            self._set_coord_box(self.p2_tac_box, None)
            self._set_coord_box(self.p2_cap_box, None)

        connected = self._robot_connected()
        if connected:
            self.p2_robot_status.setText("● ROBOT CONNECTAT")
            self.p2_robot_status.setStyleSheet(
                f"color: {C_SAFE}; background: transparent; "
                f"font-size: {SIZE_SM}px; font-weight: 700;")
            self.p2_btn_capture.setEnabled(self.active_idx is not None)
        else:
            self.p2_robot_status.setText(
                "○ Robot no connectat — anar a Robot i connectar motors")
            self.p2_robot_status.setStyleSheet(
                f"color: {C_WARN}; background: transparent; "
                f"font-size: {SIZE_SM}px; font-weight: 600;")
            self.p2_btn_capture.setEnabled(False)

        if hasattr(self, "plotter_p2"):
            self._p2_redraw_landmarks()

        self._update_next_button()

    def _p2_on_list_selection(self):
        row = self.p2_list.currentRow()
        if 0 <= row < len(self.state.landmarks):
            self.active_idx = row
            self._p2_refresh()

    def _p2_on_capture(self):
        if self.active_idx is None:
            return
        tcp = self._read_robot_tcp_mm()
        if tcp is None:
            QtWidgets.QMessageBox.warning(
                self, "TCP", "No es pot llegir la pose del TCP del robot.")
            return
        self.state.set_capture(self.active_idx, tcp)

        n = len(self.state.landmarks)
        for i in range(n):
            j = (self.active_idx + 1 + i) % n
            if not self.state.landmarks[j].is_captured:
                self.active_idx = j
                break

        self._p2_refresh()

    def _p2_on_undo(self):
        if self.active_idx is None:
            return
        self.state.clear_capture(self.active_idx)
        self._p2_refresh()

    def _refresh_tcp_display(self):
        if self.current_phase != 2:
            return
        tcp = self._read_robot_tcp_mm()
        self._set_coord_box(self.p2_tcp_box, tcp, dim=(tcp is None))

    def _read_robot_tcp_mm(self):
        if self.robot_workspace is None:
            return None
        viewer = None
        if hasattr(self.robot_workspace, "get_viewer"):
            viewer = self.robot_workspace.get_viewer()
        if viewer is None:
            return None
        try:
            poses = getattr(viewer, "poses", None)
            if poses is None or "tcp_link" not in poses:
                return None
            tcp_m = poses["tcp_link"][:3, 3]
            return tuple(float(v) * 1000.0 for v in tcp_m)
        except Exception as e:
            print(f"[registration] Error llegint TCP: {e}")
            return None

    def _robot_connected(self):
        if self.robot_workspace is None:
            return False
        if hasattr(self.robot_workspace, "is_robot_connected"):
            return self.robot_workspace.is_robot_connected()
        return False

    def _build_phase3(self):
        wrap = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(wrap)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(10)

        center = QtWidgets.QFrame()
        center.setObjectName("Panel")
        cv = QtWidgets.QVBoxLayout(center)
        cv.setContentsMargins(28, 28, 28, 28)
        cv.setSpacing(18)

        title = QtWidgets.QLabel("REGISTRE CT ↔ ROBOT")
        title.setStyleSheet(
            f"color: {C_TEXT}; background: transparent; "
            f"font-size: 20px; font-weight: 700; letter-spacing: 1px;")
        cv.addWidget(title)

        rmse_row = QtWidgets.QHBoxLayout()
        rmse_row.setSpacing(14)

        self.p3_kabsch_box = self._make_rmse_box(
            "KABSCH INICIAL", "punt ↔ punt")
        rmse_row.addWidget(self.p3_kabsch_box["frame"])

        self.p3_icp_box = self._make_rmse_box(
            "ICP FINAL", "punt ↔ superfície")
        rmse_row.addWidget(self.p3_icp_box["frame"])
        cv.addLayout(rmse_row)

        self.p3_verdict = QtWidgets.QLabel("")
        self.p3_verdict.setAlignment(Qt.AlignCenter)
        self.p3_verdict.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: 16px; font-weight: 700; letter-spacing: 2px; padding: 8px;")
        cv.addWidget(self.p3_verdict)

        self.p3_details = QtWidgets.QLabel("")
        self.p3_details.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_SM}px; font-family: {FONT_MONO}; "
            f"line-height: 160%; padding: 12px 4px;")
        cv.addWidget(self.p3_details)

        cv.addStretch()

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(12)

        self.p3_btn_compute = QtWidgets.QPushButton("Calcular registre (Kabsch + ICP)")
        self.p3_btn_compute.setFixedHeight(48)
        self.p3_btn_compute.clicked.connect(self._p3_on_compute)
        btn_row.addWidget(self.p3_btn_compute)

        self.p3_btn_apply = QtWidgets.QPushButton("Aplicar al Robot →")
        self.p3_btn_apply.setFixedHeight(48)
        self.p3_btn_apply.setStyleSheet(f"""
            QPushButton {{
                background-color: {C_SAFE}; color: white;
                border: none; border-radius: 8px;
                font-size: 14px; font-weight: 700; letter-spacing: 1px;
            }}
            QPushButton:hover {{ background-color: #48c774; }}
            QPushButton:pressed {{ background-color: #3aa060; }}
            QPushButton:disabled {{ background-color: {C_BORDER_STRONG};
                                    color: {C_TEXT_MUTED}; }}
        """)
        self.p3_btn_apply.setEnabled(False)
        self.p3_btn_apply.clicked.connect(self._p3_on_apply)
        btn_row.addWidget(self.p3_btn_apply)
        cv.addLayout(btn_row)

        h.addWidget(center, 1)
        return wrap

    def _make_rmse_box(self, title, subtitle):
        frame = QtWidgets.QFrame()
        frame.setStyleSheet(f"""
            QFrame {{
                background-color: {C_PANEL_ALT};
                border: 1px solid {C_BORDER};
                border-radius: 12px;
                padding: 16px;
            }}
        """)
        l = QtWidgets.QVBoxLayout(frame)
        l.setSpacing(4)
        l.setContentsMargins(20, 16, 20, 18)

        cap = QtWidgets.QLabel(title)
        cap.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1.2px;")
        l.addWidget(cap)

        sub = QtWidgets.QLabel(subtitle)
        sub.setStyleSheet(
            f"color: {C_TEXT_MUTED}; background: transparent; "
            f"font-size: 10px; font-style: italic;")
        l.addWidget(sub)

        val = QtWidgets.QLabel("── mm")
        val.setStyleSheet(
            f"color: {C_TEXT}; background: transparent; "
            f"font-size: 36px; font-weight: 700; font-family: {FONT_MONO};")
        l.addWidget(val)

        return {"frame": frame, "value": val, "title": cap, "subtitle": sub}

    def _set_rmse_value(self, box, value_mm, color=None):
        if value_mm is None:
            box["value"].setText("── mm")
            color = color or C_TEXT_DIM
        else:
            box["value"].setText(f"{value_mm:.2f} mm")
            color = color or C_TEXT
        box["value"].setStyleSheet(
            f"color: {color}; background: transparent; "
            f"font-size: 36px; font-weight: 700; font-family: {FONT_MONO};")

    def _p3_refresh(self):
        can = self.state.can_compute()
        self.p3_btn_compute.setEnabled(can)

        if self.state.stats_kabsch is None:
            self._set_rmse_value(self.p3_kabsch_box, None)
            self._set_rmse_value(self.p3_icp_box, None)
            self.p3_verdict.setText(
                f"Prem «Calcular registre» ({self.state.n_captured()} punts capturats)"
                if can else
                f"Necessites almenys 3 punts capturats (tens {self.state.n_captured()})")
            self.p3_verdict.setStyleSheet(
                f"color: {C_TEXT_DIM}; background: transparent; "
                f"font-size: 16px; font-weight: 700; letter-spacing: 2px; padding: 8px;")
            self.p3_details.setText("")
            self.p3_btn_apply.setEnabled(False)
            return

        self._set_rmse_value(self.p3_kabsch_box, self.state.stats_kabsch["rmse"],
                              color=C_TEXT_DIM)

        verdict, tone = self.state.verdict()
        color = {"safe": C_SAFE, "warn": C_WARN,
                 "danger": C_DANGER, "muted": C_TEXT_DIM}[tone]

        if self.state.stats_icp is not None:
            self._set_rmse_value(self.p3_icp_box, self.state.stats_icp["rmse"],
                                  color=color)
        else:
            self._set_rmse_value(self.p3_icp_box, None)

        self.p3_verdict.setText(verdict)
        self.p3_verdict.setStyleSheet(
            f"color: {color}; background: transparent; "
            f"font-size: 16px; font-weight: 700; letter-spacing: 2px; padding: 8px;")

        lines = []
        stats_k = self.state.stats_kabsch
        stats_i = self.state.stats_icp

        lines.append(f"Punts usats:      {stats_k['per_point'].shape[0]}")
        lines.append("")
        lines.append("KABSCH (punt-a-punt, virtual ↔ capturat):")
        lines.append(f"   RMSE:          {stats_k['rmse']:6.3f} mm")
        lines.append(f"   Error mitjà:   {stats_k['mean']:6.3f} mm")
        lines.append(f"   Error màxim:   {stats_k['max']:6.3f} mm")

        if stats_i is not None:
            lines.append("")
            lines.append("ICP (punt-a-superfície, refinat):")
            lines.append(f"   RMSE:          {stats_i['rmse']:6.3f} mm")
            lines.append(f"   Error mitjà:   {stats_i['mean']:6.3f} mm")
            lines.append(f"   Error màxim:   {stats_i['max']:6.3f} mm")
            lines.append(f"   Iteracions:    {len(self.state.icp_history)}")

            improvement = stats_k['rmse'] - stats_i['rmse']
            if improvement > 0:
                pct = improvement / stats_k['rmse'] * 100
                lines.append(f"   Millora ICP:   {improvement:6.3f} mm  ({pct:+.1f}%)")

            lines.append("")
            lines.append("Error final per punt (dist a superfície, mm):")
            ids, _, _ = self.state.captured_pairs()
            for pid, err in zip(ids, stats_i["per_point"]):
                name = self.state.landmarks[pid].name
                bar = "█" * min(20, int(err * 5))
                lines.append(f"   {name:>4}   {err:6.2f}   {bar}")

        self.p3_details.setText("\n".join(lines))

        self.p3_btn_apply.setEnabled(tone != "danger")

    def _p3_on_compute(self):
        try:
            self.state.compute(vertebra_polydata=self.vertebra_mesh)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Kabsch/ICP", f"Error: {e}")
            import traceback; traceback.print_exc()
            return
        self._p3_refresh()
        try:
            if self.registry.json_path:
                self.registry.save()
        except Exception as e:
            print(f"[registration] autosave failed: {e}")
        self._refresh_vertebra_combo()
        self._refresh_vertebra_status_label()

    def _p3_on_apply(self):
        if self.state.T is None:
            return
        if self.robot_workspace is None or not hasattr(
                self.robot_workspace, "load_registered_scene"):
            QtWidgets.QMessageBox.information(
                self, "Aplicar al Robot",
                "Registre calculat.\n\n"
                "El workspace del robot no implementa load_registered_scene().")
            return

        screws = self._get_screws_from_planning()

        vertebres_multi = {}
        valid_codes = self.registry.names_with_valid_T()
        for code in valid_codes:
            st = self.registry[code]
            if code == self.registry.active_name() and self.vertebra_mesh is not None:
                mesh = self.vertebra_mesh
            else:
                stl_path = self.vertebres.get(code)
                if not stl_path or not os.path.exists(stl_path):
                    print(f"[registration] STL de '{code}' no trobat, es salta")
                    continue
                try:
                    mesh = pv.read(stl_path).clean().triangulate()
                except Exception as e:
                    print(f"[registration] error carregant {stl_path}: {e}")
                    continue
            vertebres_multi[code] = {"mesh": mesh, "T": st.T}

        if not vertebres_multi:
            QtWidgets.QMessageBox.warning(self, "Aplicar",
                "No hi ha cap vèrtebra amb registre vàlid.")
            return

        try:
            self.robot_workspace.load_registered_scene(
                vertebra_mesh=None,
                screws=screws,
                T_ct_to_robot=None,
                vertebres=vertebres_multi,
            )
            best = self.state.best_stats()
            QtWidgets.QMessageBox.information(
                self, "Aplicat",
                f"S'han aplicat {len(vertebres_multi)} vèrtebres al Robot "
                f"(RMSE actual: {best['rmse']:.2f} mm).\n\n"
                "Ves al workspace «Robot» per veure-les alineades amb els "
                "cargols planificats.")
            self.registrationUpdated.emit(True)
        except Exception as e:
            QtWidgets.QMessageBox.critical(
                self, "Error", f"No s'ha pogut aplicar: {e}")
            import traceback; traceback.print_exc()

    def _get_screws_from_planning(self):
        main = self.window()
        planning_ws = None
        if hasattr(main, "_workspaces"):
            planning_ws = main._workspaces.get("planning")
        if planning_ws is None:
            return []
        pw = getattr(planning_ws, "widget", None) or planning_ws
        return list(getattr(pw, "cargols", []))

    def _show_phase(self, phase):
        self.current_phase = phase
        self.stack.setCurrentIndex(phase - 1)

        for i, pill in enumerate(self.phase_pills):
            if i + 1 == phase:
                pill.setStyleSheet(style_pill(C_ACCENT))
            else:
                pill.setStyleSheet(style_pill(C_TEXT_DIM))

        self._update_prev_next_buttons()

        if phase == 1:
            self._p1_refresh_list()
        elif phase == 2:
            if self.state.landmarks:
                for lm in self.state.landmarks:
                    if not lm.is_captured:
                        self.active_idx = lm.idx
                        break
                else:
                    self.active_idx = self.state.landmarks[0].idx
            self._p2_refresh()
        elif phase == 3:
            self._p3_refresh()

    def _update_prev_next_buttons(self):
        self.btn_prev.setEnabled(self.current_phase > 1)
        self._update_next_button()

    def _update_next_button(self):
        if self.current_phase == 1:
            can = self.state.n_landmarks() >= MIN_POINTS
            self.btn_next.setEnabled(can)
            self.btn_next.setText("Següent → (Palpar amb Robot)"
                                   if can else
                                   f"Falten {MIN_POINTS - self.state.n_landmarks()} landmarks")
        elif self.current_phase == 2:
            can = self.state.n_captured() >= 3
            self.btn_next.setEnabled(can)
            self.btn_next.setText("Següent → (Calibrar)")
        else:
            self.btn_next.setEnabled(False)
            self.btn_next.setText("—")

    def _on_prev_phase(self):
        if self.current_phase > 1:
            self._show_phase(self.current_phase - 1)

    def _on_next_phase(self):
        if self.current_phase < 3:
            self._show_phase(self.current_phase + 1)

    def cleanup(self):
        try:
            self._tcp_timer.stop()
        except Exception:
            pass
        for attr in ("plotter", "plotter_p2"):
            try:
                if hasattr(self, attr):
                    getattr(self, attr).close()
            except Exception:
                pass

