"""
Workspace de REGISTRE — calibratge TAC ↔ Robot amb superfície de vèrtebra.

FLUX DE TREBALL EN 3 FASES:

    FASE 1 — Marcar landmarks sobre la vèrtebra 3D virtual
        L'usuari veu una reconstrucció 3D interactiva de la vèrtebra STL
        (Segmentation_L_primera.stl per defecte) i clica sobre la seva
        superfície amb el ratolí. Cada clic afegeix un landmark (mínim 8).
        Els punts es projecten automàticament a la superfície via ray-picking
        (mai floten a l'aire).

    FASE 2 — Palpar amb el TCP del robot
        L'usuari mou el robot fins que la punta toqui aproximadament el
        landmark real (amb tolerància de fins a 10 mm) i prem "Capturar TCP".

    FASE 3 — Calcular (Kabsch + ICP) i aplicar
        1) Kabsch inicial amb els 8 parells (aproximació)
        2) ICP refinat contra la superfície completa de la vèrtebra
           → absorbeix la imprecisió de la palpació
        3) Aplicar al Robot: la vèrtebra + cargols apareixen ben alineats
           al workspace del robot en la seva posició física real

TOT EN MEMÒRIA: cap fitxer es guarda a disc.
"""

import os                                                                       # importem el mòdul os
import numpy as np                                                              # importem el mòdul numpy as np

import pyvista as pv                                                            # importem el mòdul pyvista as pv
from pyvistaqt import QtInteractor                                              # importem QtInteractor des de pyvistaqt
import vtk                                                                      # importem el mòdul vtk

from qtpy import QtWidgets, QtCore                                              # importem QtWidgets, QtCore des de qtpy
from qtpy.QtCore import Qt                                                      # importem Qt des de qtpy.QtCore

from ..theme import (                                                           # importem ( des de ..theme
    C_BG_WINDOW, C_PANEL_ALT, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_ACCENT,
    C_SAFE, C_WARN, C_DANGER,
    SIZE_XS, SIZE_SM, FONT_MONO,
    style_pill,
)
from ..registration.state import RegistrationState, MIN_POINTS, MultiVertebraRegistry  # importem també MultiVertebraRegistry


# ==============================================================================
# CONFIGURACIÓ
# ==============================================================================

_HERE = os.path.dirname(os.path.abspath(__file__))                              # Arrel del projecte: pugem 3 nivells des d'aquest fitxer
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))          # .../workspaces/registration.py → .../neurospine_app/ → .../scripts/ → arrel

# Prefix i sufix dels STLs de vèrtebra que acceptem a vertebres_meshes/
# Exemple: "Vertebra_T12.stl" → codi "T12"
VERTEBRA_PREFIX = "Vertebra_"
VERTEBRA_SUFFIX = ".stl"

vtk.vtkObject.GlobalWarningDisplayOff()                                         # executem GlobalWarningDisplayOff()
os.environ["VTK_SILENCE_GET_WINDOW_INFO_ERRORS"] = "1"                          # os.environ["VTK_SILENCE_GET_WINDOW_INFO_ERRORS"] = "1"


# Colors per als marcadors de landmarks
COLOR_PENDENT   = "#f0ad4e"  # taronja: encara no capturat al robot
COLOR_CAPTURAT  = "#5cb85c"  # verd: capturat
COLOR_ACTIU     = "#0a84ff"  # blau: el landmark actiu ara mateix


# ==============================================================================
# WORKSPACE PRINCIPAL
# ==============================================================================

class RegistrationWorkspace(QtWidgets.QWidget):                                 # definim la classe RegistrationWorkspace
    """
    Workspace de registre en 3 fases (marcar 3D → palpar → calibrar).
    """

    registrationUpdated = QtCore.Signal(bool)                                   # registrationUpdated ← resultat de Signal()

    def __init__(self, pacient_dir=None, robot_workspace=None, parent=None):    # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.pacient_dir = pacient_dir                                          # assignem self.pacient_dir
        self.robot_workspace = robot_workspace                                  # assignem self.robot_workspace

        # ----- MULTI-VÈRTEBRA -----
        # self.vertebres: dict {codi: ruta_STL}, ex: {"T12": ".../Vertebra_T12.stl"}
        # self.registry: estat de registre per CADA vèrtebra, serializable a JSON
        # self.state:    alias convenient a self.registry.current() (el que està actiu)
        self.vertebres = {}                                                     # mapa codi → path (es poblarà a _discover_vertebres)
        self.registry = MultiVertebraRegistry(self._registry_json_path())       # registry amb path de persistència
        self.registry.load()                                                    # carrega del disc si existeix
        self.state = None                                                       # s'assignarà en seleccionar la primera vèrtebra

        # Fase actual: 1 = marcar 3D, 2 = palpar, 3 = calibrar
        self.current_phase = 1                                                  # assignem self.current_phase
        self.active_idx = None                                                  # assignem self.active_idx

        # Vèrtebra 3D (PyVista PolyData) — es carrega a _load_active_vertebra()
        self.vertebra_mesh = None                                               # assignem self.vertebra_mesh
        self._landmark_actors = {}       # {landmark_idx: actor esfera}
        self._landmark_label_actors = {}  # {landmark_idx: actor text}
        self._vertebra_actor = None                                             # assignem self._vertebra_actor

        # Interacció de picking
        self._picker = None                                                     # assignem self._picker

        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        # ----- Descobrim vèrtebres disponibles i seleccionem la primera -----
        self._discover_vertebres()                                              # omple self.vertebres
        initial_code = self._initial_vertebra_code()                            # tria la que farem servir d'inici
        if initial_code is not None:                                            # si tenim alguna vèrtebra
            self.state = self.registry.select(initial_code)                     # fixem com activa
            self._load_active_vertebra()                                        # carreguem el seu STL
        else:                                                                   # cap vèrtebra disponible
            # Mantenim un state buit perquè la UI es construeixi sense errors.
            # L'usuari veurà el missatge d'avís al dropdown i al plotter.
            self.state = RegistrationState()                                    # estat buit (placeholder)

        self._build_ui()                                                        # executem _build_ui()
        self._show_phase(1)                                                     # mostrem fase 1

        # Timer per refresc de la pose del TCP en directe (fase 2)
        self._tcp_timer = QtCore.QTimer(self)                                   # self._tcp_timer ← resultat de QTimer()
        self._tcp_timer.timeout.connect(self._refresh_tcp_display)              # executem connect()
        self._tcp_timer.start(200)                                              # executem start()

    # ============================================================
    # HELPERS MULTI-VÈRTEBRA
    # ============================================================
    def _vertebres_dir(self):                                                   # definim la funció _vertebres_dir()
        """Carpeta on buscar els STLs de vèrtebra del pacient actiu."""
        if not self.pacient_dir:                                                # si no tenim pacient
            return None                                                         # res a fer
        return os.path.join(self.pacient_dir, "vertebres_meshes")               # retornem: path

    def _registry_json_path(self):                                              # definim la funció _registry_json_path()
        """Ruta al JSON de persistència del registry per aquest pacient."""
        if not self.pacient_dir:                                                # si no tenim pacient
            return None                                                         # sense persistència
        return os.path.join(self.pacient_dir, "registres.json")                 # retornem: path

    def _discover_vertebres(self):                                              # definim la funció _discover_vertebres()
        """
        Escaneja la carpeta vertebres_meshes/ i troba tots els STLs
        amb format 'Vertebra_<CODI>.stl'. Omple self.vertebres amb
        {codi: ruta_completa}.
        """
        self.vertebres = {}                                                     # reset
        folder = self._vertebres_dir()                                          # carpeta
        if not folder or not os.path.isdir(folder):                             # si no existeix
            print(f"[registration] carpeta de vèrtebres no trobada: {folder}")
            return
        for fname in sorted(os.listdir(folder)):                                # iterem fitxers (ordenats)
            if not fname.startswith(VERTEBRA_PREFIX):                           # ha de començar per Vertebra_
                continue
            if not fname.lower().endswith(VERTEBRA_SUFFIX):                     # i acabar en .stl
                continue
            code = fname[len(VERTEBRA_PREFIX):-len(VERTEBRA_SUFFIX)]            # extraiem el codi
            if not code:                                                        # si queda buit, saltem
                continue
            self.vertebres[code] = os.path.join(folder, fname)                  # guardem ruta
        print(f"[registration] vèrtebres descobertes: {list(self.vertebres.keys())}")

    def _initial_vertebra_code(self):                                           # definim la funció _initial_vertebra_code()
        """
        Tria quina vèrtebra mostrar primer:
          1) Si el registry en té una activa, mantenim aquella (si existeix l'STL).
          2) Si no, la primera que trobem alfabèticament.
          3) Si no n'hi ha, None.
        """
        active = self.registry.active_name()                                    # vèrtebra activa (del load)
        if active and active in self.vertebres:                                 # si encara té STL
            return active                                                       # mantenim
        if self.vertebres:                                                      # si n'hi ha alguna
            return sorted(self.vertebres.keys())[0]                             # primera alfabètica
        return None                                                             # cap

    def _load_active_vertebra(self):                                            # definim la funció _load_active_vertebra()
        """Carrega l'STL de la vèrtebra actualment seleccionada al registry."""
        name = self.registry.active_name()                                      # nom actiu
        self.vertebra_mesh = None                                               # reset
        if name is None or name not in self.vertebres:                          # si no vàlida
            return                                                              # res
        stl_path = self.vertebres[name]                                         # ruta
        try:                                                                    # intentem llegir
            mesh = pv.read(stl_path).clean().triangulate()                      # mesh
            self.vertebra_mesh = mesh                                           # assignem
            print(f"[registration] Vèrtebra '{name}' carregada: "
                  f"{mesh.n_points} vèrtexs, {mesh.n_cells} triangles")
        except Exception as e:                                                  # capturem errors
            print(f"[registration] Error carregant STL {stl_path}: {e}")

    # ============================================================
    # UI GENERAL
    # ============================================================
    def _build_ui(self):                                                        # definim la funció _build_ui()
        outer = QtWidgets.QVBoxLayout(self)                                     # outer ← resultat de QVBoxLayout()
        outer.setContentsMargins(12, 12, 12, 12)                                # executem setContentsMargins()
        outer.setSpacing(10)                                                    # executem setSpacing()

        # Selector de vèrtebra activa (nou - multi-vèrtebra)
        outer.addWidget(self._build_vertebra_selector_bar())                    # executem addWidget()

        outer.addWidget(self._build_phase_bar())                                # executem addWidget()

        self.stack = QtWidgets.QStackedWidget()                                 # self.stack ← resultat de QStackedWidget()
        self.stack.addWidget(self._build_phase1())    # index 0
        self.stack.addWidget(self._build_phase2())    # index 1
        self.stack.addWidget(self._build_phase3())    # index 2
        outer.addWidget(self.stack, 1)                                          # executem addWidget()

    # ------------------------------------------------------------
    # Barra selector de vèrtebra activa
    # ------------------------------------------------------------
    def _build_vertebra_selector_bar(self):                                     # definim la funció _build_vertebra_selector_bar()
        """
        Barra amb:
          - dropdown amb totes les vèrtebres descobertes
          - indicador d'estat de cada una (✓ registrada / ○ parcial / · buida)
          - botons: Desa tots els registres, Esborra el d'aquesta vèrtebra
        """
        bar = QtWidgets.QFrame()                                                # frame contenidor
        bar.setObjectName("Panel")                                              # estil Panel
        bar.setFixedHeight(60)                                                  # alçada fixa
        h = QtWidgets.QHBoxLayout(bar)                                          # layout horitzontal
        h.setContentsMargins(16, 10, 16, 10)
        h.setSpacing(10)

        # Etiqueta
        lbl = QtWidgets.QLabel("VÈRTEBRA ACTIVA")                                # label
        lbl.setStyleSheet(                                                      # estil
            f"color: {C_TEXT_DIM}; background: transparent; border: none; "
            f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 1px;")
        h.addWidget(lbl)                                                        # afegim

        # Dropdown
        self.vertebra_combo = QtWidgets.QComboBox()                             # dropdown
        self.vertebra_combo.setMinimumWidth(260)                                # amplada mínima
        self._refresh_vertebra_combo()                                          # omplim amb les descobertes
        self.vertebra_combo.currentIndexChanged.connect(self._on_vertebra_selected)
        h.addWidget(self.vertebra_combo)                                        # afegim

        # Indicador d'estat textual
        self.vertebra_status_lbl = QtWidgets.QLabel("")                         # label d'estat
        self.vertebra_status_lbl.setStyleSheet(                                 # estil
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_XS}px; font-family: {FONT_MONO};")
        h.addWidget(self.vertebra_status_lbl)                                   # afegim

        h.addStretch()                                                          # espai dret

        # Botó: Desa tots els registres
        self.btn_save_all = QtWidgets.QPushButton("Desa tot al disc")           # botó desa
        self.btn_save_all.clicked.connect(self._on_save_registry)
        h.addWidget(self.btn_save_all)                                          # afegim

        # Botó: Reinicia el registre d'aquesta vèrtebra
        self.btn_reset_current = QtWidgets.QPushButton("Reinicia aquesta")      # botó reset
        self.btn_reset_current.clicked.connect(self._on_reset_current_vertebra)
        h.addWidget(self.btn_reset_current)                                     # afegim

        self._refresh_vertebra_status_label()                                   # actualitzem el label d'estat
        return bar                                                              # retornem

    def _refresh_vertebra_combo(self):                                          # definim la funció _refresh_vertebra_combo()
        """Omple el dropdown amb {codi + icona d'estat}."""
        self.vertebra_combo.blockSignals(True)                                  # evitem senyals espuris
        self.vertebra_combo.clear()                                             # buidem
        if not self.vertebres:                                                  # si cap vèrtebra
            self.vertebra_combo.addItem("(cap vèrtebra a vertebres_meshes/)")   # placeholder
            self.vertebra_combo.setEnabled(False)                               # dropdown deshabilitat
        else:                                                                   # si en tenim
            self.vertebra_combo.setEnabled(True)                                # habilitem
            active = self.registry.active_name()                                # activa actual
            for i, code in enumerate(sorted(self.vertebres.keys())):            # iterem per codi ordenat
                mark = self._vertebra_status_icon(code)                         # icona segons estat
                self.vertebra_combo.addItem(f"{mark}  {code}", code)            # afegim (text + userData)
                if code == active:                                              # si és l'activa
                    self.vertebra_combo.setCurrentIndex(i)                      # la seleccionem al combo
        self.vertebra_combo.blockSignals(False)                                 # reactivem senyals

    def _vertebra_status_icon(self, code):                                      # definim la funció _vertebra_status_icon()
        """Icona d'estat segons si té registre vàlid o parcial."""
        if code not in self.registry:                                           # sense cap dada
            return "·"
        st = self.registry[code]                                                # estat
        if st.has_valid_matrix():                                               # té T calculada → ✓
            return "✓"
        if st.n_landmarks() > 0 or st.n_captured() > 0:                         # té dades en curs → ○
            return "○"
        return "·"                                                              # buit → ·

    def _refresh_vertebra_status_label(self):                                   # definim la funció _refresh_vertebra_status_label()
        """Mostra 'X / N registrades' al costat del dropdown."""
        total = len(self.vertebres)                                             # total descobertes
        done = len([c for c in self.vertebres if c in self.registry
                    and self.registry[c].has_valid_matrix()])                   # amb T vàlida
        if total == 0:                                                          # sense cap
            self.vertebra_status_lbl.setText("")
        else:                                                                   # mostra progrés
            self.vertebra_status_lbl.setText(f"{done} / {total} registrades")

    def _on_vertebra_selected(self, idx):                                       # definim la funció _on_vertebra_selected()
        """
        Callback del dropdown. Canvia la vèrtebra activa, carrega el seu STL
        i reconstrueix la visualització / llista de landmarks.
        """
        code = self.vertebra_combo.itemData(idx)                                # codi seleccionat (userData)
        if not code:                                                            # placeholder (sense vèrtebres)
            return
        if code == self.registry.active_name():                                 # ja activa
            return
        # Canvi d'activa
        self.state = self.registry.select(code)                                 # fixem com activa i obtenim estat
        self._load_active_vertebra()                                            # carreguem el seu STL
        self.active_idx = None                                                  # reset del landmark actiu
        self._rebuild_vertebra_scene()                                          # refresca plotter + landmarks
        # Reset de la fase a 1 (nova vèrtebra = començar de nou visualment,
        # però les dades es mantenen si ja en tenia)
        self._show_phase(1)
        self._refresh_vertebra_status_label()                                   # actualitzem comptador

    def _rebuild_vertebra_scene(self):                                          # definim la funció _rebuild_vertebra_scene()
        """
        Reconstrueix l'escena 3D després d'un canvi de vèrtebra.
        Treu l'STL vell, posa el nou, i redibuixa els landmarks de la nova.
        """
        # Netegem actors de landmarks i d'STL
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
        # Desactivem el picking previ: _setup_3d_scene el torna a activar.
        # Sense això, PyVista llença PyVistaPickingError("already enabled").
        try:
            self.plotter.disable_picking()
        except Exception:
            pass
        # Afegim el nou STL i reconfigurem picking + llum
        self._setup_3d_scene()
        # Redibuixem els landmarks de la nova vèrtebra activa (si en tenia)
        if self.state is not None:
            for idx in range(self.state.n_landmarks()):
                self._draw_landmark_3d(idx)
        try:
            self.plotter.render()
        except Exception:
            pass

        # IMPORTANT: també hem de reconstruir el plotter de la Fase 2,
        # altrament la vèrtebra visible allà queda la de la vèrtebra inicial
        # i només els landmarks s'actualitzen → l'STL sembla "fora de lloc".
        if hasattr(self, "plotter_p2"):
            self._rebuild_p2_scene()

    def _rebuild_p2_scene(self):                                                # definim la funció _rebuild_p2_scene()
        """
        Reconstrueix l'escena 3D de la Fase 2 quan es canvia de vèrtebra.
        Equivalent a _rebuild_vertebra_scene però per plotter_p2.
        """
        if not hasattr(self, "plotter_p2"):
            return
        # Treiem l'STL + landmarks + labels anteriors
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
        # Les llums també s'acumularien a cada canvi: les netegem.
        try:
            self.plotter_p2.remove_all_lights()
        except Exception:
            pass
        # Afegim l'STL de la nova vèrtebra + llums + landmarks
        self._setup_p2_scene()
        if self.state is not None and self.state.n_landmarks() > 0:
            self._p2_redraw_landmarks()
        try:
            self.plotter_p2.render()
        except Exception:
            pass

    def _on_save_registry(self):                                                # definim la funció _on_save_registry()
        """Desa el registry al disc com a <pacient_dir>/registres.json."""
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

    def _on_reset_current_vertebra(self):                                       # definim la funció _on_reset_current_vertebra()
        """Esborra tots els landmarks i captures de la vèrtebra activa."""
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
        # Reemplacem l'estat per un de nou
        self.registry._states[name] = RegistrationState()
        self.state = self.registry._states[name]
        self.active_idx = None
        self._rebuild_vertebra_scene()
        self._show_phase(1)
        self._refresh_vertebra_combo()
        self._refresh_vertebra_status_label()

    def _build_phase_bar(self):                                                 # definim la funció _build_phase_bar()
        """Barra amb 3 pastilles i botons Anterior/Següent."""                  # """Barra amb 3 pastilles i botons Anterior/Següent."""
        bar = QtWidgets.QFrame()                                                # bar ← resultat de QFrame()
        bar.setObjectName("Panel")                                              # executem setObjectName()
        bar.setFixedHeight(70)                                                  # executem setFixedHeight()
        h = QtWidgets.QHBoxLayout(bar)                                          # h ← resultat de QHBoxLayout()
        h.setContentsMargins(16, 10, 16, 10)                                    # executem setContentsMargins()
        h.setSpacing(10)                                                        # executem setSpacing()

        self.phase_pills = []                                                   # assignem self.phase_pills
        for i, label in enumerate(                                              # iterem: i, label in enumerate(
                ("1. MARCAR SOBRE STL 3D",
                 "2. PALPAR AMB ROBOT",
                 "3. CALIBRAR (KABSCH + ICP)")):
            pill = QtWidgets.QLabel(label)                                      # pill ← resultat de QLabel()
            pill.setAlignment(Qt.AlignCenter)                                   # executem setAlignment()
            pill.setMinimumWidth(200)                                           # executem setMinimumWidth()
            pill.setStyleSheet(style_pill(C_TEXT_DIM))                          # executem setStyleSheet()
            h.addWidget(pill)                                                   # executem addWidget()
            self.phase_pills.append(pill)                                       # executem append()

        h.addStretch()                                                          # executem addStretch()

        self.btn_prev = QtWidgets.QPushButton("← Anterior")                     # self.btn_prev ← resultat de QPushButton()
        self.btn_next = QtWidgets.QPushButton("Següent →")                      # self.btn_next ← resultat de QPushButton()
        self.btn_prev.clicked.connect(self._on_prev_phase)                      # executem connect()
        self.btn_next.clicked.connect(self._on_next_phase)                      # executem connect()
        h.addWidget(self.btn_prev)                                              # executem addWidget()
        h.addWidget(self.btn_next)                                              # executem addWidget()

        return bar                                                              # retornem: bar

    # ============================================================
    # FASE 1 — MARCAR LANDMARKS SOBRE STL 3D
    # ============================================================
    def _build_phase1(self):                                                    # definim la funció _build_phase1()
        wrap = QtWidgets.QWidget()                                              # wrap ← resultat de QWidget()
        h = QtWidgets.QHBoxLayout(wrap)                                         # h ← resultat de QHBoxLayout()
        h.setContentsMargins(0, 0, 0, 0)                                        # executem setContentsMargins()
        h.setSpacing(10)                                                        # executem setSpacing()

        # ---- ESQUERRA: llista de landmarks + botons ----
        left = QtWidgets.QFrame()                                               # left ← resultat de QFrame()
        left.setObjectName("Panel")                                             # executem setObjectName()
        left.setFixedWidth(300)                                                 # executem setFixedWidth()
        lv = QtWidgets.QVBoxLayout(left)                                        # lv ← resultat de QVBoxLayout()
        lv.setContentsMargins(16, 16, 16, 16)                                   # executem setContentsMargins()
        lv.setSpacing(10)                                                       # executem setSpacing()

        title = QtWidgets.QLabel(f"LANDMARKS (mínim {MIN_POINTS})")
        title.setObjectName("SectionTitle")                                     # executem setObjectName()
        lv.addWidget(title)                                                     # executem addWidget()

        self.p1_progress = QtWidgets.QLabel(f"0 / {MIN_POINTS} marcats")
        self.p1_progress.setStyleSheet(                                         # executem setStyleSheet()
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

        # ---- CENTRE: viewer 3D interactiu ----
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

        # PyVista viewer
        self.plotter = QtInteractor(center)
        self.plotter.set_background(C_BG_WINDOW)
        cv.addWidget(self.plotter.interactor, 1)

        h.addWidget(center, 1)

        # Un cop hi ha el plotter, hi carreguem la vèrtebra
        self._setup_3d_scene()

        return wrap

    def _setup_3d_scene(self):
        """Carrega la vèrtebra al plotter i configura el picker."""
        if self.vertebra_mesh is None:
            # Placeholder si no s'ha trobat l'STL
            self.plotter.add_text(
                "ERROR: no s'ha trobat la vèrtebra",
                position="upper_edge", font_size=14, color="red",
            )
            return

        # Reset de llums (perquè reconstruccions successives no les acumulin).
        try:
            self.plotter.remove_all_lights()
        except Exception:
            pass

        # UNA sola headlight suau que segueix la càmera: dóna sensació de
        # volum sense generar ombres dures ni brillos. Aspecte "estudi
        # fotogràfic mate".
        self.plotter.add_light(pv.Light(light_type="headlight",
                                         color="white", intensity=0.55))

        # Vèrtebra MATE PROFESSIONAL: color pedra càlid, alt ambient
        # (il·luminació base uniforme), diffuse moderat (volum suau),
        # ZERO specular (res de brillos). Aspecte clínic estètic.
        self._vertebra_actor = self.plotter.add_mesh(
            self.vertebra_mesh,
            color=(207, 200, 188),          # ⬅️ pedra càlida mate
            opacity=1.0,
            smooth_shading=True,
            specular=0.0,
            specular_power=1,
            ambient=0.65,                   # ↑ base clara i uniforme
            diffuse=0.45,                   # ↓ volum suau sense exagerar
            show_edges=False,
            pickable=True,
        )

        # Càmera centrada
        self.plotter.reset_camera()

        # Picking amb BOTÓ DRET perquè el botó esquerre quedi lliure per
        # rotar la càmera. Així no es marquen landmarks per accident quan
        # es navega per la vèrtebra.
        self.plotter.enable_point_picking(
            callback=self._on_surface_click,
            picker="cell",
            show_message=False,
            show_point=False,
            left_clicking=False,        # → botó dret
            tolerance=0.01,
            font_size=12,
        )

    def _on_surface_click(self, point):
        """
        Callback de picking: `point` és (x, y, z) en mm CT sobre la superfície
        de la vèrtebra STL. Afegeix un landmark nou.
        """
        if point is None:
            return

        # Convertim a float per si ve un tipus numpy exòtic
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        idx = self.state.add_virtual_landmark(x, y, z)
        self.active_idx = idx

        self._draw_landmark_3d(idx)
        self._p1_refresh_list()
        self._update_next_button()
        self.plotter.render()

    def _draw_landmark_3d(self, idx):
        """
        Dibuixa una esfera + etiqueta al landmark. Si ja existia, el retira
        primer i el refà (per canvis de color per estat).
        """
        if idx >= len(self.state.landmarks):
            return
        lm = self.state.landmarks[idx]

        # Retira actor anterior si n'hi havia
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

        # Colors segons estat
        if idx == self.active_idx:
            color = COLOR_ACTIU
            radius_scale = 1.4
        elif lm.is_captured:
            color = COLOR_CAPTURAT
            radius_scale = 1.0
        else:
            color = COLOR_PENDENT
            radius_scale = 1.0

        # Esfera: radi proporcional a la vèrtebra (uns 2 mm típic)
        radius_mm = 1.2 * radius_scale
        sphere = pv.Sphere(radius=radius_mm, center=lm.virtual)
        actor = self.plotter.add_mesh(
            sphere, color=color, opacity=0.95,
            smooth_shading=True, specular=0.3,
            name=f"lm_sphere_{idx}",
            pickable=False,
        )
        self._landmark_actors[idx] = actor

        # Etiqueta text
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
        """Refresca la visualització de tots els landmarks (colors, etc.)."""
        # Traiem tots i re-dibuixem
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

    # ============================================================
    # FASE 2 — PALPAR AMB EL ROBOT
    # ============================================================
    def _build_phase2(self):
        wrap = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(wrap)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(10)

        # ---- ESQUERRA: llista landmarks amb estat ----
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

        # ---- CENTRE: viewer 3D amb la vèrtebra + landmarks visibles ----
        # Perquè l'usuari vegi on ha marcat cada punt mentre palpa el real.
        # El landmark actiu es destaca amb color blau i mida més gran.
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

        # Viewer 3D dedicat a la fase 2 (només visualització, sense picking)
        self.plotter_p2 = QtInteractor(center_v)
        self.plotter_p2.set_background(C_BG_WINDOW)
        cv_lay.addWidget(self.plotter_p2.interactor, 1)

        h.addWidget(center_v, 1)

        # ---- DRETA: informació del punt actiu + botó capturar ----
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

        # Preparem l'escena 3D de la fase 2 (vèrtebra + landmarks còpia)
        self._p2_actors = {"vertebra": None, "landmarks": {}, "labels": {}}
        self._setup_p2_scene()

        return wrap

    def _setup_p2_scene(self):
        """Carrega la vèrtebra al viewer 3D de la fase 2 (només visualització)."""
        if self.vertebra_mesh is None:
            return

        # Mateix criteri que la fase 1: headlight suau + material mate.
        try:
            self.plotter_p2.remove_all_lights()
        except Exception:
            pass
        self.plotter_p2.add_light(pv.Light(light_type="headlight",
                                            color="white", intensity=0.55))

        # Vèrtebra MATE PROFESSIONAL (idèntic estil a la fase 1).
        self._p2_actors["vertebra"] = self.plotter_p2.add_mesh(
            self.vertebra_mesh,
            color=(207, 200, 188),      # mateix pedra càlida mate que a la fase 1
            opacity=1.0,
            smooth_shading=True,
            specular=0.0,
            specular_power=1,
            ambient=0.65,
            diffuse=0.45,
            show_edges=False,
            pickable=False,             # a la fase 2 NO es marca res
        )
        self.plotter_p2.reset_camera()

    def _p2_redraw_landmarks(self):
        """Repinta els landmarks al viewer 3D de la fase 2."""
        if self.vertebra_mesh is None:
            return

        # Retirar actors anteriors
        for d in ("landmarks", "labels"):
            for idx, actor in list(self._p2_actors[d].items()):
                try:
                    self.plotter_p2.remove_actor(actor)
                except Exception:
                    pass
            self._p2_actors[d].clear()

        # Redibuixar tots amb colors segons estat
        for lm in self.state.landmarks:
            if lm.idx == self.active_idx:
                color = COLOR_ACTIU
                radius_mm = 1.8            # actiu més gran
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

        # Estat del robot
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

        # Refresc del viewer 3D per ressaltar el landmark actiu
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

        # Auto-avança al següent no capturat
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
        """
        Llegeix la pose actual del TCP del robot en mm (frame del robot).
        """
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

    # ============================================================
    # FASE 3 — CALIBRAR (KABSCH + ICP) I APLICAR
    # ============================================================
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

        # Bloc RMSE en 2 columnes: Kabsch inicial + ICP final
        rmse_row = QtWidgets.QHBoxLayout()
        rmse_row.setSpacing(14)

        self.p3_kabsch_box = self._make_rmse_box(
            "KABSCH INICIAL", "punt ↔ punt")
        rmse_row.addWidget(self.p3_kabsch_box["frame"])

        self.p3_icp_box = self._make_rmse_box(
            "ICP FINAL", "punt ↔ superfície")
        rmse_row.addWidget(self.p3_icp_box["frame"])
        cv.addLayout(rmse_row)

        # Veredicte
        self.p3_verdict = QtWidgets.QLabel("")
        self.p3_verdict.setAlignment(Qt.AlignCenter)
        self.p3_verdict.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: 16px; font-weight: 700; letter-spacing: 2px; padding: 8px;")
        cv.addWidget(self.p3_verdict)

        # Detall
        self.p3_details = QtWidgets.QLabel("")
        self.p3_details.setStyleSheet(
            f"color: {C_TEXT_DIM}; background: transparent; "
            f"font-size: {SIZE_SM}px; font-family: {FONT_MONO}; "
            f"line-height: 160%; padding: 12px 4px;")
        cv.addWidget(self.p3_details)

        cv.addStretch()

        # Botons
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

        # RMSE Kabsch
        self._set_rmse_value(self.p3_kabsch_box, self.state.stats_kabsch["rmse"],
                              color=C_TEXT_DIM)

        # RMSE ICP + veredicte basat en ICP (o Kabsch si no hi ha ICP)
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

        # Detall
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

        # Aplicar al robot: només si el veredicte no és insuficient
        self.p3_btn_apply.setEnabled(tone != "danger")

    def _p3_on_compute(self):
        try:
            self.state.compute(vertebra_polydata=self.vertebra_mesh)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Kabsch/ICP", f"Error: {e}")
            import traceback; traceback.print_exc()
            return
        self._p3_refresh()
        # Autosave del registry al disc perquè no es perdi si es tanca sense Apply.
        # Silent: si falla (sense pacient_dir, etc.), no bloqueja res.
        try:
            if self.registry.json_path:
                self.registry.save()
        except Exception as e:
            print(f"[registration] autosave failed: {e}")
        # Refresquem el combo + indicador: aquesta vèrtebra ara té ✓.
        self._refresh_vertebra_combo()
        self._refresh_vertebra_status_label()

    def _p3_on_apply(self):
        """
        Aplica el registre al workspace del robot. Si hi ha MÚLTIPLES
        vèrtebres amb T vàlida al registry, les envia totes juntes
        (cadascuna amb la seva T i STL). Els cargols del planning es
        reparteixen entre les vèrtebres pel centroid més proper.
        """
        if self.state.T is None:
            return
        if self.robot_workspace is None or not hasattr(
                self.robot_workspace, "load_registered_scene"):
            QtWidgets.QMessageBox.information(
                self, "Aplicar al Robot",
                "Registre calculat.\n\n"
                "El workspace del robot no implementa load_registered_scene().")
            return

        # Cargols del planning (comuns a totes les vèrtebres)
        screws = self._get_screws_from_planning()

        # Construïm el dict {codi: {"mesh": pv_mesh, "T": T_4x4}} amb TOTES
        # les vèrtebres que tinguin un registre vàlid. Carreguem l'STL si
        # no és la vèrtebra activa (la del plotter actual està en memòria).
        vertebres_multi = {}                                                    # dict final
        valid_codes = self.registry.names_with_valid_T()                        # noms amb T
        for code in valid_codes:                                                # iterem
            st = self.registry[code]                                            # estat
            if code == self.registry.active_name() and self.vertebra_mesh is not None:
                mesh = self.vertebra_mesh                                       # reutilitzem la carregada
            else:                                                               # cal carregar l'STL
                stl_path = self.vertebres.get(code)                             # ruta al disc
                if not stl_path or not os.path.exists(stl_path):
                    print(f"[registration] STL de '{code}' no trobat, es salta")
                    continue
                try:
                    mesh = pv.read(stl_path).clean().triangulate()              # llegim STL
                except Exception as e:
                    print(f"[registration] error carregant {stl_path}: {e}")
                    continue
            vertebres_multi[code] = {"mesh": mesh, "T": st.T}

        if not vertebres_multi:                                                 # defensiu
            QtWidgets.QMessageBox.warning(self, "Aplicar",
                "No hi ha cap vèrtebra amb registre vàlid.")
            return

        try:
            self.robot_workspace.load_registered_scene(
                vertebra_mesh=None,                                             # no fem servir el param antic
                screws=screws,
                T_ct_to_robot=None,                                             # ignorat quan es passa 'vertebres'
                vertebres=vertebres_multi,                                      # NOU: multi-vèrtebra
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
        """Recupera la llista de cargols planificats."""
        main = self.window()
        planning_ws = None
        if hasattr(main, "_workspaces"):
            planning_ws = main._workspaces.get("planning")
        if planning_ws is None:
            return []
        pw = getattr(planning_ws, "widget", None) or planning_ws
        return list(getattr(pw, "cargols", []))

    # ============================================================
    # NAVEGACIÓ ENTRE FASES
    # ============================================================
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

    # ============================================================
    # CLEANUP
    # ============================================================
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
