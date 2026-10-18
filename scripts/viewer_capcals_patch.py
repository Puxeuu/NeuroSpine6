"""
Integració del selector de capçals al viewer_cobot amb disseny de targetes.

Disseny: fila horitzontal de targetes (una per capçal) amb miniatura SVG
+ nom + longitud del TCP. La seleccionada queda ressaltada amb l'accent
blau del viewer. A sota, una fila fina amb la descripció del capçal actiu.

INSTRUCCIONS D'INTEGRACIÓ (idèntiques a la versió anterior):

    Al capdamunt de viewer_cobot.py, amb els altres imports:

        from scripts.viewer_capcals_patch import (
            capcal_manager_init,
            capcal_add_selector_to_options_panel,
        )

    Dins __init__, JUST DESPRÉS de:
        self.prims = load_collisions(...)

    Afegir:
        capcal_manager_init(self)

    Al final del bloc del panell OPTIONS a _build_ui:
        capcal_add_selector_to_options_panel(self, opt)
"""

import os                                                                       # importem el mòdul os
from qtpy import QtWidgets, QtCore, QtGui                                       # importem QtWidgets, QtCore, QtGui des de qtpy

from src.model.capcal_manager import CapcalManager                              # importem CapcalManager des de src.model.capcal_manager
from src.math.transforms import Rx, Ry, Rz, make_T                              # importem Rx, Ry, Rz, make_T des de src.math.transforms
from src.kinematics.forward import forward_kinematics                           # importem forward_kinematics des de src.kinematics.forward


# ==============================================================================
# PALETA (coincideix amb la del viewer principal per consistència)
# ==============================================================================

C_ACCENT      = "#0a84ff"       # blau iOS (mateix accent_q del viewer)
C_PANEL       = "#1c1c1e"                                                       # assignem C_PANEL
C_BORDER      = "#2c2c2e"                                                       # assignem C_BORDER
C_CARD_BG     = "#2a2a2c"                                                       # assignem C_CARD_BG
C_CARD_HOVER  = "#3a3a3c"                                                       # assignem C_CARD_HOVER
C_TEXT        = "#ffffff"                                                       # assignem C_TEXT
C_TEXT_DIM    = "#8e8e93"                                                       # assignem C_TEXT_DIM
C_TEXT_MUTED  = "#636366"                                                       # assignem C_TEXT_MUTED


# ==============================================================================
# CAPCAL_MANAGER_INIT
# ==============================================================================

def capcal_manager_init(viewer):                                                # definim la funció capcal_manager_init()
    """
    Crida-la al __init__ del viewer, JUST DESPRÉS de carregar prims.
    """
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))          # ROOT ← resultat de dirname()
    viewer.capcal_manager = CapcalManager(                                      # assignem viewer.capcal_manager
        yaml_path=os.path.join(ROOT, "config", "capcals.yaml"),
        config_root=os.path.join(ROOT, "config"),
    )
    viewer.capcal_manager.apply_to(viewer.robot, viewer.prims)                  # executem apply_to()

    print(f"[capcals] Carregats: {viewer.capcal_manager.noms()}")
    print(f"[capcals] Actiu:     {viewer.capcal_manager.active_nom}")


# ==============================================================================
# MINIATURA SVG per cada capçal
# ==============================================================================
#
# En comptes d'incloure imatges externes, generem una icona vectorial simple
# a partir del propi nom del capçal. Cada capçal té una silueta característica:
#   bola    -> línia + esfera al final
#   punxo   -> línia + con punxegut
#   broca   -> línia + hèlix (rombes)
#
# Si el nom no coincideix amb cap patró conegut, es dibuixa una punxa genèrica.
# ==============================================================================

def _svg_icon(nom_capcal, color_hex, size=48):                                  # definim la funció _svg_icon()
    """
    Retorna un QIcon amb la silueta del capçal renderitzada en SVG.
    Fons transparent, contorn amb el color donat.
    """
    n = nom_capcal.lower()                                                      # n ← resultat de lower()

    # Cos comú: un rectangle prim horitzontal que representa el mànec
    body = f'''
        <rect x="6" y="21" width="22" height="6" rx="1"
              fill="{color_hex}" opacity="0.55"/>
    '''

    if "bola" in n:                                                             # si "bola" in n
        tip = f'''<circle cx="36" cy="24" r="6" fill="{color_hex}"/>'''
    elif "punx" in n or "punt" in n or "aguja" in n or "agulla" in n:           # altrament si "punx" in n or "punt" in n or "aguja"...
        tip = f'''<polygon points="28,20 44,24 28,28" fill="{color_hex}"/>'''
    elif "broca" in n or "drill" in n or "fresa" in n:                          # altrament si "broca" in n or "drill" in n or "fres...
        # Hèlix estilitzada: 3 rombes petits al llarg de l'eix
        tip = f'''
            <polygon points="30,24 33,20 36,24 33,28" fill="{color_hex}"/>
            <polygon points="34,24 37,20 40,24 37,28" fill="{color_hex}"/>
            <polygon points="38,24 41,20 44,24 41,28" fill="{color_hex}"/>
        '''
    elif "bisturi" in n or "scalpel" in n or "tall" in n:                       # altrament si "bisturi" in n or "scalpel" in n or "...
        # Fulla triangular
        tip = f'''<polygon points="28,20 44,20 44,28 28,28" fill="{color_hex}"/>
                  <polygon points="44,20 44,28 40,24" fill="{color_hex}" opacity="0.6"/>'''
    else:                                                                       # en cas contrari
        # Genèric: petita bombeta arrodonida
        tip = f'''<ellipse cx="36" cy="24" rx="8" ry="4" fill="{color_hex}"/>'''

    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 48 48">
            <line x1="4" y1="24" x2="6" y2="24" stroke="{C_TEXT_MUTED}"
                  stroke-width="2" stroke-linecap="round"/>
            {body}
            {tip}
        </svg>
    '''

    # Renderitzar SVG -> QPixmap -> QIcon
    from qtpy.QtSvg import QSvgRenderer                                         # importem QSvgRenderer des de qtpy.QtSvg
    r = QSvgRenderer(svg.encode("utf-8"))                                       # r ← resultat de QSvgRenderer()
    pm = QtGui.QPixmap(size, size)                                              # pm ← resultat de QPixmap()
    pm.fill(QtCore.Qt.transparent)                                              # executem fill()
    painter = QtGui.QPainter(pm)                                                # painter ← resultat de QPainter()
    r.render(painter)                                                           # executem render()
    painter.end()                                                               # executem end()

    return QtGui.QIcon(pm)                                                      # retornem: QtGui.QIcon(pm)


# ==============================================================================
# TARGETA DE CAPÇAL
# ==============================================================================

class CapcalCard(QtWidgets.QPushButton):                                        # definim la classe CapcalCard
    """
    Targeta seleccionable per un capçal (dins del panell d'opcions).

    Estil monocrom i cinemàtic (coherent amb el diàleg modal):
      - Icona SVG a escala amb gradients
      - Nom del capçal en majúscules
      - TCP en mono-space discret
      - Hover: borde blanc lluminós
      - Checked: fons més clar amb borde blanc més marcat
    """

    HEIGHT = 118                                                                # assignem HEIGHT
    WIDTH  = 108                                                                # assignem WIDTH

    def __init__(self, capcal, parent=None):                                    # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.capcal = capcal                                                    # assignem self.capcal
        self.setCheckable(True)                                                 # executem setCheckable()
        self.setFixedSize(self.WIDTH, self.HEIGHT)                              # executem setFixedSize()
        self.setCursor(QtCore.Qt.PointingHandCursor)                            # executem setCursor()

        layout = QtWidgets.QVBoxLayout(self)                                    # layout ← resultat de QVBoxLayout()
        layout.setContentsMargins(8, 14, 8, 12)                                 # executem setContentsMargins()
        layout.setSpacing(6)                                                    # executem setSpacing()
        layout.setAlignment(QtCore.Qt.AlignCenter)                              # executem setAlignment()

        # Icona SVG monocroma amb gradient (mateixa lògica que el diàleg gran)
        self.icon_label = QtWidgets.QLabel()                                    # self.icon_label ← resultat de QLabel()
        pix = self._icon_pixmap(capcal.nom, size=44)                            # pix ← resultat de _icon_pixmap()
        self.icon_label.setPixmap(pix)                                          # executem setPixmap()
        self.icon_label.setAlignment(QtCore.Qt.AlignCenter)                     # executem setAlignment()
        self.icon_label.setStyleSheet("background: transparent; border: none;")  # executem setStyleSheet()
        layout.addWidget(self.icon_label, alignment=QtCore.Qt.AlignCenter)      # assignem layout.addWidget(self.icon_label, alignment

        # Nom del capçal
        self.name_label = QtWidgets.QLabel(capcal.nom.upper())                  # self.name_label ← resultat de QLabel()
        self.name_label.setAlignment(QtCore.Qt.AlignCenter)                     # executem setAlignment()
        self.name_label.setStyleSheet(                                          # executem setStyleSheet()
            f"background: transparent; border: none; "
            f"color: {C_TEXT}; font-weight: 700; font-size: 11px; "
            f"letter-spacing: 1.5px;"
        )
        layout.addWidget(self.name_label)                                       # executem addWidget()

        # TCP en mono-space
        tcp_mm = capcal.tcp_offset[0] * 1000.0                                  # assignem tcp_mm
        self.tcp_label = QtWidgets.QLabel(f"{tcp_mm:+.1f} mm")
        self.tcp_label.setAlignment(QtCore.Qt.AlignCenter)                      # executem setAlignment()
        self.tcp_label.setStyleSheet(                                           # executem setStyleSheet()
            "background: transparent; border: none; "
            "color: #6e6e73; font-size: 9px; font-weight: 500; "
            "letter-spacing: 0.5px; "
            "font-family: 'SF Mono', Consolas, monospace;"
        )
        layout.addWidget(self.tcp_label)                                        # executem addWidget()

        self._apply_style()                                                     # executem _apply_style()

    # -- Icona monocroma amb gradient (mateix look que el diàleg gran) --
    def _icon_pixmap(self, nom_capcal, size=44):                                # definim la funció _icon_pixmap()
        n = nom_capcal.lower()                                                  # n ← resultat de lower()
        color = "#f5f5f7"                                                       # assignem color
        if "bola" in n:                                                         # si "bola" in n
            svg = f"""<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'>
                <defs>
                    <radialGradient id='sph' cx='40%' cy='35%' r='60%'>
                        <stop offset='0%' stop-color='#ffffff' stop-opacity='0.95'/>
                        <stop offset='60%' stop-color='{color}' stop-opacity='0.85'/>
                        <stop offset='100%' stop-color='{color}' stop-opacity='0.55'/>
                    </radialGradient>
                </defs>
                <circle cx='50' cy='50' r='36' fill='url(#sph)'/>
                <circle cx='38' cy='38' r='7' fill='#ffffff' opacity='0.5'/>
            </svg>"""
        elif "punx" in n:                                                       # altrament si "punx" in n
            svg = f"""<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'>
                <defs>
                    <linearGradient id='pnx' x1='0%' y1='0%' x2='0%' y2='100%'>
                        <stop offset='0%' stop-color='#ffffff' stop-opacity='0.95'/>
                        <stop offset='100%' stop-color='{color}' stop-opacity='0.55'/>
                    </linearGradient>
                </defs>
                <polygon points='50,8 60,88 50,74 40,88' fill='url(#pnx)'/>
                <line x1='50' y1='8' x2='50' y2='74' stroke='#ffffff'
                      stroke-width='0.8' opacity='0.6'/>
            </svg>"""
        elif "broca" in n:                                                      # altrament si "broca" in n
            svg = f"""<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'>
                <defs>
                    <linearGradient id='br' x1='0%' y1='0%' x2='100%' y2='0%'>
                        <stop offset='0%' stop-color='{color}' stop-opacity='0.65'/>
                        <stop offset='50%' stop-color='#ffffff' stop-opacity='0.95'/>
                        <stop offset='100%' stop-color='{color}' stop-opacity='0.65'/>
                    </linearGradient>
                </defs>
                <rect x='42' y='12' width='16' height='62' rx='2' fill='url(#br)'/>
                <polygon points='42,74 58,74 50,94' fill='url(#br)'/>
                <path d='M 42 22 L 58 32 M 42 36 L 58 46 M 42 50 L 58 60 M 42 64 L 58 74'
                      stroke='#000000' stroke-width='0.6' opacity='0.35' fill='none'/>
            </svg>"""
        else:                                                                   # en cas contrari
            svg = f"""<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'>
                <rect x='25' y='25' width='50' height='50' rx='10' fill='{color}' opacity='0.8'/>
            </svg>"""

        ba = QtCore.QByteArray(svg.encode("utf-8"))                             # ba ← resultat de QByteArray()
        pix = QtGui.QPixmap(size, size)                                         # pix ← resultat de QPixmap()
        pix.fill(QtCore.Qt.transparent)                                         # executem fill()
        painter = QtGui.QPainter(pix)                                           # painter ← resultat de QPainter()
        painter.setRenderHint(QtGui.QPainter.Antialiasing)                      # executem setRenderHint()
        try:                                                                    # intentem el bloc següent
            from qtpy.QtSvg import QSvgRenderer                                 # importem QSvgRenderer des de qtpy.QtSvg
            renderer = QSvgRenderer(ba)                                         # renderer ← resultat de QSvgRenderer()
            renderer.render(painter)                                            # executem render()
        except Exception:                                                       # capturem excepció: Exception
            painter.setBrush(QtGui.QColor(color))                               # executem setBrush()
            painter.setPen(QtCore.Qt.NoPen)                                     # executem setPen()
            painter.drawEllipse(6, 6, size - 12, size - 12)                     # executem drawEllipse()
        painter.end()                                                           # executem end()
        return pix                                                              # retornem: pix

    def _apply_style(self):                                                     # definim la funció _apply_style()
        # Vora VERMELLA perquè l'operador vegi clarament que ha de triar el
        # capçal muntat físicament. La seleccionada queda ressaltada amb
        # vora més intensa i un lleuger glow intern.
        self.setStyleSheet("""
            CapcalCard {
                background-color: #0e0e10;
                border: 1.5px solid rgba(255, 59, 48, 0.55);
                border-radius: 12px;
                padding: 0px;
                text-align: center;
            }
            CapcalCard:hover {
                background-color: #1a1a1c;
                border: 2px solid rgba(255, 59, 48, 0.85);
            }
            CapcalCard:checked {
                background-color: #1a1a1c;
                border: 2.5px solid #ff3b30;
            }
        """)


# ==============================================================================
# WIDGET SELECTOR SENCER (targetes + descripció del capçal actiu)
# ==============================================================================

class CapcalSelector(QtWidgets.QWidget):                                        # definim la classe CapcalSelector
    """
    Widget complet amb:
        - Fila de targetes (una per capçal)
        - Fila subtil sota amb la descripció del capçal actiu

    Emet capcalChanged(nom_capcal) quan l'usuari canvia de capçal.
    """

    capcalChanged = QtCore.Signal(str)                                          # capcalChanged ← resultat de Signal()

    def __init__(self, manager, parent=None):                                   # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.manager = manager                                                  # assignem self.manager
        self.cards = {}                     # nom -> CapcalCard
        self._button_group = QtWidgets.QButtonGroup(self)                       # self._button_group ← resultat de QButtonGroup()
        self._button_group.setExclusive(True)                                   # executem setExclusive()

        layout = QtWidgets.QVBoxLayout(self)                                    # layout ← resultat de QVBoxLayout()
        layout.setContentsMargins(0, 4, 0, 0)                                   # executem setContentsMargins()
        layout.setSpacing(10)                                                   # executem setSpacing()

        # -- Fila de targetes ------------------------------------------------
        cards_container = QtWidgets.QWidget()                                   # cards_container ← resultat de QWidget()
        cards_container.setStyleSheet("background: transparent;")               # executem setStyleSheet()
        cards_row = QtWidgets.QHBoxLayout(cards_container)                      # cards_row ← resultat de QHBoxLayout()
        cards_row.setContentsMargins(0, 0, 0, 0)                                # executem setContentsMargins()
        cards_row.setSpacing(10)                                                # executem setSpacing()
        cards_row.setAlignment(QtCore.Qt.AlignLeft)                             # executem setAlignment()

        for nom in manager.noms():                                              # iterem: nom in manager.noms()
            cap = manager.capcals[nom]                                          # assignem cap
            card = CapcalCard(cap)                                              # card ← resultat de CapcalCard()
            card.setChecked(nom == manager.active_nom)                          # assignem card.setChecked(nom
            card.clicked.connect(lambda _checked, n=nom: self._on_card_clicked(n))  # card.clicked.connect(lambda _checked, n ← resultat de _on_card_clicked()
            self._button_group.addButton(card)                                  # executem addButton()
            self.cards[nom] = card                                              # assignem self.cards[nom]
            cards_row.addWidget(card)                                           # executem addWidget()

        cards_row.addStretch()                                                  # executem addStretch()
        layout.addWidget(cards_container)                                       # executem addWidget()

        # -- Fila descripció del capçal actiu (monocrom, cinemàtic) --------
        self.desc_frame = QtWidgets.QFrame()                                    # self.desc_frame ← resultat de QFrame()
        self.desc_frame.setStyleSheet("""
            QFrame {
                background-color: rgba(255, 255, 255, 0.04);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 8px;
            }
        """)
        desc_layout = QtWidgets.QHBoxLayout(self.desc_frame)                    # desc_layout ← resultat de QHBoxLayout()
        desc_layout.setContentsMargins(12, 8, 12, 8)                            # executem setContentsMargins()
        desc_layout.setSpacing(12)                                              # executem setSpacing()

        self.desc_dot = QtWidgets.QLabel("●")                                   # self.desc_dot ← resultat de QLabel()
        self.desc_dot.setStyleSheet(                                            # executem setStyleSheet()
            "color: #ffffff; background: transparent; "
            "border: none; font-size: 12px;"
        )
        desc_layout.addWidget(self.desc_dot)                                    # executem addWidget()

        self.desc_label = QtWidgets.QLabel("")                                  # self.desc_label ← resultat de QLabel()
        self.desc_label.setStyleSheet(                                          # executem setStyleSheet()
            f"color: {C_TEXT}; background: transparent; "
            f"border: none; font-size: 12px; font-weight: 500; "
            f"letter-spacing: 0.3px;"
        )
        desc_layout.addWidget(self.desc_label, 1)                               # executem addWidget()

        self.desc_tcp = QtWidgets.QLabel("")                                    # self.desc_tcp ← resultat de QLabel()
        self.desc_tcp.setStyleSheet(                                            # executem setStyleSheet()
            "color: #6e6e73; background: transparent; "
            "border: none; font-size: 11px; font-weight: 600; "
            "letter-spacing: 0.5px; "
            "font-family: 'SF Mono', Consolas, monospace;"
        )
        desc_layout.addWidget(self.desc_tcp)                                    # executem addWidget()

        layout.addWidget(self.desc_frame)                                       # executem addWidget()

        self._update_description(manager.active_nom)                            # executem _update_description()

    # --------------------------------------------------------------------
    def _on_card_clicked(self, nom):                                            # definim la funció _on_card_clicked()
        if nom == self.manager.active_nom:                                      # si nom == self.manager.active_nom
            # Ha clicat el que ja estava seleccionat, la mantenim marcada
            self.cards[nom].setChecked(True)                                    # self.cards[nom].setChecked(True)
            return                                                              # retornem sense valor
        self._update_description(nom)                                           # executem _update_description()
        self.capcalChanged.emit(nom)                                            # executem emit()

    def _update_description(self, nom):                                         # definim la funció _update_description()
        cap = self.manager.capcals[nom]                                         # assignem cap
        self.desc_label.setText(cap.descripcio)                                 # executem setText()
        tcp_mm = cap.tcp_offset[0] * 1000.0                                     # assignem tcp_mm
        self.desc_tcp.setText(f"TCP  {tcp_mm:6.1f} mm")

    def refresh_selection(self):                                                # definim la funció refresh_selection()
        """Sincronitza les targetes amb l'estat del manager (per si canvia des de fora)."""  # """Sincronitza les targetes amb l'estat del manager (per ...
        for nom, card in self.cards.items():                                    # iterem: nom, card in self.cards.items()
            card.setChecked(nom == self.manager.active_nom)                     # assignem card.setChecked(nom
        self._update_description(self.manager.active_nom)                       # executem _update_description()


# ==============================================================================
# AFEGIR EL SELECTOR AL PANELL OPTIONS
# ==============================================================================

def capcal_add_selector_to_options_panel(viewer, options_layout):               # definim la funció capcal_add_selector_to_options_panel()
    """
    Substitueix la fila 'Capçal actiu: [combo]' pel widget de targetes.

    Es col·loca al final del panell OPTIONS.
    """
    # Títol seguint el mateix estil que "OPTIONS"
    title = QtWidgets.QLabel("END-EFFECTOR")                                    # title ← resultat de QLabel()
    title.setObjectName("Title")     # aprofita l'estil ja definit al viewer
    title.setStyleSheet("margin-top: 8px;")                                     # executem setStyleSheet()
    options_layout.addWidget(title)                                             # executem addWidget()

    viewer.capcal_selector = CapcalSelector(viewer.capcal_manager)              # viewer.capcal_selector ← resultat de CapcalSelector()
    viewer.capcal_selector.capcalChanged.connect(                               # executem connect()
        lambda nom: _on_capcal_changed(viewer, nom)
    )
    options_layout.addWidget(viewer.capcal_selector)                            # executem addWidget()


# ==============================================================================
# HANDLER DEL CANVI DE CAPÇAL (idèntic funcionalment a la versió combo)
# ==============================================================================

def _on_capcal_changed(viewer, nom):                                            # definim la funció _on_capcal_changed()
    import pyvista as pv                                                        # importem el mòdul pyvista as pv
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))          # ROOT ← resultat de dirname()

    viewer.capcal_manager.set_active(nom)                                       # executem set_active()
    viewer.capcal_manager.apply_to(viewer.robot, viewer.prims)                  # executem apply_to()
    cap = viewer.capcal_manager.active                                          # assignem cap

    # ---- Recarregar mesh visual del capçal al plotter --------------------
    if "capcal" in viewer.stl_actors:                                           # si "capcal" in viewer.stl_actors
        viewer.plotter.remove_actor(viewer.stl_actors["capcal"])                # executem remove_actor()

    link = viewer.robot.links["capcal"]                                         # assignem link
    mesh_path = os.path.join(ROOT, "config", link.mesh)                         # mesh_path ← resultat de join()

    if not os.path.exists(mesh_path):                                           # si not os.path.exists(mesh_path)
        print(f"[capcals] AVÍS: no trobo el STL {mesh_path}")
        return                                                                  # retornem sense valor

    mesh = pv.read(mesh_path)                                                   # mesh ← resultat de read()
    R_vis = Rz(link.visual_rpy[2]) @ Ry(link.visual_rpy[1]) @ Rx(link.visual_rpy[0])  # R_vis ← resultat de Rz()
    T_vis = make_T(R_vis, link.visual_xyz)                                      # T_vis ← resultat de make_T()
    mesh.transform(T_vis, inplace=True)                                         # assignem mesh.transform(T_vis, inplace

    actor = viewer.plotter.add_mesh(                                            # assignem actor
        mesh,
        color=cap.color,
        smooth_shading=False,
        specular=0.4, specular_power=10,
        ambient=0.25, diffuse=0.75,
    )
    actor.user_matrix = viewer.poses["capcal"]                                  # assignem actor.user_matrix
    viewer.stl_actors["capcal"] = actor                                         # viewer.stl_actors["capcal"] = actor

    # ---- Recalcular FK amb el nou tcp_offset ------------------------------
    viewer.poses = forward_kinematics(viewer.robot, viewer.q)                   # viewer.poses ← resultat de forward_kinematics()

    # Amb el capçal nou el TCP es mou (l'offset canvia). Cal reasignar
    # la referència IK al nou TCP i posar tots els sliders a 0.
    # sync_target_to_current() ho fa tot: actualitza T_ref_pos, R_target_initial,
    # posa target_xyz i target_rpy a 0, refresca sliders i triads.
    if hasattr(viewer, "sync_target_to_current"):                               # si hasattr(viewer, "sync_target_to_curre...
        try:                                                                    # intentem el bloc següent
            viewer.sync_target_to_current()                                     # executem sync_target_to_current()
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[capcals] error sync target: {e}")

    # Refresc del triad del TCP actual (tip_actors, no target_actors)
    for actor_ in viewer.tip_actors:                                            # iterem: actor_ in viewer.tip_actors
        actor_.user_matrix = viewer.poses["tcp_link"]                           # assignem actor_.user_matrix

    viewer.plotter.update()                                                     # executem update()
    print(f"[capcals] Actiu ara: {cap.nom} (TCP a {cap.tcp_offset[0]*1000:.1f} mm)")