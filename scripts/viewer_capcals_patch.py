import os
from qtpy import QtWidgets, QtCore, QtGui

from src.model.capcal_manager import CapcalManager
from src.math.transforms import Rx, Ry, Rz, make_T
from src.kinematics.forward import forward_kinematics


C_ACCENT      = "#0a84ff"
C_PANEL       = "#1c1c1e"
C_BORDER      = "#2c2c2e"
C_CARD_BG     = "#2a2a2c"
C_CARD_HOVER  = "#3a3a3c"
C_TEXT        = "#ffffff"
C_TEXT_DIM    = "#8e8e93"
C_TEXT_MUTED  = "#636366"


def capcal_manager_init(viewer):
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    viewer.capcal_manager = CapcalManager(
        yaml_path=os.path.join(ROOT, "config", "capcals.yaml"),
        config_root=os.path.join(ROOT, "config"),
    )
    viewer.capcal_manager.apply_to(viewer.robot, viewer.prims)

    print(f"[capcals] Carregats: {viewer.capcal_manager.noms()}")
    print(f"[capcals] Actiu:     {viewer.capcal_manager.active_nom}")


def _svg_icon(nom_capcal, color_hex, size=48):
    n = nom_capcal.lower()

    body = f'''
        <rect x="6" y="21" width="22" height="6" rx="1"
              fill="{color_hex}" opacity="0.55"/>
    '''

    if "bola" in n:
        tip = f'''<circle cx="36" cy="24" r="6" fill="{color_hex}"/>'''
    elif "punx" in n or "punt" in n or "aguja" in n or "agulla" in n:
        tip = f'''<polygon points="28,20 44,24 28,28" fill="{color_hex}"/>'''
    elif "broca" in n or "drill" in n or "fresa" in n:
        tip = f'''
            <polygon points="30,24 33,20 36,24 33,28" fill="{color_hex}"/>
            <polygon points="34,24 37,20 40,24 37,28" fill="{color_hex}"/>
            <polygon points="38,24 41,20 44,24 41,28" fill="{color_hex}"/>
        '''
    elif "bisturi" in n or "scalpel" in n or "tall" in n:
        tip = f'''<polygon points="28,20 44,20 44,28 28,28" fill="{color_hex}"/>
                  <polygon points="44,20 44,28 40,24" fill="{color_hex}" opacity="0.6"/>'''
    else:
        tip = f'''<ellipse cx="36" cy="24" rx="8" ry="4" fill="{color_hex}"/>'''

    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 48 48">
            <line x1="4" y1="24" x2="6" y2="24" stroke="{C_TEXT_MUTED}"
                  stroke-width="2" stroke-linecap="round"/>
            {body}
            {tip}
        </svg>
    '''

    from qtpy.QtSvg import QSvgRenderer
    r = QSvgRenderer(svg.encode("utf-8"))
    pm = QtGui.QPixmap(size, size)
    pm.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pm)
    r.render(painter)
    painter.end()

    return QtGui.QIcon(pm)


class CapcalCard(QtWidgets.QPushButton):
    HEIGHT = 118
    WIDTH  = 108

    def __init__(self, capcal, parent=None):
        super().__init__(parent)
        self.capcal = capcal
        self.setCheckable(True)
        self.setFixedSize(self.WIDTH, self.HEIGHT)
        self.setCursor(QtCore.Qt.PointingHandCursor)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 14, 8, 12)
        layout.setSpacing(6)
        layout.setAlignment(QtCore.Qt.AlignCenter)

        self.icon_label = QtWidgets.QLabel()
        pix = self._icon_pixmap(capcal.nom, size=44)
        self.icon_label.setPixmap(pix)
        self.icon_label.setAlignment(QtCore.Qt.AlignCenter)
        self.icon_label.setStyleSheet("background: transparent; border: none;")
        layout.addWidget(self.icon_label, alignment=QtCore.Qt.AlignCenter)

        self.name_label = QtWidgets.QLabel(capcal.nom.upper())
        self.name_label.setAlignment(QtCore.Qt.AlignCenter)
        self.name_label.setStyleSheet(
            f"background: transparent; border: none; "
            f"color: {C_TEXT}; font-weight: 700; font-size: 11px; "
            f"letter-spacing: 1.5px;"
        )
        layout.addWidget(self.name_label)

        tcp_mm = capcal.tcp_offset[0] * 1000.0
        self.tcp_label = QtWidgets.QLabel(f"{tcp_mm:+.1f} mm")
        self.tcp_label.setAlignment(QtCore.Qt.AlignCenter)
        self.tcp_label.setStyleSheet(
            "background: transparent; border: none; "
            "color: #6e6e73; font-size: 9px; font-weight: 500; "
            "letter-spacing: 0.5px; "
            "font-family: 'SF Mono', Consolas, monospace;"
        )
        layout.addWidget(self.tcp_label)

        self._apply_style()

    def _icon_pixmap(self, nom_capcal, size=44):
        n = nom_capcal.lower()
        color = "#f5f5f7"
        if "bola" in n:
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
        elif "punx" in n:
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
        elif "broca" in n:
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
        else:
            svg = f"""<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'>
                <rect x='25' y='25' width='50' height='50' rx='10' fill='{color}' opacity='0.8'/>
            </svg>"""

        ba = QtCore.QByteArray(svg.encode("utf-8"))
        pix = QtGui.QPixmap(size, size)
        pix.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        try:
            from qtpy.QtSvg import QSvgRenderer
            renderer = QSvgRenderer(ba)
            renderer.render(painter)
        except Exception:
            painter.setBrush(QtGui.QColor(color))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(6, 6, size - 12, size - 12)
        painter.end()
        return pix

    def _apply_style(self):
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


class CapcalSelector(QtWidgets.QWidget):
    capcalChanged = QtCore.Signal(str)

    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.manager = manager
        self.cards = {}
        self._button_group = QtWidgets.QButtonGroup(self)
        self._button_group.setExclusive(True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 0)
        layout.setSpacing(10)

        cards_container = QtWidgets.QWidget()
        cards_container.setStyleSheet("background: transparent;")
        cards_row = QtWidgets.QHBoxLayout(cards_container)
        cards_row.setContentsMargins(0, 0, 0, 0)
        cards_row.setSpacing(10)
        cards_row.setAlignment(QtCore.Qt.AlignLeft)

        for nom in manager.noms():
            cap = manager.capcals[nom]
            card = CapcalCard(cap)
            card.setChecked(nom == manager.active_nom)
            card.clicked.connect(lambda _checked, n=nom: self._on_card_clicked(n))
            self._button_group.addButton(card)
            self.cards[nom] = card
            cards_row.addWidget(card)

        cards_row.addStretch()
        layout.addWidget(cards_container)

        self.desc_frame = QtWidgets.QFrame()
        self.desc_frame.setStyleSheet("""
            QFrame {
                background-color: rgba(255, 255, 255, 0.04);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 8px;
            }
        """)
        desc_layout = QtWidgets.QHBoxLayout(self.desc_frame)
        desc_layout.setContentsMargins(12, 8, 12, 8)
        desc_layout.setSpacing(12)

        self.desc_dot = QtWidgets.QLabel("●")
        self.desc_dot.setStyleSheet(
            "color: #ffffff; background: transparent; "
            "border: none; font-size: 12px;"
        )
        desc_layout.addWidget(self.desc_dot)

        self.desc_label = QtWidgets.QLabel("")
        self.desc_label.setStyleSheet(
            f"color: {C_TEXT}; background: transparent; "
            f"border: none; font-size: 12px; font-weight: 500; "
            f"letter-spacing: 0.3px;"
        )
        desc_layout.addWidget(self.desc_label, 1)

        self.desc_tcp = QtWidgets.QLabel("")
        self.desc_tcp.setStyleSheet(
            "color: #6e6e73; background: transparent; "
            "border: none; font-size: 11px; font-weight: 600; "
            "letter-spacing: 0.5px; "
            "font-family: 'SF Mono', Consolas, monospace;"
        )
        desc_layout.addWidget(self.desc_tcp)

        layout.addWidget(self.desc_frame)

        self._update_description(manager.active_nom)

    def _on_card_clicked(self, nom):
        if nom == self.manager.active_nom:
            self.cards[nom].setChecked(True)
            return
        self._update_description(nom)
        self.capcalChanged.emit(nom)

    def _update_description(self, nom):
        cap = self.manager.capcals[nom]
        self.desc_label.setText(cap.descripcio)
        tcp_mm = cap.tcp_offset[0] * 1000.0
        self.desc_tcp.setText(f"TCP  {tcp_mm:6.1f} mm")

    def refresh_selection(self):
        for nom, card in self.cards.items():
            card.setChecked(nom == self.manager.active_nom)
        self._update_description(self.manager.active_nom)


def capcal_add_selector_to_options_panel(viewer, options_layout):
    title = QtWidgets.QLabel("END-EFFECTOR")
    title.setObjectName("Title")
    title.setStyleSheet("margin-top: 8px;")
    options_layout.addWidget(title)

    viewer.capcal_selector = CapcalSelector(viewer.capcal_manager)
    viewer.capcal_selector.capcalChanged.connect(
        lambda nom: _on_capcal_changed(viewer, nom)
    )
    options_layout.addWidget(viewer.capcal_selector)


def _on_capcal_changed(viewer, nom):
    import pyvista as pv
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    viewer.capcal_manager.set_active(nom)
    viewer.capcal_manager.apply_to(viewer.robot, viewer.prims)
    cap = viewer.capcal_manager.active

    if "capcal" in viewer.stl_actors:
        viewer.plotter.remove_actor(viewer.stl_actors["capcal"])

    link = viewer.robot.links["capcal"]
    mesh_path = os.path.join(ROOT, "config", link.mesh)

    if not os.path.exists(mesh_path):
        print(f"[capcals] AVÍS: no trobo el STL {mesh_path}")
        return

    mesh = pv.read(mesh_path)
    R_vis = Rz(link.visual_rpy[2]) @ Ry(link.visual_rpy[1]) @ Rx(link.visual_rpy[0])
    T_vis = make_T(R_vis, link.visual_xyz)
    mesh.transform(T_vis, inplace=True)

    actor = viewer.plotter.add_mesh(
        mesh,
        color=cap.color,
        smooth_shading=False,
        specular=0.4, specular_power=10,
        ambient=0.25, diffuse=0.75,
    )
    actor.user_matrix = viewer.poses["capcal"]
    viewer.stl_actors["capcal"] = actor

    viewer.poses = forward_kinematics(viewer.robot, viewer.q)

    if hasattr(viewer, "sync_target_to_current"):
        try:
            viewer.sync_target_to_current()
        except Exception as e:
            print(f"[capcals] error sync target: {e}")

    for actor_ in viewer.tip_actors:
        actor_.user_matrix = viewer.poses["tcp_link"]

    viewer.plotter.update()
    print(f"[capcals] Actiu ara: {cap.nom} (TCP a {cap.tcp_offset[0]*1000:.1f} mm)")

