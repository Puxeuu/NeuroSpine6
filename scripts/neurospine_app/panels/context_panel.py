"""
Panells contextuals de la columna dreta del workspace del Robot.

DISSENY:
    Un ContextPanel principal amb un QStackedWidget que canvia segons
    el mode. Cada capa és un panell específic:

        - FKPanel        Sliders de joints (moure joint per joint)
        - IK6Panel       Sliders cartesians XYZ + RPY (6 DOF)
        - IK5Panel       Sliders cartesians (5 DOF) + panell de la broca
        - MandoPanel     Estat del mando + botó activar control
        - HandguidePanel Placeholder (per la fase B)

REUTILITZACIÓ:
    Els sliders del viewer intern (viewer.joints_panel, viewer.cart_panel)
    NO es dupliquen: es fa "reparent" cap a aquests panells per mantenir
    la mateixa lògica interna. Quan es canvia de mode, es mou l'un al
    lloc actiu i s'oculta l'altre.
"""

from qtpy import QtWidgets, QtCore                                              # importem QtWidgets, QtCore des de qtpy
from qtpy.QtCore import Qt                                                      # importem Qt des de qtpy.QtCore

from ..theme import (                                                           # importem ( des de ..theme
    C_PANEL, C_PANEL_ALT, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_SAFE, C_WARN,
    SIZE_XS, SIZE_SM,
)


# ==============================================================================
# HELPER: Títol de secció net
# ==============================================================================

def _section_title(text):                                                       # definim la funció _section_title()
    lbl = QtWidgets.QLabel(text)                                                # lbl ← resultat de QLabel()
    lbl.setStyleSheet(                                                          # executem setStyleSheet()
        f"background: transparent; border: none; "
        f"color: {C_TEXT}; font-size: {SIZE_SM}px; font-weight: 700; "
        f"letter-spacing: 1.5px;"
    )
    return lbl                                                                  # retornem: lbl


def _hint(text, color=C_TEXT_DIM):                                              # definim la funció _hint()
    lbl = QtWidgets.QLabel(text)                                                # lbl ← resultat de QLabel()
    lbl.setWordWrap(True)                                                       # executem setWordWrap()
    lbl.setStyleSheet(                                                          # executem setStyleSheet()
        f"background: transparent; border: none; "
        f"color: {color}; font-size: {SIZE_XS}px; "
        f"font-style: italic; padding: 4px 0;"
    )
    return lbl                                                                  # retornem: lbl


# ==============================================================================
# PANELLS INDIVIDUALS
# ==============================================================================

class FKPanel(QtWidgets.QWidget):                                               # definim la classe FKPanel
    """
    Mode FK: joints editables + cartesians read-only (només mirar).
    Els cartesians s'actualitzen sols quan mous els joints.
    """

    def __init__(self, viewer, parent=None):                                    # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.viewer = viewer                                                    # assignem self.viewer

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(0, 0, 0, 0)                                        # executem setContentsMargins()
        v.setSpacing(10)                                                        # executem setSpacing()

        v.addWidget(_section_title("JOINTS · FORWARD KINEMATICS"))              # executem addWidget()
        v.addWidget(_hint("Mou cada joint independentment."))                   # executem addWidget()

        # Els widgets es reparenten a runtime des del ContextPanel;
        # aquí només reservem els llocs.
        self._slot_joints = QtWidgets.QVBoxLayout()                             # self._slot_joints ← resultat de QVBoxLayout()
        self._slot_joints.setContentsMargins(0, 0, 0, 0)                        # executem setContentsMargins()
        v.addLayout(self._slot_joints)                                          # executem addLayout()

        v.addSpacing(6)                                                         # executem addSpacing()

        


class IK6Panel(QtWidgets.QWidget):                                              # definim la classe IK6Panel
    """
    Mode IK 6DOF: cartesians editables + joints read-only.
    """

    def __init__(self, viewer, parent=None):                                    # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.viewer = viewer                                                    # assignem self.viewer

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(0, 0, 0, 0)                                        # executem setContentsMargins()
        v.setSpacing(10)                                                        # executem setSpacing()

        v.addWidget(_section_title("TCP · INVERSE 6 DOF"))                      # executem addWidget()
        v.addWidget(_hint(                                                      # executem addWidget()
            "Controla la posició i orientació del TCP. "
            "Calculat amb DLS."))

        self._slot_cart = QtWidgets.QVBoxLayout()                               # self._slot_cart ← resultat de QVBoxLayout()
        self._slot_cart.setContentsMargins(0, 0, 0, 0)                          # executem setContentsMargins()
        v.addLayout(self._slot_cart)                                            # executem addLayout()

        v.addSpacing(6)                                                         # executem addSpacing()

        v.addWidget(_section_title("JOINTS · read-only"))                       # executem addWidget()
        v.addWidget(_hint(                                                      # executem addWidget()
            "Posició actual dels 6 joints (calculada per IK).",
            color=C_TEXT_MUTED))

        self._slot_joints = QtWidgets.QVBoxLayout()                             # self._slot_joints ← resultat de QVBoxLayout()
        self._slot_joints.setContentsMargins(0, 0, 0, 0)                        # executem setContentsMargins()
        v.addLayout(self._slot_joints)                                          # executem addLayout()

        v.addStretch()                                                          # executem addStretch()


class IK5Panel(QtWidgets.QWidget):                                              # definim la classe IK5Panel
    """
    Mode IK 5DOF: cartesians editables + broca (J6 lliure).
    Els joints NO es mostren en aquest mode: substituïts pel panell
    de la broca que s'insereix aquí des del RobotWorkspace.
    """

    def __init__(self, viewer, parent=None):                                    # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.viewer = viewer                                                    # assignem self.viewer

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(0, 0, 0, 0)                                        # executem setContentsMargins()
        v.setSpacing(10)                                                        # executem setSpacing()

        v.addWidget(_section_title("TCP · INVERSE 5 DOF"))                      # executem addWidget()
        v.addWidget(_hint(                                                      # executem addWidget()
            "Posiciona el TCP amb 5 graus. J6 queda lliure per la broca."))

        self._slot_cart = QtWidgets.QVBoxLayout()                               # self._slot_cart ← resultat de QVBoxLayout()
        self._slot_cart.setContentsMargins(0, 0, 0, 0)                          # executem setContentsMargins()
        v.addLayout(self._slot_cart)                                            # executem addLayout()

        v.addSpacing(4)                                                         # executem addSpacing()

        # El panell de la broca s'insereix aquí des del RobotWorkspace
        # (_install_broca_in_ik5_panel). NO afegim slot_joints en aquest mode.
        # Slot fictici perquè set_mode() no falli si algú intenta reparent-ar
        # els joints aquí (queda fora del layout visible).
        self._slot_joints = None                                                # assignem self._slot_joints

        # NO addStretch() aquí: volem que la broca (afegida després) s'expandeixi
        # per ocupar l'espai que li quedi, en comptes que l'stretch li mengi l'espai.


class MandoPanel(QtWidgets.QWidget):                                            # definim la classe MandoPanel
    """
    Panell MANDO minimalista.

    Tot el flux (connectar, Set Zero, activar control) es fa automàticament
    al canvi de mode al coordinator. Sortir del mode desconnecta el mando
    també automàticament.

    Aquí només mostrem l'estat visual (dot + text discret) perquè l'usuari
    vegi que el mando està operatiu.
    """

    def __init__(self, viewer, parent=None):                                    # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.viewer = viewer                                                    # assignem self.viewer

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(0, 0, 0, 0)                                        # executem setContentsMargins()
        v.setSpacing(12)                                                        # executem setSpacing()

        v.addWidget(_section_title("HAPTIC CONTROL · MANDO 6DOF"))              # executem addWidget()
        v.addWidget(_hint(                                                      # executem addWidget()
            "El mando controla el cobot amb 'Set Zero' automàtic. "
            "La pose actual del cobot és el punt de referència."
        ))

        # ---- Estat del mando: dot indicador + text discret ----
        status_row = QtWidgets.QHBoxLayout()                                    # status_row ← resultat de QHBoxLayout()
        status_row.setContentsMargins(4, 0, 4, 0)                               # executem setContentsMargins()
        status_row.setSpacing(8)                                                # executem setSpacing()

        self.lbl_dot = QtWidgets.QLabel("●")                                    # self.lbl_dot ← resultat de QLabel()
        self.lbl_dot.setStyleSheet(                                             # executem setStyleSheet()
            f"background: transparent; color: {C_TEXT_MUTED}; "
            f"font-size: 14px;"
        )
        status_row.addWidget(self.lbl_dot)                                      # executem addWidget()

        self.lbl_connection = QtWidgets.QLabel("Desconnectat")                  # self.lbl_connection ← resultat de QLabel()
        self.lbl_connection.setStyleSheet(                                      # executem setStyleSheet()
            f"background: transparent; color: {C_TEXT_DIM}; "
            f"font-size: {SIZE_SM}px; font-weight: 500; "
            f"letter-spacing: 0.3px;"
        )
        status_row.addWidget(self.lbl_connection)                               # executem addWidget()
        status_row.addStretch()                                                 # executem addStretch()

        v.addLayout(status_row)                                                 # executem addLayout()

        v.addStretch()                                                          # executem addStretch()

        # Timer per refrescar l'estat visual
        self._timer = QtCore.QTimer(self)                                       # self._timer ← resultat de QTimer()
        self._timer.timeout.connect(self._refresh_state)                        # executem connect()
        self._timer.start(200)                                                  # executem start()

    def _refresh_state(self):                                                   # definim la funció _refresh_state()
        """Sincronitza el dot i el label amb l'estat real del mando."""         # """Sincronitza el dot i el label amb l'estat real del man...
        connected = bool(getattr(self.viewer, "mando_active", False))           # connected ← resultat de bool()

        if connected:                                                           # si connected
            self.lbl_dot.setStyleSheet(                                         # executem setStyleSheet()
                f"background: transparent; color: {C_SAFE}; "
                f"font-size: 14px;"
            )
            self.lbl_connection.setText("Connectat")                            # executem setText()
            self.lbl_connection.setStyleSheet(                                  # executem setStyleSheet()
                f"background: transparent; color: {C_TEXT}; "
                f"font-size: {SIZE_SM}px; font-weight: 500; "
                f"letter-spacing: 0.3px;"
            )
        else:                                                                   # en cas contrari
            self.lbl_dot.setStyleSheet(                                         # executem setStyleSheet()
                f"background: transparent; color: {C_TEXT_MUTED}; "
                f"font-size: 14px;"
            )
            self.lbl_connection.setText("Desconnectat")                         # executem setText()
            self.lbl_connection.setStyleSheet(                                  # executem setStyleSheet()
                f"background: transparent; color: {C_TEXT_DIM}; "
                f"font-size: {SIZE_SM}px; font-weight: 500; "
                f"letter-spacing: 0.3px;"
            )


class HandguidePanel(QtWidgets.QWidget):                                        # definim la classe HandguidePanel
    """Placeholder pel mode hand-guide (fase B)."""                             # """Placeholder pel mode hand-guide (fase B)."""

    def __init__(self, viewer, parent=None):                                    # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.viewer = viewer                                                    # assignem self.viewer

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(0, 0, 0, 0)                                        # executem setContentsMargins()
        v.setSpacing(10)                                                        # executem setSpacing()

        v.addWidget(_section_title("HAND-GUIDE · GUIAT PER FORÇA"))             # executem addWidget()

        v.addWidget(_hint(                                                      # executem addWidget()
            "Mode actiu al subprocés extern. Segueix les instruccions "
            "de la consola nova.",
            color=C_WARN
        ))

        # Placeholder: llista d'encoders (per omplir a la fase B)
        placeholder = QtWidgets.QFrame()                                        # placeholder ← resultat de QFrame()
        placeholder.setStyleSheet(f"""
            QFrame {{
                background-color: {C_PANEL_ALT};
                border: 1px dashed {C_BORDER_STRONG};
                border-radius: 8px;
            }}
        """)
        pl = QtWidgets.QVBoxLayout(placeholder)                                 # pl ← resultat de QVBoxLayout()
        pl.setContentsMargins(20, 30, 20, 30)                                   # executem setContentsMargins()
        pl.setAlignment(Qt.AlignCenter)                                         # executem setAlignment()

        icon = QtWidgets.QLabel("✦")                                            # icon ← resultat de QLabel()
        icon.setAlignment(Qt.AlignCenter)                                       # executem setAlignment()
        icon.setStyleSheet(                                                     # executem setStyleSheet()
            f"background: transparent; color: {C_WARN}; font-size: 40px;"
        )
        pl.addWidget(icon)                                                      # executem addWidget()

        msg = QtWidgets.QLabel(                                                 # assignem msg
            "Integració nativa en desenvolupament\n"
            "(Fase B del refactor)"
        )
        msg.setAlignment(Qt.AlignCenter)                                        # executem setAlignment()
        msg.setWordWrap(True)                                                   # executem setWordWrap()
        msg.setStyleSheet(                                                      # executem setStyleSheet()
            f"background: transparent; color: {C_TEXT_DIM}; "
            f"font-size: {SIZE_XS}px; font-style: italic;"
        )
        pl.addWidget(msg)                                                       # executem addWidget()

        v.addWidget(placeholder)                                                # executem addWidget()
        v.addStretch()                                                          # executem addStretch()


# ==============================================================================
# CONTEXTPANEL — Contenidor principal
# ==============================================================================

class ContextPanel(QtWidgets.QFrame):                                           # definim la classe ContextPanel
    """
    Panell contextual que canvia segons el mode actiu.

    Ús:
        panel = ContextPanel(viewer_intern)
        panel.set_mode("fk")
        ...
    """

    WIDTH = 340                                                                 # assignem WIDTH

    def __init__(self, viewer, parent=None):                                    # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.viewer = viewer                                                    # assignem self.viewer
        self._current_mode = None                                               # assignem self._current_mode
        self.setFixedWidth(self.WIDTH)                                          # executem setFixedWidth()
        self.setStyleSheet(f"""
            ContextPanel {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 12px;
            }}
        """)

        v = QtWidgets.QVBoxLayout(self)                                         # v ← resultat de QVBoxLayout()
        v.setContentsMargins(14, 14, 14, 14)                                    # executem setContentsMargins()
        v.setSpacing(0)                                                         # executem setSpacing()

        # Stacked amb un panell per mode
        self.stack = QtWidgets.QStackedWidget()                                 # self.stack ← resultat de QStackedWidget()
        self.stack.setStyleSheet("background: transparent;")                    # executem setStyleSheet()
        v.addWidget(self.stack)                                                 # executem addWidget()

        self._panels = {                                                        # assignem self._panels
            "fk":        FKPanel(viewer),
            "ik6":       IK6Panel(viewer),
            "ik5":       IK5Panel(viewer),
            "mando":     MandoPanel(viewer),
            "handguide": HandguidePanel(viewer),
        }
        for name, panel in self._panels.items():                                # iterem: name, panel in self._panels.items()
            self.stack.addWidget(panel)                                         # executem addWidget()

        # ---- Timer per sincronitzar sliders read-only ----
        #
        # En mode FK, els sliders del cart_panel s'han d'actualitzar per
        # reflectir el TCP real (calculat via forward_kinematics).
        # El viewer intern només actualitza els sliders XYZ/RPY quan
        # està en IK; en FK, els deixa quiets.
        #
        # Aquest timer llegeix la q actual del viewer, calcula el TCP,
        # i escriu els valors als sliders. Fa 10 Hz (100 ms).
        self._sync_timer = QtCore.QTimer(self)                                  # self._sync_timer ← resultat de QTimer()
        self._sync_timer.timeout.connect(self._sync_readonly_sliders)           # executem connect()
        self._sync_timer.start(100)                                             # executem start()

    def set_mode(self, mode_id):                                                # definim la funció set_mode()
        """
        Canvia el panell visible al del mode indicat, i reparenta els
        dos widgets compartits (joints_panel i cart_panel) als slots
        adequats del panell actiu.

        FK   → joints editables al top, cart read-only al bottom
        IK6  → cart editable al top, joints read-only al bottom
        IK5  → igual que IK6, però la broca queda operativa
        MANDO/HANDGUIDE → no usen aquests widgets
        """
        p = self._panels.get(mode_id)                                           # p ← resultat de get()
        if p is None:                                                           # si p is None
            return                                                              # retornem sense valor

        self._current_mode = mode_id                                            # assignem self._current_mode

        joints = getattr(self.viewer, "joints_panel", None)                     # joints ← resultat de getattr()
        cart   = getattr(self.viewer, "cart_panel",   None)                     # cart ← resultat de getattr()

        # Reparent segons el mode
        if mode_id in ("fk", "ik6", "ik5"):                                     # si mode_id in ("fk", "ik6", "ik5")
            slot_joints = getattr(p, "_slot_joints", None)                      # slot_joints ← resultat de getattr()
            slot_cart   = getattr(p, "_slot_cart",   None)                      # slot_cart ← resultat de getattr()

            if joints is not None and slot_joints is not None:                  # si joints is not None and slot_joints is...
                self._insert_into_slot(joints, slot_joints)                     # executem _insert_into_slot()
                self._set_read_only(joints, mode_id != "fk")                    # executem _set_read_only()
            elif joints is not None and mode_id == "ik5":                       # altrament si joints is not None and mode_id == "ik5"
                # En mode IK5 no mostrem els joints (substituïts per la broca).
                # Els AMAGUEM i els traiem del layout perquè no apareguin
                # com a finestra flotant ni quedin en pantalla d'un mode previ.
                joints.setVisible(False)                                        # executem setVisible()
                joints.setParent(None)                                          # executem setParent()

            if cart is not None and slot_cart is not None:                      # si cart is not None and slot_cart is not...
                self._insert_into_slot(cart, slot_cart)                         # executem _insert_into_slot()
                self._set_read_only(cart, mode_id == "fk")                      # assignem self._set_read_only(cart, mode_id
        else:                                                                   # en cas contrari
            # Modes que no usen els sliders (mando, handguide): els deixem
            # orfes (setParent(None)) perquè no ocupin lloc en cap panell.
            for w in (joints, cart):                                            # iterem: w in (joints, cart)
                if w is not None:                                               # si w is not None
                    w.setParent(None)                                           # executem setParent()

        self.stack.setCurrentWidget(p)                                          # executem setCurrentWidget()

    def _insert_into_slot(self, widget, slot_layout):                           # definim la funció _insert_into_slot()
        """
        Reparenta un widget al layout indicat.
        Si ja hi era, només s'assegura que sigui visible.
        """
        if widget.parent() is slot_layout.parentWidget():                       # si widget.parent() is slot_layout.parent...
            widget.setVisible(True)                                             # executem setVisible()
            return                                                              # retornem sense valor
        # Si venia d'un mode que l'havia amagat (IK5 amb joints),
        # cal tornar-lo a fer visible explícitament.
        widget.setVisible(True)                                                 # executem setVisible()

        # Neteja el slot per si hi tenia un altre widget
        while slot_layout.count() > 0:                                          # mentre slot_layout.count() > 0
            item = slot_layout.takeAt(0)                                        # item ← resultat de takeAt()
            w = item.widget()                                                   # w ← resultat de widget()
            if w is not None:                                                   # si w is not None
                w.setParent(None)                                               # executem setParent()
        slot_layout.addWidget(widget)                                           # executem addWidget()
        widget.setVisible(True)                                                 # executem setVisible()

    def _set_read_only(self, widget, read_only):                                # definim la funció _set_read_only()
        """
        Aplica opacity i disabled als sliders/inputs del widget indicat
        per fer-los "read-only visual".

        Els sliders segueixen actualitzant-se sols quan mous el mode
        actiu, però l'usuari no els pot arrossegar.
        """
        try:                                                                    # intentem el bloc següent
            # Buscar sliders i spinboxes dins del widget
            for w in widget.findChildren(QtWidgets.QSlider):                    # iterem: w in widget.findChildren(QtWidgets.QS...
                w.setEnabled(not read_only)                                     # executem setEnabled()
            for w in widget.findChildren(QtWidgets.QDoubleSpinBox):             # iterem: w in widget.findChildren(QtWidgets.QD...
                w.setReadOnly(read_only)                                        # executem setReadOnly()
                w.setButtonSymbols(                                             # executem setButtonSymbols()
                    QtWidgets.QAbstractSpinBox.NoButtons if read_only
                    else QtWidgets.QAbstractSpinBox.UpDownArrows
                )
            for w in widget.findChildren(QtWidgets.QSpinBox):                   # iterem: w in widget.findChildren(QtWidgets.QS...
                w.setReadOnly(read_only)                                        # executem setReadOnly()
                w.setButtonSymbols(                                             # executem setButtonSymbols()
                    QtWidgets.QAbstractSpinBox.NoButtons if read_only
                    else QtWidgets.QAbstractSpinBox.UpDownArrows
                )

            # Opacity visual al widget sencer
            if read_only:                                                       # si read_only
                effect = QtWidgets.QGraphicsOpacityEffect(widget)               # effect ← resultat de QGraphicsOpacityEffect()
                effect.setOpacity(0.55)                                         # executem setOpacity()
                widget.setGraphicsEffect(effect)                                # executem setGraphicsEffect()
            else:                                                               # en cas contrari
                widget.setGraphicsEffect(None)                                  # executem setGraphicsEffect()
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[context_panel] read-only: {e}")

    # =============================================================
    # SINCRONITZACIÓ AUTOMÀTICA DELS SLIDERS READ-ONLY
    # =============================================================
    def _sync_readonly_sliders(self):                                           # definim la funció _sync_readonly_sliders()
        """
        Cada 100ms actualitza els sliders read-only segons l'estat real
        del viewer (posició TCP calculada a partir de q).

        Aquest mètode és el que fa que en mode FK, quan mous els joints,
        els sliders XYZ/RPY del panell inferior es moguin sols.
        """
        if self._current_mode is None:                                          # si self._current_mode is None
            return                                                              # retornem sense valor

        # En mode FK: actualitzar sliders CARTESIAN (read-only) amb el
        # TCP calculat via FK a partir dels joints actuals.
        if self._current_mode == "fk":                                          # si self._current_mode == "fk"
            self._update_cart_sliders_from_current_tcp()                        # executem _update_cart_sliders_from_current_tcp()

        # En mode IK6/IK5: actualitzar sliders JOINTS (read-only) amb
        # els valors reals de q (que el viewer intern ja actualitza sol
        # perquè està en mode IK).
        elif self._current_mode in ("ik6", "ik5"):                              # altrament si self._current_mode in ("ik6", "ik5")
            self._update_joint_sliders_from_current_q()                         # executem _update_joint_sliders_from_current_q()

    def _update_cart_sliders_from_current_tcp(self):                            # definim la funció _update_cart_sliders_from_current_tcp()
        """Actualitza els sliders XYZ/RPY amb la pose actual del TCP."""        # """Actualitza els sliders XYZ/RPY amb la pose actual del ...
        viewer = self.viewer                                                    # assignem viewer

        # Necessitem poses['tcp_link'] (calculat cada frame per FK del viewer)
        poses = getattr(viewer, "poses", None)                                  # poses ← resultat de getattr()
        if poses is None or "tcp_link" not in poses:                            # si poses is None or "tcp_link" not in poses
            return                                                              # retornem sense valor

        try:                                                                    # intentem el bloc següent
            T = poses["tcp_link"]                                               # assignem T
            xyz = T[:3, 3]                                                      # assignem xyz

            # RPY a partir de la rotació 3x3
            R = T[:3, :3]                                                       # assignem R
            rpy = self._rot_to_rpy(R)                                           # rpy ← resultat de _rot_to_rpy()

            xyz_sliders = getattr(viewer, "xyz_sliders", None) or []            # assignem xyz_sliders
            xyz_labels  = getattr(viewer, "xyz_labels",  None) or []            # assignem xyz_labels
            rpy_sliders = getattr(viewer, "rpy_sliders", None) or []            # assignem rpy_sliders
            rpy_labels  = getattr(viewer, "rpy_labels",  None) or []            # assignem rpy_labels

            # Bloquejar signals per no disparar l'IK en mode FK
            for i, slider in enumerate(xyz_sliders):                            # iterem: i, slider in enumerate(xyz_sliders)
                if i >= 3:                                                      # si i >= 3
                    break                                                       # sortim del bucle
                val = float(xyz[i])                                             # val ← resultat de float()
                slider.blockSignals(True)                                       # executem blockSignals()
                # Ampliem el rang si el valor cau fora del rang actual
                v_int = int(val * 1000)                                         # v_int ← resultat de int()
                if v_int < slider.minimum() or v_int > slider.maximum():        # si v_int < slider.minimum() or v_int > s...
                    slider.setMinimum(min(slider.minimum(), v_int - 100000))    # executem setMinimum()
                    slider.setMaximum(max(slider.maximum(), v_int + 100000))    # executem setMaximum()
                slider.setValue(v_int)                                          # executem setValue()
                slider.blockSignals(False)                                      # executem blockSignals()
                if i < len(xyz_labels):                                         # si i < len(xyz_labels)
                    xyz_labels[i].setText(f"{val:.3f}")

            for i, slider in enumerate(rpy_sliders):                            # iterem: i, slider in enumerate(rpy_sliders)
                if i >= 3:                                                      # si i >= 3
                    break                                                       # sortim del bucle
                val = float(rpy[i])                                             # val ← resultat de float()
                slider.blockSignals(True)                                       # executem blockSignals()
                v_int = int(val * 1000)                                         # v_int ← resultat de int()
                if v_int < slider.minimum() or v_int > slider.maximum():        # si v_int < slider.minimum() or v_int > s...
                    slider.setMinimum(min(slider.minimum(), v_int - 10000))     # executem setMinimum()
                    slider.setMaximum(max(slider.maximum(), v_int + 10000))     # executem setMaximum()
                slider.setValue(v_int)                                          # executem setValue()
                slider.blockSignals(False)                                      # executem blockSignals()
                if i < len(rpy_labels):                                         # si i < len(rpy_labels)
                    rpy_labels[i].setText(f"{val:.3f}")

        except Exception:                                                       # capturem qualsevol excepció
            # Silenci — es crida a 10 Hz, no volem espantar la consola
            pass                                                                # no fem res (placeholder)

    def _update_joint_sliders_from_current_q(self):                             # definim la funció _update_joint_sliders_from_current_q()
        """
        Actualitza els sliders JOINTS amb els valors reals de q.
        En mode IK, el viewer intern ja crida el sync quan cal, però
        per garantia refresquem els labels aquí també.
        """
        viewer = self.viewer                                                    # assignem viewer

        q = getattr(viewer, "q", None)                                          # q ← resultat de getattr()
        joint_sliders = getattr(viewer, "joint_sliders", None) or []            # assignem joint_sliders
        joint_labels  = getattr(viewer, "joint_labels",  None) or []            # assignem joint_labels

        if q is None:                                                           # si q is None
            return                                                              # retornem sense valor

        try:                                                                    # intentem el bloc següent
            for i, slider in enumerate(joint_sliders):                          # iterem: i, slider in enumerate(joint_sliders)
                if i >= len(q):                                                 # si i >= len(q)
                    break                                                       # sortim del bucle
                v_int = int(q[i] * 1000)                                        # v_int ← resultat de int()
                slider.blockSignals(True)                                       # executem blockSignals()
                slider.setValue(v_int)                                          # executem setValue()
                slider.blockSignals(False)                                      # executem blockSignals()
                if i < len(joint_labels):                                       # si i < len(joint_labels)
                    joint_labels[i].setText(f"{q[i]:.3f}")
        except Exception:                                                       # capturem excepció: Exception
            pass                                                                # no fem res (placeholder)

    @staticmethod                                                               # decorador staticmethod
    def _rot_to_rpy(R):                                                         # definim la funció _rot_to_rpy()
        """
        Converteix una matriu de rotació 3x3 a Roll-Pitch-Yaw (ZYX).
        """
        import numpy as np                                                      # importem el mòdul numpy as np
        # Estàndard aeroespacial: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        # Es descomposa mirant els elements de la matriu.
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)                               # sy ← resultat de sqrt()
        singular = sy < 1e-6                                                    # assignem singular

        if not singular:                                                        # si not singular
            roll  = np.arctan2( R[2, 1],  R[2, 2])                              # roll ← resultat de arctan2()
            pitch = np.arctan2(-R[2, 0],  sy)                                   # pitch ← resultat de arctan2()
            yaw   = np.arctan2( R[1, 0],  R[0, 0])                              # yaw ← resultat de arctan2()
        else:                                                                   # en cas contrari
            roll  = np.arctan2(-R[1, 2],  R[1, 1])                              # roll ← resultat de arctan2()
            pitch = np.arctan2(-R[2, 0],  sy)                                   # pitch ← resultat de arctan2()
            yaw   = 0.0                                                         # assignem yaw
        return np.array([roll, pitch, yaw])                                     # retornem: np.array([roll, pitch, yaw])
