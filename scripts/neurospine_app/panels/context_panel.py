from qtpy import QtWidgets, QtCore
from qtpy.QtCore import Qt

from ..theme import (
    C_PANEL, C_PANEL_ALT, C_BORDER, C_BORDER_STRONG,
    C_TEXT, C_TEXT_DIM, C_TEXT_MUTED,
    C_SAFE, C_WARN,
    SIZE_XS, SIZE_SM,
)


def _section_title(text):
    lbl = QtWidgets.QLabel(text)
    lbl.setStyleSheet(
        f"background: transparent; border: none; "
        f"color: {C_TEXT}; font-size: {SIZE_SM}px; font-weight: 700; "
        f"letter-spacing: 1.5px;"
    )
    return lbl


def _hint(text, color=C_TEXT_DIM):
    lbl = QtWidgets.QLabel(text)
    lbl.setWordWrap(True)
    lbl.setStyleSheet(
        f"background: transparent; border: none; "
        f"color: {color}; font-size: {SIZE_XS}px; "
        f"font-style: italic; padding: 4px 0;"
    )
    return lbl


class FKPanel(QtWidgets.QWidget):
    def __init__(self, viewer, parent=None):
        super().__init__(parent)
        self.viewer = viewer

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)

        v.addWidget(_section_title("JOINTS · FORWARD KINEMATICS"))
        v.addWidget(_hint("Mou cada joint independentment."))

        self._slot_joints = QtWidgets.QVBoxLayout()
        self._slot_joints.setContentsMargins(0, 0, 0, 0)
        v.addLayout(self._slot_joints)

        v.addSpacing(6)


class IK6Panel(QtWidgets.QWidget):
    def __init__(self, viewer, parent=None):
        super().__init__(parent)
        self.viewer = viewer

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)

        v.addWidget(_section_title("TCP · INVERSE 6 DOF"))
        v.addWidget(_hint(
            "Controla la posició i orientació del TCP. "
            "Calculat amb DLS."))

        self._slot_cart = QtWidgets.QVBoxLayout()
        self._slot_cart.setContentsMargins(0, 0, 0, 0)
        v.addLayout(self._slot_cart)

        v.addSpacing(6)

        v.addWidget(_section_title("JOINTS · read-only"))
        v.addWidget(_hint(
            "Posició actual dels 6 joints (calculada per IK).",
            color=C_TEXT_MUTED))

        self._slot_joints = QtWidgets.QVBoxLayout()
        self._slot_joints.setContentsMargins(0, 0, 0, 0)
        v.addLayout(self._slot_joints)

        v.addStretch()


class IK5Panel(QtWidgets.QWidget):
    def __init__(self, viewer, parent=None):
        super().__init__(parent)
        self.viewer = viewer

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)

        v.addWidget(_section_title("TCP · INVERSE 5 DOF"))
        v.addWidget(_hint(
            "Posiciona el TCP amb 5 graus. J6 queda lliure per la broca."))

        self._slot_cart = QtWidgets.QVBoxLayout()
        self._slot_cart.setContentsMargins(0, 0, 0, 0)
        v.addLayout(self._slot_cart)

        v.addSpacing(4)

        self._slot_joints = None


class MandoPanel(QtWidgets.QWidget):
    def __init__(self, viewer, parent=None):
        super().__init__(parent)
        self.viewer = viewer

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        v.addWidget(_section_title("HAPTIC CONTROL · MANDO 6DOF"))
        v.addWidget(_hint(
            "El mando controla el cobot amb 'Set Zero' automàtic. "
            "La pose actual del cobot és el punt de referència."
        ))

        status_row = QtWidgets.QHBoxLayout()
        status_row.setContentsMargins(4, 0, 4, 0)
        status_row.setSpacing(8)

        self.lbl_dot = QtWidgets.QLabel("●")
        self.lbl_dot.setStyleSheet(
            f"background: transparent; color: {C_TEXT_MUTED}; "
            f"font-size: 14px;"
        )
        status_row.addWidget(self.lbl_dot)

        self.lbl_connection = QtWidgets.QLabel("Desconnectat")
        self.lbl_connection.setStyleSheet(
            f"background: transparent; color: {C_TEXT_DIM}; "
            f"font-size: {SIZE_SM}px; font-weight: 500; "
            f"letter-spacing: 0.3px;"
        )
        status_row.addWidget(self.lbl_connection)
        status_row.addStretch()

        v.addLayout(status_row)

        v.addStretch()

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh_state)
        self._timer.start(200)

    def _refresh_state(self):
        connected = bool(getattr(self.viewer, "mando_active", False))

        if connected:
            self.lbl_dot.setStyleSheet(
                f"background: transparent; color: {C_SAFE}; "
                f"font-size: 14px;"
            )
            self.lbl_connection.setText("Connectat")
            self.lbl_connection.setStyleSheet(
                f"background: transparent; color: {C_TEXT}; "
                f"font-size: {SIZE_SM}px; font-weight: 500; "
                f"letter-spacing: 0.3px;"
            )
        else:
            self.lbl_dot.setStyleSheet(
                f"background: transparent; color: {C_TEXT_MUTED}; "
                f"font-size: 14px;"
            )
            self.lbl_connection.setText("Desconnectat")
            self.lbl_connection.setStyleSheet(
                f"background: transparent; color: {C_TEXT_DIM}; "
                f"font-size: {SIZE_SM}px; font-weight: 500; "
                f"letter-spacing: 0.3px;"
            )


class HandguidePanel(QtWidgets.QWidget):
    def __init__(self, viewer, parent=None):
        super().__init__(parent)
        self.viewer = viewer

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)

        v.addWidget(_section_title("HAND-GUIDE · GUIAT PER FORÇA"))

        v.addWidget(_hint(
            "Mode actiu al subprocés extern. Segueix les instruccions "
            "de la consola nova.",
            color=C_WARN
        ))

        placeholder = QtWidgets.QFrame()
        placeholder.setStyleSheet(f"""
            QFrame {{
                background-color: {C_PANEL_ALT};
                border: 1px dashed {C_BORDER_STRONG};
                border-radius: 8px;
            }}
        """)
        pl = QtWidgets.QVBoxLayout(placeholder)
        pl.setContentsMargins(20, 30, 20, 30)
        pl.setAlignment(Qt.AlignCenter)

        icon = QtWidgets.QLabel("✦")
        icon.setAlignment(Qt.AlignCenter)
        icon.setStyleSheet(
            f"background: transparent; color: {C_WARN}; font-size: 40px;"
        )
        pl.addWidget(icon)

        msg = QtWidgets.QLabel(
            "Integració nativa en desenvolupament\n"
            "(Fase B del refactor)"
        )
        msg.setAlignment(Qt.AlignCenter)
        msg.setWordWrap(True)
        msg.setStyleSheet(
            f"background: transparent; color: {C_TEXT_DIM}; "
            f"font-size: {SIZE_XS}px; font-style: italic;"
        )
        pl.addWidget(msg)

        v.addWidget(placeholder)
        v.addStretch()


class ContextPanel(QtWidgets.QFrame):
    WIDTH = 340

    def __init__(self, viewer, parent=None):
        super().__init__(parent)
        self.viewer = viewer
        self._current_mode = None
        self.setFixedWidth(self.WIDTH)
        self.setStyleSheet(f"""
            ContextPanel {{
                background-color: {C_PANEL};
                border: 1px solid {C_BORDER};
                border-radius: 12px;
            }}
        """)

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(14, 14, 14, 14)
        v.setSpacing(0)

        self.stack = QtWidgets.QStackedWidget()
        self.stack.setStyleSheet("background: transparent;")
        v.addWidget(self.stack)

        self._panels = {
            "fk":        FKPanel(viewer),
            "ik6":       IK6Panel(viewer),
            "ik5":       IK5Panel(viewer),
            "mando":     MandoPanel(viewer),
            "handguide": HandguidePanel(viewer),
        }
        for name, panel in self._panels.items():
            self.stack.addWidget(panel)

        self._sync_timer = QtCore.QTimer(self)
        self._sync_timer.timeout.connect(self._sync_readonly_sliders)
        self._sync_timer.start(100)

    def set_mode(self, mode_id):
        p = self._panels.get(mode_id)
        if p is None:
            return

        self._current_mode = mode_id

        joints = getattr(self.viewer, "joints_panel", None)
        cart   = getattr(self.viewer, "cart_panel",   None)

        if mode_id in ("fk", "ik6", "ik5"):
            slot_joints = getattr(p, "_slot_joints", None)
            slot_cart   = getattr(p, "_slot_cart",   None)

            if joints is not None and slot_joints is not None:
                self._insert_into_slot(joints, slot_joints)
                self._set_read_only(joints, mode_id != "fk")
            elif joints is not None and mode_id == "ik5":
                joints.setVisible(False)
                joints.setParent(None)

            if cart is not None and slot_cart is not None:
                self._insert_into_slot(cart, slot_cart)
                self._set_read_only(cart, mode_id == "fk")
        else:
            for w in (joints, cart):
                if w is not None:
                    w.setParent(None)

        self.stack.setCurrentWidget(p)

    def _insert_into_slot(self, widget, slot_layout):
        if widget.parent() is slot_layout.parentWidget():
            widget.setVisible(True)
            return
        widget.setVisible(True)

        while slot_layout.count() > 0:
            item = slot_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
        slot_layout.addWidget(widget)
        widget.setVisible(True)

    def _set_read_only(self, widget, read_only):
        try:
            for w in widget.findChildren(QtWidgets.QSlider):
                w.setEnabled(not read_only)
            for w in widget.findChildren(QtWidgets.QDoubleSpinBox):
                w.setReadOnly(read_only)
                w.setButtonSymbols(
                    QtWidgets.QAbstractSpinBox.NoButtons if read_only
                    else QtWidgets.QAbstractSpinBox.UpDownArrows
                )
            for w in widget.findChildren(QtWidgets.QSpinBox):
                w.setReadOnly(read_only)
                w.setButtonSymbols(
                    QtWidgets.QAbstractSpinBox.NoButtons if read_only
                    else QtWidgets.QAbstractSpinBox.UpDownArrows
                )

            if read_only:
                effect = QtWidgets.QGraphicsOpacityEffect(widget)
                effect.setOpacity(0.55)
                widget.setGraphicsEffect(effect)
            else:
                widget.setGraphicsEffect(None)
        except Exception as e:
            print(f"[context_panel] read-only: {e}")

    def _sync_readonly_sliders(self):
        if self._current_mode is None:
            return

        if self._current_mode == "fk":
            self._update_cart_sliders_from_current_tcp()

        elif self._current_mode in ("ik6", "ik5"):
            self._update_joint_sliders_from_current_q()

    def _update_cart_sliders_from_current_tcp(self):
        viewer = self.viewer

        poses = getattr(viewer, "poses", None)
        if poses is None or "tcp_link" not in poses:
            return

        try:
            T = poses["tcp_link"]
            xyz = T[:3, 3]

            R = T[:3, :3]
            rpy = self._rot_to_rpy(R)

            xyz_sliders = getattr(viewer, "xyz_sliders", None) or []
            xyz_labels  = getattr(viewer, "xyz_labels",  None) or []
            rpy_sliders = getattr(viewer, "rpy_sliders", None) or []
            rpy_labels  = getattr(viewer, "rpy_labels",  None) or []

            for i, slider in enumerate(xyz_sliders):
                if i >= 3:
                    break
                val = float(xyz[i])
                slider.blockSignals(True)
                v_int = int(val * 1000)
                if v_int < slider.minimum() or v_int > slider.maximum():
                    slider.setMinimum(min(slider.minimum(), v_int - 100000))
                    slider.setMaximum(max(slider.maximum(), v_int + 100000))
                slider.setValue(v_int)
                slider.blockSignals(False)
                if i < len(xyz_labels):
                    xyz_labels[i].setText(f"{val:.3f}")

            for i, slider in enumerate(rpy_sliders):
                if i >= 3:
                    break
                val = float(rpy[i])
                slider.blockSignals(True)
                v_int = int(val * 1000)
                if v_int < slider.minimum() or v_int > slider.maximum():
                    slider.setMinimum(min(slider.minimum(), v_int - 10000))
                    slider.setMaximum(max(slider.maximum(), v_int + 10000))
                slider.setValue(v_int)
                slider.blockSignals(False)
                if i < len(rpy_labels):
                    rpy_labels[i].setText(f"{val:.3f}")

        except Exception:
            pass

    def _update_joint_sliders_from_current_q(self):
        viewer = self.viewer

        q = getattr(viewer, "q", None)
        joint_sliders = getattr(viewer, "joint_sliders", None) or []
        joint_labels  = getattr(viewer, "joint_labels",  None) or []

        if q is None:
            return

        try:
            for i, slider in enumerate(joint_sliders):
                if i >= len(q):
                    break
                v_int = int(q[i] * 1000)
                slider.blockSignals(True)
                slider.setValue(v_int)
                slider.blockSignals(False)
                if i < len(joint_labels):
                    joint_labels[i].setText(f"{q[i]:.3f}")
        except Exception:
            pass

    @staticmethod
    def _rot_to_rpy(R):
        import numpy as np
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll  = np.arctan2( R[2, 1],  R[2, 2])
            pitch = np.arctan2(-R[2, 0],  sy)
            yaw   = np.arctan2( R[1, 0],  R[0, 0])
        else:
            roll  = np.arctan2(-R[1, 2],  R[1, 1])
            pitch = np.arctan2(-R[2, 0],  sy)
            yaw   = 0.0
        return np.array([roll, pitch, yaw])

