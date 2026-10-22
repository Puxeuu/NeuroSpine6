from qtpy import QtWidgets, QtCore

from .planning_widget import PlanningWidget
from ..theme import C_BG_WINDOW, C_TEXT_DIM, C_WARN


class PlanningWorkspace(QtWidgets.QWidget):
    cargolsChanged = QtCore.Signal(list)

    def __init__(self, mando_service=None, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        try:
            self.widget = PlanningWidget(mando_service=mando_service)
            self.widget.cargolsChanged.connect(self.cargolsChanged.emit)
            layout.addWidget(self.widget)
        except Exception as e:
            self.widget = None
            self._show_error(layout, e)

    def _show_error(self, layout, exc):
        w = QtWidgets.QWidget()
        w.setStyleSheet("background: transparent;")
        v = QtWidgets.QVBoxLayout(w)
        v.setAlignment(QtCore.Qt.AlignCenter)
        v.setSpacing(16)

        icon = QtWidgets.QLabel("⚠")
        icon.setAlignment(QtCore.Qt.AlignCenter)
        icon.setStyleSheet(f"color: {C_WARN}; font-size: 48px;")
        v.addWidget(icon)

        title = QtWidgets.QLabel("No s'ha pogut carregar el workspace de planificació")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("color: white; font-size: 15px; font-weight: 600;")
        v.addWidget(title)

        msg = QtWidgets.QLabel(
            f"{type(exc).__name__}: {exc}\n\n"
            "Comprova que els arxius del TAC (.nrrd) i medul·la (.stl)\n"
            "existeixin a les rutes definides al planning_widget.py"
        )
        msg.setAlignment(QtCore.Qt.AlignCenter)
        msg.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: 12px; padding: 20px;")
        msg.setWordWrap(True)
        v.addWidget(msg)

        layout.addWidget(w)

    def get_cargols(self):
        return self.widget.cargols if self.widget else []

    def cleanup(self):
        if self.widget:
            self.widget.cleanup()

