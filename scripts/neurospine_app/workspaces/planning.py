"""
Workspace de Planificació — wrapper prim del PlanningWidget.

El widget real és a planning_widget.py. Aquest fitxer només s'encarrega
d'instanciar-lo dins d'un layout net i gestionar-ne el cicle de vida.
"""

from qtpy import QtWidgets, QtCore                                              # importem QtWidgets, QtCore des de qtpy

from .planning_widget import PlanningWidget                                     # importem PlanningWidget des de .planning_widget
from ..theme import C_BG_WINDOW, C_TEXT_DIM, C_WARN                             # importem C_BG_WINDOW, C_TEXT_DIM, C_WARN des de ..theme


class PlanningWorkspace(QtWidgets.QWidget):                                     # definim la classe PlanningWorkspace

    cargolsChanged = QtCore.Signal(list)                                        # cargolsChanged ← resultat de Signal()

    def __init__(self, mando_service=None, parent=None):                        # definim la funció __init__()
        super().__init__(parent)                                                # executem super()
        self.setStyleSheet(f"background-color: {C_BG_WINDOW};")

        layout = QtWidgets.QVBoxLayout(self)                                    # layout ← resultat de QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)                                   # executem setContentsMargins()
        layout.setSpacing(0)                                                    # executem setSpacing()

        try:                                                                    # intentem el bloc següent
            self.widget = PlanningWidget(mando_service=mando_service)           # self.widget ← resultat de PlanningWidget()
            self.widget.cargolsChanged.connect(self.cargolsChanged.emit)        # executem connect()
            layout.addWidget(self.widget)                                       # executem addWidget()
        except Exception as e:                                                  # capturem excepció: Exception as e
            self.widget = None                                                  # assignem self.widget
            self._show_error(layout, e)                                         # executem _show_error()

    def _show_error(self, layout, exc):                                         # definim la funció _show_error()
        w = QtWidgets.QWidget()                                                 # w ← resultat de QWidget()
        w.setStyleSheet("background: transparent;")                             # executem setStyleSheet()
        v = QtWidgets.QVBoxLayout(w)                                            # v ← resultat de QVBoxLayout()
        v.setAlignment(QtCore.Qt.AlignCenter)                                   # executem setAlignment()
        v.setSpacing(16)                                                        # executem setSpacing()

        icon = QtWidgets.QLabel("⚠")                                            # icon ← resultat de QLabel()
        icon.setAlignment(QtCore.Qt.AlignCenter)                                # executem setAlignment()
        icon.setStyleSheet(f"color: {C_WARN}; font-size: 48px;")
        v.addWidget(icon)                                                       # executem addWidget()

        title = QtWidgets.QLabel("No s'ha pogut carregar el workspace de planificació")  # title ← resultat de QLabel()
        title.setAlignment(QtCore.Qt.AlignCenter)                               # executem setAlignment()
        title.setStyleSheet("color: white; font-size: 15px; font-weight: 600;")  # executem setStyleSheet()
        v.addWidget(title)                                                      # executem addWidget()

        msg = QtWidgets.QLabel(                                                 # assignem msg
            f"{type(exc).__name__}: {exc}\n\n"
            "Comprova que els arxius del TAC (.nrrd) i medul·la (.stl)\n"
            "existeixin a les rutes definides al planning_widget.py"
        )
        msg.setAlignment(QtCore.Qt.AlignCenter)                                 # executem setAlignment()
        msg.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: 12px; padding: 20px;")
        msg.setWordWrap(True)                                                   # executem setWordWrap()
        v.addWidget(msg)                                                        # executem addWidget()

        layout.addWidget(w)                                                     # executem addWidget()

    def get_cargols(self):                                                      # definim la funció get_cargols()
        return self.widget.cargols if self.widget else []                       # retornem: self.widget.cargols if self.widget el...

    def cleanup(self):                                                          # definim la funció cleanup()
        if self.widget:                                                         # si self.widget
            self.widget.cleanup()                                               # executem cleanup()
