"""
Finestra principal de NeuroSpine Navigator.

Munta:
    · Header comú a dalt
    · Sidebar a l'esquerra
    · QStackedWidget al centre amb els 3 workspaces

Els workspaces es carreguen de manera diferida (lazy) per fer arrencar
l'app ràpid: només es crea el workspace la primera vegada que es clica.
"""

import os                                                                       # importem el mòdul os

from qtpy import QtWidgets, QtCore                                              # importem QtWidgets, QtCore des de qtpy

from .theme import (                                                            # importem ( des de .theme
    C_BG_WINDOW, C_TEXT, C_TEXT_DIM, C_ACCENT, C_DANGER,
    SIZE_BASE, global_stylesheet,
)
from .header import Header                                                      # importem Header des de .header
from .sidebar import Sidebar                                                    # importem Sidebar des de .sidebar
from .mando_service import get_mando_service                                    # importem get_mando_service des de .mando_service
from .preferences import load_preferences, PreferencesDialog                    # importem load_preferences, PreferencesDialog des de .preferences


WORKSPACE_TITLES = {                                                            # assignem WORKSPACE_TITLES
    "planning":     "SURGICAL PLANNING",
    "robot":        "ROBOT CONTROL",
    "registration": "TAC ↔ ROBOT REGISTRATION",
    "navigation":   "SURGICAL NAVIGATION",
}


class NeuroSpineMainWindow(QtWidgets.QMainWindow):                              # definim la classe NeuroSpineMainWindow

    def __init__(self):                                                         # definim la funció __init__()
        super().__init__()                                                      # executem super()
        self.setWindowTitle("NeuroSpine Navigator")                             # executem setWindowTitle()
        self.showMaximized()                                                    # executem showMaximized()
        self.setStyleSheet(global_stylesheet())                                 # executem setStyleSheet()

        # ---- Preferències ----
        self.prefs = load_preferences()                                         # self.prefs ← resultat de load_preferences()

        # ---- Directori del pacient (per registre, TAC, etc.) ----
        _this_dir = os.path.dirname(os.path.abspath(__file__))                  # _this_dir ← resultat de dirname()
        _root = os.path.dirname(os.path.dirname(_this_dir))    # projecte arrel
        self._pacient_id = self.prefs.get("pacient_id", "839201")               # self._pacient_id ← resultat de get()
        self._pacient_dir = os.path.join(_root, "data", "pacients", self._pacient_id)  # self._pacient_dir ← resultat de join()

        # ---- Servei de mando (compartit) ----
        # Port configurable des de preferences.json
        self.mando_service = get_mando_service(                                 # assignem self.mando_service
            port=self.prefs.get("mando_port", "COM5"),
            baud=int(self.prefs.get("mando_baudrate", 115200)),
        )
        self.mando_service.connection_ok.connect(self._on_mando_status)         # executem connect()

        # ---- Estructura de layout ----
        central = QtWidgets.QWidget()                                           # central ← resultat de QWidget()
        self.setCentralWidget(central)                                          # executem setCentralWidget()
        root = QtWidgets.QVBoxLayout(central)                                   # root ← resultat de QVBoxLayout()
        root.setContentsMargins(0, 0, 0, 0)                                     # executem setContentsMargins()
        root.setSpacing(0)                                                      # executem setSpacing()

        # Header
        self.header = Header()                                                  # self.header ← resultat de Header()
        self.header.emergencyStop.connect(self._on_estop)                       # executem connect()
        root.addWidget(self.header)                                             # executem addWidget()

        # Body: sidebar + stacked
        body = QtWidgets.QWidget()                                              # body ← resultat de QWidget()
        bl = QtWidgets.QHBoxLayout(body)                                        # bl ← resultat de QHBoxLayout()
        bl.setContentsMargins(0, 0, 0, 0)                                       # executem setContentsMargins()
        bl.setSpacing(0)                                                        # executem setSpacing()

        self.sidebar = Sidebar()                                                # self.sidebar ← resultat de Sidebar()
        self.sidebar.workspaceRequested.connect(self._switch_workspace)         # executem connect()
        self.sidebar.settingsRequested.connect(self._open_settings)             # executem connect()
        bl.addWidget(self.sidebar)                                              # executem addWidget()

        self.stack = QtWidgets.QStackedWidget()                                 # self.stack ← resultat de QStackedWidget()
        self.stack.setStyleSheet(f"background-color: {C_BG_WINDOW};")
        bl.addWidget(self.stack, 1)                                             # executem addWidget()

        root.addWidget(body, 1)                                                 # executem addWidget()

        # ---- Workspaces (lazy loading) ----
        self._workspaces = {}                                                   # assignem self._workspaces

        # Arrenca el mando
        self.mando_service.start()                                              # executem start()

        # Obrim primer el workspace ROBOT (el més lleuger, i el que ja tenies)
        self._switch_workspace("robot")                                         # executem _switch_workspace()

        # Actualitzem l'estat inicial dels indicadors
        self._refresh_status_indicators()                                       # executem _refresh_status_indicators()
        self._status_timer = QtCore.QTimer(self)                                # self._status_timer ← resultat de QTimer()
        self._status_timer.timeout.connect(self._refresh_status_indicators)     # executem connect()
        self._status_timer.start(1000)   # cada segon

    # ==========================================================================
    # WORKSPACE SWITCHING
    # ==========================================================================
    def _switch_workspace(self, name):                                          # definim la funció _switch_workspace()
        if name not in self._workspaces:                                        # si name not in self._workspaces
            self._workspaces[name] = self._create_workspace(name)               # self._workspaces[name] ← resultat de _create_workspace()
            if self._workspaces[name] is not None:                              # si self._workspaces[name] is not None
                self.stack.addWidget(self._workspaces[name])                    # executem addWidget()

        w = self._workspaces.get(name)                                          # w ← resultat de get()
        if w is not None:                                                       # si w is not None
            self.stack.setCurrentWidget(w)                                      # executem setCurrentWidget()

        self.header.set_workspace_name(WORKSPACE_TITLES.get(name, name))        # executem set_workspace_name()
        self.sidebar.set_active(name)                                           # executem set_active()

    def _create_workspace(self, name):                                          # definim la funció _create_workspace()
        """Crea un workspace amb pantalla de loading intermèdia."""             # """Crea un workspace amb pantalla de loading intermèdia."""
        loading = self._make_loading_screen(name)                               # loading ← resultat de _make_loading_screen()
        self.stack.addWidget(loading)                                           # executem addWidget()
        self.stack.setCurrentWidget(loading)                                    # executem setCurrentWidget()
        QtWidgets.QApplication.processEvents()                                  # executem processEvents()

        try:                                                                    # intentem el bloc següent
            if name == "planning":                                              # si name == "planning"
                from .workspaces.planning import PlanningWorkspace              # importem PlanningWorkspace des de .workspaces.planning
                w = PlanningWorkspace(mando_service=self.mando_service)         # w ← resultat de PlanningWorkspace()
            elif name == "robot":                                               # altrament si name == "robot"
                from .workspaces.robot import RobotWorkspace                    # importem RobotWorkspace des de .workspaces.robot
                w = RobotWorkspace()                                            # w ← resultat de RobotWorkspace()
            elif name == "registration":                                        # altrament si name == "registration"
                from .workspaces.registration import RegistrationWorkspace      # importem RegistrationWorkspace des de .workspaces.registration
                w = RegistrationWorkspace(                                      # assignem w
                    pacient_dir=self._pacient_dir,
                    robot_workspace=self._workspaces.get("robot"),
                )
                w.registrationUpdated.connect(self._on_registration_updated)    # executem connect()
            elif name == "navigation":                                          # altrament si name == "navigation"
                from .workspaces.navigation import NavigationWorkspace          # importem NavigationWorkspace des de .workspaces.navigation
                w = NavigationWorkspace(                                        # assignem w
                    mando_service=self.mando_service,
                    planning_workspace=self._workspaces.get("planning"),
                    robot_workspace=self._workspaces.get("robot"),
                )
            else:                                                               # en cas contrari
                w = None                                                        # assignem w
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[main_window] Error creant workspace '{name}': {e}")
            import traceback; traceback.print_exc()                             # importem el mòdul traceback; traceback.print_exc()
            w = self._make_error_screen(name, str(e))                           # w ← resultat de _make_error_screen()

        self.stack.removeWidget(loading)                                        # executem removeWidget()
        loading.deleteLater()                                                   # executem deleteLater()
        return w                                                                # retornem: w

    # ==========================================================================
    # PANTALLES INTERMÈDIES
    # ==========================================================================
    def _make_loading_screen(self, name):                                       # definim la funció _make_loading_screen()
        w = QtWidgets.QWidget()                                                 # w ← resultat de QWidget()
        w.setStyleSheet(f"background-color: {C_BG_WINDOW};")
        v = QtWidgets.QVBoxLayout(w)                                            # v ← resultat de QVBoxLayout()
        v.setAlignment(QtCore.Qt.AlignCenter)                                   # executem setAlignment()
        v.setSpacing(20)                                                        # executem setSpacing()

        icon = QtWidgets.QLabel("◈")                                            # icon ← resultat de QLabel()
        icon.setAlignment(QtCore.Qt.AlignCenter)                                # executem setAlignment()
        icon.setStyleSheet(f"color: {C_ACCENT}; background: transparent; font-size: 48px;")
        v.addWidget(icon)                                                       # executem addWidget()

        lbl = QtWidgets.QLabel(f"Carregant {WORKSPACE_TITLES.get(name, name)}...")
        lbl.setAlignment(QtCore.Qt.AlignCenter)                                 # executem setAlignment()
        lbl.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: {SIZE_BASE}px; letter-spacing: 1px;")
        v.addWidget(lbl)                                                        # executem addWidget()
        return w                                                                # retornem: w

    def _make_error_screen(self, name, msg):                                    # definim la funció _make_error_screen()
        w = QtWidgets.QWidget()                                                 # w ← resultat de QWidget()
        w.setStyleSheet(f"background-color: {C_BG_WINDOW};")
        v = QtWidgets.QVBoxLayout(w)                                            # v ← resultat de QVBoxLayout()
        v.setAlignment(QtCore.Qt.AlignCenter)                                   # executem setAlignment()
        v.setSpacing(16)                                                        # executem setSpacing()

        icon = QtWidgets.QLabel("⚠")                                            # icon ← resultat de QLabel()
        icon.setAlignment(QtCore.Qt.AlignCenter)                                # executem setAlignment()
        icon.setStyleSheet(f"color: {C_DANGER}; font-size: 48px;")
        v.addWidget(icon)                                                       # executem addWidget()

        lbl = QtWidgets.QLabel(f"Error carregant workspace '{name}'")
        lbl.setAlignment(QtCore.Qt.AlignCenter)                                 # executem setAlignment()
        lbl.setStyleSheet(f"color: {C_TEXT}; font-size: 15px; font-weight: 600;")
        v.addWidget(lbl)                                                        # executem addWidget()

        err = QtWidgets.QLabel(msg)                                             # err ← resultat de QLabel()
        err.setAlignment(QtCore.Qt.AlignCenter)                                 # executem setAlignment()
        err.setWordWrap(True)                                                   # executem setWordWrap()
        err.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: 11px; padding: 20px; font-family: monospace;")
        v.addWidget(err)                                                        # executem addWidget()
        return w                                                                # retornem: w

    # ==========================================================================
    # INDICADORS D'ESTAT
    # ==========================================================================
    def _on_mando_status(self, connected):                                      # definim la funció _on_mando_status()
        self.header.set_mando_status(connected)                                 # executem set_mando_status()

    def _refresh_status_indicators(self):                                       # definim la funció _refresh_status_indicators()
        # Mando
        self.header.set_mando_status(self.mando_service.is_connected())         # executem set_mando_status()

        # Robot: si el workspace del robot està carregat, comprova motor_controller
        robot_ws = self._workspaces.get("robot")                                # robot_ws ← resultat de get()
        if robot_ws is not None and hasattr(robot_ws, "is_robot_connected"):    # si robot_ws is not None and hasattr(robo...
            self.header.set_robot_status(robot_ws.is_robot_connected())         # executem set_robot_status()
        else:                                                                   # en cas contrari
            self.header.set_robot_status(False)                                 # executem set_robot_status()

        # Registre: mira si existeix el fitxer de matriu
        matrix_path = os.path.join(self._pacient_dir, "T_registre.npy")         # matrix_path ← resultat de join()
        self.header.set_registration_status(os.path.exists(matrix_path))        # executem set_registration_status()

    def _on_registration_updated(self, valid):                                  # definim la funció _on_registration_updated()
        """Cridat pel workspace Registration quan es calcula/esborra el registre."""  # """Cridat pel workspace Registration quan es calcula/esbo...
        self.header.set_registration_status(valid)                              # executem set_registration_status()

    def _open_settings(self):                                                   # definim la funció _open_settings()
        """Obre el diàleg de preferències."""                                   # """Obre el diàleg de preferències."""
        dlg = PreferencesDialog(self)                                           # dlg ← resultat de PreferencesDialog()
        dlg.exec_()                                                             # executem exec_()
        # Recarregar prefs després de tancar
        self.prefs = load_preferences()                                         # self.prefs ← resultat de load_preferences()
        # Actualitza el pacient al header per si s'ha canviat
        self.header.set_patient(self.prefs.get("pacient_id", "?"))              # executem set_patient()

    # ==========================================================================
    # EMERGENCY STOP
    # ==========================================================================
    def _on_estop(self):                                                        # definim la funció _on_estop()
        """Aturada d'emergència: intenta parar el robot i mostra un modal."""   # """Aturada d'emergència: intenta parar el robot i mostra ...
        # Si el workspace del robot està carregat: broca aturada + RESET de tots
        # els motors (el mateix que el botó STOP). Abans es cridava
        # mc.stop_all()/mc.disable(), que NO existeixen: l'E-STOP no aturava res.
        robot_ws = self._workspaces.get("robot")                                # robot_ws ← resultat de get()
        if robot_ws is not None and hasattr(robot_ws, "emergency_stop"):        # si el workspace ho suporta
            try:                                                                # intentem el bloc següent
                robot_ws.emergency_stop()                                       # executem emergency_stop()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[e-stop] Error aturant robot: {e}")

        # Missatge visual
        QtWidgets.QMessageBox.critical(                                         # executem critical()
            self, "EMERGENCY STOP",
            "S'ha activat l'aturada d'emergència.\n\n"
            "El robot ha rebut l'ordre d'aturar-se. Confirma físicament "
            "que el contactor s'ha obert i procedeix segons el protocol."
        )

    # ==========================================================================
    # CLEANUP
    # ==========================================================================
    def closeEvent(self, event):                                                # definim la funció closeEvent()
        self._status_timer.stop()                                               # executem stop()
        self.mando_service.stop()                                               # executem stop()

        for name, w in self._workspaces.items():                                # iterem: name, w in self._workspaces.items()
            if hasattr(w, "cleanup"):                                           # si hasattr(w, "cleanup")
                try:                                                            # intentem el bloc següent
                    w.cleanup()                                                 # executem cleanup()
                except Exception as e:                                          # capturem excepció: Exception as e
                    print(f"[cleanup {name}] {e}")

        event.accept()                                                          # executem accept()
