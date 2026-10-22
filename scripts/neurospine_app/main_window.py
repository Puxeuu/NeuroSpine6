import os

from qtpy import QtWidgets, QtCore

from .theme import (
    C_BG_WINDOW, C_TEXT, C_TEXT_DIM, C_ACCENT, C_DANGER,
    SIZE_BASE, global_stylesheet,
)
from .header import Header
from .sidebar import Sidebar
from .mando_service import get_mando_service
from .preferences import load_preferences, PreferencesDialog


WORKSPACE_TITLES = {
    "planning":     "SURGICAL PLANNING",
    "robot":        "ROBOT CONTROL",
    "registration": "TAC ↔ ROBOT REGISTRATION",
    "navigation":   "SURGICAL NAVIGATION",
}


class NeuroSpineMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NeuroSpine Navigator")
        self.showMaximized()
        self.setStyleSheet(global_stylesheet())

        self.prefs = load_preferences()

        _this_dir = os.path.dirname(os.path.abspath(__file__))
        _root = os.path.dirname(os.path.dirname(_this_dir))
        self._pacient_id = self.prefs.get("pacient_id", "839201")
        self._pacient_dir = os.path.join(_root, "data", "pacients", self._pacient_id)

        self.mando_service = get_mando_service(
            port=self.prefs.get("mando_port", "COM5"),
            baud=int(self.prefs.get("mando_baudrate", 115200)),
        )
        self.mando_service.connection_ok.connect(self._on_mando_status)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self.header = Header()
        self.header.emergencyStop.connect(self._on_estop)
        root.addWidget(self.header)

        body = QtWidgets.QWidget()
        bl = QtWidgets.QHBoxLayout(body)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(0)

        self.sidebar = Sidebar()
        self.sidebar.workspaceRequested.connect(self._switch_workspace)
        self.sidebar.settingsRequested.connect(self._open_settings)
        bl.addWidget(self.sidebar)

        self.stack = QtWidgets.QStackedWidget()
        self.stack.setStyleSheet(f"background-color: {C_BG_WINDOW};")
        bl.addWidget(self.stack, 1)

        root.addWidget(body, 1)

        self._workspaces = {}

        self.mando_service.start()

        self._switch_workspace("robot")

        self._refresh_status_indicators()
        self._status_timer = QtCore.QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status_indicators)
        self._status_timer.start(1000)

    def _switch_workspace(self, name):
        if name not in self._workspaces:
            self._workspaces[name] = self._create_workspace(name)
            if self._workspaces[name] is not None:
                self.stack.addWidget(self._workspaces[name])

        w = self._workspaces.get(name)
        if w is not None:
            self.stack.setCurrentWidget(w)

        self.header.set_workspace_name(WORKSPACE_TITLES.get(name, name))
        self.sidebar.set_active(name)

    def _create_workspace(self, name):
        loading = self._make_loading_screen(name)
        self.stack.addWidget(loading)
        self.stack.setCurrentWidget(loading)
        QtWidgets.QApplication.processEvents()

        try:
            if name == "planning":
                from .workspaces.planning import PlanningWorkspace
                w = PlanningWorkspace(mando_service=self.mando_service)
            elif name == "robot":
                from .workspaces.robot import RobotWorkspace
                w = RobotWorkspace()
            elif name == "registration":
                from .workspaces.registration import RegistrationWorkspace
                w = RegistrationWorkspace(
                    pacient_dir=self._pacient_dir,
                    robot_workspace=self._workspaces.get("robot"),
                )
                w.registrationUpdated.connect(self._on_registration_updated)
            elif name == "navigation":
                from .workspaces.navigation import NavigationWorkspace
                w = NavigationWorkspace(
                    mando_service=self.mando_service,
                    planning_workspace=self._workspaces.get("planning"),
                    robot_workspace=self._workspaces.get("robot"),
                )
            else:
                w = None
        except Exception as e:
            print(f"[main_window] Error creant workspace '{name}': {e}")
            import traceback; traceback.print_exc()
            w = self._make_error_screen(name, str(e))

        self.stack.removeWidget(loading)
        loading.deleteLater()
        return w

    def _make_loading_screen(self, name):
        w = QtWidgets.QWidget()
        w.setStyleSheet(f"background-color: {C_BG_WINDOW};")
        v = QtWidgets.QVBoxLayout(w)
        v.setAlignment(QtCore.Qt.AlignCenter)
        v.setSpacing(20)

        icon = QtWidgets.QLabel("◈")
        icon.setAlignment(QtCore.Qt.AlignCenter)
        icon.setStyleSheet(f"color: {C_ACCENT}; background: transparent; font-size: 48px;")
        v.addWidget(icon)

        lbl = QtWidgets.QLabel(f"Carregant {WORKSPACE_TITLES.get(name, name)}...")
        lbl.setAlignment(QtCore.Qt.AlignCenter)
        lbl.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: {SIZE_BASE}px; letter-spacing: 1px;")
        v.addWidget(lbl)
        return w

    def _make_error_screen(self, name, msg):
        w = QtWidgets.QWidget()
        w.setStyleSheet(f"background-color: {C_BG_WINDOW};")
        v = QtWidgets.QVBoxLayout(w)
        v.setAlignment(QtCore.Qt.AlignCenter)
        v.setSpacing(16)

        icon = QtWidgets.QLabel("⚠")
        icon.setAlignment(QtCore.Qt.AlignCenter)
        icon.setStyleSheet(f"color: {C_DANGER}; font-size: 48px;")
        v.addWidget(icon)

        lbl = QtWidgets.QLabel(f"Error carregant workspace '{name}'")
        lbl.setAlignment(QtCore.Qt.AlignCenter)
        lbl.setStyleSheet(f"color: {C_TEXT}; font-size: 15px; font-weight: 600;")
        v.addWidget(lbl)

        err = QtWidgets.QLabel(msg)
        err.setAlignment(QtCore.Qt.AlignCenter)
        err.setWordWrap(True)
        err.setStyleSheet(f"color: {C_TEXT_DIM}; font-size: 11px; padding: 20px; font-family: monospace;")
        v.addWidget(err)
        return w

    def _on_mando_status(self, connected):
        self.header.set_mando_status(connected)

    def _refresh_status_indicators(self):
        self.header.set_mando_status(self.mando_service.is_connected())

        robot_ws = self._workspaces.get("robot")
        if robot_ws is not None and hasattr(robot_ws, "is_robot_connected"):
            self.header.set_robot_status(robot_ws.is_robot_connected())
        else:
            self.header.set_robot_status(False)

        matrix_path = os.path.join(self._pacient_dir, "T_registre.npy")
        self.header.set_registration_status(os.path.exists(matrix_path))

    def _on_registration_updated(self, valid):
        self.header.set_registration_status(valid)

    def _open_settings(self):
        dlg = PreferencesDialog(self)
        dlg.exec_()
        self.prefs = load_preferences()
        self.header.set_patient(self.prefs.get("pacient_id", "?"))

    def _on_estop(self):
        robot_ws = self._workspaces.get("robot")
        if robot_ws is not None and hasattr(robot_ws, "emergency_stop"):
            try:
                robot_ws.emergency_stop()
            except Exception as e:
                print(f"[e-stop] Error aturant robot: {e}")

        QtWidgets.QMessageBox.critical(
            self, "EMERGENCY STOP",
            "S'ha activat l'aturada d'emergència.\n\n"
            "El robot ha rebut l'ordre d'aturar-se. Confirma físicament "
            "que el contactor s'ha obert i procedeix segons el protocol."
        )

    def closeEvent(self, event):
        self._status_timer.stop()
        self.mando_service.stop()

        for name, w in self._workspaces.items():
            if hasattr(w, "cleanup"):
                try:
                    w.cleanup()
                except Exception as e:
                    print(f"[cleanup {name}] {e}")

        event.accept()

