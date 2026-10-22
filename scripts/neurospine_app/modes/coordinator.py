from qtpy import QtWidgets
from .j6_drill import ensure_j6_position_hold


def hide_redundant_widgets(viewer):
    for attr in ("btn_mode", "btn_mando", "btn_mando_control", "btn_mando_zero"):
        w = getattr(viewer, attr, None)
        if w is not None:
            w.setVisible(False)

    for attr in ("btn_zero", "btn_connect", "btn_mirror", "btn_routes", "btn_stop"):
        w = getattr(viewer, attr, None)
        if w is not None:
            w.setVisible(False)

    for attr in ("joints_panel", "cart_panel"):
        w = getattr(viewer, attr, None)
        if w is not None:
            w.setParent(None)

    console = getattr(viewer, "console_text", None)
    if console is not None:
        console.setVisible(False)

    for attr in ("chk_collisions", "chk_target", "capcal_selector"):
        w = getattr(viewer, attr, None)
        if w is not None:
            frame = w
            for _ in range(8):
                frame = frame.parent()
                if frame is None:
                    break
                if isinstance(frame, QtWidgets.QFrame) and frame.objectName() == "ApplePanel":
                    frame.setVisible(False)
                    break

    for src_attr in ("btn_connect", "chk_collisions"):
        widget = getattr(viewer, src_attr, None)
        if widget is None:
            continue
        parent = widget
        for _ in range(6):
            parent = parent.parent()
            if parent is None:
                break
            if isinstance(parent, QtWidgets.QFrame) and parent.objectName() == "ApplePanel":
                parent.setVisible(False)
                break


def hide_redundant_buttons(viewer):
    hide_redundant_widgets(viewer)


def enter_idle(viewer):
    _safe_call(viewer, "_on_mando_control_toggle", ensure_off=("btn_mando_control", False))

    _safe_call(viewer, "_on_mando_toggle", ensure_off=("btn_mando", False))

    _safe_call(viewer, "_on_mirror_toggle", ensure_off=("btn_mirror", False))

    viewer._ik5dof = False


def _ensure_j6_included(viewer):
    try:
        ensure_j6_position_hold(viewer)
    except Exception as e:
        print(f"[coordinator] error assegurant el J6: {e}")


def enter_fk(viewer):
    viewer.mode = "FK"
    viewer._ik5dof = False
    _ensure_j6_included(viewer)
    _apply_visual_mode(viewer)


def enter_ik6(viewer):
    viewer.mode = "IK"
    viewer._ik5dof = False
    _ensure_j6_included(viewer)
    _sync_target_if_possible(viewer)
    _apply_visual_mode(viewer)


def enter_ik5(viewer):
    viewer.mode = "IK"
    viewer._ik5dof = True
    _sync_target_if_possible(viewer)
    _apply_visual_mode(viewer)


def enter_mando(viewer):
    viewer.mode = "MANDO"
    viewer._ik5dof = False
    _ensure_j6_included(viewer)
    _apply_visual_mode(viewer)

    if not getattr(viewer, "mando_active", False):
        try:
            if hasattr(viewer, "btn_mando"):
                viewer.btn_mando.setChecked(True)
                viewer._on_mando_toggle()
        except Exception as e:
            print(f"[enter_mando] connexió mando: {e}")

    if not getattr(viewer, "mando_active", False):
        print("[enter_mando] mando NO disponible al port sèrie")
        return

    try:
        if hasattr(viewer, "_on_mando_set_zero"):
            viewer._on_mando_set_zero()
            print("[enter_mando] Set Zero automàtic")
    except Exception as e:
        print(f"[enter_mando] set zero: {e}")

    try:
        if hasattr(viewer, "btn_mando_control"):
            viewer.btn_mando_control.setEnabled(True)
            if not viewer.btn_mando_control.isChecked():
                viewer.btn_mando_control.setChecked(True)
                viewer._on_mando_control_toggle()
    except Exception as e:
        print(f"[enter_mando] activació control: {e}")


def enter_handguide(viewer):
    viewer.mode = "FK"
    viewer._ik5dof = False
    _ensure_j6_included(viewer)
    _apply_visual_mode(viewer)


def _sync_target_if_possible(viewer):
    fn = getattr(viewer, "sync_target_to_current", None)
    if fn is None:
        print("[coordinator] viewer sense sync_target_to_current(): els "
              "sliders RPY poden fer salts.")
        return
    try:
        fn()
        print("[coordinator] target sincronitzat al TCP actual")
    except Exception as e:
        print(f"[coordinator] error sincronitzant target: {e}")


def _safe_call(viewer, method_name, ensure_off=None):
    if ensure_off is not None:
        btn_name, target_state = ensure_off
        btn = getattr(viewer, btn_name, None)
        if btn is not None and btn.isCheckable() and btn.isChecked() != target_state:
            btn.setChecked(target_state)
        else:
            return

    method = getattr(viewer, method_name, None)
    if method is None:
        return
    try:
        method()
    except Exception as e:
        print(f"[coordinator] {method_name}: {e}")


def _apply_visual_mode(viewer):
    fn = getattr(viewer, "_apply_mode_ui", None)
    if fn is not None:
        try:
            fn()
        except Exception:
            pass


MODE_ENTER = {
    "fk":        enter_fk,
    "ik6":       enter_ik6,
    "ik5":       enter_ik5,
    "mando":     enter_mando,
    "handguide": enter_handguide,
}

