"""
Coordinador de modes del RobotWorkspace.

RESPONSABILITATS:

    1. HIDE_REDUNDANT_WIDGETS()
       Amaga els botons del viewer_cobot original que ara són redundants
       amb el nou selector de 5 modes (btn_mode, btn_mando, etc.).

    2. Per cada canvi de mode:
         old_mode.exit()   → deixa el sistema en estat conegut
         new_mode.enter()  → configura el viewer per aquest mode 

    3. El J6 (broca) el gestiona j6_drill.py. Aquí només ens assegurem
       que, en entrar a un mode que no sigui IK5, el J6 és en POSITION_MODE
       i dins del send_q (normalment ja hi és: disarm() ho ha fet).

    4. SINCRONITZA EL TARGET IK ↔ TCP ACTUAL en entrar a IK6/IK5:
       Sense això, els sliders RPY apliquen rotacions relatives a una
       R_target_initial obsoleta (del moment previ que es va estar en IK)
       i el braç fa salts bruscos.

FILOSOFIA DEL RESET NET:

    Abans d'entrar a qualsevol mode nou, sempre passem per un estat "idle":
        - broca parada
        - mando_control OFF
        - mirror OFF (deixa el motor sense enviaments actius)
        - handguide subprocess mort si n'hi havia
        - flag _ik5dof desactivada

    D'aquesta manera, els modes no s'interfereixen entre si.
"""

from qtpy import QtWidgets                                                      # importem QtWidgets des de qtpy
from .j6_drill import ensure_j6_position_hold                                   # J6 sempre fixat on és


def hide_redundant_widgets(viewer):                                             # definim la funció hide_redundant_widgets()
    """
    Amaga tots els widgets del viewer_cobot que ara són redundants amb la
    UI nova de l'app. Fa espai perquè el viewer 3D creixi.
    """
    # 1. Botons de mode duplicats amb el nostre selector
    for attr in ("btn_mode", "btn_mando", "btn_mando_control", "btn_mando_zero"):  # iterem: attr in ("btn_mode", "btn_mando", "bt...
        w = getattr(viewer, attr, None)                                         # w ← resultat de getattr()
        if w is not None:                                                       # si w is not None
            w.setVisible(False)                                                 # executem setVisible()

    # 2. Botons operatius del viewer (re-exposats a l'ActionBar)
    for attr in ("btn_zero", "btn_connect", "btn_mirror", "btn_routes", "btn_stop"):  # iterem: attr in ("btn_zero", "btn_connect", "...
        w = getattr(viewer, attr, None)                                         # w ← resultat de getattr()
        if w is not None:                                                       # si w is not None
            w.setVisible(False)                                                 # executem setVisible()

    # 3. Panells de sliders JOINTS i CARTESIAN: els reutilitzem al context
    #    panel via reparent. Els traiem del layout original.
    for attr in ("joints_panel", "cart_panel"):                                 # iterem: attr in ("joints_panel", "cart_panel")
        w = getattr(viewer, attr, None)                                         # w ← resultat de getattr()
        if w is not None:                                                       # si w is not None
            w.setParent(None)                                                   # executem setParent()

    # 4. Consola de text del viewer (amagada però viva per rebre escriptures)
    console = getattr(viewer, "console_text", None)                             # console ← resultat de getattr()
    if console is not None:                                                     # si console is not None
        console.setVisible(False)                                               # executem setVisible()

    # 5. Panell OPTIONS del viewer amagat via els seus fills coneguts
    for attr in ("chk_collisions", "chk_target", "capcal_selector"):            # iterem: attr in ("chk_collisions", "chk_targe...
        w = getattr(viewer, attr, None)                                         # w ← resultat de getattr()
        if w is not None:                                                       # si w is not None
            frame = w                                                           # assignem frame
            for _ in range(8):                                                  # iterem: _ in range(8)
                frame = frame.parent()                                          # frame ← resultat de parent()
                if frame is None:                                               # si frame is None
                    break                                                       # sortim del bucle
                if isinstance(frame, QtWidgets.QFrame) and frame.objectName() == "ApplePanel":  # si isinstance(frame, QtWidgets.QFrame) a...
                    frame.setVisible(False)                                     # executem setVisible()
                    break                                                       # sortim del bucle

    # 6. QFrame "ApplePanel" pares de consola+botons i el bloc d'options
    for src_attr in ("btn_connect", "chk_collisions"):                          # iterem: src_attr in ("btn_connect", "chk_coll...
        widget = getattr(viewer, src_attr, None)                                # widget ← resultat de getattr()
        if widget is None:                                                      # si widget is None
            continue                                                            # saltem a la següent iteració
        parent = widget                                                         # assignem parent
        for _ in range(6):                                                      # iterem: _ in range(6)
            parent = parent.parent()                                            # parent ← resultat de parent()
            if parent is None:                                                  # si parent is None
                break                                                           # sortim del bucle
            if isinstance(parent, QtWidgets.QFrame) and parent.objectName() == "ApplePanel":  # si isinstance(parent, QtWidgets.QFrame) ...
                parent.setVisible(False)                                        # executem setVisible()
                break                                                           # sortim del bucle


# ============================================================================
# COMPATIBILITAT: alias del nom antic (per no trencar imports)
# ============================================================================
def hide_redundant_buttons(viewer):                                             # definim la funció hide_redundant_buttons()
    """Alias antic; ara amaga també més coses."""                               # """Alias antic; ara amaga també més coses."""
    hide_redundant_widgets(viewer)                                              # executem hide_redundant_widgets()


# ==============================================================================
# TRANSICIONS ENTRE MODES
# ==============================================================================

def enter_idle(viewer):                                                         # definim la funció enter_idle()
    """
    Deixa el sistema en un estat conegut i tranquil.
    Es crida abans de qualsevol canvi de mode.

    Apaguem:
      - control-del-cobot-pel-mando
      - connexió serial del mando (a més tanca la visualització 3D del
        mando al viewer, ja que _mando_disconnect crida _mando_unload_scene)
      - mirror (perquè el bucle IK deixi d'enviar paquets al bus)

    Així cada vegada que canviem de mode partim d'un estat conegut i,
    en particular, quan sortim de MANDO desapareix el visualitzador 3D
    del mando al costat del cobot.
    """
    # Aturar CONTROL del cobot pel mando
    _safe_call(viewer, "_on_mando_control_toggle", ensure_off=("btn_mando_control", False))  # executem _safe_call()

    # Aturar COMPLETAMENT el mando (serial + visualitzador 3D)
    _safe_call(viewer, "_on_mando_toggle", ensure_off=("btn_mando", False))     # executem _safe_call()

    # Aturar mirror perquè el bucle IK deixi d'enviar paquets al bus
    _safe_call(viewer, "_on_mirror_toggle", ensure_off=("btn_mirror", False))   # executem _safe_call()

    # Reset flags nostres
    viewer._ik5dof = False                                                      # assignem viewer._ik5dof


def _ensure_j6_included(viewer):
    """
    Xarxa de seguretat: garanteix que el J6 és en POSITION_MODE i dins del
    send_q del viewer. En el flux normal NO fa res, perquè el J6 ja no surt
    mai del send_q excepte mentre la broca gira (i el workspace atura la
    broca, fixant el J6 on és, abans de qualsevol canvi de mode).

    Si algun cas rar l'ha deixat fora, el fixa EXACTAMENT on és ara amb una
    posició fresca (vegeu j6_drill.hold_j6_here). Mai amb feedback vell ni
    amb mode_selection: això feia que el J6 desfés voltes en canviar de mode.
    """
    try:                                                                        # intentem el bloc següent
        ensure_j6_position_hold(viewer)                                         # executem ensure_j6_position_hold()
    except Exception as e:                                                      # capturem excepció: Exception as e
        print(f"[coordinator] error assegurant el J6: {e}")


def enter_fk(viewer):                                                           # definim la funció enter_fk()
    """Mode FK: sliders activats, IK desactivat."""                             # """Mode FK: sliders activats, IK desactivat."""
    viewer.mode = "FK"                                                          # assignem viewer.mode
    viewer._ik5dof = False                                                      # assignem viewer._ik5dof
    _ensure_j6_included(viewer)                                                 # reactivem J6 (per si veníem d'IK5)
    _apply_visual_mode(viewer)                                                  # executem _apply_visual_mode()


def enter_ik6(viewer):                                                          # definim la funció enter_ik6()
    """
    Mode IK 6DOF: DLS complet.

    IMPORTANT: cridem sync_target_to_current() DESPRÉS de posar viewer.mode
    a "IK". Això reajusta R_target_initial i target_rpy=0 amb la pose ACTUAL
    del TCP (que pot haver canviat en FK/MANDO). Sense això, el primer
    moviment d'un slider RPY dispararia un salt del braç.
    """
    viewer.mode = "IK"                                                          # assignem viewer.mode
    viewer._ik5dof = False                                                      # assignem viewer._ik5dof
    _ensure_j6_included(viewer)                                                 # reactivem J6 (per si veníem d'IK5)
    _sync_target_if_possible(viewer)                                            # executem _sync_target_if_possible()
    _apply_visual_mode(viewer)                                                  # executem _apply_visual_mode()


def enter_ik5(viewer):                                                          # definim la funció enter_ik5()
    """
    Mode IK 5DOF: IK amb J1-J5, el J6 reservat per a la broca.

      · L'IK (viewer._teleop_step) fa servir només J1-J5 i ignora el gir
        al voltant de l'eix de la broca: controla posició + direcció.
      · El J6 el gestiona j6_drill.J6Drill, que el workspace crida just
        després d'aquesta funció (arm): posició guardada + RESET (blau).
        Start → CURRENT_MODE. Stop → guarda la posició i RESET. En sortir
        d'IK5 (disarm) torna a POSITION a la posició guardada.

    IMPORTANT: mateix sync que a IK6 (veure enter_ik6).
    """
    viewer.mode = "IK"                                                          # el viewer segueix en mode IK
    viewer._ik5dof = True                                                       # IK de 5 graus (J1-J5)
    _sync_target_if_possible(viewer)                                          # executem _sync_target_if_possible()
    _apply_visual_mode(viewer)                                                  # executem _apply_visual_mode()


def enter_mando(viewer):                                                        # definim la funció enter_mando()
    """
    Mode MANDO: control teleoperat via mando físic.

    Comportament automàtic:
      1. Si el mando NO està connectat, s'intenta connectar al COM5.
         Si falla, l'error s'imprimeix i es queda en mode MANDO amb el
         control desactivat (el RobotWorkspace ho detectarà i decidirà).
      2. Un cop connectat, es fa auto-'Set Mando Zero' amb la pose actual
         del cobot com a referència. Així entrar al mode és instantani
         sense que l'usuari hagi de clicar res.
      3. S'activa el control del cobot pel mando.

    Cap reset del bus de motors: MANDO comparteix POSITION_MODE amb IK.
    """
    viewer.mode = "MANDO"                                                       # assignem viewer.mode
    viewer._ik5dof = False                                                      # assignem viewer._ik5dof
    _ensure_j6_included(viewer)                                                 # reactivem J6 (per si veníem d'IK5)
    _apply_visual_mode(viewer)                                                  # executem _apply_visual_mode()

    # ---- 1. Connectar mando si no ho està ----
    if not getattr(viewer, "mando_active", False):                              # si not getattr(viewer, "mando_active", F...
        try:                                                                    # intentem el bloc següent
            if hasattr(viewer, "btn_mando"):                                    # si hasattr(viewer, "btn_mando")
                viewer.btn_mando.setChecked(True)                               # executem setChecked()
                viewer._on_mando_toggle()                                       # executem _on_mando_toggle()
        except Exception as e:                                                  # capturem excepció: Exception as e
            print(f"[enter_mando] connexió mando: {e}")

    # Comprovem si de veritat s'ha connectat (el toggle pot haver fallat)
    if not getattr(viewer, "mando_active", False):                              # si not getattr(viewer, "mando_active", F...
        print("[enter_mando] mando NO disponible al port sèrie")                # executem print()
        return  # el workspace ho detectarà via viewer.mando_active

    # ---- 2. Auto Set Zero: pose actual del cobot com a referència ----
    try:                                                                        # intentem el bloc següent
        if hasattr(viewer, "_on_mando_set_zero"):                               # si hasattr(viewer, "_on_mando_set_zero")
            viewer._on_mando_set_zero()                                         # executem _on_mando_set_zero()
            print("[enter_mando] Set Zero automàtic")                           # executem print()
    except Exception as e:                                                      # capturem excepció: Exception as e
        print(f"[enter_mando] set zero: {e}")

    # ---- 3. Activar control del cobot pel mando ----
    try:                                                                        # intentem el bloc següent
        if hasattr(viewer, "btn_mando_control"):                                # si hasattr(viewer, "btn_mando_control")
            viewer.btn_mando_control.setEnabled(True)                           # executem setEnabled()
            if not viewer.btn_mando_control.isChecked():                        # si not viewer.btn_mando_control.isChecked()
                viewer.btn_mando_control.setChecked(True)                       # executem setChecked()
                viewer._on_mando_control_toggle()                               # executem _on_mando_control_toggle()
    except Exception as e:                                                      # capturem excepció: Exception as e
        print(f"[enter_mando] activació control: {e}")


def enter_handguide(viewer):                                                    # definim la funció enter_handguide()
    """
    Mode HAND-GUIDE: fixem el viewer en mode "FK" perquè el seu bucle
    100Hz no intenti resoldre IK ni enviï q als motors (el hand-guide
    envia currents directament).

    El workspace crearà el HandGuideController natiu i el timer que
    farà que el robot virtual segueixi el físic en temps real.
    """
    viewer.mode = "FK"                                                          # assignem viewer.mode
    viewer._ik5dof = False                                                      # assignem viewer._ik5dof
    _ensure_j6_included(viewer)                                                 # reactivem J6 (per si veníem d'IK5)
    _apply_visual_mode(viewer)                                                  # executem _apply_visual_mode()


# ==============================================================================
# HELPERS INTERNS
# ==============================================================================

def _sync_target_if_possible(viewer):                                           # definim la funció _sync_target_if_possible()
    """
    Crida viewer.sync_target_to_current() amb protecció.

    Aquest mètode viu al viewer_cobot.py i reassigna T_target, target_xyz,
    R_target_initial i target_rpy amb la pose ACTUAL del TCP (calculada per
    FK amb la q actual). Sense això, entrar a IK des de FK/MANDO deixa
    R_target_initial obsoleta i els sliders RPY fan salts.
    """
    fn = getattr(viewer, "sync_target_to_current", None)                        # fn ← resultat de getattr()
    if fn is None:                                                              # si fn is None
        # Viewer antic sense el mètode: no falla, però tampoc sincronitza.
        print("[coordinator] viewer sense sync_target_to_current(): els "       # executem print()
              "sliders RPY poden fer salts.")
        return                                                                  # retornem sense valor
    try:                                                                        # intentem el bloc següent
        fn()                                                                    # executem fn()
        print("[coordinator] target sincronitzat al TCP actual")                # executem print()
    except Exception as e:                                                      # capturem excepció: Exception as e
        print(f"[coordinator] error sincronitzant target: {e}")


def _safe_call(viewer, method_name, ensure_off=None):                           # definim la funció _safe_call()
    """
    Crida un mètode del viewer amb try/except.
    Si `ensure_off=("btn_name", False)`, primer garanteix l'estat del checkable.
    """
    if ensure_off is not None:                                                  # si ensure_off is not None
        btn_name, target_state = ensure_off                                     # assignem btn_name, target_state
        btn = getattr(viewer, btn_name, None)                                   # btn ← resultat de getattr()
        if btn is not None and btn.isCheckable() and btn.isChecked() != target_state:  # si btn is not None and btn.isCheckable()...
            btn.setChecked(target_state)                                        # executem setChecked()
        else:                                                                   # en cas contrari
            return   # el botó ja estava en l'estat desitjat, no cal cridar el toggle

    method = getattr(viewer, method_name, None)                                 # method ← resultat de getattr()
    if method is None:                                                          # si method is None
        return                                                                  # retornem sense valor
    try:                                                                        # intentem el bloc següent
        method()                                                                # executem method()
    except Exception as e:                                                      # capturem excepció: Exception as e
        print(f"[coordinator] {method_name}: {e}")


def _apply_visual_mode(viewer):                                                 # definim la funció _apply_visual_mode()
    """
    Alguns viewers tenen mètode _apply_mode_ui() per repintar l'estat.
    Si existeix, el cridem.
    """
    fn = getattr(viewer, "_apply_mode_ui", None)                                # fn ← resultat de getattr()
    if fn is not None:                                                          # si fn is not None
        try:                                                                    # intentem el bloc següent
            fn()                                                                # executem fn()
        except Exception:                                                       # capturem excepció: Exception
            pass                                                                # no fem res (placeholder)


# ==============================================================================
# TAULA DE MODES
# ==============================================================================

MODE_ENTER = {                                                                  # assignem MODE_ENTER
    "fk":        enter_fk,
    "ik6":       enter_ik6,
    "ik5":       enter_ik5,
    "mando":     enter_mando,
    "handguide": enter_handguide,
}