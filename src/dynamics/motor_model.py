"""
Conversió parell <-> corrent per als motors del cobot.

Aquest fitxer és el pont entre:
    - El càlcul dinàmic (newton_euler.py) que dona parells en Nm.
    - El motor físic que rep una comanda de CORRENT en mA.

Fórmula bàsica:
    tau (Nm) = Kt (Nm/A) · I (A)
    I (A) = tau / Kt
    I (mA) = 1000 · tau / Kt

Amb fricció:
    Els harmonic drives tenen una fricció que ha de vencer el motor abans
    que el joint es mogui. La compensem afegint un parell adicional al
    signe del moviment desitjat.
"""

import numpy as np


def torques_to_currents(tau, params):

    tau = np.asarray(tau, dtype=float)
    currents_mA = np.zeros(6)
    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]

    for i, joint_name in enumerate(joint_names):
        motor = params.motors[joint_name]
        current_A = tau[i] / motor.kt
        currents_mA[i] = current_A * 1000.0

    return currents_mA


def currents_to_torques(currents_mA, params):

    currents_mA = np.asarray(currents_mA, dtype=float)
    tau = np.zeros(6)
    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]

    for i, joint_name in enumerate(joint_names):
        motor = params.motors[joint_name]
        current_A = currents_mA[i] / 1000.0
        tau[i] = current_A * motor.kt

    return tau


# ==============================================================================
# TORQUES_TO_CURRENTS_HANDGUIDE
# Retorna els 3 corrents importants per hand-guiding en cada joint:
#
#   I_grav  = corrent per compensar gravetat
#
#   I_up    = corrent MÍNIM per començar a moure el joint contra la gravetat.
#             Vol dir: I_grav + fricció (perquè fricció s'oposa al moviment
#             "amunt").
#
#   I_down  = corrent MÀXIM abans que el joint comenci a moure's a favor de la
#             gravetat. Vol dir: I_grav - fricció.
#
# INTERPRETACIÓ:
#   Si el motor rep un corrent en el rang [I_down, I_up], el joint queda quiet
#   (la fricció aguanta l'excés de gravetat/motor).
#   Fora d'aquest rang, comença a moure's.
#
# NOTA SOBRE SIGNES:
#   I_grav pot ser positiu o negatiu segons la geometria del joint i com estan
#   posicionats els encoders. La fricció l'entenem com un MÒDUL (sempre positiu).
#
# IMPORTANT: 
#   Si la tau > 0 (la gravetat tira en sentit positiu del joint):
#       Compensar la gravetat: envia I_grav > 0 (positiu).
#       Per PUJAR contra la gravetat: cal encara MÉS positiu → I_up = I_grav + I_fric.
#       Per BAIXAR amb la gravetat: cal menys positiu → I_down = I_grav - I_fric.
#
#   Si la tau < 0 (la gravetat tira en sentit negatiu):
#       Compensar la gravetat: envia I_grav < 0 (negatiu).
#       Per PUJAR contra la gravetat: cal encara MÉS negatiu → I_up = I_grav - I_fric (més cap avall en el signe).
#       Per BAIXAR amb la gravetat: cal menys negatiu → I_down = I_grav + I_fric.
# ==============================================================================

def torques_to_currents_handguide(tau, params):

    tau = np.asarray(tau, dtype=float)
    n = 6
    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]

    I_grav = np.zeros(n)
    I_up   = np.zeros(n)
    I_down = np.zeros(n)

    for i, joint_name in enumerate(joint_names):                    # Iterem sobre cada joint. 
        motor = params.motors[joint_name]
        
        Kt = motor.kt                                               # Agafa el seu kt des de cobot.urdf
        I_grav_A = tau[i] / Kt                                      # Corrent gravitacional en Ampers. Ex: si tau = -2.115 Nm i kt = 3.25, llavors I = -0.651 A = -651 mA.
        
        fric_Nm  = params.friction.get(joint_name, 0.0)             
        I_fric_A = fric_Nm / Kt                                     # Corrent de fricció en Ampers


        sign_tau = np.sign(tau[i]) if tau[i] != 0 else 0.0
        I_grav[i] = I_grav_A * 1000.0
        I_up[i]   = (I_grav_A + sign_tau * I_fric_A) * 1000.0       # Si tau>0 ex: I_grav=800mA i I_fricc(fricc_Nm/kt)=400. Serà I_up=800+400=1200mA        Si tau<0 ex:-800mA de I_grav (tau/kt) i I_fricc(fricc_Nm/kt)=400. Serà I_up=-800-400=-1200mA
        I_down[i] = (I_grav_A - sign_tau * I_fric_A) * 1000.0       # Si tau>0 ex: I_grav=800mA i I_fricc=400. Serà I_down=800-400=400mA                    Si tau<0 ex:-800mA de I_grau i I_fricc=400. Serà I_down=-800+400=-400mA

    return {"I_grav": I_grav, "I_up": I_up, "I_down": I_down}



# ==============================================================================
# APPLY_FRICTION_COMPENSATION: Suma un parell de fricció al parell base, en la direcció de moviment.
# ==============================================================================

def apply_friction_compensation(tau_base, q_dot, params):

    tau_out = np.array(tau_base, dtype=float)
    q_dot = np.asarray(q_dot, dtype=float)

    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]

    for i, joint_name in enumerate(joint_names):
        friction = params.friction.get(joint_name, 0.0)
        if friction > 0 and abs(q_dot[i]) > 1e-4:                   # el joint s'està movent (velocitat major que un llindar petit). SINÓ SERIÀ FRICCIÓ ESTÀTICA. 
            tau_out[i] += np.sign(q_dot[i]) * friction              # suma la fricció al parell

    return tau_out


# ==============================================================================
# GET_CURRENT_LIMITS: retorna els límits de corrent màxim que podem enviar sense excedir el parell nominal del motor.
# ==============================================================================

def get_current_limits(params, safety_factor=1.0):                  # 1.0  = corrent nominal (segur)   1.5  = permet pics moderats   2.0  = pics agressius 

    RATED_CURRENT_A = {
        "TA-52": 1.04,
        "TA-40": 1.04,
    }

    limits_mA = np.zeros(6)
    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]

    for i, joint_name in enumerate(joint_names):
        motor = params.motors[joint_name]
        rated_A = RATED_CURRENT_A.get(motor.motor_type, 1.0)
        limits_mA[i] = rated_A * safety_factor * 1000.0

    return limits_mA