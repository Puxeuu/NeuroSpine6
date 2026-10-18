"""
Jacobià cinemàtic del robot.

El Jacobià és una matriu 6xN que relaciona velocitats dels joints amb
velocitats del TCP:

    [vx, vy, vz, wx, wy, wz]^T  =  J  ·  [q1_dot, ..., qn_dot]^T

Cada columna i del Jacobià respon a la pregunta:
"Si moc una miqueta el joint i, cap on i quant es mou el TCP?"

Aquesta relació és la base del control DLS: donada una velocitat
desitjada al TCP, invertim J per treure les velocitats dels joints.

Aquí calculem J numèricament amb diferències finites (perturbem cada
joint una miqueta i mesurem el canvi al TCP).
"""

import numpy as np
from src.kinematics.forward import forward_kinematics


# ==============================================================================
# ROTATION_VECTOR_FROM_R
# Passa d'una matriu de rotació 3x3 (per angles petits) a un vector 3D de velocitats
# Per angles petits   R ≈ I + θ·K    en comptes de R = I + sin(θ)·K + (1 - cos(θ))·K²
#
#        K  = [ 0   -kz    ky ]
#             [ kz    0   -kx ]
#             [-ky   kx     0 ]
#
#       TENIM:          [  1       -θ·kz    θ·ky  ]               [  1     -wz     wy  ]
#                   R = [  θ·kz    1       -θ·kx  ]     =     R = [  wz     1     -wx  ]
#                       [ -θ·ky    θ·kx     1     ]               [ -wy    wx      1   ]
#
#     wx = θ·kx = (R[2,1] - R[1,2]) / 2     Fem la mitjana per treure el soroll, ja que kx i -kx han de ser el mateix però oposats.
#     wy = θ·ky = (R[0,2] - R[2,0]) / 2
#     wz = θ·kz = (R[1,0] - R[0,1]) / 2
#
# w és un vector 3D que codifica una rotació en un sol vector. Ex: w = [0, 0, 2] rad/s
#   - Direcció de w = l'eix de rotació (un vector unitari k).
#   - Magnitud (llargada) de w = l'angle de rotació (θ).
#
# ==============================================================================

def rotation_vector_from_R(R):
    return np.array([
        (R[2, 1] - R[1, 2]) / 2,
        (R[0, 2] - R[2, 0]) / 2,
        (R[1, 0] - R[0, 1]) / 2,
    ])


# ==============================================================================
# COMPUTE_JACOBIAN
# Calcula el Jacobià numèric al voltant de la configuració q. ÉS EL MAPA DE DIRECCIONS QUE POT SEGUIR A LA POSICIÓ ACTUAL!!
#
# Entrada:
#   robot   = objecte Robot (del urdf_loader)
#   q       = configuració actual (array de 6 valors per al cobot)
#   epsilon = mida de la perturbació (per defecte 1e-6 rad)
#
# Sortida:
#   J = matriu 6x6
#       files 0-2: derivades de posició (x, y, z)
#       files 3-5: derivades d'orientació (wx, wy, wz)
#
# La derivada d'una funció es defineix com:    df/dx = lim (f(x + ε) - f(x)) / ε      =     dp/dq_i = (posició_amb_pertorbació - posició_sense) / mida_pertorbació
#                                                      ε→0
#
# EXEMPLE D'EXECUCIÓ:
# 1. Posició actual del TCP:    (0.000, 0, 1)      ← on està ara
# 2. Target desitjat:           (0.100, 0, 1)      ← on volem que vagi
# 3. Error (vector direcció):   (0.100, 0, 0)      ← diferència
# 4. Calcula Jacobià J a q actual    ← MAPA de direccions possibles
# 5. Resoldre: J · delta_q = error  (DLS)  → delta_q = quant moure cada joint
# 6. Nova q = q + delta_q
# 7. Enviar nova q als motors
# 
# QUÈ ÉS REALMENT J?
# Valor 0.25 = "si moc el joint 2 una miqueta, el TCP es mou 0.25 vegades aquella miqueta en direcció X".
# Si moves el joint 2 en 1 radian  → el TCP es mou 0.25 metres en +X
# Si moves el joint 2 en 0.1 rad   → el TCP es mou 0.025 metres en +X
# Si moves el joint 2 en -0.1 rad  → el TCP es mou -0.025 metres (direcció -X)
#
# ==============================================================================

def compute_jacobian(robot, q, epsilon=1e-6):                                   # ÉS EL MAPA DE DIRECCIONS QUE POT SEGUIR A LA POSICIÓ ACTUAL!!

    q = np.asarray(q, dtype=float)
    poses_ref = forward_kinematics(robot, q)                                    # Calcula la pose del TCP amb la q actual.
    T_ref = poses_ref["tcp_link"]
    p_ref = T_ref[:3, 3]                                                        # posició del TCP
    R_ref = T_ref[:3, :3]                                                       # rotació del TCP

    J = np.zeros((6, 6))                                                        # Preparem la matriu Jacobiana buida.

    for i in range(6):

        q_pert = q.copy()                                                       # Fem còpia
        q_pert[i] += epsilon                                                    # Pertorbem el joint i una mica

        poses_pert = forward_kinematics(robot, q_pert)                          # Calculem la nova pose pertorbada amb epsilon
        T_pert = poses_pert["tcp_link"]
        p_pert = T_pert[:3, 3]
        R_pert = T_pert[:3, :3]

        # ---- Part lineal (3 primers components) ---------------------------
        dp = (p_pert - p_ref) / epsilon                                         # Derivada numèrica de la posició.    dp/dq_i ≈ (p_pert - p_ref) / epsilon
        J[0:3, i] = dp

        # ---- Part angular (3 últims components) ---------------------------
        R_diff = R_pert @ R_ref.T                                               # Ja que R_pert = R_diff · R_ref
        w = rotation_vector_from_R(R_diff) / epsilon                            # Converteix la matriu R_diff en un vector 3D
        J[3:6, i] = w

    return J