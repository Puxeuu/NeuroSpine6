"""
DLS Solver - Damped Least Squares.

Problema: el robot té 6 motors i 6 DOF. Volem donar una velocitat
del TCP (v = [vx, vy, vz, wx, wy, wz]) i que el solver ens digui
quines velocitats aplicar a cada motor.

La solució seria invertir el Jacobià:  q_dot = J^-1 · v
Però J no és invertible a prop de singularitats. Els motors demanarien
velocitats enormes i el robot es tornaria inestable.

La solució DLS afegeix un terme λ²·I que fa la matriu sempre invertible:
    q_dot = J^T · (J · J^T + λ²·I)^-1 · v

El preu és que ja no seguim v exactament. Resolem un compromís entre
"seguir v" i "no fer q_dot enorme".

DAMPING (λ):
    λ petit (~0.001):
        Ús: lluny de singularitats.
        Efecte: seguim v amb precisió.

    λ gran (~0.5):
        Ús: prop de singularitats.
        Efecte: q_dot suau, els motors giren a velocitats raonables.
        Preu: hi ha un error residual.

"""

import numpy as np


# ==============================================================================
# DLS
# Entrada:
#   J       = Jacobià 6xN
#   v       = velocitat desitjada al TCP (vector 6D)
#   damping = λ (escalar, típicament 0.005 - 0.3)
#
# Sortida:
#   q_dot = velocitats dels joints (vector N)
# ==============================================================================

def dls(J, v, damping):
    I = np.eye(J.shape[0])
    JJt = J @ J.T
    inv = np.linalg.inv(JJt + (damping ** 2) * I)
    q_dot = J.T @ inv @ v
    return q_dot