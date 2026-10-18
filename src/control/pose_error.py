"""
Error de pose entre la posició actual del TCP i el target.

A cada iteració:
    1. Llegeix la pose actual del robot (T_actual).
    2. La compara amb la pose objectiu (T_target).
    3. Calcula un error.
    4. Genera una petita velocitat que redueix aquest error (DLS).
    5. Mou el robot una mica i torna a començar.

Com que es repeteix unes 100 vegades per segon, el robot es mou de manera
contínua i suau. El target pot ser fix (l'error tendeix a zero) o
mòbil (seguiment de trajectòria).

EN RESUM: Si ara mateix sóc aquí i vull arribar allà, quin moviment em falta?
"""

import numpy as np
from src.math.transforms import R_to_axis_angle_vector


# ==============================================================================
# POSE_ERROR
# Entrada:  T_actual, T_target  (matrius 4x4)
# Sortida:  vector 6D = [dx, dy, dz, wx, wy, wz]
#           - dx, dy, dz : error de translació (metres)
#           - wx, wy, wz : error de rotació (rad, eix·angle)
# ==============================================================================

def pose_error(T_actual, T_target):

    # ---- Error de posició ----------------------------------------------------
    p_actual = T_actual[:3, 3]
    p_target = T_target[:3, 3]
    e_pos = p_target - p_actual

    # ---- Error d'orientació --------------------------------------------------
    R_actual = T_actual[:3, :3]
    R_target = T_target[:3, :3]
    R_err = R_target @ R_actual.T

    e_ori = R_to_axis_angle_vector(R_err)           # Passem la matriu de rotació d'error a un vector 3D
    return np.concatenate([e_pos, e_ori])