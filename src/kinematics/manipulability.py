"""
Manipulabilitat de Yoshikawa i damping adaptatiu.

La manipulabilitat és un número que resumeix "com de bé pot moure's el
robot ara mateix". Serveix per detectar quan estem a prop d'una singularitat
(el robot perd un grau de llibertat i el DLS es torna inestable).

Fórmula general:
    w = sqrt( det(J · J^T) )

    w petit  ->  a prop d'una singularitat
    w gran   ->  configuració còmoda, tots els graus de llibertat disponibles

També conté el càlcul del damping adaptatiu que fa servir el DLS:
la idea és tenir molta precisió lluny de singularitats i molta estabilitat
a prop d'elles.
"""

import numpy as np


# ==============================================================================
# YOSHIKAWA: Manipulabilitat completa (posició + orientació) del TCP.
# ==============================================================================
def yoshikawa(J):
    return np.sqrt(max(0.0, np.linalg.det(J @ J.T)))


# ==============================================================================
# YOSHIKAWA_POSITION: Manipulabilitat només de posició, ignorant j6 perquè no aporta res a la posició del TCP
# ==============================================================================
def yoshikawa_position(J):
    Jp = J[0:3, 0:5]
    return np.sqrt(max(0.0, np.linalg.det(Jp @ Jp.T)))


# ==============================================================================
# YOSHIKAWA_ORIENTATION: Manipulabilitat només d'orientació
# ==============================================================================
def yoshikawa_orientation(J):
    Jw = J[3:6, :]
    return np.sqrt(max(0.0, np.linalg.det(Jw @ Jw.T)))


# ==============================================================================
# IS_NEAR_SINGULARITY: Ens diu si estem per sota d'un llindar de manipulabilitat de posició.
# ==============================================================================
def is_near_singularity(J, threshold):
    return yoshikawa_position(J) < threshold


# ==============================================================================
# ADAPTIVE_DAMPING: Retorna el damping lambda que ha de fer servir el DLS segons la manipulabilitat actual w.
#
#   Lluny de singularitats (w >= w_threshold):  lambda petit -> precisió i rapidesa.
#   A prop de singularitats (w < w_threshold):  lambda gran -> estabilitat i suavitat.
#
# La transició no és lineal, és quadràtica per suavitzar el canvi.
# ==============================================================================

def adaptive_damping(w, lambda_min, lambda_max, w_threshold):
    if w >= w_threshold:
        return lambda_min

    ratio = 1.0 - (w / w_threshold)
    return lambda_min + (lambda_max - lambda_min) * (ratio ** 2)        # λ es manté baixa mentre estem lluny de la singularitat, i només puja ràpidament quan estem a prop de singularitats.