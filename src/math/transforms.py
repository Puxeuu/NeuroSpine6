"""
Transformacions bàsiques

    - Rotacions Rx, Ry, Rz.
    - Conversió RPY -> matriu de rotació 3x3.
    - Construcció de la matriu homogènia 4x4 a partir de R i p.
    - Inversa d'una transformació homogènia.
    - Rodrigues: eix + angle -> R, i R -> eix * angle.

"""

import numpy as np                                                              # Per les mates


# ==============================================================================
# ROTACIONS ELEMENTALS
# Entrada: theta en radians.
# Sortida: matriu de rotació 3x3.
# ==============================================================================

def Rx(theta):
    c = np.cos(theta) 
    s = np.sin(theta)     
    return np.array([    
        [1, 0, 0],
        [0, c,-s],
        [0, s, c],
    ])


def Ry(theta):          
    c = np.cos(theta)   
    s = np.sin(theta)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c],
    ])


def Rz(theta):
    c = np.cos(theta) 
    s = np.sin(theta) 
    return np.array([ 
        [c,-s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])


# ==============================================================================
# RPY -> MATRIU DE ROTACIÓ
# Segueix la convenció del URDF: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
#
# Entrada:  rpy = [0, 0, π/2]
# Sortida:  matriu 3x3 equivalent a girar 90° sobre Z.
# ==============================================================================

def rpy_to_R(rpy):
    roll, pitch, yaw = rpy
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)                                       # Fem la multiplicació de matrius


# ==============================================================================
# MATRIU HOMOGÈNIA 4x4
# Rotació 3x3 + translació 3D en una única matriu 4x4.
#
# Entrada:
#   R = matriu 3x3 (identitat per exemple)
#   p = [0.10, 0.20, 0.30]
#
# Sortida:
#   [1 0 0 0.10]
#   [0 1 0 0.20]
#   [0 0 1 0.30]
#   [0 0 0 1.00]
# ==============================================================================

def make_T(R, p):
    T = np.eye(4)                                                               # Identitat 4x4
    T[:3, :3] = R
    T[:3,  3] = p
    return T


# ==============================================================================
# INVERSA D'UNA TRANSFORMACIÓ HOMOGÈNIA
# Útil per canviar de sistema de coordenades.
#
# Si T porta punts de A a B, T_inv els porta de B a A.
# ==============================================================================

def inverse_T(T):
    R = T[:3, :3]
    p = T[:3,  3]

    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T                                                         # Fem la transposada
    T_inv[:3,  3] = -R.T @ p                                                    # Fem - transposada * p
    return T_inv


# ==============================================================================
# RODRIGUES: EIX + ANGLE -> MATRIU DE ROTACIÓ
# on axis = [X,Y,Z] creem un vector respecte el (0,0,0). Aquest vector girara un angle
# i farà moure un punt, per exemple. 
#
# axis = np.array([0, 0, 1])           # eix Z
# angle = np.pi / 2                    # 90°
# R = axis_angle_to_R(axis, angle)     # això només dona una matriu 3×3
#
# P = np.array([1, 0, 0])              # el punt que vull rotar
# P_nou = R @ P                        # aquí el punt rota
#
# Fórmula:  R = I + sin(θ)·K + (1 - cos(θ))·K²
# on K és la matriu antisimètrica associada a l'eix.
#
# LIMITACIÓ IMPORTANT: sempre crea un vector respecte el 0,0,0
# SOLUCIÓ: exemple cavallet atraccions nens
# El pal gran (eix) està clavat al punt C = (5, 0, 0), a 5 metres cap a la dreta de l'origen. El pal apunta cap amunt (direcció Z).
# El nen assegut al cavallet, està al punt P = (7, 0, 0) (a 2 metres del pal). Vol girar 90° al voltant del pal.
#   1. Restem C a tot. Ara el pal és a (0,0,0) i el nen P - C = (7,0,0) - (5,0,0) = (2, 0, 0). Està a 2 metres del pal, que ara sí que és a l'origen.
#   2. Rotar amb Rodrigues
#   3. Sumem C per tornar a col·locar el pal a la seva posició real (5, 0, 0), i el nen al lloc correcte
#
# Per tant: P_final = R @ (P - C) + C
#
# ==============================================================================

def axis_angle_to_R(axis, angle):
    axis = np.asarray(axis, dtype=float)                                        # Assegurem que un vector sigui apte. 
    K = np.array([
        [       0, -axis[2],  axis[1]],
        [ axis[2],        0, -axis[0]],
        [-axis[1],  axis[0],        0],
    ])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


# ==============================================================================
# INVERSA DE RODRIGUES: MATRIU DE ROTACIÓ -> EIX * ANGLE
#
# Sabem que trace(R) = trace(I) + sin(θ)·trace(K) + (1 − cos(θ))·trace(K²)
# trace(I) = 3. La identitat 3×3 té tres 1 a la diagonal.
# trace(K) = 0
# trace(K²) = −2
# trace(R) = 3 + sin(θ)·0 + (1 − cos(θ))·(−2) = 3 − 2 + 2·cos(θ) = 1 + 2·cos(θ). Per tant; cos(θ) = (trace(R) − 1) / 2
#
# Part antisimètrica:  (R − R^T) / 2   i queda R − R^T = 2·sin(θ)·K
# Per fer R^T = I^T + sin(θ)·K^T + (1 − cos(θ))·(K²)^T = I − sin(θ)·K + (1 − cos(θ))·K²
#
# Entrada:
#   R = [ 0 -1  0 ]     (rotació de 90° sobre Z)
#       [ 1  0  0 ]
#       [ 0  0  1 ]
#
# Sortida: [0, 0, π/2]
# ==============================================================================

def R_to_axis_angle_vector(R):
    cos_theta = (np.trace(R) - 1) / 2
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)

    if theta < 1e-9:                                # cas 1: rotació nul·la
        return np.zeros(3)

    if np.abs(np.pi - theta) < 1e-9:                # cas 2: rotació de 180° (sin ≈ 0)
        return np.zeros(3)

    sin_theta = np.sin(theta)
    kx = (R[2, 1] - R[1, 2]) / (2 * sin_theta)
    ky = (R[0, 2] - R[2, 0]) / (2 * sin_theta)
    kz = (R[1, 0] - R[0, 1]) / (2 * sin_theta)

    return np.array([kx, ky, kz]) * theta