"""
Forward Kinematics (FK).

Donada una configuració de joints q = [q1, q2, ..., qn] calculem la
posició i orientació de cada link del robot respecte del 'world'.

Sortida: un diccionari { "world": T, "base_link": T, ..., "tcp_link": T }
on cada T és una matriu homogènia 4x4.

"""

import numpy as np
from src.math.transforms import Rx, Ry, Rz, rpy_to_R, make_T


# ==============================================================================
# JOINT_TRANSFORM: 
# Entrada:  joint (objecte Joint del URDF) + q (rad per revolute, m per prismatic)
# Sortida:  matriu homogènia 4x4
# ==============================================================================

def joint_transform(joint, q):

    if joint.type == "fixed":
        return np.eye(4)                                                        # el joint no es mou mai

    # --------- REVOLUTE: rotació al voltant d'axis --------------
    if joint.type == "revolute":
        ax = joint.axis

        if   ax == (1.0, 0.0, 0.0):
            R = Rx(q)
        elif ax == (0.0, 1.0, 0.0):
            R = Ry(q)
        elif ax == (0.0, 0.0, 1.0):
            R = Rz(q)

        else:
            # Eix arbitrari: fem servir la fórmula de Rodrigues. R = I + sin(q)·K + (1 - cos(q))·K²
            axis = np.array(ax, dtype=float)
            axis = axis / np.linalg.norm(axis)                                  # un vector de longitud 1 unitari
            K = np.array([
                [       0, -axis[2],  axis[1]],
                [ axis[2],        0, -axis[0]],
                [-axis[1],  axis[0],        0],
            ])
            R = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)

        return make_T(R, [0.0, 0.0, 0.0])                                       # rotació pura, translació 0



    # --------- PRISMATIC: translació al llarg d'axis --------------
    if joint.type == "prismatic":
        ax = joint.axis
        translation = [ax[0] * q, ax[1] * q, ax[2] * q]
        return make_T(np.eye(3), translation)                                   # translació pura, sense rotació

    raise ValueError(f"Tipus de joint no vàlid: {joint.type}")


# ==============================================================================
# FORWARD_KINEMATICS
# Entrada:  robot (objecte Robot del URDF) + q (array amb tants valors com joints mòbils)
# Sortida:  { link_name: matriu_4x4 } per a tots els links del robot.
# ==============================================================================

def forward_kinematics(robot, q):

    poses = {}
    poses["world"] = np.eye(4)                                                  # Comencem al world origen 0,0,0

    for link_name in robot.links.keys():                                        # Recorre tots els noms dels links del robot.
        if link_name not in poses:
            has_parent = any(j.child == link_name for j in robot.joints.values())   # Si has_parent = False, llavors cap joint el té com a fill, per tant, és orfe (com el base_link).
            if not has_parent:
                poses[link_name] = np.eye(4)                                    # Si és orfe, el col·loquem a l'origen del món. Matriu identitat


    i = 0
    for joint in robot.joints.values():

        if joint.type == "fixed":
            qj = 0.0
        else:
            qj = q[i]
            i += 1

        R_mount = rpy_to_R(joint.rpy)                                           # Obtenim la matriu 3x3 de rotació a partir del urdf rpy
        T_mount = make_T(R_mount, joint.xyz)                                    # Matriu 4x4 amb la rotació R_mount + xyz del urdf
        T_move = joint_transform(joint, qj)                                     # És la rotació qj en rad respecte l'eix de rotació del propi joint.

        T_parent = poses[joint.parent]
        T_child  = T_parent @ T_mount @ T_move

        poses[joint.child] = T_child

    return poses