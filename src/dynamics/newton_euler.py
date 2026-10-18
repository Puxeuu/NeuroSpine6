"""
Càlcul de parells de gravetat mitjançant Newton-Euler recursiu.

Objectiu: donada una configuració q del cobot, retornar els parells τ que
cada motor ha d'aplicar per mantenir el braç QUIET contra la gravetat.

"""

import numpy as np
from src.math.transforms import Rx, Ry, Rz, rpy_to_R


# ==============================================================================
# GRAVITY_TORQUES
# Retorna els parells de gravetat per cada joint.
#
# Entrada:
#   robot   = objecte Robot (URDF loader)
#   q       = configuració actual (6 valors, rad)
#   params  = DynamicParams (del dynamic_loader)
#
# Sortida:
#   tau     = array de 6 elements, parell (Nm) per cada joint
#
# IDEA GENERAL: 
#   Forward: recorre cada joint, calcula l'orientació al món de cada link, i la direcció de cada eix al món.
#   Backward: per cada joint, mira què penja d'ell, calcula el parell gravitacional de cada link penjant (r × F), projecta sobre l'eix, suma, i inverteix el signe.
#   Retorna: 6 parells de compensació de gravetat.
# ==============================================================================

def gravity_torques(robot, q, params):

    q = np.asarray(q, dtype=float)
    g_world = params.gravity

    R_world = {}
    joint_order = []
    joint_axis_world = {}

    R_world["world"] = np.eye(3)                                                # El frame món té rotació identitat (referència absoluta).

    for link_name in robot.links.keys():
        if link_name not in R_world:                                            # Per cada link sense joint pare (ex: base_link), li assigna rotació identitat. Seguretat per links orfes.
            has_parent = any(j.child == link_name for j in robot.joints.values())
            if not has_parent:
                R_world[link_name] = np.eye(3)


    q_idx = 0
    for joint in robot.joints.values():                                         # iterem sobre tots els joints del URDF

        if joint.type == "fixed":                                               # Si el joint és fix, no té angle. Si és mòbil, agafa el valor de q i avança el comptador.
            qj = 0.0
        else:
            qj = q[q_idx]
            q_idx += 1
            joint_order.append(joint.name)

        R_mount = rpy_to_R(joint.rpy)                                           # Rotació fixa del muntatge: ve del <origin rpy=...> del URDF. No depèn de q.

        if joint.type == "fixed":                                               # Rotació del moviment: depèn de qj. Si l'eix és X/Y/Z pur, fa servir funcions ràpides. Si és diagonal, fa Rodrigues.
            R_move = np.eye(3)
        elif joint.type == "revolute":
            axis = np.array(joint.axis)
            if   tuple(axis) == (1.0, 0.0, 0.0): R_move = Rx(qj)
            elif tuple(axis) == (0.0, 1.0, 0.0): R_move = Ry(qj)
            elif tuple(axis) == (0.0, 0.0, 1.0): R_move = Rz(qj)
            else:
                axis_u = axis / np.linalg.norm(axis)                            # # Rodrigues per eixos arbitraris
                K = np.array([
                    [       0, -axis_u[2],  axis_u[1]],
                    [ axis_u[2],        0, -axis_u[0]],
                    [-axis_u[1],  axis_u[0],        0],
                ])
                R_move = np.eye(3) + np.sin(qj) * K + (1 - np.cos(qj)) * (K @ K)
        else:
            R_move = np.eye(3)                                                  # No té sentit fer joints prismàtics, ja que no tenen rotació. 

        R_parent = R_world[joint.parent]
        R_child = R_parent @ R_mount @ R_move                                   # Com està ORIENTAT el link fill al món (matriu de rotació 3×3)
        R_world[joint.child] = R_child


        if joint.type == "revolute":
            axis_local = np.array(joint.axis)
            joint_axis_world[joint.name] = R_parent @ R_mount @ axis_local      # Cap on apunta L'EIX del joint al món (vector 3D unitari). NO depèn de R_move (quant s'ha girat el joint).



    # ----- BACKWARD: per cada link, calcular la força de gravetat que actua sobre ell (F = m·g al món) ------
    tau = np.zeros(6)
    descendants = _get_descendants(robot)                                       # Diu quins links queden més enllà de cada joint.

    q_idx = 0
    for joint in robot.joints.values():
        if joint.type == "fixed":                                               # Els fixos no sumen parell.
            continue

        origin_joint_world = _joint_origin_world(robot, joint.name, q, params)  # Posició de l'eix al món (per calcular braç de palanca)
        axis_world = joint_axis_world[joint.name]                               # Direcció de l'eix al món (per projectar).

        tau_joint = 0.0
        for link_name in descendants[joint.name]:
            if link_name not in params.links:
                continue

            link_params = params.links[link_name]

            com_local = link_params.com
            com_world = _com_world(robot, link_name, com_local, q, params)      # Posició del CoM al món
            F_gravity = link_params.mass * g_world                              # Força de gravetat al món: F = m · g
            r = com_world - origin_joint_world                                  # Braç de palanca: distància des de l'eix del joint fins al CoM
            tau_vec = np.cross(r, F_gravity)                                    # Parell produït per aquesta força.  τ = r × F  (producte vectorial).     IMPORTANT:     F (vector 3D) és la força que ens diu d'on ve l'empenta i cap on va.     El vector r ens diu el vector que va des de l'eix de rotació fins al punt on s'aplica la força.
            tau_joint += np.dot(tau_vec, axis_world)                            # Projectem el parell sobre l'eix del joint per obtenir el component escalar que sent aquest motor

        tau[q_idx] = -tau_joint
        q_idx += 1

    return tau



# ==============================================================================
# HELPER: descendents d'un joint
# ==============================================================================

def _get_descendants(robot):

    children = {}
    for joint in robot.joints.values():                                         # Primer de tot recorrem tots els joints. 
        children.setdefault(joint.parent, []).append(joint.child)

    result = {}
    for joint in robot.joints.values():                                         # Per cada joint, comencem una exploració amb el seu fill directe.
        descendants = []
        stack = [joint.child]
        while stack:                                                            # Repeteix fins que el stack estigui buit. ÉS UN DFS (Depth First Search)
            current = stack.pop()                                               # Agafa un link del stack (pop()).
            descendants.append(current)                                         # Afegeix-lo a la llista descendants.
            for c in children.get(current, []):                                 # Afegeix els seus fills al stack per explorar-los després.
                stack.append(c)                                                 # executem append()
        result[joint.name] = descendants

    return result



# ==============================================================================
# HELPER: origen d'un joint al món
# ==============================================================================

def _joint_origin_world(robot, joint_name, q, params):

    T = np.eye(4)
    q_idx = 0

    for joint in robot.joints.values():
        R_mount = rpy_to_R(joint.rpy)
        T_mount = np.eye(4)
        T_mount[:3, :3] = R_mount
        T_mount[:3,  3] = joint.xyz

        T = T @ T_mount                                                         # La T acumula tot el que hem viatjat des de la base fins AL JOINT ACTUAL.

        if joint.name == joint_name:                                            # És el joint que buscàvem?
            return T[:3, 3]

        if joint.type == "revolute":                                            # Si no és el que busquem, continuem
            qj = q[q_idx]
            q_idx += 1
            axis = np.array(joint.axis)
            if   tuple(axis) == (1.0, 0.0, 0.0): R_move = Rx(qj)                # Calculem la rotació d'aquest joint segons l'eix i l'angle qj. Igual que al Newton-Euler.
            elif tuple(axis) == (0.0, 1.0, 0.0): R_move = Ry(qj)
            elif tuple(axis) == (0.0, 0.0, 1.0): R_move = Rz(qj)
            else:
                axis_u = axis / np.linalg.norm(axis)
                K = np.array([
                    [       0, -axis_u[2],  axis_u[1]],
                    [ axis_u[2],        0, -axis_u[0]],
                    [-axis_u[1],  axis_u[0],        0],
                ])
                R_move = np.eye(3) + np.sin(qj) * K + (1 - np.cos(qj)) * (K @ K)

            T_move = np.eye(4)
            T_move[:3, :3] = R_move
            T = T @ T_move                                                      # Apliquem aquest gir a la cadena acumulada. Ara T reflecteix que aquest joint està girat.

    raise ValueError(f"Joint {joint_name} no trobat")



# ==============================================================================
# HELPER: CoM d'un link al món
# ==============================================================================

def _com_world(robot, link_name, com_local, q, params):

    T = np.eye(4)
    q_idx = 0

    for joint in robot.joints.values():
        R_mount = rpy_to_R(joint.rpy)
        T_mount = np.eye(4)
        T_mount[:3, :3] = R_mount
        T_mount[:3,  3] = joint.xyz
        T = T @ T_mount


        if joint.type == "revolute":
            qj = q[q_idx]
            q_idx += 1
            axis = np.array(joint.axis)
            if   tuple(axis) == (1.0, 0.0, 0.0): R_move = Rx(qj)
            elif tuple(axis) == (0.0, 1.0, 0.0): R_move = Ry(qj)
            elif tuple(axis) == (0.0, 0.0, 1.0): R_move = Rz(qj)
            else:
                axis_u = axis / np.linalg.norm(axis)
                K = np.array([
                    [       0, -axis_u[2],  axis_u[1]],
                    [ axis_u[2],        0, -axis_u[0]],
                    [-axis_u[1],  axis_u[0],        0],
                ])
                R_move = np.eye(3) + np.sin(qj) * K + (1 - np.cos(qj)) * (K @ K)

            T_move = np.eye(4)
            T_move[:3, :3] = R_move
            T = T @ T_move

        
        if joint.child == link_name:                                            # Si hem arribat al link buscat, apliquem el CoM local
            com_h = np.array([com_local[0], com_local[1], com_local[2], 1.0])   # Converteix el CoM local a coordenades homogènies (4 components), l'última és un 1 per poder fer operacions
            com_world = T @ com_h                                               # Multiplica T (acumulat des de la base) pel CoM.
            return com_world[:3]

    return np.array(com_local)