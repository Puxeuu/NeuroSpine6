"""
Carrega el YAML de geometries de col·lisió i crea objectes FCL
(Flexible Collision Library) per cada primitiva.

En comptes de fer col·lisions amb els STLs sencers (que serien massa
costosos), aproximem cada link amb primitives simples: caixes, cilindres
i esferes.

Sortida: diccionari { link_name: [Primitive, Primitive, ...] }

Cada Primitive té:
    - link       : a quin link pertany (ex: "arm3")
    - fcl_object : l'objecte de FCL (Box, Cylinder, Sphere)
    - T_local    : matriu 4x4 que col·loca la primitiva al frame local del link
"""

import yaml
import numpy as np
import fcl

from dataclasses import dataclass
from src.math.transforms import axis_angle_to_R


@dataclass
class Primitive:
    link: str
    fcl_object: object
    T_local: np.ndarray


def load_collisions(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    prims_by_link = {}

    for entry in data["primitives"]:                                            # Itera totes les primitives del YAML. Ex: {"link": "arm3", "type": "cylinder", "p0": [...], "p1": [...], "radius": 0.01}.
        link = entry["link"]                                                    # Agafa el nom del link al que pertany la primitiva (ex: "arm3").
        prim = _convert_to_primitive(entry)                                     # Converteix el dict en un objecte Primitive (Cylinder, Box o Sphere segons type).
        prims_by_link.setdefault(link, []).append(prim)                         # Crea la llista si el link no existeix, i hi afegeix la primitiva

    return prims_by_link


# ==============================================================================
# _CONVERT_TO_PRIMITIVE
# Converteix una entrada del YAML en un objecte Primitive segons el tipus:
#   - box       -> center + rpy      -> matriu 4x4 amb rpy_to_R
#   - cylinder  -> p0, p1, radius    -> rotem l'eix Z fins a (p1-p0), centrem
#   - sphere    -> center + radius   -> matriu 4x4 sense rotació
# ==============================================================================

def _convert_to_primitive(entry):
    t = entry["type"]
    link = entry["link"]

    # ---- BOX -----------------------------------------------------------------
    if t == "box":
        size = entry["size"]
        center = np.array(entry["center"], dtype=float)
        rpy = np.array(entry.get("rpy", [0.0, 0.0, 0.0]), dtype=float)

        fcl_obj = fcl.Box(size[0], size[1], size[2])
        T = _make_T_from_rpy(center, rpy)
        return Primitive(link=link, fcl_object=fcl_obj, T_local=T)

    # ---- CYLINDER ------------------------------------------------------------
    if t == "cylinder":
        p0 = np.array(entry["p0"], dtype=float)
        p1 = np.array(entry["p1"], dtype=float)
        radius = float(entry["radius"])
        length = float(np.linalg.norm(p1 - p0))
        center = (p0 + p1) / 2

        R = _rotation_z_to_direction(p1 - p0)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = center

        fcl_obj = fcl.Cylinder(radius, length)
        return Primitive(link=link, fcl_object=fcl_obj, T_local=T)

    # ---- SPHERE --------------------------------------------------------------
    if t == "sphere":
        center = np.array(entry["center"], dtype=float)
        radius = float(entry["radius"])

        fcl_obj = fcl.Sphere(radius)
        T = np.eye(4)
        T[:3, 3] = center
        return Primitive(link=link, fcl_object=fcl_obj, T_local=T)

    raise ValueError(f"Tipus no registrat: {t}")


# ==============================================================================
# _MAKE_T_FROM_RPY
# Matriu 4x4 amb translació + rotació (RPY -> R).
# ==============================================================================

def _make_T_from_rpy(xyz, rpy):
    from src.math.transforms import rpy_to_R, make_T
    R = rpy_to_R(rpy)
    return make_T(R, xyz)


# ==============================================================================
# _ROTATION_Z_TO_DIRECTION
# Retorna la matriu 3x3 que porta l'eix Z (0,0,1) cap al vector direction.
# Ho necessitem perquè FCL crea el cilindre alineat amb Z per defecte.
#
# Casos especials:
#   - direction paral·lel a Z    -> identitat
#   - direction antiparal·lel a Z-> rotació de 180°
#   - resta                       -> Rodrigues amb eix = Z × direction
# ==============================================================================

def _rotation_z_to_direction(direction):
    d = np.asarray(direction, dtype=float)
    n = np.linalg.norm(d)
    if n < 1e-12:
        return np.eye(3)
    d = d / n                                       # Converteix direction a array i el normalitza. Si té longitud zero, retorna la identitat per no petar amb divisió per zero.

    z = np.array([0.0, 0.0, 1.0])
    dot = np.dot(z, d)                              # dot = 1 → Z i direction apunten igual.   dot = -1 → apunten oposats.   dot = 0 → perpendiculars.

    if dot > 0.9999:
        return np.eye(3)
    if dot < -0.9999:
        return np.diag([1.0, -1.0, -1.0])

    axis = np.cross(z, d)                           # Producte vectorial: eix de rotació (perpendicular a Z i a d).
    axis /= np.linalg.norm(axis)                    # Normalitzem
    angle = np.arccos(dot)                          # Angle entre Z i d.

    return axis_angle_to_R(axis, angle)             # Construeix la matriu de rotació amb la fórmula de Rodrigues (eix + angle → matriu 3×3).