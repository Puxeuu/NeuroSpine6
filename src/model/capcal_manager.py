"""
Gestor de capçals intercanviables.

Llegeix config/capcals.yaml i permet canviar el capçal sense reiniciar el viewer.

En canviar de capçal, s'actualitzen 3 coses:
    1. La mesh visual del link `capcal` (STL diferent).
    2. La posició del TCP (`tcp_joint.origin`), perquè capçals diferents
       tenen puntes a distàncies diferents.
    3. Les primitives de col·lisió del link `capcal` (perquè el DLS i el
       comprovador de col·lisions treballin amb la forma correcta).

No es modifica el fitxer cobot.urdf. Els canvis són en memòria, sobre
els objectes Robot i prims que ja té carregats el viewer.
"""

import os
import yaml
import numpy as np

from src.model.urdf_loader import Link
from src.collision.collision_loader import _convert_to_primitive


# ==============================================================================
# CAPCAL
# ==============================================================================

class Capcal:

    def __init__(self, data):
        self.nom         = data["nom"]
        self.descripcio  = data.get("descripcio", data["nom"])
        self.mesh        = data["mesh"]
                                                                                # IMPORTANT: no vull posar visual_xyz perquè al solidworks sempre ho faig centrat per evitar problemes i complicacions... així sé que tot comença a 0,0,0
        self.visual_rpy  = np.array(data.get("visual_rpy", [0.0, 0.0, 0.0]))
        self.color       = data.get("color", "#8e8e93")
        self.tcp_offset  = np.array(data.get("tcp_offset", [0.0, 0.0, 0.0]))
        self.collisions  = data.get("collisions", [])

    def __repr__(self):
        return f"Capcal({self.nom}, tcp={self.tcp_offset[0]*1000:.1f}mm)"


# ==============================================================================
# CAPCAL_MANAGER
# ==============================================================================

class CapcalManager:
    """
    Carrega capcals.yaml i manté quin és el capçal actiu.

    """

    def __init__(self, yaml_path, config_root):                                 # Aquí hi posarem la nostra ruta de config/ capcals.yaml
        self.yaml_path = yaml_path
        self.config_root = config_root

        with open(yaml_path, "r", encoding="utf-8") as f:                       # Carreguem el yaml. 
            data = yaml.safe_load(f)

        self.capcals = {c["nom"]: Capcal(c) for c in data["capcals"]}           # Recorre data["capcals"] i crea un diccionari {"bola": Capcal(...), "punxo": Capcal(...), "broca": Capcal(...)}.
        if not self.capcals:
            raise ValueError(f"El fitxer {yaml_path} no defineix cap capçal.")

        default_nom = data.get("default")                                       # Llegeix quin capçal és el "per defecte" del YAML. Si no hi és, retorna None.
        if default_nom is None or default_nom not in self.capcals:              # Si el default falta o apunta a un capçal que no existeix, agafa el primer capçal del diccionari.
            default_nom = next(iter(self.capcals))                              
        self.active_nom = default_nom                                           # Guarda quin capçal està actiu ara (guarda el nom, no l'objecte)



    @property 
    def active(self):                                                           # Retorna l'objecte Capcal que està actiu ara. El @property fa que el podem utilitzar com un atribut: manager.active en lloc de manager.active().
        return self.capcals[self.active_nom]

    def noms(self):                                                             # Retorna una llista amb els noms de tots els capçals disponibles, ex: ["bola", "punxo", "broca"].
        return list(self.capcals.keys())

    def set_active(self, nom):                                                  # Canvia quin capçal està actiu. Comprova que el nom existeix; si no, llança error clar amb els disponibles.
        if nom not in self.capcals:
            raise KeyError(f"Capçal '{nom}' no existeix. Disponibles: {self.noms()}")
        self.active_nom = nom

    def resolved_mesh_path(self):                                               # Retorna la ruta absoluta al fitxer STL del capçal actiu.
        return os.path.join(self.config_root, self.active.mesh)                 # Fa servir config_root per partir de la base i no haver d'escriure-ho tot C:/.../config

    def apply_to(self, robot, prims_by_link):                                   # Modifica el model del robot perquè faci servir el capçal actiu.

        cap = self.active

        # ---- 1. Link visual ------------------------------------------------
        if "capcal" not in robot.links:                                         # IMORTANT I POSSIBLE MILLORA DE CODI: anem a buscar la pestanya capcal de robot.urdf per poder modificar el capcal. 
            raise KeyError("El URDF no té cap link anomenat 'capcal'. ")

        robot.links["capcal"] = Link(
            name="capcal",
            mesh=cap.mesh,
            visual_xyz=np.zeros(3),                                             # La classe Capcal cap no té cap.visual_xyz, llavors ho posem a 0,0,0
            visual_rpy=cap.visual_rpy,
        )



        # ---- 2. TCP offset -------------------------------------------------
        if "tcp_joint" not in robot.joints:
            raise KeyError("El URDF no té cap joint anomenat 'tcp_joint'. ")

        tcp_j = robot.joints["tcp_joint"]                                       # Obtenim el joint actual
        new_xyz = tuple(float(v) for v in cap.tcp_offset)                       # Construïm el nou xyz:    for v in cap.tcp_offset  itera els 3 valors [0.071, 0.0, 0.0].  float(v) converteix cada np.float64 a float de Python.  tuple(...) agrupa els 3 en una tupla (0.071, 0.0, 0.0).
        robot.joints["tcp_joint"] = _replace_xyz(tcp_j, new_xyz)                # Crea un Joint nou copiant tots els altres camps de l'antic i canviant només el xyz.



        # ---- 3. Primitives de col·lisió ------------------------------------
        new_prims = []                                                          # Llista buida per acumular les noves primitives
        for entry in cap.collisions:                                            # Iterem sobre el nou capcal. Ens diu de cada cilindre de col·lisió el tipus, p0, p1 i radi
            entry_with_link = dict(entry)                                       # Còpia del diccionari de cilindres
            entry_with_link["link"] = "capcal"                                  # Afegim el camp "link", ja que collision_loader espera un dict que digui a quin link pertany la primitiva
            new_prims.append(_convert_to_primitive(entry_with_link))            # Convertim el dict a un objecte Primitive

        prims_by_link["capcal"] = new_prims                                     # Substituïm les col·lisions del capcal al dict global

        return robot, prims_by_link





def _replace_xyz(joint, new_xyz):
    from dataclasses import replace
    return replace(joint, xyz=new_xyz)
