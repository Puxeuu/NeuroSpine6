"""
Construeix un DynamicParams a partir d'un objecte Robot ja carregat.

Abans, la dinàmica venia d'un YAML separat (config/dynamic_params.yaml).
Ara està al mateix cobot.urdf, dins dels blocs <inertial>, <motor>, <friction>
i <gravity> que urdf_loader.py ja llegeix. 

"""

import numpy as np
from dataclasses import dataclass
from typing import Dict


@dataclass
class LinkParams:
    name: str
    mass: float                             # kg
    com: np.ndarray                         # (3,) posició del CoM en m, frame local
    inertia: np.ndarray                     # (3, 3) tensor d'inèrcia al CoM en kg·m²


@dataclass
class MotorDynamics:
    joint_name: str
    motor_type: str                         # "TA-52" o "TA-40"
    gear_ratio: int                         # gear_ratio: int
    kt: float                               # Nm/A post-reductor


# ==============================================================================
# DYNAMICPARAMS: Contenidor global de tots els paràmetres.
# ==============================================================================
@dataclass
class DynamicParams:
    gravity: np.ndarray                     # (3,) vector gravetat al món
    links: Dict[str, LinkParams]            # {nom_link: LinkParams}
    motors: Dict[str, MotorDynamics]        # {nom_joint: MotorDynamics}
    friction: Dict[str, float]              # {nom_joint: fricció en Nm}


# ==============================================================================
# BUILD_DYNAMIC_PARAMS: Construeix un DynamicParams a partir d'un objecte Robot 
# ja carregat pel urdf_loader.
# ==============================================================================

def build_dynamic_params(robot) -> DynamicParams:                               # -> DynamicParams vol dir que la funció retornarà un objecte de tipus DynamicParams.

    # ---- Links amb inertial ----
    links = {}
    for link_name, link in robot.links.items():
        if link.inertial is None:
            continue
        links[link_name] = LinkParams(
            name=link_name,
            mass=link.inertial.mass,
            com=link.inertial.com,
            inertia=link.inertial.inertia,
        )

    # ---- Motors ----
    motors = {}
    for joint_name, motor_spec in robot.motors.items():
        motors[joint_name] = MotorDynamics(
            joint_name=joint_name,
            motor_type=motor_spec.motor_type,
            gear_ratio=motor_spec.gear_ratio,
            kt=motor_spec.kt,
        )

    # ---- Fricció ----
    friction = dict(robot.friction)

    # ---- Gravetat ----
    gravity = np.asarray(robot.gravity, dtype=float)

    return DynamicParams(
        gravity=gravity,
        links=links,
        motors=motors,
        friction=friction,
    )




def load_dynamic_params(urdf_path: str) -> DynamicParams:                       # load_dynamic_params("config/cobot.urdf")

    from src.model.urdf_loader import load_urdf
    robot = load_urdf(urdf_path)
    return build_dynamic_params(robot)