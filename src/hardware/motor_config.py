"""
Càrrega del motor_config.yaml.

Llegeix el fitxer i el converteix en un objecte MotorConfig amb un
JointMotorConfig per cada motor del robot. La resta del hardware
(converter, controller) treballa amb aquest objecte.
"""

import yaml
from dataclasses import dataclass
from typing import List

@dataclass
class JointMotorConfig:
    joint_name: str
    motor_id: int
    gear_ratio: float
    encoder_counts_per_rev: int
    direction: int
    zero_offset_counts: int
    torque_constant: float
    current_limit_mA: int
    velocity_limit_counts: int
    default_velocity_counts: int
    default_current_mA: int



@dataclass
class MotorConfig:
    serial_port: str
    baudrate: int
    joints: List[JointMotorConfig]

    @property
    def motor_ids(self) -> List[int]:
        return [j.motor_id for j in self.joints]                                # Retorna una llista amb els motor_id de tots els joints.




def load_motor_config(yaml_path: str) -> MotorConfig:                           # Llegeix el YAML i retorna un MotorConfig llest per usar.
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    joints = [JointMotorConfig(**j) for j in data["joints"]]
    return MotorConfig(
        serial_port=data["serial_port"],
        baudrate=data["baudrate"],
        joints=joints,
    )