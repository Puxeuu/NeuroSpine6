"""
Conversió entre unitats del robot i unitats dels motors.
    Joint space:  radians  (el robot pensa en rad)
    Motor space:  counts d'encoder  (el motor rep counts)

Fórmula:
    counts = (q_rad / (2π)) · direction · encoder_counts_per_rev + zero_offset

Inversa:
    q_rad = (counts - zero_offset) / encoder_counts_per_rev / direction · 2π
"""

import numpy as np
from src.hardware.motor_config import MotorConfig, JointMotorConfig

TWO_PI = 2 * np.pi

def q_to_motor_counts(q_rad: np.ndarray, config: MotorConfig) -> np.ndarray:
    counts = np.zeros(6, dtype=np.int64)
    for i, joint in enumerate(config.joints):
        counts[i] = _joint_to_counts(q_rad[i], joint)
    return counts


def motor_counts_to_q(counts: np.ndarray, config: MotorConfig) -> np.ndarray:
    q_rad = np.zeros(6, dtype=np.float64)
    for i, joint in enumerate(config.joints):
        q_rad[i] = _counts_to_joint(counts[i], joint)
    return q_rad


def _joint_to_counts(q: float, joint: JointMotorConfig) -> int:
    joint_revs = (q / TWO_PI) * joint.direction
    return int(joint_revs * joint.encoder_counts_per_rev + joint.zero_offset_counts)


def _counts_to_joint(counts: int, joint: JointMotorConfig) -> float:
    joint_revs = (counts - joint.zero_offset_counts) / joint.encoder_counts_per_rev
    return (joint_revs / joint.direction) * TWO_PI