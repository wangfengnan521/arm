"""Read-only X5A forward kinematics matching the project URDF.

The vendor RobotStatus ``end_pos`` is expressed from the SDK kinematic base,
whose origin is not the URDF ``base_link`` origin.  For calibration, compute
``base_link -> tool0`` directly from the six feedback joint positions instead.
"""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np


# Copied verbatim from x5a_moveit_config/config/X5A.urdf.xacro.  The vendor
# URDF itself remains unchanged; tool0 is the documented project addition.
JOINT_ORIGIN_XYZ = (
    (0.0, 0.0, 0.0605),
    (0.02, 0.0, 0.04),
    (-0.264, 0.0, 0.0),
    (0.245, 0.0, -0.056),
    (0.06775, 0.0005, -0.0865),
    (0.02895, 0.0, 0.0865),
)
JOINT_ORIGIN_RPY = (
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (3.1416, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (-3.1416, 0.0, 0.0),
)
JOINT_AXIS = (
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
)
LINK6_TO_TOOL0_XYZ = (0.11277, 0.0, 0.0)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _axis_matrix(axis: Sequence[float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a /= np.linalg.norm(a)
    x, y, z = a
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _transform(xyz: Sequence[float], rotation: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = np.asarray(xyz, dtype=float)
    return out


def fk_base_tool0(joints: Sequence[float]) -> np.ndarray:
    """Return ``T_base_link_tool0`` from six joint positions in radians."""
    if len(joints) < 6:
        raise ValueError(f"need 6 joints, got {len(joints)}")
    q = [float(v) for v in joints[:6]]
    if not all(math.isfinite(v) for v in q):
        raise ValueError("joint positions contain non-finite values")

    result = np.eye(4)
    for origin_xyz, origin_rpy, axis, angle in zip(
        JOINT_ORIGIN_XYZ, JOINT_ORIGIN_RPY, JOINT_AXIS, q
    ):
        result = result @ _transform(origin_xyz, _rpy_matrix(*origin_rpy))
        result = result @ _transform((0.0, 0.0, 0.0), _axis_matrix(axis, angle))
    return result @ _transform(LINK6_TO_TOOL0_XYZ, np.eye(3))
