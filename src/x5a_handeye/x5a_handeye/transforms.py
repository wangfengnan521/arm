#!/usr/bin/env python3
"""SE(3) helpers for eye-to-hand calibration."""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def R_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    m = np.asarray(R, dtype=float)
    t = float(np.trace(m))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def pose_to_T(xyz: Sequence[float], quat_xyzw: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_R(*quat_xyzw)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def T_to_pose(T: np.ndarray) -> Tuple[List[float], List[float]]:
    xyz = T[:3, 3].tolist()
    q = list(R_to_quat(T[:3, :3]))
    return xyz, q


def inv_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rvec_tvec_to_T(rvec, tvec) -> np.ndarray:
    R, _ = __import__("cv2").Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return T


def rotation_angle_deg(R: np.ndarray) -> float:
    c = (np.trace(R) - 1.0) * 0.5
    c = max(-1.0, min(1.0, float(c)))
    return math.degrees(math.acos(c))


def se3_errors(Ta: np.ndarray, Tb: np.ndarray) -> Tuple[float, float]:
    dT = inv_T(Ta) @ Tb
    t_err = float(np.linalg.norm(dT[:3, 3]))
    r_err = rotation_angle_deg(dT[:3, :3])
    return t_err, r_err
