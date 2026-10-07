from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Tuple


KINEMATICS_ENABLED = True

JOINT_ORDER = ("s1", "s2", "s3", "s4", "s5")

JOINT_LIMITS_RAD = {
    "s1": (math.radians(-130), math.radians(95)),
    "s2": (math.radians(-57), math.radians(90)),
    "s3": (math.radians(0), math.radians(142)),
    "s4": (math.radians(-35), math.radians(140)),
    "s5": (math.radians(-52), math.radians(160)),
}

BASE_TO_S1_M = 0.030
S2_TO_S3_M = 0.125
S3_TO_S4_M = 0.110
S5_TO_TOOL0_M = 0.117


Matrix4 = Tuple[Tuple[float, float, float, float], ...]
Vector3 = Tuple[float, float, float]


def forward_kinematics_deg(joints: Mapping[str, float]) -> Vector3:
    joints_rad = {name: math.radians(float(value)) for name, value in joints.items()}
    return forward_kinematics_rad(joints_rad)


def forward_kinematics_rad(joints: Mapping[str, float]) -> Vector3:
    _validate_joint_names(joints)
    q = {name: float(joints[name]) for name in JOINT_ORDER}
    _validate_joint_limits(q)

    # URDF chain:
    # Tz(base->S1) Rz(S1) Ry(S2) Tz(S2->S3) Ry(S3)
    # Tz(S3->S4) Ry(S4) Rz(S5) Tz(S5->tool0)
    transform = _identity()
    transform = _matmul(transform, _translate(0.0, 0.0, BASE_TO_S1_M))
    transform = _matmul(transform, _rotate_z(q["s1"]))
    transform = _matmul(transform, _rotate_y(q["s2"]))
    transform = _matmul(transform, _translate(0.0, 0.0, S2_TO_S3_M))
    transform = _matmul(transform, _rotate_y(q["s3"]))
    transform = _matmul(transform, _translate(0.0, 0.0, S3_TO_S4_M))
    transform = _matmul(transform, _rotate_y(q["s4"]))
    transform = _matmul(transform, _rotate_z(q["s5"]))
    transform = _matmul(transform, _translate(0.0, 0.0, S5_TO_TOOL0_M))
    return (transform[0][3], transform[1][3], transform[2][3])


def _validate_joint_names(joints: Mapping[str, float]) -> None:
    names = set(joints)
    expected = set(JOINT_ORDER)
    unknown = names - expected
    if unknown:
        raise ValueError(f"FK input contains unknown robot-space joints: {sorted(unknown)}")
    if names != expected:
        raise ValueError(f"FK input must contain exactly {list(JOINT_ORDER)}")


def _validate_joint_limits(joints: Mapping[str, float]) -> None:
    for name, angle in joints.items():
        lo, hi = JOINT_LIMITS_RAD[name]
        if angle < lo or angle > hi:
            raise ValueError(
                f"{name} angle {angle:.6f} rad is outside URDF safe limits "
                f"{lo:.6f}..{hi:.6f} rad"
            )


def _identity() -> Matrix4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _translate(x: float, y: float, z: float) -> Matrix4:
    return (
        (1.0, 0.0, 0.0, x),
        (0.0, 1.0, 0.0, y),
        (0.0, 0.0, 1.0, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _rotate_y(theta: float) -> Matrix4:
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        (c, 0.0, s, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (-s, 0.0, c, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _rotate_z(theta: float) -> Matrix4:
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        (c, -s, 0.0, 0.0),
        (s, c, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _matmul(a: Matrix4, b: Matrix4) -> Matrix4:
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4))
        for row in range(4)
    )
