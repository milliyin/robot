from typing import Dict, NamedTuple, Tuple


Vector3 = Tuple[float, float, float]
JointPoseDeg = Dict[str, float]


class ToolPoseMeasurementCm(NamedTuple):
    sim_xyz: Vector3
    real_xyz: Vector3


JOINT_POSES_DEG: Dict[str, JointPoseDeg] = {
    "home": {"s1": 0.0, "s2": 0.0, "s3": 0.0, "s4": 0.0, "s5": 0.0},
    "pose_a": {"s1": 20.0, "s2": 20.0, "s3": 30.0, "s4": 0.0, "s5": 0.0},
    "pose_b": {"s1": -20.0, "s2": 30.0, "s3": 40.0, "s4": 10.0, "s5": 20.0},
    "pose_c": {"s1": 95.0, "s2": -53.713004, "s3": 70.206658, "s4": 86.573269, "s5": 0.0},
}

MEASURED_TOOL_POSES_CM: Dict[str, ToolPoseMeasurementCm] = {
    "home": ToolPoseMeasurementCm(sim_xyz=(4.4, -0.7, 18.3), real_xyz=(-3.2, 11.55, 30.1)),
    "pose_a": ToolPoseMeasurementCm(sim_xyz=(10.0, 4.9, 13.0), real_xyz=(4.7, 16.7, 18.3)),
    "pose_b": ToolPoseMeasurementCm(sim_xyz=(10.3, -13.1, 12.0), real_xyz=(-14.1, 14.7, 10.6)),
    "pose_c": ToolPoseMeasurementCm(sim_xyz=(-0.7, 4.4, 18.3), real_xyz=(-3.2, 11.55, 30.1)),
}


def get_joint_pose_deg(name: str) -> JointPoseDeg:
    key = _normalize_pose_name(name)
    try:
        return dict(JOINT_POSES_DEG[key])
    except KeyError as exc:
        raise KeyError(f"Unknown named robot pose: {name}") from exc


def get_measurement_cm(name: str) -> ToolPoseMeasurementCm:
    key = _normalize_pose_name(name)
    try:
        return MEASURED_TOOL_POSES_CM[key]
    except KeyError as exc:
        raise KeyError(f"Unknown measured tool pose: {name}") from exc


def _normalize_pose_name(name: str) -> str:
    return name.strip().lower().replace(" ", "_").replace("-", "_")
