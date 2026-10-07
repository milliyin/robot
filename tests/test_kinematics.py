import math

import pytest

pytest.importorskip("pybullet")

from robot.kinematics import JOINT_LIMITS_RAD, forward_kinematics_deg, forward_kinematics_rad
from robot.world_frame import MOBILE_BASE_TO_ARM_BASE_HEIGHT_M, MOBILE_BASE_TO_S1_HEIGHT_M, arm_base_xyz_to_world_xyz
from simulation.pybullet_sim import PyBulletSim


POSES_DEG = {
    "home": {"s1": 0, "s2": 0, "s3": 0, "s4": 0, "s5": 0},
    "pose_a": {"s1": 20, "s2": 20, "s3": 30, "s4": 0, "s5": 0},
    "pose_b": {"s1": -20, "s2": 30, "s3": 40, "s4": 10, "s5": 20},
}


def _sim_tool_xyz(pose_deg, gripper_opening=0.052):
    sim = PyBulletSim(gui=False)
    try:
        sim.set_joint_positions_rad(
            {
                "joint_1_base": math.radians(pose_deg["s1"]),
                "joint_2_shoulder": math.radians(pose_deg["s2"]),
                "joint_3_elbow": math.radians(pose_deg["s3"]),
                "joint_4_wrist_pitch": math.radians(pose_deg["s4"]),
                "joint_5_wrist_roll": math.radians(pose_deg["s5"]),
            }
        )
        sim.set_gripper_opening_m(gripper_opening)
        return sim.get_tool_pose()[0]
    finally:
        sim.close()


def _assert_xyz_close(actual, expected, abs_tol=1e-7):
    assert len(actual) == 3
    for a, e in zip(actual, expected):
        assert math.isclose(a, e, abs_tol=abs_tol)


def test_zero_pose_gives_measured_urdf_tool_height():
    xyz = forward_kinematics_deg(POSES_DEG["home"])

    _assert_xyz_close(xyz, (0.0, 0.0, 0.382))


def test_world_frame_adds_mobile_base_mount_height():
    raw_xyz = forward_kinematics_deg(POSES_DEG["home"])

    assert math.isclose(MOBILE_BASE_TO_S1_HEIGHT_M, 0.18, abs_tol=1e-9)
    _assert_xyz_close(arm_base_xyz_to_world_xyz(raw_xyz), (0.0, 0.0, 0.382 + MOBILE_BASE_TO_ARM_BASE_HEIGHT_M))


@pytest.mark.parametrize("pose_name", ["home", "pose_a", "pose_b"])
def test_forward_kinematics_matches_pybullet_for_known_poses(pose_name):
    pose = POSES_DEG[pose_name]
    analytic_xyz = arm_base_xyz_to_world_xyz(forward_kinematics_deg(pose))
    pybullet_xyz = _sim_tool_xyz(pose)

    _assert_xyz_close(analytic_xyz, pybullet_xyz, abs_tol=1e-4)


def test_s5_wrist_roll_does_not_change_tool_xyz():
    base_pose = {"s1": 15, "s2": 20, "s3": 35, "s4": 10, "s5": 0}
    rolled_pose = {**base_pose, "s5": 90}

    _assert_xyz_close(forward_kinematics_deg(base_pose), forward_kinematics_deg(rolled_pose))
    _assert_xyz_close(_sim_tool_xyz(base_pose), _sim_tool_xyz(rolled_pose), abs_tol=1e-4)


def test_gripper_opening_does_not_change_tool_xyz_in_pybullet():
    pose = POSES_DEG["pose_a"]

    _assert_xyz_close(
        _sim_tool_xyz(pose, gripper_opening=0.0),
        _sim_tool_xyz(pose, gripper_opening=0.052),
        abs_tol=1e-4,
    )


def test_degrees_and_radians_apis_agree():
    pose_deg = POSES_DEG["pose_b"]
    pose_rad = {name: math.radians(value) for name, value in pose_deg.items()}

    _assert_xyz_close(forward_kinematics_deg(pose_deg), forward_kinematics_rad(pose_rad))


def test_boundary_joint_values_are_valid():
    boundary_pose = {name: limit[1] for name, limit in JOINT_LIMITS_RAD.items()}
    xyz = forward_kinematics_rad(boundary_pose)

    assert all(math.isfinite(value) for value in xyz)


def test_out_of_range_joint_values_are_rejected():
    pose = {name: 0.0 for name in JOINT_LIMITS_RAD}
    pose["s3"] = math.radians(143)

    with pytest.raises(ValueError, match="outside"):
        forward_kinematics_rad(pose)


def test_joint_names_must_match_robot_space_keys_exactly():
    with pytest.raises(ValueError, match="exactly"):
        forward_kinematics_deg({"s1": 0, "s2": 0, "s3": 0, "s4": 0})

    with pytest.raises(ValueError, match="unknown"):
        forward_kinematics_deg({"s1": 0, "s2": 0, "s3": 0, "s4": 0, "s5": 0, "joint_1_base": 0})
