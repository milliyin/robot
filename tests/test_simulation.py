import math

import pytest

pytest.importorskip("pybullet")

from simulation.pybullet_sim import PyBulletSim


def test_pybullet_direct_loads_robot_and_tool0_pose_is_finite():
    sim = PyBulletSim(gui=False)
    try:
        assert {
            "joint_1_base",
            "joint_2_shoulder",
            "joint_3_elbow",
            "joint_4_wrist_pitch",
            "joint_5_wrist_roll",
            "left_finger_joint",
            "right_finger_joint",
        } <= set(sim.joint_indices)
        assert "tool0" in sim.link_indices

        sim.set_joint_positions_rad(
            {
                "joint_1_base": 0.0,
                "joint_2_shoulder": 0.0,
                "joint_3_elbow": 0.0,
                "joint_4_wrist_pitch": 0.0,
                "joint_5_wrist_roll": 0.0,
            }
        )
        sim.set_gripper_opening_m(0.052)
        position, orientation = sim.get_tool_pose()
        assert len(position) == 3
        assert len(orientation) == 4
        assert all(math.isfinite(x) for x in position)
        assert all(math.isfinite(x) for x in orientation)
    finally:
        sim.close()


def test_gripper_opening_is_split_between_fingers():
    sim = PyBulletSim(gui=False)
    try:
        sim.set_gripper_opening_m(0.052)
        left = sim.get_joint_position("left_finger_joint")
        right = sim.get_joint_position("right_finger_joint")
        assert math.isclose(left, 0.026, abs_tol=1e-6)
        assert math.isclose(right, 0.026, abs_tol=1e-6)
    finally:
        sim.close()
