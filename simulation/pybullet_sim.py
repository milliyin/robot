from __future__ import annotations

import math
from pathlib import Path

import pybullet as p


class PyBulletSim:
    ARM_JOINTS = (
        "joint_1_base",
        "joint_2_shoulder",
        "joint_3_elbow",
        "joint_4_wrist_pitch",
        "joint_5_wrist_roll",
    )
    FINGER_JOINTS = ("left_finger_joint", "right_finger_joint")

    def __init__(self, gui: bool = False, urdf_path: str | Path = "robotic_arm.urdf"):
        self.client_id = p.connect(p.GUI if gui else p.DIRECT)
        if self.client_id < 0:
            raise RuntimeError("Could not connect to PyBullet")

        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        self.robot_id = p.loadURDF(
            str(Path(urdf_path).resolve()),
            useFixedBase=True,
            physicsClientId=self.client_id,
        )
        self.joint_indices: dict[str, int] = {}
        self.link_indices: dict[str, int] = {}
        self._discover_names()

    def set_joint_positions_rad(self, positions: dict[str, float]) -> None:
        for name, value in positions.items():
            if name not in self.ARM_JOINTS:
                raise KeyError(f"Unknown arm joint: {name}")
            self._set_joint(name, float(value))
        self._step()

    def set_gripper_opening_m(self, opening_m: float) -> None:
        half_opening = min(max(float(opening_m), 0.0), 0.052) / 2.0
        for name in self.FINGER_JOINTS:
            self._set_joint(name, half_opening)
        self._step()

    def get_joint_position(self, joint_name: str) -> float:
        state = p.getJointState(
            self.robot_id,
            self.joint_indices[joint_name],
            physicsClientId=self.client_id,
        )
        return float(state[0])

    def get_tool_pose(self) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        link_state = p.getLinkState(
            self.robot_id,
            self.link_indices["tool0"],
            computeForwardKinematics=True,
            physicsClientId=self.client_id,
        )
        position = tuple(float(x) for x in link_state[4])
        orientation = tuple(float(x) for x in link_state[5])
        if not all(math.isfinite(x) for x in (*position, *orientation)):
            raise RuntimeError("tool0 pose contains non-finite values")
        return position, orientation

    def close(self) -> None:
        if p.isConnected(self.client_id):
            p.disconnect(physicsClientId=self.client_id)

    def _discover_names(self) -> None:
        for i in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            self.joint_indices[joint_name] = i
            self.link_indices[link_name] = i

        missing_joints = set(self.ARM_JOINTS + self.FINGER_JOINTS) - set(self.joint_indices)
        if missing_joints:
            raise RuntimeError(f"URDF missing expected joints: {sorted(missing_joints)}")
        if "tool0" not in self.link_indices:
            raise RuntimeError("URDF missing tool0 link")

    def _set_joint(self, name: str, value: float) -> None:
        index = self.joint_indices[name]
        p.resetJointState(self.robot_id, index, value, physicsClientId=self.client_id)
        p.setJointMotorControl2(
            self.robot_id,
            index,
            p.POSITION_CONTROL,
            targetPosition=value,
            physicsClientId=self.client_id,
        )

    def _step(self) -> None:
        p.stepSimulation(physicsClientId=self.client_id)
