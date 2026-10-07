import math
from typing import Dict

from robot.calibration import Calibration


class RobotController:
    def __init__(self, calibration: Calibration, sim=None, hardware=None):
        self.calibration = calibration
        self.sim = sim
        self.hardware = hardware
        self.state_deg = {joint: 0.0 for joint in self.calibration.joint_keys()}
        self.gripper_opening_m = float(self.calibration.gripper["max_opening_m"])
        self._emit()

    def set_joint_deg(self, name: str, angle: float) -> None:
        key = name.lower()
        self.state_deg[key] = self.calibration.clamp_joint_deg(key, angle)
        self._emit()

    def set_pose_deg(self, pose: Dict[str, float]) -> None:
        for name, angle in pose.items():
            key = name.lower()
            self.state_deg[key] = self.calibration.clamp_joint_deg(key, angle)
        self._emit()

    def set_gripper_opening_m(self, opening: float) -> None:
        self.gripper_opening_m = self.calibration.clamp_gripper_opening_m(opening)
        self._emit()

    def get_tool_pose(self):
        if self.sim is None:
            raise RuntimeError("No simulation backend is attached")
        return self.sim.get_tool_pose()

    def _emit(self) -> None:
        if self.sim is not None:
            self.sim.set_joint_positions_rad(
                {
                    self.calibration.urdf_joint_name(name): math.radians(angle)
                    for name, angle in self.state_deg.items()
                }
            )
            self.sim.set_gripper_opening_m(self.gripper_opening_m)

        if self.hardware is not None:
            packet = [
                self.calibration.joint_deg_to_pwm("s1", self.state_deg["s1"]),
                self.calibration.joint_deg_to_pwm("s2", self.state_deg["s2"]),
                self.calibration.joint_deg_to_pwm("s3", self.state_deg["s3"]),
                self.calibration.joint_deg_to_pwm("s4", self.state_deg["s4"]),
                self.calibration.joint_deg_to_pwm("s5", self.state_deg["s5"]),
                self.calibration.gripper_opening_to_pwm(self.gripper_opening_m),
            ]
            self.hardware.send_pwm(packet)
