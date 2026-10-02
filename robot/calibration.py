from __future__ import annotations

import json
from pathlib import Path


class Calibration:
    def __init__(self, path: str | Path):
        with Path(path).open("r", encoding="utf-8") as f:
            self.data = json.load(f)
        self.joints = self.data["joints"]
        self.gripper = self.data["gripper"]

    def joint_keys(self) -> list[str]:
        return list(self.joints.keys())

    def urdf_joint_name(self, joint_name: str) -> str:
        return self._joint(joint_name)["joint_name"]

    def joint_limits_deg(self, joint_name: str) -> tuple[float, float]:
        joint = self._joint(joint_name)
        return float(joint["min_deg"]), float(joint["max_deg"])

    def clamp_joint_deg(self, joint_name: str, angle_deg: float) -> float:
        lo, hi = self.joint_limits_deg(joint_name)
        return min(max(float(angle_deg), lo), hi)

    def joint_deg_to_pwm(self, joint_name: str, angle_deg: float) -> int:
        joint = self._joint(joint_name)
        angle = self.clamp_joint_deg(joint_name, angle_deg)
        ref_deg = float(joint["reference_deg"])
        ref_pwm = float(joint["reference_pwm"])

        if angle <= ref_deg:
            a0 = float(joint["min_deg"])
            p0 = float(joint["min_pwm"])
        else:
            a0 = ref_deg
            p0 = ref_pwm

        if angle <= ref_deg:
            a1 = ref_deg
            p1 = ref_pwm
        else:
            a1 = float(joint["max_deg"])
            p1 = float(joint["max_pwm"])

        if a0 == a1:
            pwm = p0
        else:
            pwm = p0 + (angle - a0) * (p1 - p0) / (a1 - a0)
        return self._clamp_pwm(joint, round(pwm))

    def clamp_gripper_opening_m(self, opening_m: float) -> float:
        max_opening = float(self.gripper["max_opening_m"])
        return min(max(float(opening_m), 0.0), max_opening)

    def gripper_opening_to_pwm(self, opening_m: float) -> int:
        opening = self.clamp_gripper_opening_m(opening_m)
        max_opening = float(self.gripper["max_opening_m"])
        closed_pwm = float(self.gripper["closed_pwm"])
        open_pwm = float(self.gripper["open_pwm"])
        pwm = closed_pwm + (opening / max_opening) * (open_pwm - closed_pwm)
        return self._clamp_pwm(self.gripper, round(pwm))

    def startup_pwm(self) -> list[int]:
        return [int(x) for x in self.data["startup_pwm"]]

    def _joint(self, joint_name: str) -> dict:
        try:
            return self.joints[joint_name.lower()]
        except KeyError as exc:
            raise KeyError(f"Unknown calibrated joint: {joint_name}") from exc

    @staticmethod
    def _clamp_pwm(config: dict, pwm: int) -> int:
        lo, hi = config["safe_pwm"]
        return int(min(max(int(pwm), int(lo)), int(hi)))
