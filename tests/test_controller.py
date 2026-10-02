import math

from robot.calibration import Calibration
from robot.controller import RobotController


class FakeSim:
    def __init__(self):
        self.joint_positions = []
        self.gripper_openings = []

    def set_joint_positions_rad(self, values):
        self.joint_positions.append(values)

    def set_gripper_opening_m(self, opening):
        self.gripper_openings.append(opening)

    def get_tool_pose(self):
        return (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)


class FakeHardware:
    def __init__(self):
        self.packets = []

    def send_pwm(self, values):
        self.packets.append(values)
        return True


def test_set_joint_deg_clamps_state_and_fans_out_to_sim_and_hardware():
    sim = FakeSim()
    hardware = FakeHardware()
    controller = RobotController(Calibration("config/servo_calibration.json"), sim=sim, hardware=hardware)

    controller.set_joint_deg("s1", 999)

    assert controller.state_deg["s1"] == 95
    assert math.isclose(sim.joint_positions[-1]["joint_1_base"], math.radians(95), abs_tol=1e-9)
    assert hardware.packets[-1] == [90, 200, 550, 150, 190, 250]


def test_set_pose_deg_updates_multiple_joints_from_one_robot_state():
    sim = FakeSim()
    hardware = FakeHardware()
    controller = RobotController(Calibration("config/servo_calibration.json"), sim=sim, hardware=hardware)

    controller.set_pose_deg({"s1": -130, "s2": 90, "s3": 142})

    assert controller.state_deg["s1"] == -130
    assert controller.state_deg["s2"] == 90
    assert controller.state_deg["s3"] == 142
    assert hardware.packets[-1] == [540, 420, 250, 150, 190, 250]


def test_set_gripper_opening_clamps_and_fans_out():
    sim = FakeSim()
    hardware = FakeHardware()
    controller = RobotController(Calibration("config/servo_calibration.json"), sim=sim, hardware=hardware)

    controller.set_gripper_opening_m(99)

    assert controller.gripper_opening_m == 0.052
    assert sim.gripper_openings[-1] == 0.052
    assert hardware.packets[-1] == [284, 200, 550, 150, 190, 250]


def test_get_tool_pose_delegates_to_simulation():
    sim = FakeSim()
    controller = RobotController(Calibration("config/servo_calibration.json"), sim=sim)

    assert controller.get_tool_pose() == ((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
