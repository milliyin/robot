import math
import xml.etree.ElementTree as ET


def _root():
    return ET.parse("robotic_arm.urdf").getroot()


def test_urdf_has_tool0_and_two_fingers():
    root = _root()
    links = {x.get("name") for x in root.findall("link")}
    assert {"left_finger_link", "right_finger_link", "tool0"} <= links


def test_measured_joint_limits_are_present():
    root = _root()
    expected = {
        "joint_1_base": (-130, 95),
        "joint_2_shoulder": (-57, 90),
        "joint_3_elbow": (0, 142),
        "joint_4_wrist_pitch": (-35, 140),
        "joint_5_wrist_roll": (-52, 160),
    }
    joints = {j.get("name"): j for j in root.findall("joint")}
    for name, (lo, hi) in expected.items():
        limit = joints[name].find("limit")
        assert math.isclose(float(limit.get("lower")), math.radians(lo), abs_tol=1e-3)
        assert math.isclose(float(limit.get("upper")), math.radians(hi), abs_tol=1e-3)


def test_tool0_is_fixed_to_gripper_base_at_measured_tip():
    root = _root()
    joints = {j.get("name"): j for j in root.findall("joint")}
    joint = joints["tool0_fixed_joint"]
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "gripper_base_link"
    assert joint.find("child").get("link") == "tool0"
    xyz = [float(x) for x in joint.find("origin").get("xyz").split()]
    assert xyz == [0.0, 0.0, 0.117]


def test_gripper_uses_symmetric_prismatic_fingers():
    root = _root()
    joints = {j.get("name"): j for j in root.findall("joint")}
    left = joints["left_finger_joint"]
    right = joints["right_finger_joint"]
    assert left.get("type") == "prismatic"
    assert right.get("type") == "prismatic"
    assert left.find("axis").get("xyz") == "1 0 0"
    assert right.find("axis").get("xyz") == "-1 0 0"
    assert math.isclose(float(left.find("limit").get("upper")), 0.026, abs_tol=1e-6)
    assert math.isclose(float(right.find("limit").get("upper")), 0.026, abs_tol=1e-6)
