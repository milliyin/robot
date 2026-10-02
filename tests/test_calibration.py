from robot.calibration import Calibration


def test_s1_reference_and_safe_endpoints():
    c = Calibration("config/servo_calibration.json")
    assert c.joint_deg_to_pwm("s1", 0) == 284
    assert c.joint_deg_to_pwm("s1", 95) == 90
    assert c.joint_deg_to_pwm("s1", -130) == 540


def test_s2_reference_and_safe_endpoints():
    c = Calibration("config/servo_calibration.json")
    assert c.joint_deg_to_pwm("s2", 0) == 200
    assert c.joint_deg_to_pwm("s2", -57) == 75
    assert c.joint_deg_to_pwm("s2", 90) == 420


def test_all_references_and_safe_endpoints_are_exact():
    c = Calibration("config/servo_calibration.json")
    expected = {
        "s3": [(0, 550), (142, 250)],
        "s4": [(0, 150), (-35, 75), (140, 470)],
        "s5": [(0, 190), (-52, 75), (160, 555)],
    }
    for joint, points in expected.items():
        for angle, pwm in points:
            assert c.joint_deg_to_pwm(joint, angle) == pwm


def test_joint_angles_are_clamped_before_pwm_conversion():
    c = Calibration("config/servo_calibration.json")
    assert c.clamp_joint_deg("s1", 999) == 95
    assert c.clamp_joint_deg("s1", -999) == -130
    assert c.joint_deg_to_pwm("s1", 999) == 90
    assert c.joint_deg_to_pwm("s1", -999) == 540


def test_gripper_endpoints():
    c = Calibration("config/servo_calibration.json")
    assert c.gripper_opening_to_pwm(0.052) == 250
    assert c.gripper_opening_to_pwm(0.0) == 400
    assert c.gripper_opening_to_pwm(99) == 250
    assert c.gripper_opening_to_pwm(-1) == 400
