from robot.poses import JOINT_POSES_DEG, MEASURED_TOOL_POSES_CM, get_joint_pose_deg, get_measurement_cm


def test_named_joint_poses_are_available_for_home_pose_a_and_pose_b():
    assert JOINT_POSES_DEG["home"] == {"s1": 0.0, "s2": 0.0, "s3": 0.0, "s4": 0.0, "s5": 0.0}
    assert JOINT_POSES_DEG["pose_a"] == {"s1": 20.0, "s2": 20.0, "s3": 30.0, "s4": 0.0, "s5": 0.0}
    assert JOINT_POSES_DEG["pose_b"] == {"s1": -20.0, "s2": 30.0, "s3": 40.0, "s4": 10.0, "s5": 20.0}
    assert JOINT_POSES_DEG["pose_c"] == {
        "s1": 95.0,
        "s2": -53.713004,
        "s3": 70.206658,
        "s4": 86.573269,
        "s5": 0.0,
    }


def test_measurements_store_raw_sim_and_real_xyz_in_cm():
    home = MEASURED_TOOL_POSES_CM["home"]
    assert home.sim_xyz == (4.4, -0.7, 18.3)
    assert home.real_xyz == (-3.2, 11.55, 30.1)

    pose_a = MEASURED_TOOL_POSES_CM["pose_a"]
    assert pose_a.sim_xyz == (10.0, 4.9, 13.0)
    assert pose_a.real_xyz == (4.7, 16.7, 18.3)

    pose_b = MEASURED_TOOL_POSES_CM["pose_b"]
    assert pose_b.sim_xyz == (10.3, -13.1, 12.0)
    assert pose_b.real_xyz == (-14.1, 14.7, 10.6)

    pose_c = MEASURED_TOOL_POSES_CM["pose_c"]
    assert pose_c.sim_xyz == (-0.7, 4.4, 18.3)
    assert pose_c.real_xyz == (-3.2, 11.55, 30.1)


def test_lookup_returns_copies_so_callers_cannot_mutate_pose_constants():
    pose = get_joint_pose_deg("pose_a")
    pose["s1"] = 999

    assert get_joint_pose_deg("pose_a")["s1"] == 20.0
    assert get_measurement_cm("pose_b").real_xyz == (-14.1, 14.7, 10.6)


def test_lookup_names_are_case_and_space_friendly():
    assert get_joint_pose_deg("Pose A") == get_joint_pose_deg("pose_a")
    assert get_measurement_cm("Pose B") == get_measurement_cm("pose_b")
    assert get_measurement_cm("Pose C") == get_measurement_cm("pose_c")
