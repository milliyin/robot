import main


class FakeController:
    def __init__(self):
        self.state_deg = {"s1": 0.0, "s2": 0.0, "s3": 0.0, "s4": 0.0, "s5": 0.0}
        self.poses = []

    def set_pose_deg(self, pose):
        self.state_deg.update(pose)
        self.poses.append(dict(pose))


def test_move_to_named_pose_interpolates_to_pose_a(monkeypatch):
    monkeypatch.setattr(main, "POSE_STEPS", 5)
    monkeypatch.setattr(main, "POSE_STEP_DELAY_S", 0.0)

    controller = FakeController()

    main.move_to_named_pose(controller, "pose_a")

    assert len(controller.poses) == 5
    assert controller.poses[-1] == {"s1": 20.0, "s2": 20.0, "s3": 30.0, "s4": 0.0, "s5": 0.0}


def test_move_to_named_pose_interpolates_to_pose_c(monkeypatch):
    monkeypatch.setattr(main, "POSE_STEPS", 5)
    monkeypatch.setattr(main, "POSE_STEP_DELAY_S", 0.0)

    controller = FakeController()

    main.move_to_named_pose(controller, "pose_c")

    assert len(controller.poses) == 5
    assert controller.poses[-1] == {
        "s1": 95.0,
        "s2": -53.713004,
        "s3": 70.206658,
        "s4": 86.573269,
        "s5": 0.0,
    }
