import argparse
import time

from hardware.arduino_serial import ArduinoSerial
from hardware.jetson_pca9685 import JetsonPCA9685
from robot.calibration import Calibration
from robot.controller import RobotController
from robot.poses import get_joint_pose_deg
from simulation.pybullet_sim import PyBulletSim


STEP_DEG = 2.0
GRIPPER_STEP_M = 0.004
POSE_STEPS = 25
POSE_STEP_DELAY_S = 0.04


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manual robot arm digital twin runner")
    parser.add_argument("--hardware", action="store_true", help="Mirror safe PWM commands to Arduino")
    parser.add_argument("--port", default="COM6", help="Arduino serial port")
    parser.add_argument("--jetson-pca9685", action="store_true", help="Mirror safe PWM commands directly to PCA9685 over Jetson I2C")
    parser.add_argument("--i2c-address", type=lambda value: int(value, 0), default=0x40, help="PCA9685 I2C address for Jetson mode")
    parser.add_argument("--no-gui", action="store_true", help="Run PyBullet in DIRECT mode")
    return parser


def print_help() -> None:
    print("Setup: pip install -r requirements.txt")
    print("Jetson setup: pip install -r requirements-jetson.txt")
    print("Simulation: python main.py")
    print("Hardware: python main.py --hardware --port COM6")
    print("Jetson: python main.py --jetson-pca9685")
    print(
        "Keys: 1/q S1, 2/w S2, 3/e S3, 4/r S4, 5/t S5, "
        "o/p gripper, h home, a Pose A, b Pose B, c Pose C, x pose, Esc quit"
    )


def main() -> None:
    args = build_parser().parse_args()
    print_help()

    calibration = Calibration("config/servo_calibration.json")
    sim = PyBulletSim(gui=not args.no_gui)
    hardware = create_hardware_backend(args)

    controller = RobotController(calibration, sim=sim, hardware=hardware)
    try:
        run_keyboard_loop(controller)
    finally:
        if hardware is not None:
            hardware.close()
        sim.close()


def create_hardware_backend(args):
    if args.hardware and args.jetson_pca9685:
        raise ValueError("Choose either --hardware for Arduino or --jetson-pca9685 for direct I2C, not both")

    if args.hardware:
        hardware = ArduinoSerial(port=args.port)
        if not hardware.connected:
            print(f"Hardware disabled: could not open {args.port}: {hardware.error}")
            return None
        return hardware

    if args.jetson_pca9685:
        hardware = JetsonPCA9685(address=args.i2c_address)
        if not hardware.connected:
            print(f"Jetson PCA9685 disabled: could not open I2C address {hex(args.i2c_address)}: {hardware.error}")
            return None
        return hardware

    return None


def run_keyboard_loop(controller: RobotController) -> None:
    import msvcrt

    bindings = {
        "1": ("s1", STEP_DEG),
        "q": ("s1", -STEP_DEG),
        "2": ("s2", STEP_DEG),
        "w": ("s2", -STEP_DEG),
        "3": ("s3", STEP_DEG),
        "e": ("s3", -STEP_DEG),
        "4": ("s4", STEP_DEG),
        "r": ("s4", -STEP_DEG),
        "5": ("s5", STEP_DEG),
        "t": ("s5", -STEP_DEG),
    }

    while True:
        if not msvcrt.kbhit():
            time.sleep(0.02)
            continue

        key = msvcrt.getch()
        if key == b"\x1b":
            break
        char = key.decode("ascii", errors="ignore").lower()
        if char in bindings:
            joint, delta = bindings[char]
            controller.set_joint_deg(joint, controller.state_deg[joint] + delta)
        elif char == "o":
            controller.set_gripper_opening_m(controller.gripper_opening_m + GRIPPER_STEP_M)
        elif char == "p":
            controller.set_gripper_opening_m(controller.gripper_opening_m - GRIPPER_STEP_M)
        elif char == "h":
            move_to_named_pose(controller, "home")
            controller.set_gripper_opening_m(controller.calibration.gripper["max_opening_m"])
        elif char == "a":
            move_to_named_pose(controller, "pose_a")
        elif char == "b":
            move_to_named_pose(controller, "pose_b")
        elif char == "c":
            move_to_named_pose(controller, "pose_c")
        elif char == "x":
            position, _ = controller.get_tool_pose()
            print(f"tool0 xyz: {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}")


def move_to_named_pose(controller: RobotController, name: str) -> None:
    target = get_joint_pose_deg(name)
    start = dict(controller.state_deg)
    for step in range(1, POSE_STEPS + 1):
        ratio = step / POSE_STEPS
        intermediate = {
            joint: start[joint] + (target[joint] - start[joint]) * ratio
            for joint in target
        }
        controller.set_pose_deg(intermediate)
        time.sleep(POSE_STEP_DELAY_S)


if __name__ == "__main__":
    main()
