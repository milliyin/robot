# Robot Arm Digital Twin Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an IK-ready PyBullet digital twin that mirrors safe joint commands to the Arduino/PCA9685 physical robot.

**Architecture:** Keep robot geometry in URDF and physical servo calibration in JSON. A controller owns one joint-state vector and applies it to PyBullet plus an optional serial hardware backend, so simulation and hardware share the same robot-space commands.

**Tech Stack:** Python 3, PyBullet, pyserial, pytest, URDF/XML, Arduino C++, Adafruit PCA9685 library.

**Spec:** `docs/superpowers/specs/2026-09-19-robot-digital-twin-design.md`

## Global Constraints
- Use the measured link distances from the spec without changing them.
- Use S1 safe PWM 90..540; never use the hard-stop values 74 or 555 as operating limits.
- Start in simulation-only mode; hardware requires explicit enablement.
- Keep robot joint angles separate from PCA9685 PWM calibration.
- S6 is gripper opening, not a sixth end-effector pose DOF.
- Do not implement autonomous IK execution to hardware in this phase.

## Review Focus
- Reversed or asymmetric servo mappings must still reproduce each measured calibration endpoint exactly.
- Missing COM port must leave simulation fully usable instead of crashing.
- Malformed serial packets must never produce unclamped servo motion.
- Gripper open/close mapping must preserve 52 mm maximum opening and symmetric simulated fingers.
- URDF `tool0` must remain fixed at the gripper-center tip and not move with either finger.

---

### Task 1: Correct the URDF and add the tool/gripper structure

**Files:**
- Modify: `robotic_arm.urdf`
- Test: `tests/test_urdf.py`

**Interfaces:**
- Consumes: measured geometry and safe joint limits from the spec.
- Produces: joints `joint_1_base` through `joint_5_wrist_roll`, gripper finger joints, and fixed link `tool0`.

- [ ] **Step 1: Write failing URDF structure tests**

```python
import math
import xml.etree.ElementTree as ET


def test_urdf_has_tool0_and_two_fingers():
    root = ET.parse('robotic_arm.urdf').getroot()
    links = {x.get('name') for x in root.findall('link')}
    assert {'left_finger_link', 'right_finger_link', 'tool0'} <= links


def test_measured_joint_limits_are_present():
    root = ET.parse('robotic_arm.urdf').getroot()
    expected = {
        'joint_1_base': (-130, 95),
        'joint_2_shoulder': (-57, 90),
        'joint_3_elbow': (0, 142),
        'joint_4_wrist_pitch': (-35, 140),
        'joint_5_wrist_roll': (-52, 160),
    }
    joints = {j.get('name'): j for j in root.findall('joint')}
    for name, (lo, hi) in expected.items():
        limit = joints[name].find('limit')
        assert math.isclose(float(limit.get('lower')), math.radians(lo), abs_tol=1e-3)
        assert math.isclose(float(limit.get('upper')), math.radians(hi), abs_tol=1e-3)
```

- [ ] **Step 2: Run tests and verify they fail**

Run: `pytest tests/test_urdf.py -v`
Expected: FAIL because `right_finger_link` and `tool0` do not exist and limits differ.

- [ ] **Step 3: Update URDF**

Implement visual and collision primitives for each existing rigid link, use the measured safe limits, replace the single revolute S6 finger with two symmetric prismatic simulation fingers, and add a fixed `tool0` at the center of the 0.117 m tool tip.

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_urdf.py -v`
Expected: PASS.

### Task 2: Add measured servo calibration mapping

**Files:**
- Create: `config/servo_calibration.json`
- Create: `robot/calibration.py`
- Test: `tests/test_calibration.py`

**Interfaces:**
- Produces: `Calibration.joint_deg_to_pwm(joint_name: str, angle_deg: float) -> int`, `Calibration.clamp_joint_deg(...) -> float`, and gripper opening conversion.

- [ ] **Step 1: Write failing endpoint tests**

```python
from robot.calibration import Calibration


def test_s1_reference_and_safe_endpoints():
    c = Calibration('config/servo_calibration.json')
    assert c.joint_deg_to_pwm('s1', 0) == 284
    assert c.joint_deg_to_pwm('s1', 95) == 90
    assert c.joint_deg_to_pwm('s1', -130) == 540


def test_s2_reference_and_safe_endpoints():
    c = Calibration('config/servo_calibration.json')
    assert c.joint_deg_to_pwm('s2', 0) == 200
    assert c.joint_deg_to_pwm('s2', -57) == 75
    assert c.joint_deg_to_pwm('s2', 90) == 420


def test_gripper_endpoints():
    c = Calibration('config/servo_calibration.json')
    assert c.gripper_opening_to_pwm(0.052) == 250
    assert c.gripper_opening_to_pwm(0.0) == 400
```

- [ ] **Step 2: Run and verify failure**
Run: `pytest tests/test_calibration.py -v`
Expected: FAIL because calibration module/config do not exist.

- [ ] **Step 3: Implement piecewise-linear mapping**
Use the measured zero plus safe endpoints for each joint. Clamp in robot-space before interpolation. Preserve asymmetric endpoints exactly.

- [ ] **Step 4: Run tests**
Run: `pytest tests/test_calibration.py -v`
Expected: PASS.

### Task 3: Add PyBullet simulation backend

**Files:**
- Create: `simulation/pybullet_sim.py`
- Test: `tests/test_simulation.py`

**Interfaces:**
- Produces: `PyBulletSim(gui: bool)`, `set_joint_positions_rad(dict[str, float])`, `set_gripper_opening_m(float)`, `get_tool_pose()`, `close()`.

- [ ] **Step 1: Write a DIRECT-mode smoke test**
Load `robotic_arm.urdf`, assert required joints exist, apply zero pose, and assert `tool0` pose contains finite XYZ values.

- [ ] **Step 2: Run test and verify failure**
Run: `pytest tests/test_simulation.py -v`
Expected: FAIL because simulator module does not exist.

- [ ] **Step 3: Implement PyBullet backend**
Use `pybullet.DIRECT` for tests and `pybullet.GUI` for interactive mode. Load a plane, load the URDF with fixed base, discover joint indices by name, and control S1-S5 plus symmetric gripper fingers.

- [ ] **Step 4: Run simulation tests**
Run: `pytest tests/test_simulation.py -v`
Expected: PASS.

### Task 4: Add rate-limited serial hardware backend

**Files:**
- Create: `hardware/arduino_serial.py`
- Test: `tests/test_serial_bridge.py`

**Interfaces:**
- Produces: `ArduinoSerial(port, baud=115200, max_hz=30)`, `send_pwm(list[int])`, `close()`.

- [ ] **Step 1: Write tests with an injected fake serial object**
Verify newline-terminated six-value packets, value clamping, and rate limiting.

- [ ] **Step 2: Run and verify failure**
Run: `pytest tests/test_serial_bridge.py -v`
Expected: FAIL because backend does not exist.

- [ ] **Step 3: Implement backend**
Allow dependency injection for tests, set `timeout=0.05`, send at no more than 30 Hz, and expose connection failure without killing simulation.

- [ ] **Step 4: Run tests**
Run: `pytest tests/test_serial_bridge.py -v`
Expected: PASS.

### Task 5: Add the single-state robot controller

**Files:**
- Create: `robot/controller.py`
- Test: `tests/test_controller.py`

**Interfaces:**
- Consumes: `Calibration`, `PyBulletSim`, optional `ArduinoSerial`.
- Produces: `RobotController.set_joint_deg(name, angle)`, `set_gripper_opening_m(opening)`, `set_pose_deg(dict)`, `get_tool_pose()`.

- [ ] **Step 1: Write failing fan-out tests**
Assert a robot-space command is clamped once, converted to radians for simulation, converted to calibrated PWM for hardware, and stored as the current state.

- [ ] **Step 2: Run and verify failure**
Run: `pytest tests/test_controller.py -v`
Expected: FAIL because controller does not exist.

- [ ] **Step 3: Implement controller**
Keep the canonical state in robot-space units. Simulation and hardware are outputs, never sources of calibration truth.

- [ ] **Step 4: Run tests**
Run: `pytest tests/test_controller.py -v`
Expected: PASS.

### Task 6: Harden Arduino firmware

**Files:**
- Create: `firmware/robot_arm.ino`

**Interfaces:**
- Consumes: six comma-separated PCA9685 pulse targets.
- Produces: clamped PCA9685 commands on channels 0..5.

- [ ] **Step 1: Define firmware safe ranges**
Use exactly: S1 90..540, S2 75..420, S3 250..550, S4 75..470, S5 75..555, S6 250..400.

- [ ] **Step 2: Implement strict packet parsing**
Accept exactly six integer fields plus newline; ignore malformed packets instead of partially applying them.

- [ ] **Step 3: Clamp and command**
Clamp each pulse independently and call `pwm.setPWM(channel, 0, pulse)`.

- [ ] **Step 4: Add safe startup values**
Use measured references: S1 284, S2 200, S3 550, S4 150, S5 190, S6 250.

### Task 7: Add interactive manual runner

**Files:**
- Create: `main.py`
- Create: `requirements.txt`

**Interfaces:**
- CLI flags: `--hardware`, `--port`, `--no-gui`.

- [ ] **Step 1: Add simulation-only startup**
Default to PyBullet GUI without opening serial.

- [ ] **Step 2: Add manual keyboard/UI controls**
Provide safe incremental S1-S5 movement, S6 open/close, home, and print `tool0` XYZ.

- [ ] **Step 3: Add explicit hardware opt-in**
Only create `ArduinoSerial` when `--hardware` is supplied. Default Windows port can be `COM6`, but allow override.

- [ ] **Step 4: Document setup**
Print concise startup help including `pip install -r requirements.txt`, `python main.py`, and `python main.py --hardware --port COM6`.

### Task 8: Whole-system verification

**Files:**
- Test: all test files above.

**Interfaces:** none.

- [ ] **Step 1: Run Python tests**
Run: `pytest -q`
Expected: all tests PASS.

- [ ] **Step 2: Parse URDF independently**
Run: `python -c "import xml.etree.ElementTree as ET; ET.parse('robotic_arm.urdf'); print('URDF OK')"`
Expected: `URDF OK`.

- [ ] **Step 3: Run PyBullet DIRECT smoke test**
Run a short script that loads the model, sets the measured zero pose, opens/closes the simulated gripper, and prints finite `tool0` XYZ values.

- [ ] **Step 4: Hardware bench test**
With the arm supported and workspace clear, enable hardware and verify one joint at a time at zero, then one small positive/negative movement, then S6 open/close. Do not run full-range automated sweeps.
