# Robot Arm Digital Twin Progress

Date: 2026-09-19

## Current Status

The PC-side digital twin and safe hardware bridge have been implemented. The project now has a measured URDF robot model, servo calibration config, PyBullet simulation backend, Arduino serial bridge, controller, safe Arduino firmware, manual runner, and automated tests.

Simulation is working. PyBullet GUI loads the robot. Forward kinematics is enabled and can be validated against PyBullet.

## Verification Completed

Commands run:

```bash
python -m pytest -q
```

Result:

```txt
31 passed
```

URDF parse check:

```bash
python -c "import xml.etree.ElementTree as ET; ET.parse('robotic_arm.urdf'); print('URDF OK')"
```

Result:

```txt
URDF OK
```

PyBullet DIRECT smoke test:

```bash
python -c "from simulation.pybullet_sim import PyBulletSim; sim=PyBulletSim(gui=False); sim.set_joint_positions_rad({'joint_1_base':0,'joint_2_shoulder':0,'joint_3_elbow':0,'joint_4_wrist_pitch':0,'joint_5_wrist_roll':0}); sim.set_gripper_opening_m(0.052); print('tool0', sim.get_tool_pose()[0]); sim.set_gripper_opening_m(0.0); sim.close()"
```

Result:

```txt
tool0 (0.0, 0.0, 0.38199999928474426)
```

## Files Created Or Updated

### Design And Handoff

- `2026-09-19-robot-digital-twin-design.md`
- `2026-09-19-robot-digital-twin.md`
- `progress.md`

### Robot Model

- `robotic_arm.urdf`

Implemented:

- Corrected measured geometry.
- S1-S5 revolute joints.
- Safe measured joint limits.
- Two symmetric prismatic gripper fingers.
- Fixed `tool0` frame at the gripper center tip.

Measured geometry used:

- Base plate to S1: `0.03 m`
- S2 to S3: `0.125 m`
- S3 to S4: `0.110 m`
- S5 to tool tip: `0.117 m`
- Gripper max opening: `0.052 m`

### Calibration

- `config/servo_calibration.json`
- `robot/__init__.py`
- `robot/calibration.py`

Implemented:

- Robot-space degrees to PWM conversion.
- Piecewise-linear measured calibration.
- Joint clamping before PWM conversion.
- Gripper opening to PWM conversion.

Safe PWM ranges:

- S1: `90..540`
- S2: `75..420`
- S3: `250..550`
- S4: `75..470`
- S5: `75..555`
- S6: `250..400`

Startup PWM:

```txt
S1 284, S2 200, S3 550, S4 150, S5 190, S6 250
```

### Simulation

- `simulation/__init__.py`
- `simulation/pybullet_sim.py`

Implemented:

- PyBullet `GUI` and `DIRECT` support.
- URDF loading with fixed base.
- Joint discovery by name.
- S1-S5 joint control in radians.
- Symmetric gripper control in meters.
- `tool0` pose query for later FK/IK work.

### Hardware Serial Bridge

- `hardware/__init__.py`
- `hardware/arduino_serial.py`

Implemented:

- Six-channel PWM packet protocol.
- Newline-terminated packets.
- Safe PWM clamping.
- Rate limiting, default `30 Hz`.
- Non-fatal missing COM port behavior.

Protocol sent to Arduino:

```txt
284,200,550,150,190,250\n
```

Important: this is PWM, not old `0..180` angle packets.

### Controller

- `robot/controller.py`

Implemented:

- One canonical robot-space state.
- Simulation fan-out using radians.
- Hardware fan-out using calibrated PWM.
- Gripper fan-out to simulation and hardware.
- `get_tool_pose()` delegation to PyBullet.

### Forward Kinematics

- `robot/kinematics.py`
- `tests/test_kinematics.py`

Implemented:

- Raw URDF/base-frame FK for S1-S5.
- Radians-first API:

```python
forward_kinematics_rad({"s1": ..., "s2": ..., "s3": ..., "s4": ..., "s5": ...})
```

- Degrees wrapper:

```python
forward_kinematics_deg({"s1": ..., "s2": ..., "s3": ..., "s4": ..., "s5": ...})
```

- Joint-name validation so missing or unknown names cannot silently produce a wrong pose.
- URDF safe-limit validation; out-of-range FK inputs are rejected.
- FK remains independent from PWM, Arduino, display frames, real-world coordinate transforms, and gripper opening.

Transform chain used:

```txt
Tz(0.030)
Rz(S1)
Ry(S2)
Tz(0.125)
Ry(S3)
Tz(0.110)
Ry(S4)
Rz(S5)
Tz(0.117)
```

This follows the actual URDF:

- S1 axis: `0 0 1`
- S2 axis: `0 1 0`
- S3 axis: `0 1 0`
- S4 axis: `0 1 0`
- S5 axis: `0 0 1`
- `tool0_fixed_joint`: `0 0 0.117`

FK vs PyBullet comparison:

```txt
pose,fk_x,fk_y,fk_z,pybullet_x,pybullet_y,pybullet_z,max_abs_diff
Home,0.000000,0.000000,0.382000,0.000000,0.000000,0.382000,0.000000
Pose A,0.203579,0.074097,0.293374,0.203605,0.074106,0.293341,0.000034
Pose B,0.264137,-0.096138,0.196192,0.264143,-0.096140,0.196164,0.000029
```

Notes:

- S5 wrist roll changes orientation but does not materially change `tool0` XYZ because the tool offset lies on the roll axis.
- Gripper opening does not change `tool0` XYZ.
- No PWM or hardware calibration constants were changed for FK.
- The observed real-world X/Y convention mismatch should still be handled later as a separate coordinate-frame transform, not inside raw FK.

### Arduino Firmware

- `firmware/robot_arm.ino`
- `firmware/robot_arm/robot_arm.ino`

Implemented:

- Uses `Adafruit_PWMServoDriver`.
- Accepts exactly six comma-separated integer PWM values.
- Rejects malformed packets.
- Clamps each channel independently on-device.
- Applies safe startup PWM values.

Required Arduino libraries:

- `Adafruit PWM Servo Driver Library`
- `Adafruit BusIO`

### Manual Runner

- `main.py`
- `requirements.txt`

Run simulation:

```bash
python main.py
```

Run simulation without GUI:

```bash
python main.py --no-gui
```

Run simulation plus hardware:

```bash
python main.py --hardware --port COM6
```

Keyboard controls are read from the terminal window, not from the PyBullet GUI:

```txt
1 / q  -> S1 base
2 / w  -> S2 shoulder
3 / e  -> S3 elbow
4 / r  -> S4 wrist pitch
5 / t  -> S5 wrist roll
o / p  -> gripper open/close
h      -> home
x      -> print tool0 XYZ
Esc    -> quit
```

### Tests

- `tests/test_urdf.py`
- `tests/test_calibration.py`
- `tests/test_simulation.py`
- `tests/test_serial_bridge.py`
- `tests/test_controller.py`
- `tests/test_firmware.py`
- `tests/test_kinematics.py`

Coverage includes:

- URDF structure and limits.
- `tool0` fixed link.
- Symmetric gripper joints.
- Calibration endpoints.
- Gripper PWM endpoints.
- PyBullet DIRECT load and pose query.
- Serial packet format, clamping, and rate limiting.
- Controller simulation/hardware fan-out.
- Firmware safe constants and strict packet parsing.
- Raw FK against PyBullet for Home, Pose A, and Pose B.
- S5-only and gripper-only XYZ invariance.
- FK degrees/radians API agreement.

## Difference From Old Code

Old setup:

- Python sent servo angles such as:

```txt
90.0,90.0,90.0,90.0,90.0,45.0\n
```

- Arduino mapped `0..180` angles to PWM using old `pulseMin` and `pulseMax`.

New setup:

- Python sends calibrated PWM directly:

```txt
284,200,550,150,190,250\n
```

- Arduino clamps PWM again and rejects malformed packets.

Do not use the old Ursina sender with the new firmware unless it is updated to send integer PWM packets.

## Remaining Work

### Hardware Validation

Still needs physical bench testing:

1. Upload `firmware/robot_arm.ino` to Arduino Mega.
2. Install Arduino libraries:
   - `Adafruit PWM Servo Driver Library`
   - `Adafruit BusIO`
3. Connect PCA9685, servo power, and common ground.
4. Run:

```bash
python main.py --hardware --port COM6
```

5. Test one joint at a time with small movements.
6. Confirm each servo direction is correct.
7. Confirm S1 never approaches hard contact and remains in `90..540` PWM.
8. Test gripper open/close last.

### Future Phase

Not implemented yet:

- Real-world forward kinematics validation against measured poses.
- Inverse kinematics.
- Autonomous IK-to-hardware execution.
- Trajectory planner for larger smooth hardware moves.
- Better GUI sliders for the new calibrated controller.

## Notes For Main Agent

- This folder is not a Git repository.
- A local execution ledger also exists at:

```txt
.superpowers/sdd/2026-09-19-robot-digital-twin/progress.md
```

- Current verified state: PyBullet simulation loads; raw FK is enabled and should be validated with `tests/test_kinematics.py` before using it for later coordinate-frame work.
- The next meaningful tasks are real hardware validation with the Arduino/PCA9685/servos connected and a separate raw-URDF-to-real-world coordinate-frame investigation.
