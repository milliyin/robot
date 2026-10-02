# Robot Arm Digital Twin Design

## Goal
Build a PC-based PyBullet digital twin that uses the measured robot geometry and servo calibration, can run in simulation-only mode, and can optionally mirror commands to the Arduino Mega/PCA9685 physical arm. This phase prepares the model for later forward/inverse kinematics without implementing autonomous IK yet.

## Known geometry
- Base plate to S1 axis: 0.03 m vertical.
- S1 to S2 axis-center gap: 0.00 m.
- S2 to S3 axis-center distance: 0.125 m.
- S3 to S4 axis-center distance: 0.110 m.
- S4 to S5 axis-center gap: 0.00 m.
- S5 axis to gripper/tool tip: 0.117 m.
- Gripper maximum opening: 0.052 m.

## Measured calibration

| Joint | Zero/reference | Safe PWM | Measured joint range |
|---|---|---|---|
| S1 base | PWM 284 at physical reference 180 deg = joint 0 deg | 90..540 | -130 deg .. +95 deg |
| S2 shoulder | PWM 200 at physical reference 90 deg = joint 0 deg | 75..420 | -57 deg .. +90 deg |
| S3 elbow | PWM 550 at straight reference 93 deg = joint 0 deg | 250..550 | 0 deg .. +142 deg |
| S4 wrist pitch | PWM 150 at physical reference 90 deg = joint 0 deg | 75..470 | -35 deg .. +140 deg |
| S5 wrist roll | PWM 190 at physical reference 90 deg = joint 0 deg | 75..555 | -52 deg .. +160 deg |
| S6 gripper | PWM 250 open, PWM 400 closed | 250..400 | opening 0.052 m .. 0.0 m |

S1 hard mechanical contacts were observed at PWM 74 and 555. Normal software must clamp to 90..540.

## Coordinate conventions
- S1: revolute around +Z.
- S2: revolute around +Y.
- S3: revolute around +Y.
- S4: revolute around +Y.
- S5: revolute around +Z.
- S6: gripper opening, represented in the simulator as two symmetric sliding fingers driven by one normalized command.
- `tool0` is a fixed frame at the center of the gripper tip, 0.117 m from S5 along the arm/tool centerline at the zero pose.

## Architecture
- `robotic_arm.urdf`: kinematic model, visual/collision primitives, safe joint limits, two gripper fingers, fixed `tool0`.
- `config/servo_calibration.json`: measured PWM and joint mappings. Hardware calibration stays out of the URDF.
- `robot/calibration.py`: piecewise-linear mapping between robot joint angles and measured PWM endpoints/reference points.
- `hardware/arduino_serial.py`: rate-limited USB serial protocol for six commands.
- `simulation/pybullet_sim.py`: PyBullet loader, joint discovery, joint control, gripper control, tool pose reading.
- `robot/controller.py`: single robot state and fan-out to simulation and optional hardware.
- `main.py`: manual test UI/CLI with simulation-only and simulation+hardware modes.
- `firmware/robot_arm.ino`: accepts validated PWM targets or normalized joint packets, clamps again on-device, and drives PCA9685.

## Safety
- Never command S1 outside 90..540 PWM.
- All joints clamp to measured safe operating ranges before serial transmission.
- Arduino clamps again independently.
- Startup is simulation-only unless hardware is explicitly enabled.
- Hardware mode moves through a bounded-rate trajectory rather than jumping directly to far targets.
- No automatic IK-to-hardware execution in this phase.

## Validation
1. URDF parses successfully.
2. PyBullet loads the URDF with all expected joints and `tool0`.
3. Each simulated joint reaches the measured safe limit without exceeding URDF limits.
4. Calibration mapping reproduces all measured reference/end points.
5. Simulation-only manual control works with no serial device.
6. Hardware mode sends bounded commands at a fixed maximum update rate.
7. Arduino rejects/clamps out-of-range commands.
8. Tool pose can be queried for later FK/IK validation.
