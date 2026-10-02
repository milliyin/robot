You have the complete robot-arm project locally. Continue from the current verified state and implement/validate FORWARD KINEMATICS (FK) properly.

CURRENT PROJECT STATE

The project already contains:

- robotic_arm.urdf
- config/servo_calibration.json
- robot/calibration.py
- robot/controller.py
- simulation/pybullet_sim.py
- hardware/arduino_serial.py
- firmware/robot_arm.ino
- main.py
- tests/

Current verification:
- `python -m pytest -q` => 21 passed
- URDF parses correctly
- PyBullet GUI and DIRECT mode load correctly
- simulation + optional hardware controller exists
- tool0 link exists
- calibration mapping is implemented
- serial bridge is implemented
- Arduino firmware accepts direct PWM packets

IMPORTANT CALIBRATION

S1:
- reference joint angle = 0 deg
- reference PWM = 284
- joint min = -130 deg
- joint max = +95 deg
- safe PWM = 90..540

S2:
- reference = 0 deg
- reference PWM = 200
- joint min = -57 deg
- joint max = +90 deg
- safe PWM = 75..420

S3:
- reference = 0 deg
- reference PWM = 550
- joint min = 0 deg
- joint max = +142 deg
- safe PWM = 250..550

S4:
- reference = 0 deg
- reference PWM = 150
- joint min = -35 deg
- joint max = +140 deg
- safe PWM = 75..470

S5:
- reference = 0 deg
- reference PWM = 190
- joint min = -52 deg
- joint max = +160 deg
- safe PWM = 75..555

S6:
- open PWM = 250
- closed PWM = 400
- max opening = 0.052 m

Measured geometry:
- base to S1 = 0.030 m
- S2 to S3 = 0.125 m
- S3 to S4 = 0.110 m
- S5 to tool0 = 0.117 m

The PyBullet raw zero-joint pose previously produced approximately:
tool0 = (0.0, 0.0, 0.382 m)

GOAL

Implement a proper FK layer that takes robot-space joint angles:

S1, S2, S3, S4, S5

and returns the tool0 pose, initially prioritizing:

X, Y, Z

in the RAW ROBOT/URDF BASE FRAME.

Do NOT mix the physical-world/display coordinate frame into the core FK.

We have evidence that the real-world measurement convention may have X/Y swapped relative to PyBullet. That should be handled later as a separate coordinate transform, not by changing the robot's FK equations.

REQUIREMENTS

1. Inspect the existing URDF and simulation code first.
2. Derive FK from the actual URDF joint chain and axes.
3. Do not assume sign conventions from generic robotics examples if the URDF says otherwise.
4. Keep FK independent from:
   - PWM
   - Arduino
   - servo calibration
   - display/world coordinate offsets
5. FK inputs must be robot-space joint angles.
6. Prefer radians internally; expose a clean degrees API if that matches the current project style.
7. S5 wrist roll should affect orientation but should not materially change tool0 XYZ if tool0 lies on the roll axis.
8. S6/gripper opening should not affect tool0 XYZ.
9. Preserve all current working simulation/hardware behavior.
10. Do not modify calibration constants unless an existing inconsistency is proven.

SUGGESTED STRUCTURE

Prefer something like:

robot/kinematics.py

with APIs similar to:

def forward_kinematics_rad(joints: dict[str, float]) -> tuple[float, float, float]:
    ...

and optionally:

def forward_kinematics_deg(joints: dict[str, float]) -> tuple[float, float, float]:
    ...

If it makes sense, return full pose later, but XYZ validation is the current priority.

MOST IMPORTANT VALIDATION

Do not trust the hand-derived equations alone.

For multiple poses:

1. Set S1-S5 to known robot-space angles.
2. Compute XYZ using our FK implementation.
3. Set exactly the same joint values in PyBullet.
4. Query `tool0` using the existing `get_tool_pose()`.
5. Compare FK XYZ against PyBullet XYZ.

They should match within a small numerical tolerance.

At minimum test:

HOME:
S1=0
S2=0
S3=0
S4=0
S5=0

Expected:
XYZ approximately (0, 0, 0.382 m)

POSE A:
S1=20
S2=20
S3=30
S4=0
S5=0

POSE B:
S1=-20
S2=30
S3=40
S4=10
S5=20

Also test:

- changing only S5 does not change XYZ significantly
- changing only gripper opening does not change tool0 XYZ
- FK respects the same joint sign convention as PyBullet
- valid boundary joint values work
- invalid/out-of-range values are either rejected or clamped consistently with the existing project architecture

TESTS

Add focused tests, probably:

tests/test_kinematics.py

Required tests:

1. zero pose gives ~0,0,0.382
2. analytic FK matches PyBullet for Pose A
3. analytic FK matches PyBullet for Pose B
4. S5-only change leaves XYZ unchanged
5. degrees/radians APIs agree, if both are implemented
6. joint naming/order cannot silently produce wrong FK

Run:

python -m pytest -q

All existing tests plus the new FK tests must pass.

VERY IMPORTANT

Do NOT implement inverse kinematics yet.

Do NOT compensate for real-world measurement differences by changing FK.

We need these layers separated:

robot joint angles
        ↓
RAW FK
        ↓
URDF/base-frame XYZ
        ↓
optional world-coordinate transform
        ↓
display / real-world XYZ

If a world-frame transform is needed, create it as a clearly separate component/function.

CURRENT REAL-WORLD MEASUREMENTS

These were collected, but DO NOT use them to alter raw FK yet:

Home:
Sim:  X=4.4, Y=-0.7, Z=18.3 cm
Real: X=-3.2, Y=11.55, Z=30.1 cm

Pose A:
Sim:  X=10, Y=4.9, Z=13 cm
Real: X=4.7, Y=16.7, Z=18.3 cm

Pose B:
Sim:  X=10.3, Y=-13.1, Z=12 cm
Real: X=-14.1, Y=14.7, Z=10.6 cm

There is a strong possibility that:
Real X corresponds approximately to Sim Y
Real Y corresponds approximately to Sim X

But do not bake this into FK.
Treat that as a separate coordinate-frame investigation after raw FK is verified.

DELIVERABLES

When finished, provide:

1. Files changed/created.
2. FK equations/transform-chain approach used.
3. Test results.
4. FK vs PyBullet comparison table for Home, Pose A, Pose B.
5. Any URDF/sign inconsistencies discovered.
6. Confirmation that no PWM/hardware calibration was changed unnecessarily.
7. Recommendation for the next step after FK passes.

Do the work directly in the existing project. Inspect existing patterns before creating new abstractions, and keep the implementation small and testable.