# Kinematics, reference poses and joint conventions

Implemented in [`firmware/src/kinematics.h`](../firmware/src/kinematics.h) (host-tested in
`firmware/test_host/test_kinematics.cpp`) and mirrored by the URDF in
[`hg_description`](../ros2_ws/src/hg_description/urdf/hg.urdf.xacro).

## Geometry

- Body frame (`base_link`): x forward, y left, z up, origin at the body centre. Body box
  360 x 150 x 150 mm, centred on `base_link`.
- Legs FL, FR, RL, RR. Each leg is planar (the body's x-z plane) with two pitch joints: hip,
  then knee. Hip pitch axes at x = +93 mm (front) / -93 mm (rear), y = +95 mm (left) / -95 mm
  (right), z = 0.
- Femur L1 = 108 mm, tibia L2 = 130 mm to the foot contact point.
- The same joint conventions apply to all four legs. Servo direction signs and offsets are a
  later calibration concern.

## Joint conventions

| Joint | Zero / reference | Positive direction | Limits |
| --- | --- | --- | --- |
| Hip | 0 = femur straight down | femur forward | -45 to +100 deg |
| Knee | interior angle, 180 = straight leg | opening the knee | 30 to 150 deg |

The knee points forward: the femur is forward of the hip-to-foot line.

## Inverse and forward kinematics

Foot target (x, z) in the leg plane, relative to the hip axis (x forward, z up), in mm:

- r = hypot(x, z), clamped to [|L1 - L2| + 5, L1 + L2 - 5] = [27, 233] mm along the same
  direction.
- knee = acos((L1² + L2² - r²) / (2 L1 L2))
- hip = atan2(x, -z) + acos((L1² + r² - L2²) / (2 L1 r))
- Both joints are then clamped to their limits, and the result reports which clamps applied.
  The knee limits alone restrict the reach to 65.2 to 230.0 mm.

Forward kinematics: knee at (L1 sin hip, -L1 cos hip). Tibia direction
t = hip + knee - 180 deg (measured like the hip). Foot = knee + (L2 sin t, -L2 cos t).

The host tests check IK followed by FK to within 0.1 mm over the workspace (the worst case
measured is about 0.0002 mm). They also check the STAND, LIE and SIT angles, the reach and
joint clamping, and degenerate targets.

## Reference poses

In every pose each foot rests on a flat floor directly below its hip axis.

| Pose | Body | Hip axis height above floor | Front hip / knee (deg) | Rear hip / knee (deg) |
| --- | --- | --- | --- | --- |
| LIE | level | 75 mm | 88.62 / 35.22 | 88.62 / 35.22 |
| SIT | pitched 25 deg, front raised | front 210 mm, rear 131.4 mm | 6.04 / 123.59 | 39.92 / 66.27 |
| STAND | level | 190 mm | 41.23 / 105.57 | 41.23 / 105.57 |

- **SIT, chosen for this design:**
  - **Heights:** pitch 25 deg; front hip axes 210 mm above the floor. The rear hip axes are
    2 x 93 mm x sin 25 deg = 78.6 mm lower, at 131.4 mm.
  - **Legs:** the front legs are extended (knee 123.6 deg) and the rear legs folded (knee
    66.3 deg). Every joint is at least 26 deg inside its limits.
  - **Clearance:** with the 150 mm body box centred on the hip axes, the body's rear-bottom edge
    is 26.7 mm above the floor.
  - **Leg-frame targets:** because the body is pitched, a foot vertically below the hip lies at
    (-h sin 25 deg, -h cos 25 deg) in the leg frame. That is (-88.7, -190.3) mm at the front and
    (-55.5, -119.1) mm at the rear.
- **WALK** uses the STAND pose until gaits exist.
- **TORQUE_OFF** holds the last commanded angles.

## Transitions

During a posture transition (1.6 s for LIE to STAND, 1.0 s otherwise), the firmware moves all
eight joint angles from the angles commanded at the start of the transition to the target pose,
using a minimum-jerk profile: s = 10p³ - 15p⁴ + 6p⁵, with zero velocity and acceleration at both
ends. An aborted transition, for example the forced SIT after a heartbeat loss, starts from the
angles commanded at that moment. There are no servos yet, so the angles are only reported.

## JOINT_STATE and /joint_states

The firmware sends JOINT_STATE at 50 Hz: 8 x (int16 position in 0.01 deg, int16 current, int8
temperature), ordered FL hip, FL knee, FR hip, FR knee, RL hip, RL knee, RR hip, RR knee. Until
servos exist, position is the commanded angle and current and temperature are 0.

The bridge publishes `sensor_msgs/JointState` on `/joint_states` in radians. It uses the URDF
joint names `fl_hip_joint`, `fl_knee_joint`, ... `rr_knee_joint`, and leaves `effort` and
`velocity` empty.

**URDF mapping:** the URDF joint value is the convention angle above, in radians, with no
offset or sign change:

- `<leg>_hip_joint` rotates about -y, so a positive value moves the femur forward.
- `<leg>_knee_joint` has its joint origin rotated by pi about y. At q = pi the tibia continues
  straight down the femur; as q decreases the tibia folds back, so the knee points forward.
- The URDF limits are the same: hip -0.7854 to 1.7453 rad, knee 0.5236 to 2.6180 rad.

Expected `/joint_states` values in the reference poses (the firmware resolution is 0.01 deg):

| Pose | Joints | Hip | Knee |
| --- | --- | --- | --- |
| LIE | all legs | 88.62 deg = 1.5467 rad | 35.22 deg = 0.6147 rad |
| SIT | FL, FR | 6.04 deg = 0.1054 rad | 123.59 deg = 2.1571 rad |
| SIT | RL, RR | 39.92 deg = 0.6967 rad | 66.27 deg = 1.1566 rad |
| STAND | all legs | 41.23 deg = 0.7196 rad | 105.57 deg = 1.8425 rad |
