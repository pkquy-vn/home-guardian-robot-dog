# hg_teensy_bridge

ROS 2 node `teensy_bridge`: owns the serial link to the Teensy 4.1. It sends HEARTBEAT, MODE,
CLEAR_FAULT and the servo commands, and publishes what the Teensy reports. The `servo`
command-line tool calibrates and tests joints through the bridge. Protocol details are in
[docs/protocol.md](../../../docs/protocol.md); servo motion and safety in
[docs/servo-motion.md](../../../docs/servo-motion.md).

The heartbeat is sent from a timer on the node's executor, never from a separate thread. If the
node stalls or stops, heartbeats stop and the Teensy raises `HEARTBEAT_LOST` within 500 ms. If the
serial port is missing or disappears (USB re-enumeration, reflash), the node logs it once,
retries every second and recovers on its own.

## Run

```bash
ros2 launch hg_bringup robot.launch.py        # bridge + robot_state_publisher
ros2 launch hg_teensy_bridge bridge.launch.py # bridge only
```

## Parameters

| Name | Default | Description |
| --- | --- | --- |
| `port` | `/dev/ttyACM0` | Serial port of the Teensy |
| `baud` | `115200` | Baud rate (ignored by USB serial) |
| `heartbeat_rate_hz` | `20.0` | HEARTBEAT send rate |
| `status_timeout_s` | `0.5` | The link counts as connected while STATUS arrived within this time |
| `calibration_file` | `~/.config/home_guardian/servo_calibration.yaml` | The Pi's copy of the joint calibration. Empty: keep no copy |

All five are also launch arguments. With `ros2 run`, give the float parameters a decimal point,
for example `-p heartbeat_rate_hz:=20.0`.

## Interfaces

| Name | Type | Description |
| --- | --- | --- |
| `/teensy/mode` (pub) | `std_msgs/UInt8` | STATUS mode, about 10 Hz |
| `/teensy/faults` (pub) | `std_msgs/UInt16` | STATUS fault bits (bit 0 `HEARTBEAT_LOST`) |
| `/teensy/connected` (pub) | `std_msgs/Bool` | Link state; 2 Hz and on change |
| `/joint_states` (pub) | `sensor_msgs/JointState` | Joint angles in radians, 50 Hz, names `fl_hip_joint` … `rr_knee_joint`. Measured for joints whose servo is present and calibrated, commanded otherwise |
| `/diagnostics` (pub) | `diagnostic_msgs/DiagnosticArray` | 1 Hz. Teensy link: state, STATUS rate, mode, faults and their causes, speed limit, CRC errors, heartbeats. One status per servo, see below |
| `/teensy/command_result` (pub) | `std_msgs/String` | The Teensy's answer to every command, as `OK: ...` or `FAILED: ...` |
| `/teensy/cmd_mode` (sub) | `std_msgs/UInt8` | Requested mode, sent as MODE |
| `/teensy/servo_command` (sub) | `hg_msgs/ServoCommand` | Calibration and joint test commands, published by the `servo` tool |
| `/teensy/clear_fault` (srv) | `std_srvs/Trigger` | Sends CLEAR_FAULT. Success means sent: the answer is on `/teensy/command_result` |

The bridge also logs every answer, and the cause of every servo fault.

## The `servo` tool

```bash
ros2 run hg_teensy_bridge servo show                  # calibration and state of every joint
ros2 run hg_teensy_bridge servo calibrate fl_hip fl_knee
ros2 run hg_teensy_bridge servo flip fl_knee          # invert the direction sign
ros2 run hg_teensy_bridge servo angle fl_knee 100     # drive one joint, in degrees
ros2 run hg_teensy_bridge servo speed 30              # speed limit, in deg/s
ros2 run hg_teensy_bridge servo clear-fault
ros2 run hg_teensy_bridge servo restore               # send the Pi's calibration copy back
ros2 run hg_teensy_bridge servo off                   # torque off, now
```

The procedure and the safety rules are in
[docs/servo-motion.md](../../../docs/servo-motion.md).

**Calibration copy:** the Teensy stores the calibration. The bridge mirrors it into
`calibration_file`, which is outside the repository, whenever it changes. A joint the Teensy
reports as uncalibrated keeps its saved entry; `servo restore` sends the saved entries back.

## Servo diagnostics

`/diagnostics` carries one status per servo, named by joint: `servo 1 FL hip` to
`servo 8 RR knee`. Servo values: `present`, `position_raw`, `position_deg` (the servo's own
angle), `speed_raw`, `load_percent`, `current_ma`, `current_raw`, `voltage_v`, `temperature_c`,
`status` and `status_bits`. Joint values: `calibrated`, `direction`, `reference_raw`, `torque`
(as the servo reports it), `driven`, `test_target`, `target_deg`, `measured_deg` and
`drive_block` (why the joint cannot be given torque, or `none`).

| Level | When |
| --- | --- |
| OK | Present and normal |
| WARN | 60 deg C or above, or a status bit is set |
| ERROR | Missing, or 70 deg C or above |
| STALE | No SERVO_STATE from the Teensy for 1 s |

The node also logs one summary when the servo states are first known, then one line whenever a
servo appears or goes missing.

## Examples

Request STAND (modes: 0 LIE, 1 SIT, 2 STAND, 3 WALK, 9 TORQUE_OFF):

```bash
ros2 topic pub --once /teensy/cmd_mode std_msgs/msg/UInt8 "{data: 2}"
```

Clear a latched fault:

```bash
ros2 service call /teensy/clear_fault std_srvs/srv/Trigger
```

Watch the Teensy's answers:

```bash
ros2 topic echo /teensy/command_result
```
