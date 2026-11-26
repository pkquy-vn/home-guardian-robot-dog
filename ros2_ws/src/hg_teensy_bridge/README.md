# hg_teensy_bridge

ROS 2 node `teensy_bridge`: owns the serial link to the Teensy 4.1. It sends HEARTBEAT, MODE and
CLEAR_FAULT, and publishes what the Teensy reports in STATUS, JOINT_STATE and SERVO_STATE.
Protocol details are in [docs/protocol.md](../../../docs/protocol.md).

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

All four are also launch arguments. With `ros2 run`, give the float parameters a decimal point,
for example `-p heartbeat_rate_hz:=20.0`.

## Interfaces

| Name | Type | Description |
| --- | --- | --- |
| `/teensy/mode` (pub) | `std_msgs/UInt8` | STATUS mode, about 10 Hz |
| `/teensy/faults` (pub) | `std_msgs/UInt16` | STATUS fault bits (bit 0 `HEARTBEAT_LOST`) |
| `/teensy/connected` (pub) | `std_msgs/Bool` | Link state; 2 Hz and on change |
| `/joint_states` (pub) | `sensor_msgs/JointState` | Joint angles in radians, 50 Hz, names `fl_hip_joint` … `rr_knee_joint` |
| `/diagnostics` (pub) | `diagnostic_msgs/DiagnosticArray` | 1 Hz. Teensy link: state, STATUS rate, mode, faults, CRC errors, heartbeats. One status per servo, see below |
| `/teensy/cmd_mode` (sub) | `std_msgs/UInt8` | Requested mode, sent as MODE |
| `/teensy/clear_fault` (srv) | `std_srvs/Trigger` | Sends CLEAR_FAULT. Success means sent, not accepted: check `/teensy/faults` |

## Servo diagnostics

`/diagnostics` carries one status per servo, named by joint: `servo 1 FL hip` to
`servo 8 RR knee`. Values: `present`, `position_raw`, `position_deg` (raw, no calibration yet),
`speed_raw`, `load_percent`, `current_ma`, `current_raw`, `voltage_v`, `temperature_c`, `status`
and `status_bits`.

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
