# hg_teensy_bridge

ROS 2 node `teensy_bridge`: owns the serial link to the Teensy 4.1. It sends HEARTBEAT, MODE and
CLEAR_FAULT, and publishes what the Teensy reports in STATUS and JOINT_STATE. Protocol details are
in [docs/protocol.md](../../../docs/protocol.md).

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
| `/diagnostics` (pub) | `diagnostic_msgs/DiagnosticArray` | Link state, STATUS rate, mode, faults, CRC errors, heartbeats; 1 Hz |
| `/teensy/cmd_mode` (sub) | `std_msgs/UInt8` | Requested mode, sent as MODE |
| `/teensy/clear_fault` (srv) | `std_srvs/Trigger` | Sends CLEAR_FAULT. Success means sent, not accepted: check `/teensy/faults` |

## Examples

Request STAND (modes: 0 LIE, 1 SIT, 2 STAND, 3 WALK, 9 TORQUE_OFF):

```bash
ros2 topic pub --once /teensy/cmd_mode std_msgs/msg/UInt8 "{data: 2}"
```

Clear a latched fault:

```bash
ros2 service call /teensy/clear_fault std_srvs/srv/Trigger
```
