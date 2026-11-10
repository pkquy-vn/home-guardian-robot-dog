# Home Guardian

[![Firmware build](https://github.com/pkquy-vn/home-guardian-robot-dog/actions/workflows/firmware.yml/badge.svg)](https://github.com/pkquy-vn/home-guardian-robot-dog/actions/workflows/firmware.yml)
[![ROS 2 build](https://github.com/pkquy-vn/home-guardian-robot-dog/actions/workflows/ros2.yml/badge.svg)](https://github.com/pkquy-vn/home-guardian-robot-dog/actions/workflows/ros2.yml)

Home Guardian is a custom 8-DOF quadruped robot being built for autonomous indoor patrol. A
Raspberry Pi 5 runs the high-level software on ROS 2 Jazzy, and a Teensy 4.1 runs the real-time
controller; the two talk over a framed binary serial link. The project is at an early stage: this
repository currently contains the serial protocol, minimal Teensy firmware that exchanges
heartbeat and status messages with the Pi, the `hg_teensy_bridge` ROS 2 package, a link test
tool and a host-side protocol test. The other ROS 2 packages are planned but not written yet.

## Architecture

| | Raspberry Pi 5 (8 GB) | Teensy 4.1 |
| --- | --- | --- |
| Role | High-level computer | Real-time controller |
| Software | Ubuntu Server 24.04, ROS 2 Jazzy | Bare-metal Arduino-framework firmware, built with PlatformIO |
| Today | `hg_teensy_bridge` (heartbeat, mode commands, status, joint states), `hg_description` robot model, `tools/test_link.py` | Posture modes with leg kinematics and interpolated joint angles (no servos yet), latching heartbeat watchdog, 10 Hz status, 50 Hz joint state, status LED |
| Planned | Localization, navigation, docking, perception, events, audio, behavior (ROS 2) | Joint control and gaits; joint state, IMU, odometry, foot contact and battery data |

### Serial link

The Pi and the Teensy are connected over **USB serial** (the Teensy appears as `/dev/ttyACM0`).
The hardware UART (Teensy `Serial7` on pins 28/29 to Pi GPIO 14/15) is deferred because the
M.2 HAT+ covers the Pi's GPIO header; see
[ADR 0001](docs/decisions/0001-usb-serial-link.md).

Every message is one frame; all multi-byte fields are little-endian:

| Byte(s) | Field | Notes |
| --- | --- | --- |
| 0 | start | `0xA5` |
| 1 | type | message type, table below |
| 2 | length | payload length, 0 to 64 |
| 3 … 3+length-1 | payload | |
| last 2 | CRC | CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) over type, length and payload |

The C++ implementation is [`common/include/protocol.h`](common/include/protocol.h) (header-only,
no Arduino dependencies). Its single Python mirror is
[`hg_teensy_bridge/protocol.py`](ros2_ws/src/hg_teensy_bridge/hg_teensy_bridge/protocol.py), used
by the ROS 2 bridge and by `tools/test_link.py`. The
receiving parser rejects and counts frames with a bad CRC or length and resynchronizes on the
next start byte.

| Type | ID | Direction | Payload | Status |
| --- | --- | --- | --- | --- |
| HEARTBEAT | `0x01` | Pi → Teensy | `uint32` sequence | Implemented |
| CMD_VEL | `0x02` | Pi → Teensy | not defined yet | Reserved |
| BODY_POSE | `0x03` | Pi → Teensy | not defined yet | Reserved |
| MODE | `0x04` | Pi → Teensy | `uint8` requested mode | Implemented |
| LED | `0x05` | Pi → Teensy | not defined yet | Reserved |
| IR_LED | `0x06` | Pi → Teensy | not defined yet | Reserved |
| CONFIG | `0x07` | Pi → Teensy | not defined yet | Reserved |
| CLEAR_FAULT | `0x08` | Pi → Teensy | empty | Implemented |
| JOINT_STATE | `0x11` | Teensy → Pi | 8 x (`int16` position in 0.01 deg, `int16` current, `int8` temperature), FL hip, FL knee, FR hip, FR knee, RL hip, RL knee, RR hip, RR knee | Implemented (commanded angles; current and temperature 0 until servos exist) |
| IMU | `0x12` | Teensy → Pi | not defined yet | Reserved |
| ODOM | `0x13` | Teensy → Pi | not defined yet | Reserved |
| CONTACT | `0x14` | Teensy → Pi | not defined yet | Reserved |
| BATTERY | `0x15` | Teensy → Pi | not defined yet | Reserved |
| STATUS | `0x16` | Teensy → Pi | `uint8` mode, `uint8` gait phase, `uint16` fault bits | Implemented (gait phase is always 0 for now) |
| FAULT | `0x17` | Teensy → Pi | not defined yet | Reserved |

Modes: 0 `LIE`, 1 `SIT`, 2 `STAND`, 3 `WALK`, 9 `TORQUE_OFF`. The firmware boots in LIE and
runs the posture state machine in [`firmware/src/posture.h`](firmware/src/posture.h). Each mode
has a joint-level reference pose computed with the leg kinematics; during a transition all eight
joint angles move to the target pose with a smooth (minimum-jerk) profile. LIE to STAND takes
1.6 s, every other transition 1.0 s, and TORQUE_OFF is immediate and holds the last angles.
There are no servos yet: the commanded angles are only reported in JOINT_STATE. Geometry, joint
conventions and poses: [docs/kinematics.md](docs/kinematics.md). STATUS reports the old mode until a
transition completes. LIE, SIT and STAND can be requested from each other, WALK only from
STAND, from WALK only STAND, and from TORQUE_OFF only LIE. TORQUE_OFF is accepted from anywhere,
at any time. Other requests are rejected (and counted on the Teensy) when they are illegal,
arrive during a transition or arrive while a fault bit is set.

Fault bits: bit 0 `HEARTBEAT_LOST`. It is set at boot and clears on the first heartbeat. After
that, 500 ms without a heartbeat latches it: it stays set after heartbeats resume, until a
CLEAR_FAULT arrives while heartbeats are healthy (or the Teensy is reset). On a loss the robot
moves to SIT and turns torque off 5 s later; from LIE it stays in LIE. During a transition the
target decides. Only an accepted CLEAR_FAULT cancels the pending torque-off. See
[ADR 0002](docs/decisions/0002-heartbeat-loss-latches.md). The on-board LED blinks at 1 Hz while
the link is OK and no fault is set, and at 5 Hz otherwise.

## Repository layout

```
.
├── .github/workflows/   CI: protocol host test, firmware build, ROS 2 colcon build
├── cad/                 Mechanical design (SpotMicro-derived); attribution and license notes
├── common/
│   ├── include/         Code shared by the firmware and the Pi side (protocol.h)
│   └── test/            Host-side protocol test (g++ vs Python)
├── docs/
│   ├── decisions/       Architecture decision records
│   └── setup/           How the Raspberry Pi was set up
├── firmware/            PlatformIO project for the Teensy 4.1
├── ros2_ws/src/         colcon workspace: hg_teensy_bridge, hg_description, hg_bringup
└── tools/               Pi-side utilities (test_link.py)
```

## Prerequisites

On the Raspberry Pi (details in [docs/setup/raspberry-pi.md](docs/setup/raspberry-pi.md)):

- Ubuntu Server 24.04 and ROS 2 Jazzy (`ros-base` and `ros-dev-tools`) for `hg_teensy_bridge`
- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) (`pio`)
- PJRC and PlatformIO udev rules, your user in the `dialout` group, ModemManager disabled
- Python 3 with pyserial: `sudo apt install python3-serial`
- `g++` with C++17 for the host-side protocol test

## Quick start

These are run on the Pi from the repository root. The firmware upload and the link test need the
Teensy 4.1 connected over USB.

Clone the repository:

```bash
git clone git@github.com:pkquy-vn/home-guardian-robot-dog.git
```

```bash
cd home-guardian-robot-dog
```

Run the protocol host test (no hardware needed):

```bash
python3 common/test/test_protocol.py
```

Build the firmware:

```bash
pio run -d firmware
```

Upload the firmware to the Teensy (make sure the robot is in a safe state first). If the upload
waits for the device, press the program button on the Teensy:

```bash
pio run -d firmware -t upload
```

Run the link test (stop it with Ctrl+C):

```bash
python3 tools/test_link.py /dev/ttyACM0
```

What to expect, once the firmware is running: the script prints one line per second with the
number of heartbeats sent, about 10 STATUS frames per second, the mode, the decoded fault bits
and the CRC error count. Fault bits should read `0x0000 (none)` and the Teensy LED should blink
at 1 Hz. After you stop the script, the LED should switch to 5 Hz within about half a second.
Because the fault now latches, it stays set (`0x0001`) on the next run until CLEAR_FAULT is sent
through the ROS bridge or the Teensy is reset.
Only one program can use the serial port at a time, so close any serial monitor first.

## ROS 2: hg_teensy_bridge, hg_description, hg_bringup

The `teensy_bridge` node owns the serial link on the Pi: it sends HEARTBEAT and publishes what
the Teensy reports in STATUS and JOINT_STATE. `hg_description` holds the robot model (xacro
URDF; its joint values equal the firmware's hip and knee angles in radians, see
[docs/kinematics.md](docs/kinematics.md)), and `hg_bringup` starts the bridge together with
`robot_state_publisher`. Only one program can use the port, so do not run
`tools/test_link.py` at the same time.

The heartbeat is sent from a timer on the node's executor, never from a separate thread: if the
node stalls or is stopped, heartbeats stop and the Teensy raises `HEARTBEAT_LOST` within 500 ms.
If the port is missing or disappears (USB re-enumeration, reflash), the node logs it once,
retries every second and recovers on its own.

Build (on the Pi, from the repository root):

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
cd ros2_ws && rosdep install --from-paths src --ignore-src -y && colcon build --symlink-install
```

Run:

```bash
source install/setup.bash
```

```bash
ros2 launch hg_bringup robot.launch.py
```

(`ros2 launch hg_teensy_bridge bridge.launch.py` starts the bridge alone.)

Parameters (launch arguments of the same name): `port` (default `/dev/ttyACM0`), `baud`
(`115200`, ignored by USB serial), `heartbeat_rate_hz` (`20.0`), `status_timeout_s` (`0.5`).
With `ros2 run`, give the float parameters a decimal point, for example
`-p heartbeat_rate_hz:=20.0`.

| Topic | Type | Content |
| --- | --- | --- |
| `/teensy/mode` | `std_msgs/UInt8` | STATUS mode, on every STATUS (about 10 Hz) |
| `/teensy/faults` | `std_msgs/UInt16` | STATUS fault bits (bit 0 `HEARTBEAT_LOST`), on every STATUS |
| `/teensy/connected` | `std_msgs/Bool` | True while STATUS arrived within `status_timeout_s`; 2 Hz and on change |
| `/joint_states` | `sensor_msgs/JointState` | Joint angles in radians from JOINT_STATE (50 Hz), names `fl_hip_joint` ... `rr_knee_joint` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz: link state, STATUS rate, mode and mode name, decoded faults, CRC errors, heartbeats sent |
| `/teensy/cmd_mode` (subscribed) | `std_msgs/UInt8` | Requested mode; each message is sent as MODE |
| `/teensy/clear_fault` (service) | `std_srvs/Trigger` | Sends CLEAR_FAULT. Success means sent, not accepted: check `/teensy/faults` |

Request STAND, then clear a latched fault:

```bash
ros2 topic pub --once /teensy/cmd_mode std_msgs/msg/UInt8 "{data: 2}"
```

```bash
ros2 service call /teensy/clear_fault std_srvs/srv/Trigger
```

## Project status

**Works today**

- Protocol (`common/include/protocol.h`): frame encoding, CRC, a resynchronizing parser and the
  HEARTBEAT, MODE, CLEAR_FAULT, STATUS and JOINT_STATE payloads. The host test checks the C++
  and Python implementations against each other and runs in CI.
- Firmware: posture mode state machine with a latching heartbeat watchdog, leg kinematics with
  joint-level reference poses (both host-tested in `firmware/test_host`), interpolated joint
  angles, STATUS at 10 Hz, JOINT_STATE at 50 Hz, status LED. Built by CI on every push
  and pull request.
- `tools/test_link.py`: heartbeat sender and STATUS monitor.
- `hg_teensy_bridge`: ROS 2 node for HEARTBEAT, MODE, CLEAR_FAULT, STATUS and `/joint_states`.
  `hg_description` (robot model) and `hg_bringup` (launch). Built with colcon in CI.

**Not yet verified on hardware**

- The joint-level postures, `/joint_states` and the robot model have not yet been run on the
  robot or viewed in RViz.

**Next**

- Run the firmware and link test on the robot.
- Drive the servos with the commanded angles once they are fitted (calibrate direction signs
  and offsets).
- Define payloads for the remaining reserved message types.
- ROS 2 packages: `hg_msgs`, `hg_localization`, `hg_navigation`, `hg_docking`, `hg_perception`,
  `hg_events`, `hg_audio`, `hg_behavior`, `hg_sim`.
- Move the link to the hardware UART once a taller stacking header is fitted (ADR 0001).
- Add the CAD files with upstream attribution and license ([cad/README.md](cad/README.md)).

## License

The code is licensed under the [Apache License 2.0](LICENSE). The CAD files in `cad/` are derived
from an open-source SpotMicro model and follow the upstream model's license, which is still to be
confirmed; see [cad/README.md](cad/README.md).
