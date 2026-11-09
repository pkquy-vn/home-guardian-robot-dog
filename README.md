# Home Guardian

[![Firmware build](https://github.com/pkquy-vn/home-guardian-robot-dog/actions/workflows/firmware.yml/badge.svg)](https://github.com/pkquy-vn/home-guardian-robot-dog/actions/workflows/firmware.yml)

Home Guardian is a custom 8-DOF quadruped robot being built for autonomous indoor patrol. A
Raspberry Pi 5 runs the high-level software on ROS 2 Jazzy, and a Teensy 4.1 runs the real-time
controller; the two talk over a framed binary serial link. The project is at an early stage: this
repository currently contains the serial protocol, minimal Teensy firmware that exchanges
heartbeat and status messages with the Pi, a link test tool and a host-side protocol test. The
ROS 2 packages are planned but not written yet.

## Architecture

| | Raspberry Pi 5 (8 GB) | Teensy 4.1 |
| --- | --- | --- |
| Role | High-level computer | Real-time controller |
| Software | Ubuntu Server 24.04, ROS 2 Jazzy | Bare-metal Arduino-framework firmware, built with PlatformIO |
| Today | Runs `tools/test_link.py` | Heartbeat watchdog, 10 Hz status, status LED |
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
no Arduino dependencies); [`tools/test_link.py`](tools/test_link.py) mirrors it in Python. The
receiving parser rejects and counts frames with a bad CRC or length and resynchronizes on the
next start byte.

| Type | ID | Direction | Payload | Status |
| --- | --- | --- | --- | --- |
| HEARTBEAT | `0x01` | Pi → Teensy | `uint32` sequence | Implemented |
| CMD_VEL | `0x02` | Pi → Teensy | not defined yet | Reserved |
| BODY_POSE | `0x03` | Pi → Teensy | not defined yet | Reserved |
| MODE | `0x04` | Pi → Teensy | not defined yet | Reserved |
| LED | `0x05` | Pi → Teensy | not defined yet | Reserved |
| IR_LED | `0x06` | Pi → Teensy | not defined yet | Reserved |
| CONFIG | `0x07` | Pi → Teensy | not defined yet | Reserved |
| CLEAR_FAULT | `0x08` | Pi → Teensy | not defined yet | Reserved |
| JOINT_STATE | `0x11` | Teensy → Pi | not defined yet | Reserved |
| IMU | `0x12` | Teensy → Pi | not defined yet | Reserved |
| ODOM | `0x13` | Teensy → Pi | not defined yet | Reserved |
| CONTACT | `0x14` | Teensy → Pi | not defined yet | Reserved |
| BATTERY | `0x15` | Teensy → Pi | not defined yet | Reserved |
| STATUS | `0x16` | Teensy → Pi | `uint8` mode, `uint8` gait phase, `uint16` fault bits | Implemented (mode and gait phase are always 0 for now) |
| FAULT | `0x17` | Teensy → Pi | not defined yet | Reserved |

Fault bits: bit 0 `HEARTBEAT_LOST`. The firmware sets it at boot and whenever no valid
HEARTBEAT has arrived for 500 ms, and clears it when heartbeats resume. The on-board LED blinks
at 1 Hz while the link is OK and at 5 Hz while the heartbeat is lost.

## Repository layout

```
.
├── .github/workflows/   CI: protocol host test and firmware build
├── cad/                 Mechanical design (SpotMicro-derived); attribution and license notes
├── common/
│   ├── include/         Code shared by the firmware and the Pi side (protocol.h)
│   └── test/            Host-side protocol test (g++ vs Python)
├── docs/
│   ├── decisions/       Architecture decision records
│   └── setup/           How the Raspberry Pi was set up
├── firmware/            PlatformIO project for the Teensy 4.1
├── ros2_ws/src/         colcon workspace (empty; packages planned)
└── tools/               Pi-side utilities (test_link.py)
```

## Prerequisites

On the Raspberry Pi (details in [docs/setup/raspberry-pi.md](docs/setup/raspberry-pi.md)):

- Ubuntu Server 24.04 and ROS 2 Jazzy (ROS 2 is not needed by anything in the repository yet)
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
Only one program can use the serial port at a time, so close any serial monitor first.

## Project status

**Works today**

- Protocol (`common/include/protocol.h`): frame encoding, CRC, a resynchronizing parser and the
  HEARTBEAT and STATUS payloads. The host test checks the C++ and Python implementations against
  each other and runs in CI.
- Firmware: HEARTBEAT watchdog (500 ms), STATUS at 10 Hz, status LED. Built by CI on every push
  and pull request.
- `tools/test_link.py`: heartbeat sender and STATUS monitor.

**Not yet verified on hardware**

- The firmware and link test in this repository have not yet been run against a Teensy.

**Next**

- Run the firmware and link test on the robot.
- Define payloads for the reserved message types.
- ROS 2 packages: `hg_msgs`, `hg_teensy_bridge`, `hg_description`, `hg_bringup`,
  `hg_localization`, `hg_navigation`, `hg_docking`, `hg_perception`, `hg_events`, `hg_audio`,
  `hg_behavior`, `hg_sim`.
- Move the link to the hardware UART once a taller stacking header is fitted (ADR 0001).
- Add the CAD files with upstream attribution and license ([cad/README.md](cad/README.md)).

## License

The code is licensed under the [Apache License 2.0](LICENSE). The CAD files in `cad/` are derived
from an open-source SpotMicro model and follow the upstream model's license, which is still to be
confirmed; see [cad/README.md](cad/README.md).
