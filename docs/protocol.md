# Pi–Teensy protocol, posture modes and faults

## Link

The Raspberry Pi and the Teensy 4.1 are connected over **USB serial** (the Teensy appears as
`/dev/ttyACM0`). The hardware UART (Teensy `Serial7` on pins 28/29 to Pi GPIO 14/15) is
deferred; see [ADR 0001](decisions/0001-usb-serial-link.md).

## Frame format

Every message is one frame. All multi-byte fields are little-endian.

| Byte(s) | Field | Notes |
| --- | --- | --- |
| 0 | start | `0xA5` |
| 1 | type | message type, table below |
| 2 | length | payload length, 0 to 64 |
| 3 … 3+length-1 | payload | |
| last 2 | CRC | CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) over type, length and payload |

The C++ implementation is [`common/include/protocol.h`](../common/include/protocol.h)
(header-only, no Arduino dependencies). The only Python implementation is
[`hg_teensy_bridge/protocol.py`](../ros2_ws/src/hg_teensy_bridge/hg_teensy_bridge/protocol.py),
used by the ROS 2 bridge and `tools/test_link.py`. `common/test/test_protocol.py` checks the
two against each other. The parser rejects and counts frames with a bad CRC or length and
resynchronizes on the next start byte.

## Messages

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
| CALIBRATION | `0x09` | Pi → Teensy | `uint8` operation, `uint8` joint, `int8` direction, `int16` reference | Implemented |
| JOINT_TARGET | `0x0A` | Pi → Teensy | `uint8` joint, `int16` angle in 0.01 deg | Implemented |
| SPEED_LIMIT | `0x0B` | Pi → Teensy | `uint16` maximum joint speed in 0.1 deg/s | Implemented |
| JOINT_STATE | `0x11` | Teensy → Pi | 8 x (`int16` position in 0.01 deg, `int16` current in mA, `int8` temperature in deg C), FL hip, FL knee, FR hip, FR knee, RL hip, RL knee, RR hip, RR knee | Implemented |
| IMU | `0x12` | Teensy → Pi | not defined yet | Reserved |
| ODOM | `0x13` | Teensy → Pi | not defined yet | Reserved |
| CONTACT | `0x14` | Teensy → Pi | not defined yet | Reserved |
| BATTERY | `0x15` | Teensy → Pi | not defined yet | Reserved |
| STATUS | `0x16` | Teensy → Pi | `uint8` mode, `uint8` gait phase, `uint16` fault bits | Implemented (gait phase is always 0 for now) |
| FAULT | `0x17` | Teensy → Pi | `uint16` fault bit, `uint8` joint, `int16` value | Implemented |
| SERVO_STATE | `0x18` | Teensy → Pi | `uint8` first servo ID (1 or 5), then 4 x (`uint8` flags, `int16` position, `int16` speed, `int16` load, `int16` current, `uint8` voltage, `uint8` temperature, `uint8` status) | Implemented |
| JOINT_STATUS | `0x19` | Teensy → Pi | `uint16` speed limit in 0.1 deg/s, then 8 x (`uint8` flags, `uint8` block, `int16` reference, `int16` target in 0.01 deg) | Implemented |
| ACK | `0x1A` | Teensy → Pi | `uint8` command, `uint8` result, `uint8` joint, `int16` value | Implemented |

Joints are numbered 0 to 7 in the JOINT_STATE order; 255 means "no joint".

## SERVO_STATE

Telemetry of the bus servos, IDs 1 to 8 (see [servo-bus.md](servo-bus.md)). Eight servos do not
fit in one 64-byte payload, so each message carries a group of four: IDs 1-4 or 5-8. The two
groups alternate, each at 10 Hz. Payload: 49 bytes.

| Field | Type | Unit |
| --- | --- | --- |
| flags | `uint8` | bit 0 = present |
| position | `int16` | counts, 4096 per turn. Raw: no calibration |
| speed | `int16` | counts per second |
| load | `int16` | 0.1 %, sign = direction |
| current | `int16` | counts, about 6.5 mA each (to be checked against a meter) |
| voltage | `uint8` | 0.1 V |
| temperature | `uint8` | deg C |
| status | `uint8` | the servo's status byte, 0 = no fault |

The servo's sign-magnitude values are decoded to two's complement by the firmware. All fields are
0 for a servo that is not present.

## Servo motion messages

Behaviour is described in [servo-motion.md](servo-motion.md).

- **JOINT_STATE** (50 Hz): for a joint whose servo is present and calibrated, the measured
  joint angle, current and temperature. For every other joint, the commanded posture angle,
  with current and temperature 0.
- **CALIBRATION:** operation 1 stores the servo's current position as the joint's reference
  pose; 2 sets the direction sign (+1 or -1); 3 sets direction and reference (servo counts) from
  the Pi's saved copy. Accepted only while every servo is limp.
- **JOINT_TARGET:** drives one joint to an angle until the next MODE command. The angle is
  clamped to the joint limits.
- **SPEED_LIMIT:** the firmware clamps it to 5 to 180 deg/s. Not stored across a reset.
- **JOINT_STATUS** (10 Hz). Flags: bit 0 calibrated, 1 direction is -1, 2 the servo reports
  torque on, 3 driven by the firmware, 4 a test target is active, 5 JOINT_STATE is measured for
  this joint. `block` says why the joint cannot be given torque: 0 none, 1 servo not present,
  2 not calibrated, 3 joint range does not fit the servo range, 4 servo not in position mode,
  5 measured angle far outside the joint limits. `reference` is the calibration in servo counts.
- **FAULT:** the cause of one latched servo fault: the joint, and the temperature in deg C or
  the current in mA. For `SERVO_TORQUE_LOST` the value is 0 if the servo did not take torque
  and 1 if it lost it. Sent when the fault latches and once per second while it stays latched.
- **ACK:** sent once for every MODE, CLEAR_FAULT, CALIBRATION, JOINT_TARGET and SPEED_LIMIT.
  `command` is the message type answered. `value` is the requested mode, the stored reference,
  the clamped target in 0.01 deg, or the speed limit in use in 0.1 deg/s.

| ACK result | Meaning |
| --- | --- |
| 0 | OK |
| 1 | No change: already in the requested state |
| 2 | Bad request: wrong size, or unknown mode, operation, joint or value |
| 3 | Refused: a fault is latched |
| 4 | Refused: a posture transition is in progress |
| 5 | Refused: not allowed in the current mode |
| 6 | Refused: a servo has torque (calibration) |
| 7 | Refused: the servo is not present |
| 8 | Refused: the joint is not calibrated |
| 9 | Refused: no healthy heartbeat (CLEAR_FAULT) |
| 10 | The calibration is in use but could not be saved |
| 11 | The joint range does not fit the servo range. For CALIBRATION: stored, but the joint will not get torque |
| 12 | Refused: the servo is not in position mode |
| 13 | Refused: the measured angle is far outside the joint limits |

## Posture modes

Modes: 0 `LIE`, 1 `SIT`, 2 `STAND`, 3 `WALK`, 9 `TORQUE_OFF`. The firmware boots in LIE and runs
the posture state machine in [`firmware/src/posture.h`](../firmware/src/posture.h).

- **Transitions:** LIE to STAND takes 1.6 s, every other transition 1.0 s. All eight joint
  angles move smoothly to the target pose; see [kinematics.md](kinematics.md). STATUS reports the
  old mode until a transition completes.
- **Allowed requests:** LIE, SIT and STAND from each other. WALK only from STAND, and from WALK
  only STAND. From TORQUE_OFF only LIE.
- **TORQUE_OFF** is accepted from anywhere at any time and takes effect immediately.
- **Rejected:** any other request that is illegal, arrives during a transition, or arrives while
  a fault bit is set. Every request is answered with an ACK.

Joints whose servo is present and calibrated follow the posture: SIT, STAND and WALK give them
torque, LIE moves them to the lie pose and then releases them. See
[servo-motion.md](servo-motion.md).

## Faults and the heartbeat watchdog

| Bit | Fault | Response |
| --- | --- | --- |
| 0 | `HEARTBEAT_LOST` | below |
| 1 | `SERVO_HOT` (65 deg C or more) | move to LIE |
| 2 | `SERVO_OVERHEAT` (70 deg C or more) | torque off |
| 3 | `SERVO_OVERCURRENT` (above 3.5 A for 500 ms) | torque off |
| 4 | `SERVO_MISSING` (a driven servo stopped answering) | torque off |
| 5 | `SERVO_TORQUE_LOST` (a servo did not take or keep torque) | torque off |

Every fault latches until a CLEAR_FAULT arrives while heartbeats are healthy, or the Teensy is
reset. The servo faults are described in [servo-motion.md](servo-motion.md).

`HEARTBEAT_LOST`, see [ADR 0002](decisions/0002-heartbeat-loss-latches.md):

- **Boot:** the bit is set at boot and clears on the first heartbeat.
- **Latch:** after that, 500 ms without a heartbeat latches the bit. It stays set after
  heartbeats resume, until a CLEAR_FAULT arrives while heartbeats are healthy, or the Teensy is
  reset.
- **Loss response:** the robot moves to SIT and turns torque off 5 s later. From LIE it stays in
  LIE. If a transition is in progress, its target decides. Only an accepted CLEAR_FAULT cancels
  the pending torque-off.
- **LED:** the on-board LED blinks at 1 Hz while the link is OK and no fault is set, and at 5 Hz
  otherwise.

## Link test

`tools/test_link.py` exercises the link without ROS. Only one program can use the port, so stop
the ROS bridge and any serial monitor first.

```bash
python3 tools/test_link.py /dev/ttyACM0
```

It sends HEARTBEAT at 20 Hz and prints one line per second: heartbeats sent, STATUS rate (about
10 per second), mode, decoded fault bits and the CRC error count. After the script stops, the LED
switches to 5 Hz within about half a second. The latched fault (`0x0001`) then stays set until a
CLEAR_FAULT is sent through the ROS bridge or the Teensy is reset.
