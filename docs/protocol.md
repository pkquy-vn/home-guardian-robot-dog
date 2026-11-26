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
| JOINT_STATE | `0x11` | Teensy → Pi | 8 x (`int16` position in 0.01 deg, `int16` current, `int8` temperature), FL hip, FL knee, FR hip, FR knee, RL hip, RL knee, RR hip, RR knee | Implemented (commanded angles; current and temperature 0 until servos exist) |
| IMU | `0x12` | Teensy → Pi | not defined yet | Reserved |
| ODOM | `0x13` | Teensy → Pi | not defined yet | Reserved |
| CONTACT | `0x14` | Teensy → Pi | not defined yet | Reserved |
| BATTERY | `0x15` | Teensy → Pi | not defined yet | Reserved |
| STATUS | `0x16` | Teensy → Pi | `uint8` mode, `uint8` gait phase, `uint16` fault bits | Implemented (gait phase is always 0 for now) |
| FAULT | `0x17` | Teensy → Pi | not defined yet | Reserved |
| SERVO_STATE | `0x18` | Teensy → Pi | `uint8` first servo ID (1 or 5), then 4 x (`uint8` flags, `int16` position, `int16` speed, `int16` load, `int16` current, `uint8` voltage, `uint8` temperature, `uint8` status) | Implemented |

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

## Posture modes

Modes: 0 `LIE`, 1 `SIT`, 2 `STAND`, 3 `WALK`, 9 `TORQUE_OFF`. The firmware boots in LIE and runs
the posture state machine in [`firmware/src/posture.h`](../firmware/src/posture.h).

- **Transitions:** LIE to STAND takes 1.6 s, every other transition 1.0 s. All eight joint
  angles move smoothly to the target pose; see [kinematics.md](kinematics.md). STATUS reports the
  old mode until a transition completes.
- **Allowed requests:** LIE, SIT and STAND from each other. WALK only from STAND, and from WALK
  only STAND. From TORQUE_OFF only LIE.
- **TORQUE_OFF** is accepted from anywhere at any time, takes effect immediately and holds the
  last angles.
- **Rejected:** any other request that is illegal, arrives during a transition, or arrives while
  a fault bit is set. Rejections are counted on the Teensy.

The commanded angles are only reported in JOINT_STATE: they are not sent to the servos yet.

## Faults and the heartbeat watchdog

Fault bit 0 is `HEARTBEAT_LOST`. See [ADR 0002](decisions/0002-heartbeat-loss-latches.md).

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
