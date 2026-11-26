# Servo bus (Feetech STS3250)

The leg joints use Feetech STS3250 bus servos. The Teensy talks to them on one half-duplex
serial line. Implemented in `firmware/src/`: `servo_registers.h` (register addresses and unit
conversions), `servo_packet.h` (packets), `servo_bus.h` (transactions) and `servo_manager.h`
(presence, telemetry and the writes). When a servo gets torque and where it goes is decided in
`joint_control.h`; see [servo-motion.md](servo-motion.md).

> **Writes are limited to two things:** torque enable (0 or 1) and the goal registers (40 to
> 47). `ServoBus` refuses every other write, so the firmware cannot write a servo's EEPROM, its
> EEPROM lock, or the value 128 to torque enable (which would store a new zero offset in the
> servo). All writes are refused until `set_writes_enabled(true)`, which `main.cpp` calls once.

## Wiring and IDs

- Teensy `Serial1` at 1,000,000 baud, `SERIAL_HALF_DUPLEX`: data on pin 1 only, with a 1 kOhm
  pull-up to 3.3 V.
- Servo IDs follow the JOINT_STATE order plus one: 1 FL hip, 2 FL knee, 3 FR hip, 4 FR knee,
  5 RL hip, 6 RL knee, 7 RR hip, 8 RR knee. IDs are set with a separate bench sketch; the
  firmware has no ID-change or servo EEPROM-write code.

## Packets

```
Request:  FF FF ID LEN INST PARAM...   CHECKSUM     LEN = number of params + 2
Reply:    FF FF ID LEN STATUS DATA...  CHECKSUM     LEN = number of data bytes + 2
CHECKSUM = ~(ID + LEN + INST or STATUS + params or data), low 8 bits
```

Instructions: PING `0x01`, READ `0x02`, WRITE `0x03`, SYNC WRITE `0x83` (broadcast ID `0xFE`, no
reply). Multi-byte register values are little-endian. Reference: Feetech's FTServo_Arduino
library (`INST.h`, `SCS.cpp`, `SMS_STS.h`).

**Echo:** on a one-wire bus the Teensy can receive its own request before the reply. The reply
parser skips the first received packet that is byte-identical to the request. It skips only the
first one, because a PING reply with status `0x01` is byte-identical to the PING request.

## Registers

| Register | Address | Size | Unit |
| --- | --- | --- | --- |
| Operating mode (EEPROM, read only) | 33 | 1 | 0 = position servo. A servo in another mode is never driven |
| Torque enable (written) | 40 | 1 | 0 = off, 1 = on |
| Acceleration (written) | 41 | 1 | written as 0 |
| Goal position (written) | 42 | 2 | counts; always within 0 to 4095 |
| Goal time (written) | 44 | 2 | written as 0 |
| Goal speed (written) | 46 | 2 | counts per second; 0 would mean maximum, so at least 1 |
| Present position | 56 | 2 | counts, 4096 per turn; bit 15 = sign |
| Present speed | 58 | 2 | counts per second; bit 15 = sign |
| Present load | 60 | 2 | 0.1 %; bit 10 = direction |
| Present voltage | 62 | 1 | 0.1 V |
| Present temperature | 63 | 1 | deg C |
| Present current | 69 | 2 | counts, about 6.5 mA each (to be checked against a meter); bit 15 = sign |

The firmware reads registers 40 to 70 in one transaction, so the servo's torque state and goal
come back with the feedback, and reads the operating mode once when a servo appears. Goals are
written with one SYNC WRITE of registers 41 to 47 to all driven servos; torque with a SYNC WRITE
of register 40. The servo's status byte comes with every reply (0 = no fault). The Feetech library does not name its bits; the names used here
(bit 0 voltage, 1 sensor, 2 temperature, 3 current, 4 angle, 5 overload) are from Feetech's STS
memory table and still need to be confirmed for the STS3250.

## Timing

The bus must never stall the 200 Hz control loop:

- No bus I/O happens in the timer interrupt. Everything runs from `loop()`.
- Per control tick at most these transactions are started, one after the other: torque off,
  goals, torque on (one SYNC WRITE each, only when needed), then one read or ping. With eight
  servos that is under 3 ms of bus time in the worst case.
- Nothing waits: a request is only written when the port can take it at once, and only bytes
  that have already arrived are read. A write that cannot be sent is skipped for that tick.
- A goal SYNC WRITE to eight servos is 72 bytes, more than the port's 64-byte transmit buffer,
  so `main.cpp` adds buffer memory to `Serial1` (`addMemoryForWrite`, `addMemoryForRead`).

Schedule of the read slot:

- **Boot:** IDs 1 to 8 are pinged once each, on the first 8 ticks.
- **Polling:** each tick reads the next tracked servo, round-robin. Eight servos are each read
  25 times per second; fewer servos are read more often.
- **Missing IDs:** every 100 ms one tick pings one untracked ID instead. A missing ID is normal.
- **Dropout:** a servo whose read fails 3 times in a row is dropped until a ping finds it again.
  A servo is reported as present after its first successful read.

## Telemetry to the Pi

The firmware sends SERVO_STATE (see [protocol.md](protocol.md)) for servo IDs 1-4 and 5-8
alternately, each group at 10 Hz, and JOINT_STATUS at 10 Hz. The bridge publishes one
`DiagnosticStatus` per servo on `/diagnostics` and logs when a servo appears or goes missing.
