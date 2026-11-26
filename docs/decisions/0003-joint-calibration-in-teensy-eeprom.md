# ADR 0003: Joint calibration lives in the Teensy's EEPROM; the servos' EEPROM is never written

- **Status:** Accepted
- **Date:** 2026-11-26

## Context

To drive a joint, the firmware needs the servo position at a known joint angle and the
direction of rotation. The Feetech STS3250 can store a position offset in its own EEPROM (and
its ID, baud rate and angle limits), so the calibration could live in the servo, on the Teensy,
or on the Pi.

## Decision

- Per joint, the Teensy stores the servo position at the reference pose (hip 0 deg, knee
  interior angle 90 deg) and a direction sign, in its own EEPROM, as one record with a version
  byte and a CRC-16. A record that fails a check is ignored and every joint is uncalibrated.
- **The Teensy is the source of truth.** The Pi keeps a copy in a YAML file outside the
  repository and can send it back.
- **The firmware never writes a servo's EEPROM.** `ServoBus` only allows writes to torque enable
  (0 or 1) and the goal registers (40 to 47). Servo IDs are set with a separate bench sketch.
- **A joint that is not calibrated never gets torque.**

## Consequences

- A replaced servo needs its ID set on the bench and the joint calibrated again; nothing else.
- A replaced or wiped Teensy is uncalibrated until `servo restore` sends the Pi's copy.
- The calibration is lost if both the Teensy's EEPROM and the Pi's file are lost.
- The servos' own offset and angle-limit registers stay at whatever they were set to: the joint
  limits are enforced by the firmware only.
- Changing the record layout needs a new version number; older records are then ignored.
