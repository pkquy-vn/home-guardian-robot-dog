# ADR 0002: HEARTBEAT_LOST latches; heartbeat loss forces SIT, then TORQUE_OFF

- **Status:** Accepted
- **Date:** 2026-11-10

## Context

The Pi sends HEARTBEAT at 20 Hz. Until now the Teensy set `HEARTBEAT_LOST` after 500 ms without
a heartbeat and cleared it as soon as heartbeats resumed. With posture modes, a link dropout
needs a defined physical response, and a fault that clears itself can hide a dropout that
nobody saw.

## Decision

Implemented in `firmware/src/posture.h` and host-tested in `firmware/test_host/`.

- **Latch.** `HEARTBEAT_LOST` is set at boot and clears by itself on the first heartbeat. Once a
  link has been established, 500 ms without a heartbeat latches it. It stays set after
  heartbeats resume, until CLEAR_FAULT arrives while heartbeats are healthy. A Teensy reset also
  clears it.
- **Loss response.** From SIT, STAND or WALK the robot moves to SIT (1.0 s), then TORQUE_OFF
  5 s after the loss. From LIE it stays in LIE, and TORQUE_OFF stays TORQUE_OFF. During a
  transition the target decides: heading to LIE or SIT finishes the move; heading to STAND or
  WALK aborts into SIT.
- **Cancelling.** Only an accepted CLEAR_FAULT cancels the pending TORQUE_OFF. Heartbeats
  resuming do not, and a second loss does not postpone it.
- **While latched**, every mode request is rejected except TORQUE_OFF, which is accepted from
  anywhere at any time.

## Consequences

- Every dropout is visible and needs a deliberate CLEAR_FAULT (`/teensy/clear_fault`) before the
  robot accepts mode commands again.
- A dropout longer than 500 ms always ends with the robot sitting and, unless cleared within
  5 s, with torque off. Short glitches on a standing robot therefore cost a sit-down.
- `tools/test_link.py` cannot send CLEAR_FAULT. After a test run that ends the heartbeat, the
  fault stays latched until it is cleared through the ROS bridge or the Teensy is reset.
- The timings are stand-ins until servos and joint interpolation exist; revisit them then.
