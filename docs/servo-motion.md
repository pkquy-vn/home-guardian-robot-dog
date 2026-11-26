# Servo motion, calibration and safety

How the firmware drives the leg servos. Implemented in `firmware/src/`: `joint_control.h`
(torque, motion and faults), `calibration.h` (angle mapping and storage) and `servo_manager.h`
(bus I/O). The Pi-side tool is `servo`, in `hg_teensy_bridge`.

> **Not host-tested.** Only the message packing is covered by a test. The motion and safety
> logic has been reviewed and compiles, and is verified on the hardware by the owner.

## Calibration

Each joint has a **reference position** (the servo position, in counts, at the reference pose)
and a **direction sign**:

```
joint angle = reference angle + direction x (position - reference) x 360 / 4096
```

- **Reference pose:** hip 0 deg (femur vertical), knee interior angle 90 deg.
- **Direction:** +1 means the servo's counts increase in the joint's positive direction (hip:
  femur forward; knee: opening). The default is +1 for every joint; it is flipped from the Pi.
- **Storage:** the Teensy's EEPROM, as one record with a version and a CRC. A record that fails
  any check is ignored as a whole, and every joint is then uncalibrated. The servo's own EEPROM
  is never written; see [ADR 0003](decisions/0003-joint-calibration-in-teensy-eeprom.md).
- **The Pi's copy:** the bridge mirrors the Teensy's calibration into
  `~/.config/home_guardian/servo_calibration.yaml` (parameter `calibration_file`), outside the
  repository. A joint the Teensy reports as uncalibrated keeps its saved entry, so a wiped
  Teensy never erases the copy. `servo restore` sends the copy back.
- Calibration only changes while every servo is limp. Saving it pauses the loop for a few
  milliseconds (flash write).

A joint is driven only if all of this holds; `servo show` prints the reason otherwise:

| Check | Why |
| --- | --- |
| The servo answers | |
| The servo is in position mode (its register 33 is 0) | In the other modes the goal registers mean something else |
| The joint is calibrated | |
| Both joint limits map inside counts 20 to 4075 | A goal is never wrapped past 0/4095: flip the direction or remount the horn |
| The measured angle is within the joint limits plus 35 deg | Otherwise the calibration does not match the joint |

## Torque

- At boot every joint is limp, and torque off is sent to all eight IDs.
- A joint gets torque only from an explicit command: SIT, STAND or WALK (every joint that can be
  driven) or a joint test target (that joint). Sending the mode that is already held again
  gives torque to joints that have become available since.
- **Engaging:** the goal is set to the measured position; torque is enabled only after the servo
  has reported that goal back, within 16 counts of its position. The commanded angle then
  starts at the measured angle.
- **LIE** never gives torque. Driven joints move to the lie pose, hold for 0.3 s, then go limp.
- **SIT, STAND, WALK** hold torque. WALK is the STAND pose for now.
- **TORQUE_OFF** removes torque at once. In this mode nothing gets torque: a joint test is
  refused until LIE has been requested (LIE itself gives no torque).
- A limp servo that reports torque on is told to turn it off, and torque off is sent to all IDs
  every 200 ms while no joint is driven.

## Motion

- The target of a driven joint is its posture angle ([kinematics.md](kinematics.md)) or its test
  target, clamped to the joint limits (hip -45 to +100 deg, knee 30 to 150 deg).
- Each 5 ms tick the commanded angle moves toward the target by at most the **speed limit**:
  60 deg/s by default, settable from 5 to 180 deg/s, not stored across a reset. At 60 deg/s the
  LIE to STAND transition is slower than its nominal 1.6 s.
- Goals go out as one sync write per tick. The servo's own speed register is set to 1.5 times
  the limit as a second cap; its acceleration register is 0.
- A test target stays active until the next mode command, a fault, or a heartbeat loss.

## Faults

Checked on the Teensy, independent of the Pi. Each latches (STATUS fault bits) until CLEAR_FAULT
and is reported in a FAULT message with the joint and the value. While any fault is latched,
mode requests and test targets are refused; TORQUE_OFF is always accepted.

| Fault | Trigger | Response |
| --- | --- | --- |
| `SERVO_HOT` | any servo at 65 deg C or more | move to LIE, then limp |
| `SERVO_OVERHEAT` | any servo at 70 deg C or more | torque off on all servos, mode TORQUE_OFF |
| `SERVO_OVERCURRENT` | any servo above 3.5 A for 500 ms | same |
| `SERVO_MISSING` | a driven servo stops answering (3 failed reads, or no reading for 300 ms) | same |
| `SERVO_TORQUE_LOST` | a servo does not take torque within 1 s, or a driven servo reports torque off | same |
| `HEARTBEAT_LOST` | see [protocol.md](protocol.md) | SIT, torque off 5 s after the loss |

After a torque-off fault the mode is TORQUE_OFF: clear the fault, request LIE, then SIT or STAND.
The current threshold uses 6.5 mA per count, which has not been checked against a meter.

## The `servo` tool

Run it while the bridge is running. Joints: `fl_hip`, `fl_knee`, `fr_hip`, `fr_knee`, `rl_hip`,
`rl_knee`, `rr_hip`, `rr_knee`.

```bash
ros2 run hg_teensy_bridge servo show                  # calibration and state of every joint
ros2 run hg_teensy_bridge servo calibrate fl_hip fl_knee
ros2 run hg_teensy_bridge servo flip fl_knee          # invert the direction sign
ros2 run hg_teensy_bridge servo angle fl_knee 100     # drive one joint, in degrees
ros2 run hg_teensy_bridge servo speed 30              # speed limit, in deg/s
ros2 run hg_teensy_bridge servo clear-fault
ros2 run hg_teensy_bridge servo restore               # send the Pi's copy to the Teensy
ros2 run hg_teensy_bridge servo off                   # torque off, now
```

Every command prints the Teensy's answer. Postures use the mode topic, `/teensy/cmd_mode`.

## Calibrating a joint

1. Torque off (`servo off`). Hold the leg at the reference pose: femur vertical, knee at 90 deg.
2. `servo calibrate <joints>`, then `servo show`: the joints read about 0 deg (hip) and 90 deg
   (knee), and "can drive" is yes.
3. Check the direction with a small move at low speed, for example `servo speed 20`, then
   `servo angle fl_knee 100` (the knee must open) or `servo angle fl_hip 10` (the femur must
   move forward).
4. If a joint moves the wrong way: `servo off`, `servo flip <joint>`, request LIE (mode 0, to
   leave TORQUE_OFF; the leg stays limp), and repeat step 3. Do this before any posture: with a
   wrong sign, a posture mirrors the joint about its reference pose and can run it into a stop.

The direction can also be checked without torque: with the joint limp, move it by hand in its
positive direction and watch `measured` in `servo show` increase.
