// SPDX-License-Identifier: Apache-2.0
//
// Joint motion control and servo safety (HIGH RISK: this decides when a servo gets torque and
// where it goes). Plain C++, no Arduino dependencies. Described in docs/servo-motion.md.
//
// Torque:
//   - Every joint is limp at boot. A joint gets torque only from an explicit command: a SIT,
//     STAND or WALK request (every joint that can be driven) or a JOINT_TARGET (that joint).
//   - A joint can be driven only if its servo answers, is in position mode and is calibrated,
//     its joint range fits the servo's position range, and its measured angle is plausible
//     (drive_block()). All other joints stay limp.
//   - Engaging: the goal is set to the freshly measured position first. Torque is enabled only
//     after the servo has reported that goal back, within kPresetToleranceCounts of its
//     position, so nothing jumps. The commanded angle then starts at the measured angle.
//   - A limp servo is never sent a goal.
//   - LIE never gives torque; a driven joint moves to the lie pose and is then released. SIT,
//     STAND and WALK hold torque. TORQUE_OFF releases everything at once.
//   - A limp servo that reports torque on is told to turn it off again, and torque off is sent
//     to all eight IDs at boot and regularly while no joint is driven.
//
// Motion: each control tick the commanded angle of a driven joint moves toward its target by at
// most speed limit x tick time. The target is the posture angle (joint_motion.h) or the joint's
// test target, clamped to the joint limits. The servo's own speed register is set to 1.5 x the
// limit as a second, independent cap.
//
// Faults (independent of the Pi; each latches until clear_faults()):
//   - SERVO_HOT: a servo at 65 deg C or more. Test targets are dropped; the posture state
//     machine moves to LIE.
//   - SERVO_OVERHEAT: a servo at 70 deg C or more.
//   - SERVO_OVERCURRENT: a servo above 3.5 A for 500 ms.
//   - SERVO_MISSING: a driven servo stopped answering.
//   - SERVO_TORQUE_LOST: a servo did not take torque within 1 s (value 0), or a driven servo
//     reports torque off (value 1).
//   The last four release every joint at once.

#ifndef HG_FIRMWARE_JOINT_CONTROL_H_
#define HG_FIRMWARE_JOINT_CONTROL_H_

#include <cstddef>
#include <cstdint>

#include "calibration.h"
#include "kinematics.h"
#include "posture.h"
#include "protocol.h"
#include "servo_io.h"

namespace hg {

constexpr float kControlPeriodS = 0.005f;  // update() is called once per 200 Hz control tick.

constexpr float kDefaultSpeedLimitDegS = 60.0f;
constexpr float kMinSpeedLimitDegS = 5.0f;
constexpr float kMaxSpeedLimitDegS = 180.0f;
constexpr float kServoSpeedCapFactor = 1.5f;

constexpr uint8_t kHotTemperatureC = 65;
constexpr uint8_t kOverheatTemperatureC = 70;
constexpr int32_t kOvercurrentMilliamps = 3500;
constexpr uint32_t kOvercurrentTimeMs = 500;
constexpr uint32_t kSampleStaleMs = 300;     // A driven servo not read for this long is missing.
constexpr uint32_t kEngageTimeoutMs = 1000;  // From the command to torque reported on.
constexpr uint8_t kTorqueLostSamples = 2;    // Consecutive readings with torque off.

constexpr int16_t kPresetToleranceCounts = 16;  // About 1.4 deg.
constexpr float kEngageMarginDeg = 35.0f;  // Measured angle may be this far outside the limits.
constexpr uint32_t kLieSettleMs = 300;     // Torque is held this long after reaching the lie pose.

constexpr uint8_t kTorqueOffRepeatTicks = 3;      // A torque-off is sent this many ticks in a row.
constexpr uint8_t kBootTorqueOffTicks = 10;       // Torque off to all IDs for the first ticks.
constexpr uint32_t kIdleTorqueOffPeriodMs = 200;  // And this often while no joint is driven.

class JointController {
 public:
  // `samples` points at kNumServos samples that stay valid (ServoManager::samples()).
  // `calibration` is the stored calibration (all invalid if there is none).
  void begin(const ServoSample* samples, const JointCalibration (&calibration)[kNumJoints]);

  // --- Commands, called when the message arrives -----------------------------------------------

  // Call after a MODE request was accepted or was a no-change. Drops all test targets. SIT,
  // STAND and WALK engage every joint that can be driven; TORQUE_OFF releases everything.
  void on_mode_command(Mode mode, uint32_t now_ms);

  // JOINT_TARGET: drives one joint to `deg`, clamped to the joint limits, until the next mode
  // command. The caller checks the posture state (no fault, not TORQUE_OFF) first.
  AckResult set_test_target(size_t joint, float deg, uint32_t now_ms, float* clamped_deg);

  // SPEED_LIMIT: returns the limit now in use, in 0.1 deg/s.
  uint16_t set_speed_limit(uint16_t speed_ddeg_s);

  // CALIBRATION: changes one joint's calibration. Refused unless every servo is limp. On kOk and
  // kRangeError the calibration changed and the caller must store it; *reference receives the
  // joint's reference position.
  AckResult calibrate(const CalibrationPayload& request, int16_t* reference);

  // CLEAR_FAULT (after the caller's own checks). Returns true if a fault was latched.
  bool clear_faults();

  // --- Control tick ------------------------------------------------------------------------------

  // Call once per control tick, after PostureStateMachine::tick() and JointMotion::update().
  // Checks the servos, advances every joint and builds the plan of servo writes for this tick.
  void update(uint32_t now_ms, const PostureStateMachine& posture, const RobotPose& pose);

  // --- Outputs -----------------------------------------------------------------------------------

  const ServoBusPlan& plan() const { return plan_; }
  uint16_t fault_bits() const { return fault_bits_; }
  // The cause of one latched fault bit. False if that bit is not latched.
  bool fault_cause(uint16_t fault_bit, FaultPayload* cause) const;

  const JointCalibration& calibration(size_t joint) const { return calibration_[joint]; }
  bool driven(size_t joint) const { return phase_[joint] != Phase::kLimp; }
  bool has_test_target(size_t joint) const { return has_test_target_[joint]; }
  float target_deg(size_t joint) const { return target_deg_[joint]; }
  // Why a limp joint cannot be given torque right now; kNone for a driven joint.
  JointBlock block(size_t joint) const;
  bool all_limp() const;
  float speed_limit_deg_s() const { return speed_limit_deg_s_; }

 private:
  enum class Phase : uint8_t {
    kLimp,      // No torque.
    kPreset,    // Writing goal = measured position; waiting for the servo to report it back.
    kEnabling,  // Goal confirmed: writing torque on; waiting for the servo to report it.
    kDriven,    // Torque on; following the target.
  };

  JointBlock drive_block(size_t joint) const;
  bool engage(size_t joint, uint32_t now_ms);
  void release(size_t joint);
  void release_all(bool always_broadcast);
  void clear_test_targets();
  void latch(uint16_t fault_bit, size_t joint, int32_t value);
  void monitor(uint32_t now_ms);
  void advance_joint(size_t joint, uint32_t now_ms, bool abort_engage, bool lie_steady);
  uint16_t servo_speed_cap() const;

  const ServoSample* samples_ = nullptr;
  JointCalibration calibration_[kNumJoints];
  float speed_limit_deg_s_ = kDefaultSpeedLimitDegS;

  Phase phase_[kNumJoints] = {};
  uint32_t engage_start_ms_[kNumJoints] = {};  // When the joint left kLimp.
  uint32_t phase_sequence_[kNumJoints] = {};   // Sample sequence when the phase was entered.
  uint32_t seen_sequence_[kNumJoints] = {};    // Last sample checked for torque while driven.
  uint8_t torque_off_samples_[kNumJoints] = {};
  float commanded_deg_[kNumJoints] = {};
  float target_deg_[kNumJoints] = {};
  bool has_test_target_[kNumJoints] = {};
  float test_target_deg_[kNumJoints] = {};
  bool settling_[kNumJoints] = {};  // At the lie pose, waiting kLieSettleMs before release.
  uint32_t settle_start_ms_[kNumJoints] = {};

  uint8_t off_ticks_[kNumJoints] = {};  // Ticks left to repeat torque off for one joint.
  uint8_t all_off_ticks_ = 0;           // Ticks left to repeat torque off for all IDs.
  uint32_t last_all_off_ms_ = 0;

  bool over_current_[kNumJoints] = {};
  uint32_t over_current_since_ms_[kNumJoints] = {};

  uint16_t fault_bits_ = 0;
  FaultPayload fault_causes_[16];  // Indexed by fault bit number.

  ServoBusPlan plan_;
};

}  // namespace hg

#endif  // HG_FIRMWARE_JOINT_CONTROL_H_
