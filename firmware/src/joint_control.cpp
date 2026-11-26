// SPDX-License-Identifier: Apache-2.0
//
// Joint motion control and servo safety. Behaviour: joint_control.h.

#include "joint_control.h"

#include "servo_registers.h"

namespace hg {

namespace {

float posture_angle(const RobotPose& pose, size_t joint) {
  const LegAngles& leg = pose.legs[joint / 2];
  return is_knee_joint(joint) ? leg.knee_deg : leg.hip_deg;
}

// Moves `current` toward `target` by at most `max_step`. A NaN target holds the position.
float step_toward(float current, float target, float max_step) {
  const float delta = target - current;
  if (!(delta == delta)) return current;
  if (delta > max_step) return current + max_step;
  if (delta < -max_step) return current - max_step;
  return target;
}

int16_t clamp_position(int16_t position) {
  if (position < sts::kPositionMin) return sts::kPositionMin;
  if (position > sts::kPositionMax) return sts::kPositionMax;
  return position;
}

int16_t saturate_i16(int32_t value) {
  if (value > INT16_MAX) return INT16_MAX;
  if (value < INT16_MIN) return INT16_MIN;
  return static_cast<int16_t>(value);
}

size_t bit_number(uint16_t bit) {
  size_t n = 0;
  while (n < 15 && !(bit & (1u << n))) ++n;
  return n;
}

AckResult ack_for(JointBlock block) {
  switch (block) {
    case JointBlock::kNone:
      return AckResult::kOk;
    case JointBlock::kNotPresent:
      return AckResult::kNotPresent;
    case JointBlock::kNotCalibrated:
      return AckResult::kNotCalibrated;
    case JointBlock::kRangeError:
      return AckResult::kRangeError;
    case JointBlock::kServoMode:
      return AckResult::kServoMode;
    case JointBlock::kOutOfRange:
      return AckResult::kOutOfRange;
  }
  return AckResult::kNotAllowed;
}

}  // namespace

void JointController::begin(const ServoSample* samples,
                            const JointCalibration (&calibration)[kNumJoints]) {
  samples_ = samples;
  for (size_t i = 0; i < kNumJoints; ++i) {
    calibration_[i] = calibration[i];
    if (!is_valid_direction(calibration_[i].direction)) {
      calibration_[i].direction = default_direction(i);
    }
    phase_[i] = Phase::kLimp;
    has_test_target_[i] = false;
    settling_[i] = false;
    off_ticks_[i] = 0;
    over_current_[i] = false;
    torque_off_samples_[i] = 0;
  }
  speed_limit_deg_s_ = kDefaultSpeedLimitDegS;
  fault_bits_ = 0;
  all_off_ticks_ = kBootTorqueOffTicks;  // Whatever the servos were doing before this boot.
  last_all_off_ms_ = 0;
  plan_ = ServoBusPlan();
}

// ---------------------------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------------------------

void JointController::on_mode_command(Mode mode, uint32_t now_ms) {
  clear_test_targets();
  if (mode == Mode::kTorqueOff) {
    release_all(true);
    return;
  }
  if (mode == Mode::kSit || mode == Mode::kStand || mode == Mode::kWalk) {
    // Joints that cannot be driven stay limp; block() says why.
    for (size_t i = 0; i < kNumJoints; ++i) engage(i, now_ms);
  }
  // LIE gives no torque: joints that are driven move to the lie pose and are then released.
}

AckResult JointController::set_test_target(size_t joint, float deg, uint32_t now_ms,
                                           float* clamped_deg) {
  if (joint >= kNumJoints || !(deg == deg) || clamped_deg == nullptr) {
    return AckResult::kBadRequest;
  }
  if (samples_ == nullptr || fault_bits_ != 0) return AckResult::kFaultLatched;
  const float target = clamp_to_joint_range(joint, deg);
  *clamped_deg = target;
  if (phase_[joint] == Phase::kLimp) {
    const JointBlock blocked = drive_block(joint);
    if (blocked != JointBlock::kNone) return ack_for(blocked);
    if (!engage(joint, now_ms)) return AckResult::kNotAllowed;
  }
  has_test_target_[joint] = true;
  test_target_deg_[joint] = target;
  settling_[joint] = false;
  return AckResult::kOk;
}

uint16_t JointController::set_speed_limit(uint16_t speed_ddeg_s) {
  float deg_s = static_cast<float>(speed_ddeg_s) * 0.1f;
  if (deg_s < kMinSpeedLimitDegS) deg_s = kMinSpeedLimitDegS;
  if (deg_s > kMaxSpeedLimitDegS) deg_s = kMaxSpeedLimitDegS;
  speed_limit_deg_s_ = deg_s;
  return static_cast<uint16_t>(deg_s * 10.0f + 0.5f);
}

AckResult JointController::calibrate(const CalibrationPayload& request, int16_t* reference) {
  if (request.joint >= kNumJoints || reference == nullptr) return AckResult::kBadRequest;
  const size_t joint = request.joint;
  JointCalibration cal = calibration_[joint];
  *reference = cal.reference;
  if (samples_ == nullptr || !all_limp()) return AckResult::kTorqueOn;

  switch (request.operation) {
    case kCalCapture: {
      const ServoSample& s = samples_[joint];
      if (!s.present) return AckResult::kNotPresent;
      if (s.position < sts::kPositionMin || s.position > sts::kPositionMax) {
        return AckResult::kOutOfRange;
      }
      cal.valid = true;
      cal.reference = s.position;
      break;
    }
    case kCalSetDirection:
      if (!is_valid_direction(request.direction)) return AckResult::kBadRequest;
      if (cal.direction == request.direction) return AckResult::kNoChange;
      cal.direction = request.direction;
      break;
    case kCalRestore:
      if (!is_valid_direction(request.direction) || request.reference < sts::kPositionMin ||
          request.reference > sts::kPositionMax) {
        return AckResult::kBadRequest;
      }
      cal.valid = true;
      cal.direction = request.direction;
      cal.reference = request.reference;
      break;
    default:
      return AckResult::kBadRequest;
  }
  calibration_[joint] = cal;
  *reference = cal.reference;
  return (cal.valid && !range_fits(cal, joint)) ? AckResult::kRangeError : AckResult::kOk;
}

bool JointController::clear_faults() {
  const bool had_fault = fault_bits_ != 0;
  fault_bits_ = 0;
  for (size_t i = 0; i < kNumJoints; ++i) over_current_[i] = false;
  return had_fault;
}

// ---------------------------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------------------------

bool JointController::fault_cause(uint16_t fault_bit, FaultPayload* cause) const {
  if (cause == nullptr || fault_bit == 0 || (fault_bits_ & fault_bit) != fault_bit) return false;
  *cause = fault_causes_[bit_number(fault_bit)];
  return true;
}

JointBlock JointController::block(size_t joint) const {
  if (phase_[joint] != Phase::kLimp) return JointBlock::kNone;
  return drive_block(joint);
}

bool JointController::all_limp() const {
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (phase_[i] != Phase::kLimp) return false;
    if (samples_ != nullptr && samples_[i].present && samples_[i].torque_enabled) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------------------------
// Torque
// ---------------------------------------------------------------------------------------------

JointBlock JointController::drive_block(size_t joint) const {
  if (samples_ == nullptr) return JointBlock::kNotPresent;
  const ServoSample& s = samples_[joint];
  const JointCalibration& cal = calibration_[joint];
  if (!s.present) return JointBlock::kNotPresent;
  if (!s.position_mode) return JointBlock::kServoMode;
  if (!cal.valid) return JointBlock::kNotCalibrated;
  if (!range_fits(cal, joint)) return JointBlock::kRangeError;
  // A measured angle far outside the limits means the calibration does not match the joint, or
  // the servo is past its 0/4095 point. Driving from there could go the long way round.
  if (s.position < sts::kPositionMin || s.position > sts::kPositionMax) {
    return JointBlock::kOutOfRange;
  }
  const float deg = position_to_deg(cal, joint, s.position);
  if (!(deg >= joint_min_deg(joint) - kEngageMarginDeg) ||
      !(deg <= joint_max_deg(joint) + kEngageMarginDeg)) {
    return JointBlock::kOutOfRange;
  }
  return JointBlock::kNone;
}

// Starts giving one joint torque. Only called for an explicit command.
bool JointController::engage(size_t joint, uint32_t now_ms) {
  if (phase_[joint] != Phase::kLimp) return true;
  if (samples_ == nullptr || fault_bits_ != 0) return false;
  if (drive_block(joint) != JointBlock::kNone) return false;
  phase_[joint] = Phase::kPreset;
  engage_start_ms_[joint] = now_ms;
  phase_sequence_[joint] = samples_[joint].sequence;
  commanded_deg_[joint] = position_to_deg(calibration_[joint], joint, samples_[joint].position);
  settling_[joint] = false;
  // Torque-off repeats still pending from an earlier release must not undo this.
  off_ticks_[joint] = 0;
  all_off_ticks_ = 0;
  return true;
}

void JointController::release(size_t joint) {
  phase_[joint] = Phase::kLimp;
  has_test_target_[joint] = false;
  settling_[joint] = false;
  off_ticks_[joint] = kTorqueOffRepeatTicks;
  plan_.write_goal[joint] = false;
  plan_.torque_on[joint] = false;
}

void JointController::release_all(bool always_broadcast) {
  bool any_engaged = false;
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (phase_[i] != Phase::kLimp) any_engaged = true;
    phase_[i] = Phase::kLimp;
    has_test_target_[i] = false;
    settling_[i] = false;
    plan_.write_goal[i] = false;
    plan_.torque_on[i] = false;
  }
  if (any_engaged || always_broadcast) all_off_ticks_ = kTorqueOffRepeatTicks;
}

void JointController::clear_test_targets() {
  for (size_t i = 0; i < kNumJoints; ++i) has_test_target_[i] = false;
}

void JointController::latch(uint16_t fault_bit, size_t joint, int32_t value) {
  if (fault_bits_ & fault_bit) return;  // Keep the first cause.
  fault_bits_ = static_cast<uint16_t>(fault_bits_ | fault_bit);
  FaultPayload& cause = fault_causes_[bit_number(fault_bit)];
  cause.fault = fault_bit;
  cause.joint = static_cast<uint8_t>(joint);
  cause.value = saturate_i16(value);
}

uint16_t JointController::servo_speed_cap() const {
  const float counts = speed_limit_deg_s_ * sts::kCountsPerDeg * kServoSpeedCapFactor;
  if (!(counts >= 1.0f)) return 1;  // Never 0: that means "maximum speed" to the servo.
  if (counts >= 32767.0f) return 0x7FFF;
  return static_cast<uint16_t>(counts + 0.5f);
}

// ---------------------------------------------------------------------------------------------
// Control tick
// ---------------------------------------------------------------------------------------------

// Checks every servo that answers, driven or not, and that every driven servo still answers.
void JointController::monitor(uint32_t now_ms) {
  for (size_t i = 0; i < kNumJoints; ++i) {
    const ServoSample& s = samples_[i];
    if (phase_[i] != Phase::kLimp) {
      // Signed, so a sample taken just after `now_ms` was read does not count as old.
      const int32_t age_ms = static_cast<int32_t>(now_ms - s.time_ms);
      if (!s.present || age_ms > static_cast<int32_t>(kSampleStaleMs)) {
        latch(kFaultServoMissing, i, 0);
      }
    }
    if (!s.present) {
      over_current_[i] = false;
      continue;
    }

    if (s.temperature >= kOverheatTemperatureC) {
      latch(kFaultServoOverheat, i, s.temperature);
    } else if (s.temperature >= kHotTemperatureC) {
      latch(kFaultServoHot, i, s.temperature);
    }

    int32_t milliamps = sts::current_to_milliamps(s.current);
    if (milliamps < 0) milliamps = -milliamps;
    if (milliamps > kOvercurrentMilliamps) {
      if (!over_current_[i]) {
        over_current_[i] = true;
        over_current_since_ms_[i] = now_ms;
      } else if (now_ms - over_current_since_ms_[i] >= kOvercurrentTimeMs) {
        latch(kFaultServoOvercurrent, i, milliamps);
      }
    } else {
      over_current_[i] = false;
    }
  }
}

void JointController::advance_joint(size_t joint, uint32_t now_ms, bool abort_engage,
                                    bool lie_steady) {
  const ServoSample& s = samples_[joint];
  const JointCalibration& cal = calibration_[joint];

  switch (phase_[joint]) {
    case Phase::kLimp:
      return;

    case Phase::kPreset:
    case Phase::kEnabling: {
      // No reason left to give this joint torque: stop before it gets any.
      if (abort_engage || (lie_steady && !has_test_target_[joint])) {
        release(joint);
        return;
      }
      if (now_ms - engage_start_ms_[joint] > kEngageTimeoutMs) {
        latch(kFaultServoTorqueLost, joint, 0);  // update() then releases everything.
        return;
      }
      // Nothing is written until a reading taken after the command has arrived: the first goal
      // must be where the joint is now, not where it was a moment ago. (Some Feetech firmware
      // turns torque on by itself when a goal is written; this keeps that harmless too.)
      const bool fresh = s.sequence != phase_sequence_[joint];
      if (phase_[joint] == Phase::kPreset && !fresh) return;

      // Keep the goal on the measured position, so the joint stays where it is when torque
      // comes on. The commanded angle follows, so the motion later starts from there.
      commanded_deg_[joint] = position_to_deg(cal, joint, s.position);
      plan_.write_goal[joint] = true;
      plan_.goal[joint] = clamp_position(s.position);

      if (phase_[joint] == Phase::kPreset) {
        // Torque on is only sent after the servo itself has reported a goal next to its
        // position, in a reading taken after this joint was engaged.
        const bool in_range = s.position >= sts::kPositionMin && s.position <= sts::kPositionMax;
        const int32_t error = static_cast<int32_t>(s.goal) - static_cast<int32_t>(s.position);
        if (in_range && error >= -kPresetToleranceCounts && error <= kPresetToleranceCounts) {
          phase_[joint] = Phase::kEnabling;
          phase_sequence_[joint] = s.sequence;
        }
        return;
      }
      plan_.torque_on[joint] = true;
      if (fresh && s.torque_enabled) {
        phase_[joint] = Phase::kDriven;
        seen_sequence_[joint] = s.sequence;
        torque_off_samples_[joint] = 0;
      }
      return;
    }

    case Phase::kDriven: {
      if (s.sequence != seen_sequence_[joint]) {
        seen_sequence_[joint] = s.sequence;
        if (s.torque_enabled) {
          torque_off_samples_[joint] = 0;
        } else if (++torque_off_samples_[joint] >= kTorqueLostSamples) {
          latch(kFaultServoTorqueLost, joint, 1);  // update() then releases everything.
          return;
        }
      }

      // Rate limit. The commanded angle only ever moves toward the (clamped) target, so it
      // never moves further outside the joint limits than where the joint was engaged.
      const float max_step = speed_limit_deg_s_ * kControlPeriodS;
      commanded_deg_[joint] = step_toward(commanded_deg_[joint], target_deg_[joint], max_step);
      plan_.write_goal[joint] = true;
      plan_.goal[joint] = deg_to_position(cal, joint, commanded_deg_[joint]);

      // LIE: at the lie pose, hold briefly so the servo catches up, then release.
      const bool at_target = commanded_deg_[joint] == target_deg_[joint];
      if (lie_steady && !has_test_target_[joint] && at_target) {
        if (!settling_[joint]) {
          settling_[joint] = true;
          settle_start_ms_[joint] = now_ms;
        } else if (now_ms - settle_start_ms_[joint] >= kLieSettleMs) {
          release(joint);
        }
      } else {
        settling_[joint] = false;
      }
      return;
    }
  }
}

void JointController::update(uint32_t now_ms, const PostureStateMachine& posture,
                             const RobotPose& pose) {
  plan_ = ServoBusPlan();
  plan_.goal_speed = servo_speed_cap();
  if (samples_ == nullptr) return;

  monitor(now_ms);

  const bool link_fault = (posture.fault_bits() & kFaultHeartbeatLost) != 0;
  const bool steady = !posture.in_transition();
  const Mode mode = posture.reported_mode();
  if ((fault_bits_ & kTorqueOffFaults) != 0 || (mode == Mode::kTorqueOff && steady)) {
    release_all(false);
  }
  // A fault or a lost link ends every test (the joints follow the posture again) and stops
  // joints that were still on their way to getting torque.
  const bool abort_engage = fault_bits_ != 0 || link_fault;
  if (abort_engage) clear_test_targets();
  const bool lie_steady = mode == Mode::kLie && steady;

  for (size_t i = 0; i < kNumJoints; ++i) {
    const float wanted = has_test_target_[i] ? test_target_deg_[i] : posture_angle(pose, i);
    target_deg_[i] = clamp_to_joint_range(i, wanted);
    advance_joint(i, now_ms, abort_engage, lie_steady);
  }
  // A fault latched while advancing (no torque in time, torque lost) releases everything too.
  if ((fault_bits_ & kTorqueOffFaults) != 0) release_all(false);

  // Limp servos must have torque off: after a release, and whenever one reports otherwise.
  bool any_engaged = false;
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (phase_[i] != Phase::kLimp) {
      any_engaged = true;
      continue;
    }
    if (off_ticks_[i] > 0) {
      --off_ticks_[i];
      plan_.torque_off[i] = true;
    }
    if (samples_[i].present && samples_[i].torque_enabled) plan_.torque_off[i] = true;
  }
  if (all_off_ticks_ > 0) {
    --all_off_ticks_;
    plan_.torque_off_all = true;
  } else if (!any_engaged && now_ms - last_all_off_ms_ >= kIdleTorqueOffPeriodMs) {
    // Also reaches a servo that holds torque but is not answering reads.
    plan_.torque_off_all = true;
  }
  if (plan_.torque_off_all) last_all_off_ms_ = now_ms;
}

}  // namespace hg
