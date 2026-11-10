// SPDX-License-Identifier: Apache-2.0
//
// Commanded joint angles for the posture modes. Behaviour: joint_motion.h.

#include "joint_motion.h"

namespace hg {

namespace {

// Minimum-jerk profile: 0 -> 1 with zero velocity and acceleration at both ends.
float smooth_progress(float p) {
  if (p <= 0.0f) return 0.0f;
  if (p >= 1.0f) return 1.0f;
  return p * p * p * (10.0f + p * (-15.0f + 6.0f * p));
}

float lerp(float a, float b, float s) { return a + (b - a) * s; }

}  // namespace

void JointMotion::init(Mode boot_mode) {
  reference_pose(Mode::kLie, &lie_);
  reference_pose(Mode::kSit, &sit_);
  reference_pose(Mode::kStand, &stand_);
  if (!pose_for(boot_mode, &commanded_)) commanded_ = lie_;
  tracking_transition_ = false;
}

bool JointMotion::pose_for(Mode mode, RobotPose* pose) const {
  switch (mode) {
    case Mode::kLie:
      *pose = lie_;
      return true;
    case Mode::kSit:
      *pose = sit_;
      return true;
    case Mode::kStand:
    case Mode::kWalk:  // STAND pose until gaits exist.
      *pose = stand_;
      return true;
    case Mode::kTorqueOff:
    default:
      return false;  // Hold the last commanded angles.
  }
}

void JointMotion::update(const PostureStateMachine& posture, uint32_t now_ms) {
  if (!posture.in_transition()) {
    tracking_transition_ = false;
    RobotPose pose;
    if (pose_for(posture.reported_mode(), &pose)) commanded_ = pose;
    return;
  }

  if (!tracking_transition_ || posture.transition_count() != tracked_transition_) {
    // A new transition (or an abort into another one): start from the current angles.
    tracking_transition_ = true;
    tracked_transition_ = posture.transition_count();
    start_ = commanded_;
    if (!pose_for(posture.transition_target(), &target_)) target_ = commanded_;
  }

  const uint32_t length = posture.transition_length_ms();
  const uint32_t elapsed = now_ms - posture.transition_start_ms();
  const float p = length == 0 ? 1.0f : static_cast<float>(elapsed) / static_cast<float>(length);
  const float s = smooth_progress(p);
  for (int i = 0; i < kNumLegs; ++i) {
    // Start and target are both within the joint limits and s is in [0, 1], so every
    // interpolated angle is too; clamp anyway before it can ever reach a servo.
    const LegAngles a{lerp(start_.legs[i].hip_deg, target_.legs[i].hip_deg, s),
                      lerp(start_.legs[i].knee_deg, target_.legs[i].knee_deg, s)};
    commanded_.legs[i] = clamp_to_joint_limits(a);
  }
}

}  // namespace hg
