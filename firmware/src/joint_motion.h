// SPDX-License-Identifier: Apache-2.0
//
// Commanded joint angles for the posture modes.
//
// During a posture transition all eight joint angles move from the angles commanded when the
// transition started (so an aborted transition starts from where the legs are) to the target
// reference pose, over the transition time, with a minimum-jerk profile (zero velocity and
// acceleration at both ends). In a stable mode the reference pose is held; in TORQUE_OFF the
// last commanded angles are held. No servos yet: the angles are only reported in JOINT_STATE.

#ifndef HG_FIRMWARE_JOINT_MOTION_H_
#define HG_FIRMWARE_JOINT_MOTION_H_

#include <cstdint>

#include "kinematics.h"
#include "posture.h"

namespace hg {

class JointMotion {
 public:
  // Computes the reference poses and starts at the pose of `boot_mode` (LIE).
  void init(Mode boot_mode);
  // Call after PostureStateMachine::tick() in the control loop.
  void update(const PostureStateMachine& posture, uint32_t now_ms);
  const RobotPose& commanded() const { return commanded_; }

 private:
  bool pose_for(Mode mode, RobotPose* pose) const;

  RobotPose lie_;
  RobotPose sit_;
  RobotPose stand_;
  RobotPose commanded_;
  RobotPose start_;
  RobotPose target_;
  bool tracking_transition_ = false;
  uint32_t tracked_transition_ = 0;
};

}  // namespace hg

#endif  // HG_FIRMWARE_JOINT_MOTION_H_
