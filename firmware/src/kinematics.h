// SPDX-License-Identifier: Apache-2.0
//
// Leg kinematics (planar, two pitch joints per leg) and reference poses.
//
// Plain C++, no Arduino dependencies: host-tested by firmware/test_host/test_kinematics.cpp.
// Conventions (docs/kinematics.md), identical on all four legs:
//   - Body frame: x forward, y left, z up. Hip pitch axes at x = +/-93 mm (front/rear),
//     y = +/-95 mm (left/right), z = 0.
//   - Leg plane: origin at the hip axis, x forward, z up (the body's x-z plane).
//   - Hip angle: 0 = femur straight down, positive = femur forward. Limits -45..+100 deg.
//   - Knee angle: interior angle between femur and tibia, 180 = straight. Limits 30..150 deg.
//     The knee points forward (the femur is forward of the hip-to-foot line).
//   - Femur L1 = 108 mm, tibia L2 = 130 mm to the foot contact point.

#ifndef HG_FIRMWARE_KINEMATICS_H_
#define HG_FIRMWARE_KINEMATICS_H_

#include <cstdint>

#include "protocol.h"

namespace hg {

constexpr float kFemurMm = 108.0f;
constexpr float kTibiaMm = 130.0f;
constexpr float kReachMarginMm = 5.0f;
constexpr float kMinReachMm =
    (kTibiaMm > kFemurMm ? kTibiaMm - kFemurMm : kFemurMm - kTibiaMm) + kReachMarginMm;  // 27 mm
constexpr float kMaxReachMm = kFemurMm + kTibiaMm - kReachMarginMm;                      // 233 mm

constexpr float kHipMinDeg = -45.0f;
constexpr float kHipMaxDeg = 100.0f;
constexpr float kKneeMinDeg = 30.0f;
constexpr float kKneeMaxDeg = 150.0f;

constexpr float kHipXMm = 93.0f;  // Hip axes at x = +93 (front) / -93 (rear).
constexpr float kHipYMm = 95.0f;  // Hip axes at y = +95 (left) / -95 (right).

// Legs in JOINT_STATE order.
enum class Leg : uint8_t { kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3 };
constexpr int kNumLegs = 4;

inline bool is_front(Leg leg) { return leg == Leg::kFrontLeft || leg == Leg::kFrontRight; }

// Foot position in the leg plane, relative to the hip axis (x forward, z up), in mm.
struct FootPosition {
  float x_mm = 0.0f;
  float z_mm = 0.0f;
};

struct LegAngles {
  float hip_deg = 0.0f;
  float knee_deg = 180.0f;
};

// leg_ik() status bits. 0 means the target was reached exactly.
constexpr uint8_t kIkReachClamped = 1u << 0;  // Target distance clamped to [kMin, kMax]ReachMm.
constexpr uint8_t kIkHipLimited = 1u << 1;    // Hip angle clamped to its limits.
constexpr uint8_t kIkKneeLimited = 1u << 2;   // Knee angle clamped to its limits.

struct IkResult {
  LegAngles angles;
  uint8_t status = 0;
};

// Inverse kinematics, knee-forward solution. The reach is clamped first (keeping the direction
// from the hip), then both joints are clamped to their limits; the result is always within the
// joint limits and finite.
IkResult leg_ik(const FootPosition& target);

// Forward kinematics: foot position for the given joint angles (no clamping).
FootPosition leg_fk(const LegAngles& angles);

LegAngles clamp_to_joint_limits(const LegAngles& angles);

// Foot on a flat floor directly below the hip axis, with the hip axis `hip_height_mm` above the
// floor and the body pitched by `body_pitch_deg` (positive = front raised), expressed in the
// (pitched) leg plane.
FootPosition foot_below_hip(float hip_height_mm, float body_pitch_deg);

// Joint angles for all legs, indexed by Leg.
struct RobotPose {
  LegAngles legs[kNumLegs];
};

// Reference poses (docs/kinematics.md). Feet directly below the hip axes in all of them.
constexpr float kStandHipHeightMm = 190.0f;
constexpr float kLieHipHeightMm = 75.0f;
constexpr float kSitPitchDeg = 25.0f;           // Front raised.
constexpr float kSitFrontHipHeightMm = 210.0f;  // Front legs extended.
// Rear hip axes are 2 * kHipXMm * sin(25 deg) = 78.6 mm lower: 131.4 mm (rear legs folded).

// Writes the reference pose for `mode` and returns true, or returns false for TORQUE_OFF,
// which holds the last commanded angles. WALK uses the STAND pose until gaits exist.
// `status` (optional) receives the OR of the leg_ik() status bits of all legs.
bool reference_pose(Mode mode, RobotPose* pose, uint8_t* status = nullptr);

}  // namespace hg

#endif  // HG_FIRMWARE_KINEMATICS_H_
