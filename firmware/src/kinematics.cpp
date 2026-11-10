// SPDX-License-Identifier: Apache-2.0
//
// Leg kinematics and reference poses. Conventions and equations: kinematics.h and
// docs/kinematics.md.

#include "kinematics.h"

#include <cmath>

namespace hg {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegPerRad = 180.0f / kPi;
constexpr float kRadPerDeg = kPi / 180.0f;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// acos with its argument clamped to [-1, 1] against rounding.
float safe_acos(float c) { return std::acos(clampf(c, -1.0f, 1.0f)); }

}  // namespace

LegAngles clamp_to_joint_limits(const LegAngles& angles) {
  LegAngles out;
  out.hip_deg = clampf(angles.hip_deg, kHipMinDeg, kHipMaxDeg);
  out.knee_deg = clampf(angles.knee_deg, kKneeMinDeg, kKneeMaxDeg);
  return out;
}

IkResult leg_ik(const FootPosition& target) {
  IkResult result;
  float x = target.x_mm;
  float z = target.z_mm;
  float r = std::sqrt(x * x + z * z);

  // Reach clamp along the hip-to-target direction (straight down if the target is the hip).
  if (!(r >= kMinReachMm && r <= kMaxReachMm)) {  // Also true for NaN.
    const float clamped = r > kMaxReachMm ? kMaxReachMm : kMinReachMm;
    if (std::isfinite(r) && r > 1e-6f) {
      x *= clamped / r;
      z *= clamped / r;
    } else {
      x = 0.0f;
      z = -clamped;
    }
    r = clamped;
    result.status |= kIkReachClamped;
  }

  // Interior knee angle from the law of cosines.
  const float knee =
      safe_acos((kFemurMm * kFemurMm + kTibiaMm * kTibiaMm - r * r) / (2.0f * kFemurMm * kTibiaMm));
  // Hip: direction of the hip-to-foot line (0 = straight down, positive forward), plus the angle
  // between that line and the femur (femur forward of the line: knee points forward).
  const float line = std::atan2(x, -z);
  const float alpha =
      safe_acos((kFemurMm * kFemurMm + r * r - kTibiaMm * kTibiaMm) / (2.0f * kFemurMm * r));
  float hip_deg = (line + alpha) * kDegPerRad;
  if (hip_deg > 180.0f) hip_deg -= 360.0f;

  const LegAngles raw{hip_deg, knee * kDegPerRad};
  result.angles = clamp_to_joint_limits(raw);
  if (result.angles.hip_deg != raw.hip_deg) result.status |= kIkHipLimited;
  if (result.angles.knee_deg != raw.knee_deg) result.status |= kIkKneeLimited;
  return result;
}

FootPosition leg_fk(const LegAngles& angles) {
  const float hip = angles.hip_deg * kRadPerDeg;
  // Tibia direction, measured like the hip (from straight down, positive forward).
  const float tibia = (angles.hip_deg + angles.knee_deg - 180.0f) * kRadPerDeg;
  FootPosition foot;
  foot.x_mm = kFemurMm * std::sin(hip) + kTibiaMm * std::sin(tibia);
  foot.z_mm = -kFemurMm * std::cos(hip) - kTibiaMm * std::cos(tibia);
  return foot;
}

FootPosition foot_below_hip(float hip_height_mm, float body_pitch_deg) {
  // World "down" expressed in a body frame pitched front-up by body_pitch_deg.
  const float pitch = body_pitch_deg * kRadPerDeg;
  FootPosition foot;
  foot.x_mm = -hip_height_mm * std::sin(pitch);
  foot.z_mm = -hip_height_mm * std::cos(pitch);
  return foot;
}

bool reference_pose(Mode mode, RobotPose* pose, uint8_t* status) {
  float front_height = 0.0f;
  float rear_height = 0.0f;
  float pitch = 0.0f;
  switch (mode) {
    case Mode::kLie:
      front_height = rear_height = kLieHipHeightMm;
      break;
    case Mode::kSit:
      pitch = kSitPitchDeg;
      front_height = kSitFrontHipHeightMm;
      rear_height = kSitFrontHipHeightMm - 2.0f * kHipXMm * std::sin(pitch * kRadPerDeg);
      break;
    case Mode::kStand:
    case Mode::kWalk:
      front_height = rear_height = kStandHipHeightMm;
      break;
    case Mode::kTorqueOff:
    default:
      return false;
  }
  uint8_t all_status = 0;
  for (int i = 0; i < kNumLegs; ++i) {
    const float height = is_front(static_cast<Leg>(i)) ? front_height : rear_height;
    const IkResult ik = leg_ik(foot_below_hip(height, pitch));
    pose->legs[i] = ik.angles;
    all_status |= ik.status;
  }
  if (status != nullptr) *status = all_status;
  return true;
}

}  // namespace hg
