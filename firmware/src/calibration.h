// SPDX-License-Identifier: Apache-2.0
//
// Joint calibration: the mapping between servo position (counts) and joint angle (deg), and its
// stored form. Plain C++, no Arduino dependencies.
//
// Per joint the calibration is the servo position at the reference pose and a direction sign:
//   angle = reference angle + direction * (position - reference) / counts per degree
// Reference pose: hip 0 deg (femur vertical), knee interior angle 90 deg. Joint conventions and
// limits: kinematics.h.
//
// A joint can only be driven if it is calibrated AND its whole range (the joint limits plus a
// margin) maps inside the servo's position range 0..4095. Otherwise the range crosses the
// servo's wrap point and the joint must never be driven: flip its direction or remount the horn.
//
// Stored form (kCalibrationRecordSize bytes, for the Teensy's EEPROM): magic "HGCL", a version
// byte, 8 x (flags, int16 reference), and a CRC-16/CCITT-FALSE. A record with a wrong magic,
// version, CRC or field is rejected as a whole, so garbage is never used.

#ifndef HG_FIRMWARE_CALIBRATION_H_
#define HG_FIRMWARE_CALIBRATION_H_

#include <cstddef>
#include <cstdint>

#include "kinematics.h"
#include "protocol.h"
#include "servo_registers.h"

namespace hg {

constexpr float kHipReferenceDeg = 0.0f;    // Femur vertical.
constexpr float kKneeReferenceDeg = 90.0f;  // Interior angle 90 deg (set with a square).

// Margin kept between the mapped joint range and the ends of the servo's position range.
constexpr float kPositionMarginCounts = 20.0f;  // About 1.8 deg.

struct JointCalibration {
  bool valid = false;     // A reference has been stored.
  int8_t direction = 1;   // +1 or -1: sign of servo counts per joint degree.
  int16_t reference = 0;  // Servo position in counts at the reference pose.
};

// Joints are in JointIndex order: even = hip, odd = knee.
inline bool is_knee_joint(size_t joint) { return (joint % 2) == 1; }
inline float joint_reference_deg(size_t joint) {
  return is_knee_joint(joint) ? kKneeReferenceDeg : kHipReferenceDeg;
}
inline float joint_min_deg(size_t joint) { return is_knee_joint(joint) ? kKneeMinDeg : kHipMinDeg; }
inline float joint_max_deg(size_t joint) { return is_knee_joint(joint) ? kKneeMaxDeg : kHipMaxDeg; }

// Direction used until one is set from the Pi. +1 means the servo's counts increase in the
// joint's positive direction (docs/kinematics.md: hip femur forward, knee opening). How each
// servo is mounted is not recorded anywhere, so every joint starts at +1 and must be checked
// with a small test move.
inline int8_t default_direction(size_t /*joint*/) { return 1; }

inline bool is_valid_direction(int8_t direction) { return direction == 1 || direction == -1; }

// Clamps an angle to the joint's limits. NaN is returned unchanged (callers hold position).
inline float clamp_to_joint_range(size_t joint, float deg) {
  const float lo = joint_min_deg(joint);
  const float hi = joint_max_deg(joint);
  return deg < lo ? lo : (deg > hi ? hi : deg);
}

// Joint angle for a servo position. Meaningful only when cal.valid.
inline float position_to_deg(const JointCalibration& cal, size_t joint, int16_t position) {
  return joint_reference_deg(joint) + static_cast<float>(cal.direction) *
                                          static_cast<float>(position - cal.reference) *
                                          sts::kDegPerCount;
}

// Servo position for a joint angle, as an unclamped real number of counts.
inline float deg_to_position_unclamped(const JointCalibration& cal, size_t joint, float deg) {
  return static_cast<float>(cal.reference) + static_cast<float>(cal.direction) *
                                                 (deg - joint_reference_deg(joint)) *
                                                 sts::kCountsPerDeg;
}

// Servo goal for a joint angle: rounded and clamped to the servo's range, never wrapped.
inline int16_t deg_to_position(const JointCalibration& cal, size_t joint, float deg) {
  const float counts = deg_to_position_unclamped(cal, joint, deg);
  if (!(counts == counts)) return cal.reference;  // NaN (never expected): the reference pose.
  if (counts <= static_cast<float>(sts::kPositionMin)) return sts::kPositionMin;
  if (counts >= static_cast<float>(sts::kPositionMax)) return sts::kPositionMax;
  return static_cast<int16_t>(counts + 0.5f);
}

// True if both joint limits map inside the servo's position range, with a margin. A calibrated
// joint whose range does not fit is never driven.
inline bool range_fits(const JointCalibration& cal, size_t joint) {
  if (!cal.valid || !is_valid_direction(cal.direction)) return false;
  const float lo = static_cast<float>(sts::kPositionMin) + kPositionMarginCounts;
  const float hi = static_cast<float>(sts::kPositionMax) - kPositionMarginCounts;
  const float at_min = deg_to_position_unclamped(cal, joint, joint_min_deg(joint));
  const float at_max = deg_to_position_unclamped(cal, joint, joint_max_deg(joint));
  return at_min >= lo && at_min <= hi && at_max >= lo && at_max <= hi;
}

// ---------------------------------------------------------------------------------------------
// Stored record
// ---------------------------------------------------------------------------------------------

constexpr uint8_t kCalibrationVersion = 1;
constexpr size_t kCalibrationRecordSize = 4 + 1 + kNumJoints * 3 + 2;  // 31
constexpr uint8_t kCalStoredValid = 1u << 0;
constexpr uint8_t kCalStoredNegative = 1u << 1;

inline void serialize_calibration(const JointCalibration (&joints)[kNumJoints], uint8_t* out) {
  out[0] = 'H';
  out[1] = 'G';
  out[2] = 'C';
  out[3] = 'L';
  out[4] = kCalibrationVersion;
  for (size_t i = 0; i < kNumJoints; ++i) {
    uint8_t* entry = out + 5 + i * 3;
    entry[0] = static_cast<uint8_t>((joints[i].valid ? kCalStoredValid : 0) |
                                    (joints[i].direction < 0 ? kCalStoredNegative : 0));
    put_u16le(entry + 1, static_cast<uint16_t>(joints[i].reference));
  }
  put_u16le(out + kCalibrationRecordSize - 2, crc16_ccitt_false(out, kCalibrationRecordSize - 2));
}

// Fills `joints` and returns true only for a complete, consistent record. On false, `joints` is
// left untouched.
inline bool deserialize_calibration(const uint8_t* in, JointCalibration (&joints)[kNumJoints]) {
  if (in[0] != 'H' || in[1] != 'G' || in[2] != 'C' || in[3] != 'L') return false;
  if (in[4] != kCalibrationVersion) return false;
  if (get_u16le(in + kCalibrationRecordSize - 2) !=
      crc16_ccitt_false(in, kCalibrationRecordSize - 2)) {
    return false;
  }
  JointCalibration parsed[kNumJoints];
  for (size_t i = 0; i < kNumJoints; ++i) {
    const uint8_t* entry = in + 5 + i * 3;
    if (entry[0] & ~(kCalStoredValid | kCalStoredNegative)) return false;
    parsed[i].valid = (entry[0] & kCalStoredValid) != 0;
    parsed[i].direction = (entry[0] & kCalStoredNegative) ? -1 : 1;
    parsed[i].reference = static_cast<int16_t>(get_u16le(entry + 1));
    if (parsed[i].valid &&
        (parsed[i].reference < sts::kPositionMin || parsed[i].reference > sts::kPositionMax)) {
      return false;
    }
  }
  for (size_t i = 0; i < kNumJoints; ++i) joints[i] = parsed[i];
  return true;
}

}  // namespace hg

#endif  // HG_FIRMWARE_CALIBRATION_H_
