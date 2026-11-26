// SPDX-License-Identifier: Apache-2.0
//
// Feetech STS3250 (SMS/STS protocol): register addresses and unit conversions, all in this one
// header. Plain C++, no Arduino dependencies.
//
// Reference: Feetech's FTServo_Arduino library (SMS_STS.h, SMS_STS.cpp). Multi-byte registers are
// little-endian. Servo IDs on the bus: 1 FL hip, 2 FL knee, 3 FR hip, 4 FR knee, 5 RL hip,
// 6 RL knee, 7 RR hip, 8 RR knee (ID = JOINT_STATE index + 1).
//
// The firmware reads registers 40..70 (and the operating mode, register 33) and writes only two
// things: torque enable (0 or 1) and the goal block (acceleration, position, time, speed). It
// never writes a servo's EEPROM (registers below 40): ServoBus refuses every other write, and
// IDs are set with a separate bench sketch.

#ifndef HG_FIRMWARE_SERVO_REGISTERS_H_
#define HG_FIRMWARE_SERVO_REGISTERS_H_

#include <cstdint>

namespace hg {
namespace sts {

// ---------------------------------------------------------------------------------------------
// Register addresses
// ---------------------------------------------------------------------------------------------

// EEPROM register, READ ONLY here. The operating mode decides what the goal registers mean:
// 0 = position servo; 1 = constant speed (the goal speed makes it spin), 2 = PWM, 3 = step. A
// servo is only ever driven in mode 0.
constexpr uint8_t kRegOperatingMode = 33;
constexpr uint8_t kModePosition = 0;

// Control registers (SRAM, read/write).
constexpr uint8_t kRegTorqueEnable = 40;  // 0 = off, 1 = on. See kTorqueOff / kTorqueOn.
constexpr uint8_t kRegAcceleration = 41;  // 100 counts/s^2 per unit; 0 = maximum
constexpr uint8_t kRegGoalPosition = 42;  // 2 bytes, sign bit 15, counts
constexpr uint8_t kRegGoalTime = 44;      // 2 bytes; written as 0 (SMS_STS::WritePosEx)
constexpr uint8_t kRegGoalSpeed = 46;     // 2 bytes, counts per second; 0 = MAXIMUM speed
constexpr uint8_t kRegTorqueLimit = 48;   // 2 bytes, 0.1 %. Not written by the firmware.

// The only values the firmware may write to kRegTorqueEnable. NEVER write 128 there: it makes
// the servo store its current position as a new zero offset in its own EEPROM
// (SMS_STS::CalibrationOfs).
constexpr uint8_t kTorqueOff = 0;
constexpr uint8_t kTorqueOn = 1;

// The block written to set a goal: registers 41..47 (SMS_STS::SyncWritePosEx).
constexpr uint8_t kGoalBlockStart = kRegAcceleration;
constexpr uint8_t kGoalBlockLength = 7;

// The only registers that may be written: torque enable and the goal block (40..47). Everything
// below 40 is the servo's EEPROM and register 55 is the EEPROM lock: neither is ever written.
constexpr uint8_t kFirstWritableRegister = kRegTorqueEnable;
constexpr uint8_t kLastWritableRegister = kGoalBlockStart + kGoalBlockLength - 1;  // 47

// True if writing `count` bytes of `data` at `address` is allowed: inside registers 40..47, and
// torque enable only ever 0 or 1. ServoBus checks every WRITE and SYNC WRITE with this.
inline bool is_allowed_write(uint8_t address, uint8_t count, const uint8_t* data) {
  if (data == nullptr || count == 0 || address < kFirstWritableRegister) return false;
  if (static_cast<unsigned>(address) + count - 1u > kLastWritableRegister) return false;
  if (address == kRegTorqueEnable && data[0] != kTorqueOff && data[0] != kTorqueOn) return false;
  return true;
}

// Feedback registers (SRAM, read-only).
constexpr uint8_t kRegPresentPosition = 56;     // 2 bytes, sign bit 15, counts
constexpr uint8_t kRegPresentSpeed = 58;        // 2 bytes, sign bit 15, counts per second
constexpr uint8_t kRegPresentLoad = 60;         // 2 bytes, direction bit 10, magnitude in 0.1 %
constexpr uint8_t kRegPresentVoltage = 62;      // 1 byte, 0.1 V
constexpr uint8_t kRegPresentTemperature = 63;  // 1 byte, deg C
constexpr uint8_t kRegMoving = 66;              // 1 byte
constexpr uint8_t kRegPresentCurrent = 69;      // 2 bytes, sign bit 15, counts

// The feedback block: registers 56..70.
constexpr uint8_t kFeedbackStart = kRegPresentPosition;
constexpr uint8_t kFeedbackLength = 15;

// The block read from each servo in one transaction: registers 40..70, so that the servo's own
// torque state comes back with the feedback.
constexpr uint8_t kPollStart = kRegTorqueEnable;
constexpr uint8_t kPollLength = kFeedbackStart + kFeedbackLength - kPollStart;  // 31

// ---------------------------------------------------------------------------------------------
// Status byte
// ---------------------------------------------------------------------------------------------

// Every reply packet carries the servo's status byte (FTServo_Arduino: SCS::getState()); 0 means
// no fault. The library does not name the bits. The names below are from Feetech's STS memory
// table and still need to be confirmed against the STS3250 documentation.
constexpr uint8_t kStatusVoltage = 1u << 0;
constexpr uint8_t kStatusSensor = 1u << 1;
constexpr uint8_t kStatusTemperature = 1u << 2;
constexpr uint8_t kStatusCurrent = 1u << 3;
constexpr uint8_t kStatusAngle = 1u << 4;
constexpr uint8_t kStatusOverload = 1u << 5;

// ---------------------------------------------------------------------------------------------
// Unit conversions
// ---------------------------------------------------------------------------------------------

constexpr float kCountsPerRev = 4096.0f;  // Position: 12-bit, 4096 counts per turn.
constexpr float kDegPerCount = 360.0f / kCountsPerRev;
constexpr float kCountsPerDeg = kCountsPerRev / 360.0f;
// The position range of the servo in its normal (single-turn) mode. A goal is never wrapped
// around it: wrapping would send the joint the long way round.
constexpr int16_t kPositionMin = 0;
constexpr int16_t kPositionMax = 4095;
constexpr float kLoadPercentPerCount = 0.1f;  // Load magnitude: 0..1000 = 0..100 %.
constexpr float kVoltsPerCount = 0.1f;
constexpr float kMilliampsPerCount = 6.5f;  // Approximate: to be checked against a meter.

// Position, speed and current are sign-magnitude words: bit 15 is the sign (SMS_STS::ReadPos).
inline int16_t decode_sign15(uint16_t raw) {
  const int16_t magnitude = static_cast<int16_t>(raw & 0x7FFF);
  return (raw & 0x8000) ? static_cast<int16_t>(-magnitude) : magnitude;
}

// Inverse of decode_sign15(), for goal registers (SMS_STS::WritePosEx).
inline uint16_t encode_sign15(int16_t value) {
  if (value >= 0) return static_cast<uint16_t>(value);
  // -32768 has no sign-magnitude form in 15 bits; saturate to -32767.
  const uint16_t magnitude = value == INT16_MIN ? 0x7FFF : static_cast<uint16_t>(-value);
  return static_cast<uint16_t>(magnitude | 0x8000);
}

// Load: bit 10 is the direction, bits 0-9 the magnitude in 0.1 % (SMS_STS::ReadLoad).
inline int16_t decode_load(uint16_t raw) {
  const int16_t magnitude = static_cast<int16_t>(raw & 0x03FF);
  return (raw & 0x0400) ? static_cast<int16_t>(-magnitude) : magnitude;
}

inline uint16_t read_u16le(const uint8_t* bytes) {
  return static_cast<uint16_t>(bytes[0] | (static_cast<uint16_t>(bytes[1]) << 8));
}

// One servo's feedback, decoded to signed servo units (no physical-unit scaling).
struct Feedback {
  int16_t position = 0;     // counts (kDegPerCount)
  int16_t speed = 0;        // counts per second
  int16_t load = 0;         // 0.1 % (kLoadPercentPerCount), sign = direction
  int16_t current = 0;      // counts (kMilliampsPerCount)
  uint8_t voltage = 0;      // 0.1 V (kVoltsPerCount)
  uint8_t temperature = 0;  // deg C
};

inline int32_t current_to_milliamps(int16_t counts) {
  return static_cast<int32_t>(static_cast<float>(counts) * kMilliampsPerCount);
}

// Goal block for one servo (kGoalBlockLength bytes, written at kGoalBlockStart): the goal
// position in counts, and the servo's own speed cap in counts per second. A speed of 0 would
// mean "maximum" to the servo, so it is raised to 1.
inline void encode_goal_block(int16_t position, uint16_t speed, uint8_t* out) {
  const uint16_t position_word = encode_sign15(position);
  const uint16_t speed_word = speed == 0 ? 1 : (speed > 0x7FFF ? 0x7FFF : speed);
  out[0] = 0;  // Acceleration: maximum. The firmware's own rate limit shapes the motion.
  out[1] = static_cast<uint8_t>(position_word & 0xFF);
  out[2] = static_cast<uint8_t>(position_word >> 8);
  out[3] = 0;  // Goal time: unused.
  out[4] = 0;
  out[5] = static_cast<uint8_t>(speed_word & 0xFF);
  out[6] = static_cast<uint8_t>(speed_word >> 8);
}

// Decodes a feedback block: kFeedbackLength bytes starting at register kFeedbackStart.
inline Feedback decode_feedback(const uint8_t* block) {
  Feedback f;
  f.position = decode_sign15(read_u16le(block + (kRegPresentPosition - kFeedbackStart)));
  f.speed = decode_sign15(read_u16le(block + (kRegPresentSpeed - kFeedbackStart)));
  f.load = decode_load(read_u16le(block + (kRegPresentLoad - kFeedbackStart)));
  f.voltage = block[kRegPresentVoltage - kFeedbackStart];
  f.temperature = block[kRegPresentTemperature - kFeedbackStart];
  f.current = decode_sign15(read_u16le(block + (kRegPresentCurrent - kFeedbackStart)));
  return f;
}

// One servo's poll block (kPollLength bytes read from kPollStart), decoded.
struct PollData {
  uint8_t torque_enable = 0;  // Register 40 as read: 0 = torque off.
  int16_t goal = 0;           // Goal position register, counts.
  Feedback feedback;
};

inline PollData decode_poll(const uint8_t* block) {
  PollData d;
  d.torque_enable = block[kRegTorqueEnable - kPollStart];
  d.goal = decode_sign15(read_u16le(block + (kRegGoalPosition - kPollStart)));
  d.feedback = decode_feedback(block + (kFeedbackStart - kPollStart));
  return d;
}

}  // namespace sts
}  // namespace hg

#endif  // HG_FIRMWARE_SERVO_REGISTERS_H_
