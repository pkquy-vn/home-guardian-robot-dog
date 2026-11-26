// SPDX-License-Identifier: Apache-2.0
//
// Feetech STS3250 (SMS/STS protocol): register addresses and unit conversions, all in this one
// header. Plain C++, no Arduino dependencies.
//
// Reference: Feetech's FTServo_Arduino library (SMS_STS.h, SMS_STS.cpp). Multi-byte registers are
// little-endian. Servo IDs on the bus: 1 FL hip, 2 FL knee, 3 FR hip, 4 FR knee, 5 RL hip,
// 6 RL knee, 7 RR hip, 8 RR knee (ID = JOINT_STATE index + 1).
//
// S1: the firmware only reads the feedback block. The control registers are listed for S2 and
// are not written anywhere. There are deliberately no EEPROM registers here: IDs are set with a
// separate bench sketch.

#ifndef HG_FIRMWARE_SERVO_REGISTERS_H_
#define HG_FIRMWARE_SERVO_REGISTERS_H_

#include <cstdint>

namespace hg {
namespace sts {

// ---------------------------------------------------------------------------------------------
// Register addresses
// ---------------------------------------------------------------------------------------------

// Control registers (SRAM, read/write). NOT written in S1.
constexpr uint8_t kRegTorqueEnable = 40;
constexpr uint8_t kRegAcceleration = 41;
constexpr uint8_t kRegGoalPosition = 42;  // 2 bytes, sign bit 15
constexpr uint8_t kRegGoalTime = 44;      // 2 bytes
constexpr uint8_t kRegGoalSpeed = 46;     // 2 bytes
constexpr uint8_t kRegTorqueLimit = 48;   // 2 bytes

// Feedback registers (SRAM, read-only).
constexpr uint8_t kRegPresentPosition = 56;     // 2 bytes, sign bit 15, counts
constexpr uint8_t kRegPresentSpeed = 58;        // 2 bytes, sign bit 15, counts per second
constexpr uint8_t kRegPresentLoad = 60;         // 2 bytes, direction bit 10, magnitude in 0.1 %
constexpr uint8_t kRegPresentVoltage = 62;      // 1 byte, 0.1 V
constexpr uint8_t kRegPresentTemperature = 63;  // 1 byte, deg C
constexpr uint8_t kRegMoving = 66;              // 1 byte
constexpr uint8_t kRegPresentCurrent = 69;      // 2 bytes, sign bit 15, counts

// The feedback block read in one transaction: registers 56..70.
constexpr uint8_t kFeedbackStart = kRegPresentPosition;
constexpr uint8_t kFeedbackLength = 15;

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
constexpr float kLoadPercentPerCount = 0.1f;  // Load magnitude: 0..1000 = 0..100 %.
constexpr float kVoltsPerCount = 0.1f;
constexpr float kMilliampsPerCount = 6.5f;  // Approximate: to be checked against a meter.

// Position, speed and current are sign-magnitude words: bit 15 is the sign (SMS_STS::ReadPos).
inline int16_t decode_sign15(uint16_t raw) {
  const int16_t magnitude = static_cast<int16_t>(raw & 0x7FFF);
  return (raw & 0x8000) ? static_cast<int16_t>(-magnitude) : magnitude;
}

// Inverse of decode_sign15(), for goal registers (SMS_STS::WritePosEx). Unused in S1.
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

// Decodes a feedback block: kFeedbackLength bytes read from kFeedbackStart.
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

}  // namespace sts
}  // namespace hg

#endif  // HG_FIRMWARE_SERVO_REGISTERS_H_
