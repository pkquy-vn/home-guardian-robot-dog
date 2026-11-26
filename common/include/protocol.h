// SPDX-License-Identifier: Apache-2.0
//
// Home Guardian Pi <-> Teensy serial protocol.
//
// Header-only, no Arduino dependencies: the same file is compiled into the Teensy firmware and
// into host-side tools and tests.
//
// Frame layout (all multi-byte fields little-endian):
//
//   +------+------+--------+-------------------+--------+--------+
//   | 0xA5 | type | length | payload[length]   | crc lo | crc hi |
//   +------+------+--------+-------------------+--------+--------+
//
//   length  0..kMaxPayload bytes.
//   crc     CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR)
//           computed over type, length and payload (not the start byte).
//
// Python mirror: ros2_ws/src/hg_teensy_bridge/hg_teensy_bridge/protocol.py (used by the ROS 2
// bridge and tools/test_link.py). Keep both in sync; common/test/test_protocol.py checks that
// they agree.

#ifndef HG_COMMON_PROTOCOL_H_
#define HG_COMMON_PROTOCOL_H_

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace hg {

constexpr uint8_t kStartByte = 0xA5;
constexpr size_t kMaxPayload = 64;
constexpr size_t kHeaderSize = 3;  // start, type, length
constexpr size_t kCrcSize = 2;
constexpr size_t kMaxFrameSize = kHeaderSize + kMaxPayload + kCrcSize;

constexpr uint16_t kCrcInit = 0xFFFF;
constexpr uint16_t kCrcPoly = 0x1021;

// Message type IDs. The IDs marked "reserved" have no payload definition yet; their payloads
// will be specified when they are implemented.
enum class MsgType : uint8_t {
  // Pi -> Teensy
  kHeartbeat = 0x01,    // HEARTBEAT
  kCmdVel = 0x02,       // CMD_VEL
  kBodyPose = 0x03,     // BODY_POSE
  kMode = 0x04,         // MODE
  kLed = 0x05,          // LED
  kIrLed = 0x06,        // IR_LED
  kConfig = 0x07,       // CONFIG
  kClearFault = 0x08,   // CLEAR_FAULT
  kCalibration = 0x09,  // CALIBRATION
  kJointTarget = 0x0A,  // JOINT_TARGET
  kSpeedLimit = 0x0B,   // SPEED_LIMIT
  // Teensy -> Pi
  kJointState = 0x11,   // JOINT_STATE
  kImu = 0x12,          // IMU
  kOdom = 0x13,         // ODOM
  kContact = 0x14,      // CONTACT
  kBattery = 0x15,      // BATTERY
  kStatus = 0x16,       // STATUS
  kFault = 0x17,        // FAULT
  kServoState = 0x18,   // SERVO_STATE
  kJointStatus = 0x19,  // JOINT_STATUS
  kAck = 0x1A,          // ACK
};

// Posture modes: requested with MODE, reported in StatusPayload::mode.
enum class Mode : uint8_t {
  kLie = 0,        // LIE
  kSit = 1,        // SIT
  kStand = 2,      // STAND
  kWalk = 3,       // WALK
  kTorqueOff = 9,  // TORQUE_OFF
};

inline bool is_valid_mode(uint8_t value) {
  return value == static_cast<uint8_t>(Mode::kLie) || value == static_cast<uint8_t>(Mode::kSit) ||
         value == static_cast<uint8_t>(Mode::kStand) ||
         value == static_cast<uint8_t>(Mode::kWalk) ||
         value == static_cast<uint8_t>(Mode::kTorqueOff);
}

// Fault bits reported in StatusPayload::fault_bits. Every fault latches until CLEAR_FAULT
// (HEARTBEAT_LOST before the first heartbeat after boot clears by itself).
constexpr uint16_t kFaultHeartbeatLost = 1u << 0;     // HEARTBEAT_LOST
constexpr uint16_t kFaultServoHot = 1u << 1;          // SERVO_HOT: >= 65 C, went to LIE
constexpr uint16_t kFaultServoOverheat = 1u << 2;     // SERVO_OVERHEAT: >= 70 C, torque off
constexpr uint16_t kFaultServoOvercurrent = 1u << 3;  // SERVO_OVERCURRENT: torque off
constexpr uint16_t kFaultServoMissing = 1u << 4;     // SERVO_MISSING: driven servo gone, torque off
constexpr uint16_t kFaultServoTorqueLost = 1u << 5;  // SERVO_TORQUE_LOST: see docs, torque off
// Faults whose response is torque off on all servos at once.
constexpr uint16_t kTorqueOffFaults =
    kFaultServoOverheat | kFaultServoOvercurrent | kFaultServoMissing | kFaultServoTorqueLost;
constexpr uint16_t kServoFaults = kFaultServoHot | kTorqueOffFaults;

// Joint value meaning "no joint" in FAULT and ACK.
constexpr uint8_t kNoJoint = 0xFF;

// ---------------------------------------------------------------------------------------------
// Little-endian helpers
// ---------------------------------------------------------------------------------------------

inline void put_u16le(uint8_t* out, uint16_t v) {
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>(v >> 8);
}

inline void put_u32le(uint8_t* out, uint32_t v) {
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  out[3] = static_cast<uint8_t>(v >> 24);
}

inline uint16_t get_u16le(const uint8_t* in) {
  return static_cast<uint16_t>(in[0] | (static_cast<uint16_t>(in[1]) << 8));
}

inline uint32_t get_u32le(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
}

// ---------------------------------------------------------------------------------------------
// CRC-16/CCITT-FALSE
// ---------------------------------------------------------------------------------------------

inline uint16_t crc16_ccitt_false_update(uint16_t crc, uint8_t byte) {
  crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(byte) << 8));
  for (int bit = 0; bit < 8; ++bit) {
    if (crc & 0x8000) {
      crc = static_cast<uint16_t>((crc << 1) ^ kCrcPoly);
    } else {
      crc = static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline uint16_t crc16_ccitt_false(const uint8_t* data, size_t len, uint16_t crc = kCrcInit) {
  for (size_t i = 0; i < len; ++i) crc = crc16_ccitt_false_update(crc, data[i]);
  return crc;
}

// ---------------------------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------------------------

// Writes one frame into `out`. `payload` may be null when `len` is 0.
// Returns the number of bytes written (kHeaderSize + len + kCrcSize), or 0 if `len` exceeds
// kMaxPayload or `out_size` is too small.
inline size_t encode_frame(uint8_t type, const uint8_t* payload, size_t len, uint8_t* out,
                           size_t out_size) {
  if (len > kMaxPayload) return 0;
  const size_t frame_size = kHeaderSize + len + kCrcSize;
  if (out == nullptr || out_size < frame_size) return 0;
  if (len > 0 && payload == nullptr) return 0;

  out[0] = kStartByte;
  out[1] = type;
  out[2] = static_cast<uint8_t>(len);
  if (len > 0) std::memcpy(out + kHeaderSize, payload, len);
  const uint16_t crc = crc16_ccitt_false(out + 1, 2 + len);
  put_u16le(out + kHeaderSize + len, crc);
  return frame_size;
}

inline size_t encode_frame(MsgType type, const uint8_t* payload, size_t len, uint8_t* out,
                           size_t out_size) {
  return encode_frame(static_cast<uint8_t>(type), payload, len, out, out_size);
}

// ---------------------------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------------------------

// Byte-at-a-time frame parser.
//
// Bytes are buffered from a start byte until a complete frame has arrived. A frame with a bad
// CRC or an oversize length is rejected and counted, and the parser resynchronizes by rescanning
// the rejected bytes for the next start byte, so a real frame that began inside a false or
// corrupted one is not lost. Because of that rescan, more than one complete frame can be waiting
// after a rejection; drain them with poll():
//
//   if (parser.feed(byte)) {
//     do {
//       handle(parser.type(), parser.payload(), parser.length());
//     } while (parser.poll());
//   }
//
// type(), length() and payload() describe the most recent valid frame and stay valid until the
// next call to feed() or poll().
class FrameParser {
 public:
  // Adds one received byte. Returns true if a valid frame is available.
  bool feed(uint8_t byte) {
    if (count_ == 0) {
      if (byte != kStartByte) {
        ++dropped_bytes_;
        return false;
      }
    } else if (count_ >= kMaxFrameSize) {
      // Unreachable while poll() keeps its invariants; guard against overflow regardless.
      discard_and_resync();
      return feed(byte);
    }
    buf_[count_++] = byte;
    return poll();
  }

  // Looks for a complete frame among bytes that are already buffered. Returns true if a valid
  // frame is available.
  bool poll() {
    while (count_ >= kHeaderSize) {
      const size_t len = buf_[2];
      if (len > kMaxPayload) {
        ++length_errors_;
        discard_and_resync();
        continue;
      }
      const size_t frame_size = kHeaderSize + len + kCrcSize;
      if (count_ < frame_size) return false;

      const uint16_t received = get_u16le(buf_ + kHeaderSize + len);
      const uint16_t computed = crc16_ccitt_false(buf_ + 1, 2 + len);
      if (received != computed) {
        ++crc_errors_;
        discard_and_resync();
        continue;
      }

      type_ = buf_[1];
      length_ = static_cast<uint8_t>(len);
      if (len > 0) std::memcpy(payload_, buf_ + kHeaderSize, len);
      ++frames_ok_;
      consume(frame_size);
      return true;
    }
    return false;
  }

  // Clears buffered bytes. Counters are kept.
  void reset() { count_ = 0; }

  uint8_t type() const { return type_; }
  uint8_t length() const { return length_; }
  const uint8_t* payload() const { return payload_; }

  uint32_t frames_ok() const { return frames_ok_; }
  uint32_t crc_errors() const { return crc_errors_; }
  uint32_t length_errors() const { return length_errors_; }
  // Bytes skipped while searching for a start byte.
  uint32_t dropped_bytes() const { return dropped_bytes_; }

 private:
  // Drops the start byte of the rejected candidate and realigns on the next start byte.
  void discard_and_resync() { consume(1); }

  // Removes `n` bytes from the front of the buffer, then skips to the next start byte.
  void consume(size_t n) {
    size_t next = n;
    while (next < count_ && buf_[next] != kStartByte) ++next;
    dropped_bytes_ += static_cast<uint32_t>(next - n);
    const size_t remaining = count_ - next;
    if (remaining > 0) std::memmove(buf_, buf_ + next, remaining);
    count_ = remaining;
  }

  uint8_t buf_[kMaxFrameSize] = {};
  size_t count_ = 0;

  uint8_t type_ = 0;
  uint8_t length_ = 0;
  uint8_t payload_[kMaxPayload] = {};

  uint32_t frames_ok_ = 0;
  uint32_t crc_errors_ = 0;
  uint32_t length_errors_ = 0;
  uint32_t dropped_bytes_ = 0;
};

// ---------------------------------------------------------------------------------------------
// Payloads
// ---------------------------------------------------------------------------------------------

// HEARTBEAT (Pi -> Teensy): uint32 sequence.
constexpr size_t kHeartbeatPayloadSize = 4;

inline void pack_heartbeat(uint32_t sequence, uint8_t* out) { put_u32le(out, sequence); }

inline bool unpack_heartbeat(const uint8_t* payload, size_t len, uint32_t* sequence) {
  if (len != kHeartbeatPayloadSize || payload == nullptr || sequence == nullptr) return false;
  *sequence = get_u32le(payload);
  return true;
}

// MODE (Pi -> Teensy): uint8 requested mode (a Mode value; the firmware rejects others).
constexpr size_t kModePayloadSize = 1;

inline void pack_mode(uint8_t mode, uint8_t* out) { out[0] = mode; }

inline bool unpack_mode(const uint8_t* payload, size_t len, uint8_t* mode) {
  if (len != kModePayloadSize || payload == nullptr || mode == nullptr) return false;
  *mode = payload[0];
  return true;
}

// CLEAR_FAULT (Pi -> Teensy): empty payload.
constexpr size_t kClearFaultPayloadSize = 0;

// STATUS (Teensy -> Pi): uint8 mode, uint8 gait phase, uint16 fault bits.
constexpr size_t kStatusPayloadSize = 4;

struct StatusPayload {
  uint8_t mode = 0;
  uint8_t gait_phase = 0;
  uint16_t fault_bits = 0;
};

inline void pack_status(const StatusPayload& status, uint8_t* out) {
  out[0] = status.mode;
  out[1] = status.gait_phase;
  put_u16le(out + 2, status.fault_bits);
}

inline bool unpack_status(const uint8_t* payload, size_t len, StatusPayload* status) {
  if (len != kStatusPayloadSize || payload == nullptr || status == nullptr) return false;
  status->mode = payload[0];
  status->gait_phase = payload[1];
  status->fault_bits = get_u16le(payload + 2);
  return true;
}

// JOINT_STATE (Teensy -> Pi): 8 x (int16 position in 0.01 deg, int16 current in mA, int8
// temperature in deg C), in JointIndex order. For a joint whose servo is present and calibrated
// (JOINT_STATUS flag kJointMeasured) these are measured values in the joint conventions of
// docs/kinematics.md. For every other joint the position is the commanded posture angle and
// current and temperature are 0.
enum class JointIndex : uint8_t {
  kFlHip = 0,
  kFlKnee = 1,
  kFrHip = 2,
  kFrKnee = 3,
  kRlHip = 4,
  kRlKnee = 5,
  kRrHip = 6,
  kRrKnee = 7,
};
constexpr size_t kNumJoints = 8;
constexpr size_t kJointStateEntrySize = 5;
constexpr size_t kJointStatePayloadSize = kNumJoints * kJointStateEntrySize;  // 40

struct JointStateEntry {
  int16_t position_cdeg = 0;  // 0.01 deg
  int16_t current = 0;        // mA
  int8_t temperature = 0;     // deg C
};

inline void pack_joint_state(const JointStateEntry (&joints)[kNumJoints], uint8_t* out) {
  for (size_t i = 0; i < kNumJoints; ++i) {
    uint8_t* entry = out + i * kJointStateEntrySize;
    put_u16le(entry, static_cast<uint16_t>(joints[i].position_cdeg));
    put_u16le(entry + 2, static_cast<uint16_t>(joints[i].current));
    entry[4] = static_cast<uint8_t>(joints[i].temperature);
  }
}

inline bool unpack_joint_state(const uint8_t* payload, size_t len,
                               JointStateEntry (&joints)[kNumJoints]) {
  if (len != kJointStatePayloadSize || payload == nullptr) return false;
  for (size_t i = 0; i < kNumJoints; ++i) {
    const uint8_t* entry = payload + i * kJointStateEntrySize;
    joints[i].position_cdeg = static_cast<int16_t>(get_u16le(entry));
    joints[i].current = static_cast<int16_t>(get_u16le(entry + 2));
    joints[i].temperature = static_cast<int8_t>(entry[4]);
  }
  return true;
}

// SERVO_STATE (Teensy -> Pi): telemetry of the bus servos, IDs 1..8 (ID = JointIndex + 1).
// Eight servos do not fit in one 64-byte payload, so each message carries a group of four:
//   uint8 first servo ID of the group (1 or 5), then for IDs first .. first + 3:
//   uint8 flags (bit 0 = present), int16 position, int16 speed, int16 load, int16 current,
//   uint8 voltage, uint8 temperature, uint8 status.
// Values are in servo units, with the servo's sign-magnitude words decoded to two's complement:
// position in counts (4096 per turn, raw: no calibration), speed in counts per second, load in
// 0.1 % (sign = direction), current in counts (about 6.5 mA each), voltage in 0.1 V, temperature
// in deg C, status = the servo's status byte (0 = no fault). All fields are 0 when not present.
constexpr size_t kNumServos = 8;
constexpr size_t kServoStateGroupSize = 4;
constexpr size_t kServoStateEntrySize = 12;
constexpr size_t kServoStatePayloadSize = 1 + kServoStateGroupSize * kServoStateEntrySize;  // 49
constexpr uint8_t kServoFlagPresent = 1u << 0;

struct ServoStateEntry {
  bool present = false;
  int16_t position = 0;     // counts
  int16_t speed = 0;        // counts per second
  int16_t load = 0;         // 0.1 %
  int16_t current = 0;      // counts
  uint8_t voltage = 0;      // 0.1 V
  uint8_t temperature = 0;  // deg C
  uint8_t status = 0;
};

// True for the first servo ID of a group: 1 or 5.
inline bool is_valid_servo_group(uint8_t first_id) {
  return first_id >= 1 && first_id + kServoStateGroupSize - 1 <= kNumServos &&
         (first_id - 1) % kServoStateGroupSize == 0;
}

inline void pack_servo_state(uint8_t first_id,
                             const ServoStateEntry (&servos)[kServoStateGroupSize], uint8_t* out) {
  out[0] = first_id;
  for (size_t i = 0; i < kServoStateGroupSize; ++i) {
    uint8_t* entry = out + 1 + i * kServoStateEntrySize;
    entry[0] = servos[i].present ? kServoFlagPresent : 0;
    put_u16le(entry + 1, static_cast<uint16_t>(servos[i].position));
    put_u16le(entry + 3, static_cast<uint16_t>(servos[i].speed));
    put_u16le(entry + 5, static_cast<uint16_t>(servos[i].load));
    put_u16le(entry + 7, static_cast<uint16_t>(servos[i].current));
    entry[9] = servos[i].voltage;
    entry[10] = servos[i].temperature;
    entry[11] = servos[i].status;
  }
}

inline bool unpack_servo_state(const uint8_t* payload, size_t len, uint8_t* first_id,
                               ServoStateEntry (&servos)[kServoStateGroupSize]) {
  if (len != kServoStatePayloadSize || payload == nullptr || first_id == nullptr) return false;
  if (!is_valid_servo_group(payload[0])) return false;
  *first_id = payload[0];
  for (size_t i = 0; i < kServoStateGroupSize; ++i) {
    const uint8_t* entry = payload + 1 + i * kServoStateEntrySize;
    servos[i].present = (entry[0] & kServoFlagPresent) != 0;
    servos[i].position = static_cast<int16_t>(get_u16le(entry + 1));
    servos[i].speed = static_cast<int16_t>(get_u16le(entry + 3));
    servos[i].load = static_cast<int16_t>(get_u16le(entry + 5));
    servos[i].current = static_cast<int16_t>(get_u16le(entry + 7));
    servos[i].voltage = entry[9];
    servos[i].temperature = entry[10];
    servos[i].status = entry[11];
  }
  return true;
}

// ---------------------------------------------------------------------------------------------
// Servo motion: calibration, test targets, status, faults and acknowledgements
// ---------------------------------------------------------------------------------------------

// CALIBRATION (Pi -> Teensy): uint8 operation, uint8 joint (JointIndex), int8 direction,
// int16 reference. Changes one joint's calibration, stored in the Teensy's EEPROM. Accepted
// only while no servo has torque.
//   kCalCapture:      store the servo's current position as the reference pose (hip 0 deg, knee
//                     interior 90 deg). direction and reference are ignored.
//   kCalSetDirection: set the direction sign to +1 or -1. reference is ignored.
//   kCalRestore:      set direction and reference (servo counts) from the Pi's saved copy.
constexpr uint8_t kCalCapture = 1;
constexpr uint8_t kCalSetDirection = 2;
constexpr uint8_t kCalRestore = 3;
constexpr size_t kCalibrationPayloadSize = 5;

struct CalibrationPayload {
  uint8_t operation = 0;
  uint8_t joint = 0;
  int8_t direction = 1;
  int16_t reference = 0;
};

inline void pack_calibration(const CalibrationPayload& c, uint8_t* out) {
  out[0] = c.operation;
  out[1] = c.joint;
  out[2] = static_cast<uint8_t>(c.direction);
  put_u16le(out + 3, static_cast<uint16_t>(c.reference));
}

inline bool unpack_calibration(const uint8_t* payload, size_t len, CalibrationPayload* c) {
  if (len != kCalibrationPayloadSize || payload == nullptr || c == nullptr) return false;
  c->operation = payload[0];
  c->joint = payload[1];
  c->direction = static_cast<int8_t>(payload[2]);
  c->reference = static_cast<int16_t>(get_u16le(payload + 3));
  return true;
}

// JOINT_TARGET (Pi -> Teensy): uint8 joint, int16 angle in 0.01 deg. Test command: drives one
// joint to an angle (clamped to the joint limits, rate-limited) until the next MODE command.
constexpr size_t kJointTargetPayloadSize = 3;

inline void pack_joint_target(uint8_t joint, int16_t angle_cdeg, uint8_t* out) {
  out[0] = joint;
  put_u16le(out + 1, static_cast<uint16_t>(angle_cdeg));
}

inline bool unpack_joint_target(const uint8_t* payload, size_t len, uint8_t* joint,
                                int16_t* angle_cdeg) {
  if (len != kJointTargetPayloadSize || payload == nullptr || joint == nullptr ||
      angle_cdeg == nullptr) {
    return false;
  }
  *joint = payload[0];
  *angle_cdeg = static_cast<int16_t>(get_u16le(payload + 1));
  return true;
}

// SPEED_LIMIT (Pi -> Teensy): uint16 maximum joint speed in 0.1 deg/s. The firmware clamps it
// to its own range and reports the value in use in ACK and JOINT_STATUS. Not stored: the default
// applies again after a reset.
constexpr size_t kSpeedLimitPayloadSize = 2;

inline void pack_speed_limit(uint16_t speed_ddeg_s, uint8_t* out) { put_u16le(out, speed_ddeg_s); }

inline bool unpack_speed_limit(const uint8_t* payload, size_t len, uint16_t* speed_ddeg_s) {
  if (len != kSpeedLimitPayloadSize || payload == nullptr || speed_ddeg_s == nullptr) return false;
  *speed_ddeg_s = get_u16le(payload);
  return true;
}

// FAULT (Teensy -> Pi): uint16 fault (exactly one fault bit), uint8 joint (JointIndex, or
// kNoJoint), int16 value. The cause of one latched servo fault: the joint that tripped it and
// the value that did (temperature in deg C, current in mA, otherwise 0). Sent when the fault
// latches and once per second while it stays latched.
constexpr size_t kFaultPayloadSize = 5;

struct FaultPayload {
  uint16_t fault = 0;
  uint8_t joint = kNoJoint;
  int16_t value = 0;
};

inline void pack_fault(const FaultPayload& f, uint8_t* out) {
  put_u16le(out, f.fault);
  out[2] = f.joint;
  put_u16le(out + 3, static_cast<uint16_t>(f.value));
}

inline bool unpack_fault(const uint8_t* payload, size_t len, FaultPayload* f) {
  if (len != kFaultPayloadSize || payload == nullptr || f == nullptr) return false;
  f->fault = get_u16le(payload);
  f->joint = payload[2];
  f->value = static_cast<int16_t>(get_u16le(payload + 3));
  return true;
}

// JOINT_STATUS (Teensy -> Pi, 10 Hz): uint16 speed limit in 0.1 deg/s, then 8 x (uint8 flags,
// uint8 block, int16 reference, int16 target in 0.01 deg), in JointIndex order. reference is the
// calibration: the servo position in counts at the reference pose. target is the angle the joint
// is (or would be) driven to: the posture or test target, clamped to the joint limits.
constexpr uint8_t kJointCalibrated = 1u << 0;         // A reference is stored.
constexpr uint8_t kJointDirectionNegative = 1u << 1;  // Direction sign is -1.
constexpr uint8_t kJointTorqueOn = 1u << 2;           // The servo reports torque on.
constexpr uint8_t kJointDriven = 1u << 3;             // The firmware is driving this joint.
constexpr uint8_t kJointTestTarget = 1u << 4;         // A JOINT_TARGET test target is active.
constexpr uint8_t kJointMeasured = 1u << 5;           // JOINT_STATE carries measured values for it.

// JointStatusEntry::block: why the joint cannot be given torque right now. kNone: it can.
enum class JointBlock : uint8_t {
  kNone = 0,
  kNotPresent = 1,     // The servo is not answering.
  kNotCalibrated = 2,  // No reference stored.
  kRangeError = 3,     // The joint range crosses the servo's 0/4095 point: flip or remount.
  kServoMode = 4,      // The servo is not in position mode (its register 33 is not 0).
  kOutOfRange = 5,     // The measured angle is far outside the joint limits.
};

constexpr size_t kJointStatusEntrySize = 6;
constexpr size_t kJointStatusPayloadSize = 2 + kNumJoints * kJointStatusEntrySize;  // 50

struct JointStatusEntry {
  uint8_t flags = 0;
  uint8_t block = 0;        // A JointBlock value.
  int16_t reference = 0;    // servo counts
  int16_t target_cdeg = 0;  // 0.01 deg
};

inline void pack_joint_status(uint16_t speed_limit_ddeg_s,
                              const JointStatusEntry (&joints)[kNumJoints], uint8_t* out) {
  put_u16le(out, speed_limit_ddeg_s);
  for (size_t i = 0; i < kNumJoints; ++i) {
    uint8_t* entry = out + 2 + i * kJointStatusEntrySize;
    entry[0] = joints[i].flags;
    entry[1] = joints[i].block;
    put_u16le(entry + 2, static_cast<uint16_t>(joints[i].reference));
    put_u16le(entry + 4, static_cast<uint16_t>(joints[i].target_cdeg));
  }
}

inline bool unpack_joint_status(const uint8_t* payload, size_t len, uint16_t* speed_limit_ddeg_s,
                                JointStatusEntry (&joints)[kNumJoints]) {
  if (len != kJointStatusPayloadSize || payload == nullptr || speed_limit_ddeg_s == nullptr) {
    return false;
  }
  *speed_limit_ddeg_s = get_u16le(payload);
  for (size_t i = 0; i < kNumJoints; ++i) {
    const uint8_t* entry = payload + 2 + i * kJointStatusEntrySize;
    joints[i].flags = entry[0];
    joints[i].block = entry[1];
    joints[i].reference = static_cast<int16_t>(get_u16le(entry + 2));
    joints[i].target_cdeg = static_cast<int16_t>(get_u16le(entry + 4));
  }
  return true;
}

// ACK (Teensy -> Pi): uint8 command (the MsgType answered), uint8 result (AckResult), uint8
// joint (JointIndex, or kNoJoint), int16 value. Sent once for every MODE, CLEAR_FAULT,
// CALIBRATION, JOINT_TARGET and SPEED_LIMIT received. value: MODE: the requested mode;
// CALIBRATION: the stored reference in counts; JOINT_TARGET: the clamped target in 0.01 deg;
// SPEED_LIMIT: the speed limit in use in 0.1 deg/s; otherwise 0.
enum class AckResult : uint8_t {
  kOk = 0,
  kNoChange = 1,       // Already in the requested state; nothing to do.
  kBadRequest = 2,     // Wrong payload size, or an unknown mode, operation, joint or value.
  kFaultLatched = 3,   // Refused: a fault is latched. Send CLEAR_FAULT first.
  kBusy = 4,           // Refused: a posture transition is in progress.
  kNotAllowed = 5,     // Refused: not allowed in the current mode.
  kTorqueOn = 6,       // Refused: calibration only changes while every servo is limp.
  kNotPresent = 7,     // Refused: the joint's servo is not answering.
  kNotCalibrated = 8,  // Refused: the joint is not calibrated.
  kLinkUnhealthy = 9,  // CLEAR_FAULT refused: no healthy heartbeat.
  kStorageError = 10,  // The calibration is in use but could not be saved to the EEPROM.
  // The joint cannot be driven (see JointBlock). For CALIBRATION, kRangeError means: stored, but
  // the joint will not get torque until its direction is flipped or the horn is remounted.
  kRangeError = 11,
  kServoMode = 12,
  kOutOfRange = 13,
};
constexpr size_t kAckPayloadSize = 5;

struct AckPayload {
  uint8_t command = 0;
  uint8_t result = 0;
  uint8_t joint = kNoJoint;
  int16_t value = 0;
};

inline void pack_ack(const AckPayload& a, uint8_t* out) {
  out[0] = a.command;
  out[1] = a.result;
  out[2] = a.joint;
  put_u16le(out + 3, static_cast<uint16_t>(a.value));
}

inline bool unpack_ack(const uint8_t* payload, size_t len, AckPayload* a) {
  if (len != kAckPayloadSize || payload == nullptr || a == nullptr) return false;
  a->command = payload[0];
  a->result = payload[1];
  a->joint = payload[2];
  a->value = static_cast<int16_t>(get_u16le(payload + 3));
  return true;
}

}  // namespace hg

#endif  // HG_COMMON_PROTOCOL_H_
