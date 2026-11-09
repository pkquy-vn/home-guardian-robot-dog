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

// Message type IDs. Only kHeartbeat and kStatus have defined payloads so far; the others are
// reserved IDs whose payloads will be specified when they are implemented.
enum class MsgType : uint8_t {
  // Pi -> Teensy
  kHeartbeat = 0x01,   // HEARTBEAT
  kCmdVel = 0x02,      // CMD_VEL
  kBodyPose = 0x03,    // BODY_POSE
  kMode = 0x04,        // MODE
  kLed = 0x05,         // LED
  kIrLed = 0x06,       // IR_LED
  kConfig = 0x07,      // CONFIG
  kClearFault = 0x08,  // CLEAR_FAULT
  // Teensy -> Pi
  kJointState = 0x11,  // JOINT_STATE
  kImu = 0x12,         // IMU
  kOdom = 0x13,        // ODOM
  kContact = 0x14,     // CONTACT
  kBattery = 0x15,     // BATTERY
  kStatus = 0x16,      // STATUS
  kFault = 0x17,       // FAULT
};

// Fault bits reported in StatusPayload::fault_bits.
constexpr uint16_t kFaultHeartbeatLost = 1u << 0;  // HEARTBEAT_LOST

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

}  // namespace hg

#endif  // HG_COMMON_PROTOCOL_H_
