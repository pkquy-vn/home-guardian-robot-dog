// SPDX-License-Identifier: Apache-2.0
//
// Feetech SMS/STS serial packets: builders and a reply parser. Plain C++, no Arduino
// dependencies. Reference: Feetech's FTServo_Arduino library (INST.h, SCS.cpp).
//
//   Request:  FF FF ID LEN INST PARAM...   CHECKSUM     LEN = number of params + 2
//   Reply:    FF FF ID LEN STATUS DATA...  CHECKSUM     LEN = number of data bytes + 2
//   CHECKSUM = ~(ID + LEN + INST or STATUS + params or data), low 8 bits.
//
// The bus is one half-duplex wire, so the host may receive an echo of its own request before the
// servo's reply. ReplyParser skips one received packet that is byte-identical to the request.

#ifndef HG_FIRMWARE_SERVO_PACKET_H_
#define HG_FIRMWARE_SERVO_PACKET_H_

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace hg {
namespace sts {

constexpr uint8_t kHeaderByte = 0xFF;
constexpr uint8_t kBroadcastId = 0xFE;
constexpr uint8_t kMaxServoId = 0xFD;

constexpr uint8_t kInstPing = 0x01;
constexpr uint8_t kInstRead = 0x02;
constexpr uint8_t kInstWrite = 0x03;
constexpr uint8_t kInstSyncWrite = 0x83;

// Largest packet built or parsed: fits a sync write of 7 bytes to 8 servos (72 bytes).
constexpr size_t kMaxPacketSize = 80;
constexpr size_t kPacketOverhead = 6;  // FF FF ID LEN INST/STATUS ... CHECKSUM
constexpr size_t kMaxParams = kMaxPacketSize - kPacketOverhead;

// Checksum of a complete packet buffer of `size` bytes: over ID .. last param.
inline uint8_t packet_checksum(const uint8_t* packet, size_t size) {
  uint8_t sum = 0;
  for (size_t i = 2; i + 1 < size; ++i) sum = static_cast<uint8_t>(sum + packet[i]);
  return static_cast<uint8_t>(~sum);
}

// Builders write one request packet into `out` and return its size, or 0 if the arguments are
// invalid or the packet (or, for a read, its reply) would not fit in kMaxPacketSize / out_size.

inline size_t build_packet(uint8_t id, uint8_t instruction, const uint8_t* params,
                           size_t param_count, uint8_t* out, size_t out_size) {
  const size_t size = param_count + kPacketOverhead;
  if (out == nullptr || param_count > kMaxParams || size > out_size) return 0;
  if (id > kBroadcastId) return 0;
  if (param_count > 0 && params == nullptr) return 0;
  out[0] = kHeaderByte;
  out[1] = kHeaderByte;
  out[2] = id;
  out[3] = static_cast<uint8_t>(param_count + 2);
  out[4] = instruction;
  if (param_count > 0) std::memcpy(out + 5, params, param_count);
  out[size - 1] = packet_checksum(out, size);
  return size;
}

inline size_t build_ping(uint8_t id, uint8_t* out, size_t out_size) {
  if (id > kMaxServoId) return 0;
  return build_packet(id, kInstPing, nullptr, 0, out, out_size);
}

// Read `count` bytes starting at register `address`.
inline size_t build_read(uint8_t id, uint8_t address, uint8_t count, uint8_t* out,
                         size_t out_size) {
  if (id > kMaxServoId || count == 0 || count > kMaxParams) return 0;
  const uint8_t params[2] = {address, count};
  return build_packet(id, kInstRead, params, sizeof(params), out, out_size);
}

// Write `count` bytes starting at register `address`.
inline size_t build_write(uint8_t id, uint8_t address, const uint8_t* data, uint8_t count,
                          uint8_t* out, size_t out_size) {
  if (id > kMaxServoId || data == nullptr || count == 0 || count + 1u > kMaxParams) return 0;
  uint8_t params[kMaxParams];
  params[0] = address;
  std::memcpy(params + 1, data, count);
  return build_packet(id, kInstWrite, params, count + 1u, out, out_size);
}

// Sync write (broadcast, no reply): writes `bytes_per_servo` bytes starting at `address` to each
// of `servo_count` servos. `data` holds servo_count * bytes_per_servo bytes, in the order of
// `ids`.
inline size_t build_sync_write(uint8_t address, uint8_t bytes_per_servo, const uint8_t* ids,
                               const uint8_t* data, uint8_t servo_count, uint8_t* out,
                               size_t out_size) {
  if (ids == nullptr || data == nullptr || bytes_per_servo == 0 || servo_count == 0) return 0;
  const size_t param_count = 2u + static_cast<size_t>(servo_count) * (bytes_per_servo + 1u);
  if (param_count > kMaxParams) return 0;
  uint8_t params[kMaxParams];
  params[0] = address;
  params[1] = bytes_per_servo;
  size_t n = 2;
  for (uint8_t i = 0; i < servo_count; ++i) {
    if (ids[i] > kMaxServoId) return 0;
    params[n++] = ids[i];
    std::memcpy(params + n, data + static_cast<size_t>(i) * bytes_per_servo, bytes_per_servo);
    n += bytes_per_servo;
  }
  return build_packet(kBroadcastId, kInstSyncWrite, params, param_count, out, out_size);
}

// Byte-at-a-time parser for the reply to one request.
//
// Call begin() with the request just sent, then feed() every received byte. feed() returns true
// when a complete packet with a valid checksum has arrived that is not the echo of the request.
// Packets with a bad checksum or length are dropped and the parser hunts for the next header.
//
// Echo: the first received packet that is byte-identical to the request is taken to be the echo
// and skipped. Only the first: a PING reply with status 0x01 is byte-identical to the PING
// request, so on a bus that echoes, the second identical packet is the reply. (On a bus that
// does not echo, that one reply would be mistaken for the echo and the ping would time out.)
class ReplyParser {
 public:
  // `request` must stay valid and unchanged until the next begin().
  void begin(const uint8_t* request, size_t request_size) {
    request_ = request;
    request_size_ = request_size;
    echo_skipped_ = false;
    state_ = State::kHeader1;
    count_ = 0;
  }

  bool feed(uint8_t byte) {
    switch (state_) {
      case State::kHeader1:
        if (byte == kHeaderByte) state_ = State::kHeader2;
        break;
      case State::kHeader2:
        state_ = byte == kHeaderByte ? State::kId : State::kHeader1;
        break;
      case State::kId:
        if (byte == kHeaderByte) break;  // A third 0xFF: still the header.
        packet_[0] = kHeaderByte;
        packet_[1] = kHeaderByte;
        packet_[2] = byte;
        state_ = State::kLength;
        break;
      case State::kLength:
        if (byte < 2 || byte + 4u > kMaxPacketSize) {
          state_ = byte == kHeaderByte ? State::kHeader2 : State::kHeader1;
          break;
        }
        packet_[3] = byte;
        count_ = 4;
        state_ = State::kBody;
        break;
      case State::kBody:
        packet_[count_++] = byte;
        if (count_ < packet_[3] + 4u) break;
        state_ = State::kHeader1;
        if (packet_checksum(packet_, count_) != packet_[count_ - 1]) break;
        if (!echo_skipped_ && is_request()) {
          echo_skipped_ = true;
          break;
        }
        return true;
    }
    return false;
  }

  // The packet for which feed() last returned true.
  uint8_t id() const { return packet_[2]; }
  uint8_t status() const { return packet_[4]; }
  const uint8_t* data() const { return packet_ + 5; }
  uint8_t data_length() const { return static_cast<uint8_t>(packet_[3] - 2); }
  bool echo_skipped() const { return echo_skipped_; }

 private:
  enum class State : uint8_t { kHeader1, kHeader2, kId, kLength, kBody };

  bool is_request() const {
    return request_ != nullptr && count_ == request_size_ &&
           std::memcmp(packet_, request_, count_) == 0;
  }

  const uint8_t* request_ = nullptr;
  size_t request_size_ = 0;
  bool echo_skipped_ = false;
  State state_ = State::kHeader1;
  size_t count_ = 0;
  uint8_t packet_[kMaxPacketSize] = {};
};

}  // namespace sts
}  // namespace hg

#endif  // HG_FIRMWARE_SERVO_PACKET_H_
