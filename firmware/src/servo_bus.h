// SPDX-License-Identifier: Apache-2.0
//
// Feetech servo bus: one request/reply transaction at a time on a half-duplex serial port, as a
// non-blocking state machine.
//
// Never blocks:
//   - start_*() refuses (returns false) unless the port can take the whole request at once, so
//     the port's write() never waits for buffer space;
//   - poll() reads only bytes that have already arrived, a bounded number per call;
//   - a transaction ends on its reply or about 1 ms after the request has been sent, whichever
//     comes first. Nothing waits for the servo.
// Not for use from an interrupt: call everything from loop().
//
// Never moves a servo unless told to: WRITE and SYNC WRITE are refused until
// set_writes_enabled(true). Nothing calls that in S1, so the firmware only sends PING and READ.

#ifndef HG_FIRMWARE_SERVO_BUS_H_
#define HG_FIRMWARE_SERVO_BUS_H_

#include <Arduino.h>

#include "servo_packet.h"

namespace hg {

class ServoBus {
 public:
  enum class Result : uint8_t {
    kIdle,      // No transaction in progress.
    kBusy,      // Waiting for the reply (or for the request to finish sending).
    kOk,        // Reply received and it matches the request (or a sync write was sent).
    kNoReply,   // No valid reply within the timeout.
    kBadReply,  // A valid packet arrived, but from another ID or with another length.
  };

  // `port` must already be open in half-duplex mode at `baud`.
  void begin(Stream& port, uint32_t baud);

  bool busy() const { return state_ != State::kIdle; }

  // Each start_*() sends one request and returns true, or returns false without sending when a
  // transaction is in progress, the arguments are invalid, or the port cannot take the request
  // without blocking. `now_us` is micros().
  bool start_ping(uint8_t id, uint32_t now_us);
  bool start_read(uint8_t id, uint8_t address, uint8_t count, uint32_t now_us);
  // Refused unless writes are enabled (see set_writes_enabled).
  bool start_write(uint8_t id, uint8_t address, const uint8_t* data, uint8_t count,
                   uint32_t now_us);
  // Broadcast, no reply. Refused unless writes are enabled. A sync write to 8 servos is longer
  // than the Teensy's default 64-byte transmit buffer: S2 must add transmit memory to the port.
  bool start_sync_write(uint8_t address, uint8_t bytes_per_servo, const uint8_t* ids,
                        const uint8_t* data, uint8_t servo_count, uint32_t now_us);

  // Safety interlock for everything that can change a servo's registers. Off by default.
  void set_writes_enabled(bool enabled) { writes_enabled_ = enabled; }
  bool writes_enabled() const { return writes_enabled_; }

  // Advances the transaction. Returns kBusy while it runs, its outcome (kOk, kNoReply or
  // kBadReply) exactly once when it ends, and kIdle otherwise. Call on every loop().
  Result poll(uint32_t now_us);

  // The reply of the last transaction that ended with kOk; valid until the next start_*().
  uint8_t reply_status() const { return parser_.status(); }
  const uint8_t* reply_data() const { return parser_.data(); }
  uint8_t reply_length() const { return parser_.data_length(); }

 private:
  enum class State : uint8_t { kIdle, kWaitReply, kWaitSent };

  bool transmit(size_t size, bool expect_reply, uint8_t reply_id, uint8_t reply_length,
                uint32_t now_us);

  Stream* port_ = nullptr;
  uint32_t us_per_byte_ = 10;
  bool writes_enabled_ = false;

  State state_ = State::kIdle;
  uint32_t start_us_ = 0;
  uint32_t timeout_us_ = 0;
  uint8_t expected_id_ = 0;
  uint8_t expected_length_ = 0;

  uint8_t request_[sts::kMaxPacketSize] = {};
  sts::ReplyParser parser_;
};

}  // namespace hg

#endif  // HG_FIRMWARE_SERVO_BUS_H_
