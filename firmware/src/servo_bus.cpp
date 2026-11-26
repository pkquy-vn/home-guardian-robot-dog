// SPDX-License-Identifier: Apache-2.0
//
// Feetech servo bus transactions. Behaviour and guarantees: servo_bus.h.

#include "servo_bus.h"

namespace hg {

namespace {

// Time allowed for the reply after the request has been sent. A reply takes well under 0.5 ms
// at 1 Mbps.
constexpr uint32_t kReplyTimeoutUs = 1000;
// Extra time after a request that has no reply (sync write) before the bus is free again.
constexpr uint32_t kNoReplySettleUs = 200;
// Bounds on the work done in one call. The port's receive buffer is far smaller than either.
constexpr int kMaxDrainBytes = 256;
constexpr int kMaxBytesPerPoll = 128;

}  // namespace

void ServoBus::begin(Stream& port, uint32_t baud) {
  port_ = &port;
  // 10 bits per byte (8N1), rounded up.
  us_per_byte_ = baud > 0 ? (10000000u + baud - 1) / baud : 10;
  state_ = State::kIdle;
}

bool ServoBus::start_ping(uint8_t id, uint32_t now_us) {
  if (busy()) return false;
  return transmit(sts::build_ping(id, request_, sizeof(request_)), true, id, 0, now_us);
}

bool ServoBus::start_read(uint8_t id, uint8_t address, uint8_t count, uint32_t now_us) {
  if (busy()) return false;
  return transmit(sts::build_read(id, address, count, request_, sizeof(request_)), true, id, count,
                  now_us);
}

bool ServoBus::start_write(uint8_t id, uint8_t address, const uint8_t* data, uint8_t count,
                           uint32_t now_us) {
  if (!writes_enabled_ || busy()) return false;
  if (!sts::is_allowed_write(address, count, data)) return false;
  return transmit(sts::build_write(id, address, data, count, request_, sizeof(request_)), true, id,
                  0, now_us);
}

bool ServoBus::start_sync_write(uint8_t address, uint8_t bytes_per_servo, const uint8_t* ids,
                                const uint8_t* data, uint8_t servo_count, uint32_t now_us) {
  if (!writes_enabled_ || busy() || data == nullptr) return false;
  for (uint8_t i = 0; i < servo_count; ++i) {
    const uint8_t* servo_data = data + static_cast<size_t>(i) * bytes_per_servo;
    if (!sts::is_allowed_write(address, bytes_per_servo, servo_data)) return false;
  }
  return transmit(sts::build_sync_write(address, bytes_per_servo, ids, data, servo_count, request_,
                                        sizeof(request_)),
                  false, 0, 0, now_us);
}

// Sends request_[0 .. size). `size` is 0 when the builder rejected the arguments.
bool ServoBus::transmit(size_t size, bool expect_reply, uint8_t reply_id, uint8_t reply_length,
                        uint32_t now_us) {
  if (port_ == nullptr || size == 0) return false;

  // Drop anything left over from an earlier transaction (a late reply, the echo of a sync
  // write, line noise) so it cannot be taken for this reply.
  for (int i = 0; i < kMaxDrainBytes && port_->available() > 0; ++i) port_->read();

  // write() waits when the transmit buffer is full, so only send what fits right now.
  if (port_->availableForWrite() < static_cast<int>(size)) return false;
  port_->write(request_, size);

  parser_.begin(request_, size);
  expected_id_ = reply_id;
  expected_length_ = reply_length;
  start_us_ = now_us;
  timeout_us_ = static_cast<uint32_t>(size) * us_per_byte_ +
                (expect_reply ? kReplyTimeoutUs : kNoReplySettleUs);
  state_ = expect_reply ? State::kWaitReply : State::kWaitSent;
  return true;
}

ServoBus::Result ServoBus::poll(uint32_t now_us) {
  if (state_ == State::kIdle) return Result::kIdle;

  if (state_ == State::kWaitReply) {
    // Bytes already received are parsed before the timeout is checked, so a reply that arrived
    // in time still counts when loop() was late.
    for (int i = 0; i < kMaxBytesPerPoll && port_->available() > 0; ++i) {
      const int c = port_->read();
      if (c < 0) break;
      if (!parser_.feed(static_cast<uint8_t>(c))) continue;
      state_ = State::kIdle;
      const bool matches =
          parser_.id() == expected_id_ && parser_.data_length() == expected_length_;
      return matches ? Result::kOk : Result::kBadReply;
    }
  }

  if (now_us - start_us_ < timeout_us_) return Result::kBusy;  // Wrap-safe (unsigned).
  const bool was_waiting_for_reply = state_ == State::kWaitReply;
  state_ = State::kIdle;
  return was_waiting_for_reply ? Result::kNoReply : Result::kOk;
}

}  // namespace hg
