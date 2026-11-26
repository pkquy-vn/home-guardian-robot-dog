// SPDX-License-Identifier: Apache-2.0
//
// Servo presence and telemetry. Behaviour: servo_manager.h.

#include "servo_manager.h"

#include "servo_registers.h"

namespace hg {

namespace {

constexpr uint32_t kRepingPeriodMs = 100;  // One untracked ID is pinged this often.
constexpr uint8_t kMaxFailedReads = 3;     // Consecutive failed reads before a servo is dropped.

}  // namespace

void ServoManager::begin(Stream& port, uint32_t baud) {
  bus_.begin(port, baud);
  for (size_t i = 0; i < kNumServos; ++i) drop(i);
  boot_scan_id_ = 1;
  read_cursor_ = 0;
  ping_cursor_ = 0;
  pending_ = Pending::kNone;
}

void ServoManager::drop(size_t index) {
  tracked_[index] = false;
  failed_reads_[index] = 0;
  telemetry_[index] = ServoStateEntry();
}

// Next servo ID at or after *cursor (wrapping) whose tracked state equals `tracked`, or 0 if
// there is none. Advances *cursor past the returned ID.
uint8_t ServoManager::next_id(uint8_t* cursor, bool tracked) const {
  for (size_t step = 0; step < kNumServos; ++step) {
    const size_t index = (*cursor + step) % kNumServos;
    if (tracked_[index] == tracked) {
      *cursor = static_cast<uint8_t>((index + 1) % kNumServos);
      return static_cast<uint8_t>(index + 1);
    }
  }
  return 0;
}

void ServoManager::poll(uint32_t now_us) {
  const ServoBus::Result result = bus_.poll(now_us);
  if (result == ServoBus::Result::kIdle || result == ServoBus::Result::kBusy) return;

  const Pending pending = pending_;
  pending_ = Pending::kNone;
  if (pending_id_ < 1 || pending_id_ > kNumServos) return;
  const size_t index = pending_id_ - 1u;
  const bool ok = result == ServoBus::Result::kOk;

  if (pending == Pending::kPing) {
    if (ok && !tracked_[index]) {
      tracked_[index] = true;  // Reported as present after its first successful read.
      failed_reads_[index] = 0;
    }
    return;
  }

  if (pending == Pending::kRead && tracked_[index]) {
    if (ok && bus_.reply_length() == sts::kFeedbackLength) {
      const sts::Feedback feedback = sts::decode_feedback(bus_.reply_data());
      ServoStateEntry& entry = telemetry_[index];
      entry.present = true;
      entry.position = feedback.position;
      entry.speed = feedback.speed;
      entry.load = feedback.load;
      entry.current = feedback.current;
      entry.voltage = feedback.voltage;
      entry.temperature = feedback.temperature;
      entry.status = bus_.reply_status();
      failed_reads_[index] = 0;
    } else if (++failed_reads_[index] >= kMaxFailedReads) {
      drop(index);
    }
  }
}

void ServoManager::on_control_tick(uint32_t now_ms, uint32_t now_us) {
  // One transaction per tick at most. A transaction ends about 1 ms after it starts, so the bus
  // is normally free again by the next tick; if it is not, this tick is skipped.
  if (bus_.busy()) return;

  uint8_t id = 0;
  Pending kind = Pending::kNone;
  if (boot_scan_id_ <= kNumServos) {
    id = boot_scan_id_++;
    kind = Pending::kPing;
    last_reping_ms_ = now_ms;
  } else if (now_ms - last_reping_ms_ >= kRepingPeriodMs &&
             (id = next_id(&ping_cursor_, false)) != 0) {
    kind = Pending::kPing;
    last_reping_ms_ = now_ms;
  } else if ((id = next_id(&read_cursor_, true)) != 0) {
    kind = Pending::kRead;
  }
  if (kind == Pending::kNone) return;

  const bool started = kind == Pending::kPing
                           ? bus_.start_ping(id, now_us)
                           : bus_.start_read(id, sts::kFeedbackStart, sts::kFeedbackLength, now_us);
  if (started) {
    pending_ = kind;
    pending_id_ = id;
  }
}

}  // namespace hg
