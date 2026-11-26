// SPDX-License-Identifier: Apache-2.0
//
// Servo I/O: presence, telemetry and the planned writes. Behaviour: servo_manager.h.

#include "servo_manager.h"

#include "servo_registers.h"

namespace hg {

namespace {

constexpr uint32_t kRepingPeriodMs = 100;  // One untracked ID is pinged this often.
constexpr uint8_t kMaxFailedReads = 3;     // Consecutive failed reads before a servo is dropped.

bool any_of(const bool (&flags)[kNumServos]) {
  for (size_t i = 0; i < kNumServos; ++i) {
    if (flags[i]) return true;
  }
  return false;
}

}  // namespace

void ServoManager::begin(Stream& port, uint32_t baud) {
  bus_.begin(port, baud);
  for (size_t i = 0; i < kNumServos; ++i) drop(i);
  boot_scan_id_ = 1;
  read_cursor_ = 0;
  ping_cursor_ = 0;
  plan_ = ServoBusPlan();
  torque_off_due_ = false;
  goal_due_ = false;
  torque_on_due_ = false;
  read_due_ = false;
  pending_ = Pending::kNone;
}

void ServoManager::drop(size_t index) {
  tracked_[index] = false;
  mode_known_[index] = false;
  failed_reads_[index] = 0;
  telemetry_[index] = ServoStateEntry();
  const uint32_t sequence = samples_[index].sequence;  // Never goes back.
  samples_[index] = ServoSample();
  samples_[index].sequence = sequence;
}

void ServoManager::failed_read(size_t index) {
  if (++failed_reads_[index] >= kMaxFailedReads) drop(index);
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

void ServoManager::poll(uint32_t now_ms, uint32_t now_us) {
  const ServoBus::Result result = bus_.poll(now_us);
  if (result == ServoBus::Result::kBusy) return;
  if (result != ServoBus::Result::kIdle) finish(result, now_ms);
  start_next(now_us);
}

// Handles the end of the transaction that was in progress.
void ServoManager::finish(ServoBus::Result result, uint32_t now_ms) {
  const Pending pending = pending_;
  pending_ = Pending::kNone;
  if (pending == Pending::kWrite || pending == Pending::kNone) return;  // Nothing to learn.
  if (pending_id_ < 1 || pending_id_ > kNumServos) return;
  const size_t index = pending_id_ - 1u;
  const bool ok = result == ServoBus::Result::kOk;

  if (pending == Pending::kPing) {
    if (ok && !tracked_[index]) {
      tracked_[index] = true;  // Reported as present after its first successful read.
      mode_known_[index] = false;
      failed_reads_[index] = 0;
    }
    return;
  }
  if (!tracked_[index]) return;

  if (pending == Pending::kReadMode) {
    if (ok && bus_.reply_length() == 1) {
      mode_[index] = bus_.reply_data()[0];
      mode_known_[index] = true;
      failed_reads_[index] = 0;
    } else {
      failed_read(index);
    }
    return;
  }

  // Pending::kReadPoll
  if (!ok || bus_.reply_length() != sts::kPollLength) {
    failed_read(index);
    return;
  }
  const sts::PollData data = sts::decode_poll(bus_.reply_data());
  failed_reads_[index] = 0;

  ServoStateEntry& entry = telemetry_[index];
  entry.present = true;
  entry.position = data.feedback.position;
  entry.speed = data.feedback.speed;
  entry.load = data.feedback.load;
  entry.current = data.feedback.current;
  entry.voltage = data.feedback.voltage;
  entry.temperature = data.feedback.temperature;
  entry.status = bus_.reply_status();

  ServoSample& sample = samples_[index];
  sample.present = true;
  ++sample.sequence;
  sample.time_ms = now_ms;
  sample.torque_enabled = data.torque_enable != sts::kTorqueOff;
  sample.position_mode = mode_[index] == sts::kModePosition;
  sample.position = data.feedback.position;
  sample.goal = data.goal;
  sample.current = data.feedback.current;
  sample.temperature = data.feedback.temperature;
}

void ServoManager::on_control_tick(const ServoBusPlan& plan, uint32_t now_ms, uint32_t now_us) {
  // Whatever was left of the previous tick's work is replaced, not queued.
  plan_ = plan;
  torque_off_due_ = plan_.torque_off_all || any_of(plan_.torque_off);
  goal_due_ = any_of(plan_.write_goal);
  torque_on_due_ = any_of(plan_.torque_on);
  read_due_ = true;
  tick_ms_ = now_ms;
  start_next(now_us);
}

// Starts the next transaction that is due, if the bus is free. A stage that cannot be sent is
// skipped for this tick rather than retried, so a write can never hold up the reads.
void ServoManager::start_next(uint32_t now_us) {
  if (bus_.busy()) return;
  if (torque_off_due_) {
    torque_off_due_ = false;
    if (start_torque_write(sts::kTorqueOff, plan_.torque_off, plan_.torque_off_all, now_us)) {
      return;
    }
  }
  if (goal_due_) {
    goal_due_ = false;
    if (start_goal_write(now_us)) return;
  }
  if (torque_on_due_) {
    torque_on_due_ = false;
    if (start_torque_write(sts::kTorqueOn, plan_.torque_on, false, now_us)) return;
  }
  if (read_due_) {
    read_due_ = false;
    start_read(now_us);
  }
}

// Writes torque enable = `value` to the selected servos (`all`: to all eight IDs, answering or
// not; only ever used for torque off).
bool ServoManager::start_torque_write(uint8_t value, const bool (&selected)[kNumServos], bool all,
                                      uint32_t now_us) {
  uint8_t ids[kNumServos];
  uint8_t data[kNumServos];
  uint8_t count = 0;
  for (size_t i = 0; i < kNumServos; ++i) {
    if (!all && !selected[i]) continue;
    // Torque on only ever goes to a servo that is answering.
    if (value != sts::kTorqueOff && !samples_[i].present) continue;
    ids[count] = static_cast<uint8_t>(i + 1);
    data[count] = value;
    ++count;
  }
  if (count == 0) return false;
  if (!bus_.start_sync_write(sts::kRegTorqueEnable, 1, ids, data, count, now_us)) return false;
  pending_ = Pending::kWrite;
  return true;
}

bool ServoManager::start_goal_write(uint32_t now_us) {
  uint8_t ids[kNumServos];
  uint8_t data[kNumServos * sts::kGoalBlockLength];
  uint8_t count = 0;
  for (size_t i = 0; i < kNumServos; ++i) {
    if (!plan_.write_goal[i] || !samples_[i].present) continue;
    const int16_t goal = plan_.goal[i];
    if (goal < sts::kPositionMin || goal > sts::kPositionMax) continue;  // Never wrap a goal.
    ids[count] = static_cast<uint8_t>(i + 1);
    sts::encode_goal_block(goal, plan_.goal_speed, data + count * sts::kGoalBlockLength);
    ++count;
  }
  if (count == 0) return false;
  if (!bus_.start_sync_write(sts::kGoalBlockStart, sts::kGoalBlockLength, ids, data, count,
                             now_us)) {
    return false;
  }
  pending_ = Pending::kWrite;
  return true;
}

void ServoManager::start_read(uint32_t now_us) {
  uint8_t id = 0;
  Pending kind = Pending::kNone;
  if (boot_scan_id_ <= kNumServos) {
    id = boot_scan_id_++;
    kind = Pending::kPing;
    last_reping_ms_ = tick_ms_;
  } else if (tick_ms_ - last_reping_ms_ >= kRepingPeriodMs &&
             (id = next_id(&ping_cursor_, false)) != 0) {
    kind = Pending::kPing;
    last_reping_ms_ = tick_ms_;
  } else if ((id = next_id(&read_cursor_, true)) != 0) {
    kind = mode_known_[id - 1u] ? Pending::kReadPoll : Pending::kReadMode;
  }
  if (kind == Pending::kNone) return;

  bool started = false;
  switch (kind) {
    case Pending::kPing:
      started = bus_.start_ping(id, now_us);
      break;
    case Pending::kReadMode:
      started = bus_.start_read(id, sts::kRegOperatingMode, 1, now_us);
      break;
    default:
      started = bus_.start_read(id, sts::kPollStart, sts::kPollLength, now_us);
      break;
  }
  if (started) {
    pending_ = kind;
    pending_id_ = id;
  }
}

}  // namespace hg
