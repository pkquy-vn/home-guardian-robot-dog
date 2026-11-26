// SPDX-License-Identifier: Apache-2.0
//
// Servo I/O for servo IDs 1..8 (hg::kNumServos) on one ServoBus: presence, telemetry, and the
// writes the motion logic asks for. This class decides nothing about motion: it sends the
// ServoBusPlan it is given (joint_control.h builds it) and reports what the servos say.
//
// Scheduling. Nothing here blocks, and nothing runs in the timer interrupt. Per 200 Hz control
// tick at most these transactions are started, one after the other, each from loop() once the
// previous one has ended:
//   1. torque off (one sync write), if the plan asks for it;
//   2. goal positions (one sync write to all driven servos), if the plan has any;
//   3. torque on (one sync write), if the plan asks for it;
//   4. one read or ping:
//        - boot: the first 8 ticks ping IDs 1..8 once each;
//        - every 100 ms one untracked ID is pinged, round-robin (a missing ID is normal);
//        - otherwise the next tracked servo is read, round-robin: its operating mode once
//          (register 33), then registers 40..70 (torque enable, goal, feedback).
// With all eight servos that is under 3 ms of bus time in the worst case. A stage that cannot
// be sent is skipped for that tick, never waited for; what is left over when the next tick
// arrives is replaced by that tick's plan.
//
// A servo is reported as present once its first read of registers 40..70 has succeeded. A servo
// whose read fails 3 times in a row is dropped (reported as not present) until a ping finds it
// again.

#ifndef HG_FIRMWARE_SERVO_MANAGER_H_
#define HG_FIRMWARE_SERVO_MANAGER_H_

#include <Arduino.h>

#include "protocol.h"
#include "servo_bus.h"
#include "servo_io.h"

namespace hg {

class ServoManager {
 public:
  // `port` must already be open in half-duplex mode at `baud`.
  void begin(Stream& port, uint32_t baud);

  // Allows writes to the servos (torque enable and goals). Off until this is called.
  void set_writes_enabled(bool enabled) { bus_.set_writes_enabled(enabled); }

  // Call on every loop(): finishes the transaction in progress and starts the next one that is
  // due. Never blocks.
  void poll(uint32_t now_ms, uint32_t now_us);

  // Call once per control tick from loop() (not from the timer interrupt) with the writes for
  // this tick. Never blocks.
  void on_control_tick(const ServoBusPlan& plan, uint32_t now_ms, uint32_t now_us);

  // Latest telemetry of servo ID index + 1 (index 0..kNumServos-1). All zero while not present.
  const ServoStateEntry& servo(size_t index) const { return telemetry_[index]; }
  // kNumServos samples for the motion logic; the pointer stays valid.
  const ServoSample* samples() const { return samples_; }

 private:
  enum class Pending : uint8_t { kNone, kPing, kReadMode, kReadPoll, kWrite };

  uint8_t next_id(uint8_t* cursor, bool tracked) const;
  void drop(size_t index);
  void failed_read(size_t index);
  void finish(ServoBus::Result result, uint32_t now_ms);
  void start_next(uint32_t now_us);
  bool start_torque_write(uint8_t value, const bool (&selected)[kNumServos], bool all,
                          uint32_t now_us);
  bool start_goal_write(uint32_t now_us);
  void start_read(uint32_t now_us);

  ServoBus bus_;

  bool tracked_[kNumServos] = {};          // Answered a ping; being read.
  bool mode_known_[kNumServos] = {};       // Its operating mode has been read.
  uint8_t mode_[kNumServos] = {};          // Operating mode register.
  uint8_t failed_reads_[kNumServos] = {};  // Consecutive failed reads.
  ServoStateEntry telemetry_[kNumServos];
  ServoSample samples_[kNumServos];

  uint8_t boot_scan_id_ = 1;  // Next ID of the boot scan; kNumServos + 1 when it is done.
  uint8_t read_cursor_ = 0;   // Index after the servo read last.
  uint8_t ping_cursor_ = 0;   // Index after the ID pinged last.
  uint32_t last_reping_ms_ = 0;
  uint32_t tick_ms_ = 0;

  // This tick's work that has not been started yet.
  ServoBusPlan plan_;
  bool torque_off_due_ = false;
  bool goal_due_ = false;
  bool torque_on_due_ = false;
  bool read_due_ = false;

  Pending pending_ = Pending::kNone;
  uint8_t pending_id_ = 0;
};

}  // namespace hg

#endif  // HG_FIRMWARE_SERVO_MANAGER_H_
