// SPDX-License-Identifier: Apache-2.0
//
// Servo presence and telemetry for servo IDs 1..8 (hg::kNumServos) on one ServoBus.
//
// S1: read-only. This class only starts PING and READ transactions: it never enables torque and
// never writes a goal position (and the bus refuses writes anyway, see servo_bus.h).
//
// Scheduling: on_control_tick() starts at most ONE transaction per 200 Hz control tick, and
// poll() advances it without blocking. So the bus can never stall the control loop.
//   - Boot: the first 8 ticks ping IDs 1..8 once each.
//   - Afterwards, every tick reads the feedback block (registers 56..70) of the next tracked
//     servo, round-robin: 8 servos are each refreshed 25 times per second, fewer servos more
//     often.
//   - Every 100 ms one tick is used to ping one untracked ID instead, round-robin. A missing ID
//     is normal: it is simply pinged again later.
//   - A servo whose read fails 3 times in a row is dropped (untracked and reported as not
//     present) until a ping finds it again.
// A servo is reported as present once its first read has succeeded.

#ifndef HG_FIRMWARE_SERVO_MANAGER_H_
#define HG_FIRMWARE_SERVO_MANAGER_H_

#include <Arduino.h>

#include "protocol.h"
#include "servo_bus.h"

namespace hg {

class ServoManager {
 public:
  // `port` must already be open in half-duplex mode at `baud`.
  void begin(Stream& port, uint32_t baud);

  // Call on every loop(): finishes the transaction in progress, if any. Never blocks.
  void poll(uint32_t now_us);

  // Call once per control tick from loop() (not from the timer interrupt): starts at most one
  // transaction. Never blocks.
  void on_control_tick(uint32_t now_ms, uint32_t now_us);

  // Latest telemetry of servo ID index + 1 (index 0..kNumServos-1). All zero while not present.
  const ServoStateEntry& servo(size_t index) const { return telemetry_[index]; }

 private:
  enum class Pending : uint8_t { kNone, kPing, kRead };

  uint8_t next_id(uint8_t* cursor, bool tracked) const;
  void drop(size_t index);

  ServoBus bus_;

  bool tracked_[kNumServos] = {};          // Answered a ping; being polled.
  uint8_t failed_reads_[kNumServos] = {};  // Consecutive failed reads.
  ServoStateEntry telemetry_[kNumServos];

  uint8_t boot_scan_id_ = 1;  // Next ID of the boot scan; kNumServos + 1 when it is done.
  uint8_t read_cursor_ = 0;   // Index after the servo read last.
  uint8_t ping_cursor_ = 0;   // Index after the ID pinged last.
  uint32_t last_reping_ms_ = 0;

  Pending pending_ = Pending::kNone;
  uint8_t pending_id_ = 0;
};

}  // namespace hg

#endif  // HG_FIRMWARE_SERVO_MANAGER_H_
