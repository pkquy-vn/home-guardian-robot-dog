// SPDX-License-Identifier: Apache-2.0
//
// The data exchanged between the servo bus (ServoManager) and the motion logic
// (JointController), once per control tick: what the servos last reported, and what should be
// written to them. Plain C++, no Arduino dependencies.

#ifndef HG_FIRMWARE_SERVO_IO_H_
#define HG_FIRMWARE_SERVO_IO_H_

#include <cstdint>

#include "protocol.h"

namespace hg {

// What servo ID index + 1 last reported. Everything but `sequence` is cleared when the servo
// stops answering.
struct ServoSample {
  bool present = false;         // The servo answers; the fields below are from its last reply.
  uint32_t sequence = 0;        // Incremented on every successful read; never reset.
  uint32_t time_ms = 0;         // millis() of the last successful read.
  bool torque_enabled = false;  // The servo's torque enable register is not 0.
  bool position_mode = false;   // The servo's operating mode is 0 (position servo).
  int16_t position = 0;         // Present position, counts.
  int16_t goal = 0;             // The servo's goal position register, counts.
  int16_t current = 0;          // counts (sts::kMilliampsPerCount)
  uint8_t temperature = 0;      // deg C
};

// What to write to the servos in one control tick. Indexed like ServoSample.
struct ServoBusPlan {
  bool torque_off_all = false;       // Torque off to all eight IDs, answering or not.
  bool torque_off[kNumServos] = {};  // Torque off to these servos.
  bool write_goal[kNumServos] = {};  // Goal position (and speed cap) to these servos.
  int16_t goal[kNumServos] = {};     // Goal position, counts, within 0..4095.
  uint16_t goal_speed = 1;           // The servo's own speed cap, counts per second.
  bool torque_on[kNumServos] = {};   // Torque on to these servos.
};

}  // namespace hg

#endif  // HG_FIRMWARE_SERVO_IO_H_
