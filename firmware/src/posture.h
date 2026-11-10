// SPDX-License-Identifier: Apache-2.0
//
// Posture mode state machine and heartbeat-loss response (safety logic).
//
// Plain C++, no Arduino dependencies: host-tested by firmware/test_host/test_posture.cpp.
// Every input takes the current time in milliseconds (millis() on the Teensy); all time
// arithmetic is unsigned and therefore safe across the 32-bit wrap.
//
// Modes and transitions (no servos yet: timed transitions stand in for joint interpolation):
//   - Boot mode is LIE. STATUS reports the old mode until a transition completes.
//   - LIE, SIT and STAND can be requested from each other. WALK only from STAND; from WALK only
//     STAND. From TORQUE_OFF only LIE. LIE -> STAND takes 1.6 s, every other transition 1.0 s.
//   - TORQUE_OFF can be requested from anywhere, also during a transition and while a fault is
//     latched, and takes effect immediately.
//   - Any other request is rejected and counted if it is unknown or illegal, arrives during a
//     transition, or arrives while a fault bit is set. Requesting the current mode is a no-op.
//
// Heartbeat watchdog (see docs/decisions/0002-heartbeat-loss-latches.md):
//   - HEARTBEAT_LOST is set at boot and clears by itself on the first heartbeat.
//   - Once a link is established, 500 ms without a heartbeat is a loss: HEARTBEAT_LOST latches
//     and stays set after heartbeats resume, until CLEAR_FAULT arrives while heartbeats are
//     healthy.
//   - Loss response, by the mode being held or, during a transition, by its target:
//       LIE: stay in (or finish moving to) LIE.
//       SIT: stay in (or finish moving to) SIT; TORQUE_OFF 5 s after the loss.
//       STAND, WALK: move to SIT (1.0 s); TORQUE_OFF 5 s after the loss.
//       TORQUE_OFF: stay.
//   - The pending TORQUE_OFF is cancelled only by an accepted CLEAR_FAULT, not by heartbeats
//     resuming.

#ifndef HG_FIRMWARE_POSTURE_H_
#define HG_FIRMWARE_POSTURE_H_

#include <cstdint>

#include "protocol.h"

namespace hg {

constexpr uint32_t kHeartbeatTimeoutMs = 500;
constexpr uint32_t kLieToStandMs = 1600;
constexpr uint32_t kTransitionMs = 1000;
constexpr uint32_t kLossTorqueOffDelayMs = 5000;

enum class RequestResult : uint8_t {
  kAccepted,         // Transition started (or TORQUE_OFF entered).
  kNoChange,         // Requested the mode already held; nothing to do. Not counted.
  kRejectedInvalid,  // Unknown mode value.
  kRejectedFault,    // A fault bit is set.
  kRejectedBusy,     // A transition is in progress.
  kRejectedIllegal,  // Not an allowed transition from the current mode.
};

enum class ClearResult : uint8_t {
  kCleared,                // HEARTBEAT_LOST latch cleared; pending TORQUE_OFF cancelled.
  kNotLatched,             // Heartbeats healthy and nothing latched; no effect.
  kRejectedLinkUnhealthy,  // No healthy heartbeat right now; latch kept. Counted.
};

class PostureStateMachine {
 public:
  PostureStateMachine() = default;

  // Inputs. Call tick() from the 200 Hz control loop; the others when the message arrives.
  void on_heartbeat(uint32_t now_ms);
  RequestResult request_mode(uint8_t mode, uint32_t now_ms);
  ClearResult clear_fault(uint32_t now_ms);
  void tick(uint32_t now_ms);

  // Outputs.
  Mode reported_mode() const { return current_; }
  bool in_transition() const { return in_transition_; }
  Mode transition_target() const { return target_; }
  uint16_t fault_bits() const;
  bool fault_latched() const { return latched_; }
  bool torque_off_pending() const { return torque_off_pending_; }
  bool heartbeat_healthy(uint32_t now_ms) const;
  uint32_t rejected_requests() const { return rejected_requests_; }
  uint32_t rejected_clears() const { return rejected_clears_; }

  // Allowed requested transitions (TORQUE_OFF, which is allowed from anywhere, excluded).
  static bool is_allowed(Mode from, Mode to);
  static uint32_t transition_duration_ms(Mode from, Mode to);

 private:
  void advance(uint32_t now_ms);
  void update_link(uint32_t now_ms);
  void on_loss(uint32_t now_ms);
  void start_transition(Mode to, uint32_t now_ms);
  void enter_torque_off();

  Mode current_ = Mode::kLie;
  Mode target_ = Mode::kLie;
  bool in_transition_ = false;
  uint32_t transition_start_ms_ = 0;
  uint32_t transition_duration_ms_ = 0;

  bool link_established_ = false;  // At least one heartbeat since boot.
  bool link_lost_ = false;         // Currently more than kHeartbeatTimeoutMs since the last one.
  uint32_t last_heartbeat_ms_ = 0;
  bool latched_ = false;

  bool torque_off_pending_ = false;
  uint32_t torque_off_loss_ms_ = 0;  // Time of the loss that scheduled the pending TORQUE_OFF.

  uint32_t rejected_requests_ = 0;
  uint32_t rejected_clears_ = 0;
};

}  // namespace hg

#endif  // HG_FIRMWARE_POSTURE_H_
