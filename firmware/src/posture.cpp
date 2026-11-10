// SPDX-License-Identifier: Apache-2.0
//
// Posture mode state machine and heartbeat-loss response. Behaviour is specified in posture.h.

#include "posture.h"

namespace hg {

namespace {

// Wrap-safe elapsed time.
uint32_t elapsed_ms(uint32_t since_ms, uint32_t now_ms) { return now_ms - since_ms; }

}  // namespace

bool PostureStateMachine::is_allowed(Mode from, Mode to) {
  switch (to) {
    case Mode::kLie:
      return from == Mode::kSit || from == Mode::kStand || from == Mode::kTorqueOff;
    case Mode::kSit:
      return from == Mode::kLie || from == Mode::kStand;
    case Mode::kStand:
      return from == Mode::kLie || from == Mode::kSit || from == Mode::kWalk;
    case Mode::kWalk:
      return from == Mode::kStand;
    case Mode::kTorqueOff:
      return false;  // Handled separately: allowed from anywhere, immediate.
  }
  return false;
}

uint32_t PostureStateMachine::transition_duration_ms(Mode from, Mode to) {
  if (from == Mode::kLie && to == Mode::kStand) return kLieToStandMs;
  return kTransitionMs;
}

uint16_t PostureStateMachine::fault_bits() const {
  // Before the first heartbeat the bit is set but not latched; it clears on that heartbeat.
  return (!link_established_ || latched_) ? kFaultHeartbeatLost : 0;
}

bool PostureStateMachine::heartbeat_healthy(uint32_t now_ms) const {
  return link_established_ && elapsed_ms(last_heartbeat_ms_, now_ms) < kHeartbeatTimeoutMs;
}

void PostureStateMachine::on_heartbeat(uint32_t now_ms) {
  advance(now_ms);  // A heartbeat arriving after a gap must not hide the loss.
  link_established_ = true;
  link_lost_ = false;
  last_heartbeat_ms_ = now_ms;
}

RequestResult PostureStateMachine::request_mode(uint8_t mode, uint32_t now_ms) {
  advance(now_ms);
  if (!is_valid_mode(mode)) {
    ++rejected_requests_;
    return RequestResult::kRejectedInvalid;
  }
  const Mode requested = static_cast<Mode>(mode);

  if (requested == Mode::kTorqueOff) {
    // Always honoured: during transitions and while a fault is latched.
    if (current_ == Mode::kTorqueOff && !in_transition_) return RequestResult::kNoChange;
    enter_torque_off();
    return RequestResult::kAccepted;
  }
  if (fault_bits() != 0) {
    ++rejected_requests_;
    return RequestResult::kRejectedFault;
  }
  if (in_transition_) {
    ++rejected_requests_;
    return RequestResult::kRejectedBusy;
  }
  if (requested == current_) return RequestResult::kNoChange;
  if (!is_allowed(current_, requested)) {
    ++rejected_requests_;
    return RequestResult::kRejectedIllegal;
  }
  start_transition(requested, now_ms);
  return RequestResult::kAccepted;
}

ClearResult PostureStateMachine::clear_fault(uint32_t now_ms) {
  advance(now_ms);
  if (!heartbeat_healthy(now_ms)) {
    ++rejected_clears_;
    return ClearResult::kRejectedLinkUnhealthy;
  }
  if (!latched_) return ClearResult::kNotLatched;
  latched_ = false;
  torque_off_pending_ = false;
  return ClearResult::kCleared;
}

void PostureStateMachine::tick(uint32_t now_ms) { advance(now_ms); }

// Brings the state up to now_ms. Called first by every input, so the result does not depend on
// whether a control tick happened to run between two events.
void PostureStateMachine::advance(uint32_t now_ms) {
  update_link(now_ms);
  if (torque_off_pending_ && elapsed_ms(torque_off_loss_ms_, now_ms) >= kLossTorqueOffDelayMs) {
    enter_torque_off();
  }
  if (in_transition_ && elapsed_ms(transition_start_ms_, now_ms) >= transition_duration_ms_) {
    current_ = target_;
    in_transition_ = false;
  }
}

void PostureStateMachine::update_link(uint32_t now_ms) {
  if (!link_established_ || link_lost_) return;
  if (elapsed_ms(last_heartbeat_ms_, now_ms) < kHeartbeatTimeoutMs) return;
  link_lost_ = true;
  latched_ = true;
  on_loss(now_ms);
}

void PostureStateMachine::on_loss(uint32_t now_ms) {
  // Decide by the posture being held, or by where an in-progress transition is heading.
  const Mode effective = in_transition_ ? target_ : current_;
  bool schedule_torque_off = false;
  switch (effective) {
    case Mode::kLie:
    case Mode::kTorqueOff:
      break;  // Stay (or finish moving to LIE).
    case Mode::kSit:
      schedule_torque_off = true;  // Stay (or finish moving to SIT).
      break;
    case Mode::kStand:
    case Mode::kWalk:
      start_transition(Mode::kSit, now_ms);  // Aborts any transition in progress.
      schedule_torque_off = true;
      break;
  }
  // A repeated loss never postpones a TORQUE_OFF that is already pending.
  if (schedule_torque_off && !torque_off_pending_) {
    torque_off_pending_ = true;
    torque_off_loss_ms_ = now_ms;
  }
}

void PostureStateMachine::start_transition(Mode to, uint32_t now_ms) {
  transition_duration_ms_ = transition_duration_ms(current_, to);
  target_ = to;
  transition_start_ms_ = now_ms;
  in_transition_ = true;
  ++transition_count_;
}

void PostureStateMachine::enter_torque_off() {
  current_ = Mode::kTorqueOff;
  target_ = Mode::kTorqueOff;
  in_transition_ = false;
  torque_off_pending_ = false;
}

}  // namespace hg
