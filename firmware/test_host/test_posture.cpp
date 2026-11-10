// SPDX-License-Identifier: Apache-2.0
//
// Host tests for the posture state machine (firmware/src/posture.h). No framework: a failing
// check prints its location and the program exits non-zero. Run with
// firmware/test_host/run_tests.sh.

#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <utility>

#include "posture.h"

using hg::ClearResult;
using hg::Mode;
using hg::PostureStateMachine;
using hg::RequestResult;

namespace {

int g_checks = 0;
int g_failures = 0;
std::string g_context;

#define CHECK(cond)                                                                                \
  do {                                                                                             \
    ++g_checks;                                                                                    \
    if (!(cond)) {                                                                                 \
      ++g_failures;                                                                                \
      std::printf("%s:%d: [%s] CHECK(%s) failed\n", __FILE__, __LINE__, g_context.c_str(), #cond); \
    }                                                                                              \
  } while (0)

#define CHECK_EQ(a, b)                                                                       \
  do {                                                                                       \
    ++g_checks;                                                                              \
    const long long va = static_cast<long long>(a);                                          \
    const long long vb = static_cast<long long>(b);                                          \
    if (va != vb) {                                                                          \
      ++g_failures;                                                                          \
      std::printf("%s:%d: [%s] CHECK_EQ(%s, %s) failed: %lld != %lld\n", __FILE__, __LINE__, \
                  g_context.c_str(), #a, #b, va, vb);                                        \
    }                                                                                        \
  } while (0)

const Mode kAllModes[] = {Mode::kLie, Mode::kSit, Mode::kStand, Mode::kWalk, Mode::kTorqueOff};

const char* name(Mode m) {
  switch (m) {
    case Mode::kLie:
      return "LIE";
    case Mode::kSit:
      return "SIT";
    case Mode::kStand:
      return "STAND";
    case Mode::kWalk:
      return "WALK";
    case Mode::kTorqueOff:
      return "TORQUE_OFF";
  }
  return "?";
}

// The spec's allowed requests, written out independently of PostureStateMachine::is_allowed.
// TORQUE_OFF (allowed from anywhere) is checked separately.
const std::set<std::pair<Mode, Mode>> kAllowed = {
    {Mode::kLie, Mode::kSit},    {Mode::kLie, Mode::kStand},  {Mode::kSit, Mode::kLie},
    {Mode::kSit, Mode::kStand},  {Mode::kStand, Mode::kLie},  {Mode::kStand, Mode::kSit},
    {Mode::kStand, Mode::kWalk}, {Mode::kWalk, Mode::kStand}, {Mode::kTorqueOff, Mode::kLie},
};

uint32_t expected_duration(Mode from, Mode to) {
  return (from == Mode::kLie && to == Mode::kStand) ? 1600u : 1000u;
}

// Drives the state machine in 1 ms steps: a control tick every step and, while enabled, a
// heartbeat every 50 ms (the bridge's 20 Hz).
struct Rig {
  PostureStateMachine sm;
  uint32_t now;
  bool heartbeats_on = false;
  uint32_t last_hb = 0;

  explicit Rig(uint32_t start_ms = 10000) : now(start_ms) {}

  void heartbeat() {
    sm.on_heartbeat(now);
    last_hb = now;
  }
  void connect() {
    heartbeats_on = true;
    heartbeat();
  }
  // Stops heartbeats; returns the time the loss will be detected (last heartbeat + 500 ms).
  uint32_t stop_heartbeats() {
    heartbeats_on = false;
    return last_hb + hg::kHeartbeatTimeoutMs;
  }
  void resume_heartbeats() { connect(); }

  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
      ++now;
      if (heartbeats_on && now - last_hb >= 50) heartbeat();
      sm.tick(now);
    }
  }
  // Runs until now == ref + offset (wrap-safe; the target must not be in the past).
  void run_until(uint32_t ref, uint32_t offset) {
    const uint32_t target = ref + offset;
    const uint32_t ahead = target - now;
    CHECK(ahead < 0x80000000u);
    run(ahead);
  }

  RequestResult request(Mode m) { return sm.request_mode(static_cast<uint8_t>(m), now); }

  // From boot (LIE, connected), reaches a stable `m` through allowed transitions.
  void go_to(Mode m) {
    switch (m) {
      case Mode::kLie:
        break;
      case Mode::kSit:
        CHECK_EQ(request(Mode::kSit), RequestResult::kAccepted);
        run(1000);
        break;
      case Mode::kStand:
        CHECK_EQ(request(Mode::kStand), RequestResult::kAccepted);
        run(1600);
        break;
      case Mode::kWalk:
        go_to(Mode::kStand);
        CHECK_EQ(request(Mode::kWalk), RequestResult::kAccepted);
        run(1000);
        break;
      case Mode::kTorqueOff:
        CHECK_EQ(request(Mode::kTorqueOff), RequestResult::kAccepted);
        break;
    }
    CHECK_EQ(sm.reported_mode(), m);
    CHECK(!sm.in_transition());
  }
};

// ---------------------------------------------------------------------------------------------
// Boot and link establishment
// ---------------------------------------------------------------------------------------------

void test_boot_state() {
  g_context = "boot";
  Rig r;
  CHECK_EQ(r.sm.reported_mode(), Mode::kLie);
  CHECK(!r.sm.in_transition());
  CHECK_EQ(r.sm.fault_bits(), hg::kFaultHeartbeatLost);
  CHECK(!r.sm.fault_latched());
  CHECK(!r.sm.torque_off_pending());

  // Before the first heartbeat: requests rejected (fault bit set), CLEAR_FAULT rejected.
  CHECK_EQ(r.request(Mode::kStand), RequestResult::kRejectedFault);
  CHECK_EQ(r.sm.rejected_requests(), 1);
  CHECK_EQ(r.sm.clear_fault(r.now), ClearResult::kRejectedLinkUnhealthy);
  CHECK_EQ(r.sm.rejected_clears(), 1);
  r.run(10000);  // No heartbeats: still LIE, bit still set but never latched, nothing scheduled.
  CHECK_EQ(r.sm.reported_mode(), Mode::kLie);
  CHECK_EQ(r.sm.fault_bits(), hg::kFaultHeartbeatLost);
  CHECK(!r.sm.fault_latched());
  CHECK(!r.sm.torque_off_pending());

  // The first heartbeat clears the bit by itself.
  r.connect();
  CHECK_EQ(r.sm.fault_bits(), 0);
  CHECK(!r.sm.fault_latched());
  CHECK_EQ(r.request(Mode::kStand), RequestResult::kAccepted);
}

void test_torque_off_before_first_heartbeat() {
  g_context = "torque_off before heartbeat";
  Rig r;
  CHECK_EQ(r.request(Mode::kTorqueOff), RequestResult::kAccepted);
  CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);
  CHECK_EQ(r.request(Mode::kLie), RequestResult::kRejectedFault);  // Still no heartbeat.
}

// ---------------------------------------------------------------------------------------------
// Transitions
// ---------------------------------------------------------------------------------------------

void test_is_allowed_matches_spec() {
  for (Mode from : kAllModes) {
    for (Mode to : kAllModes) {
      g_context = std::string("is_allowed ") + name(from) + "->" + name(to);
      const bool expected = kAllowed.count({from, to}) > 0;
      CHECK_EQ(PostureStateMachine::is_allowed(from, to), expected);
    }
  }
}

void test_allowed_transitions_and_timing() {
  for (const auto& t : kAllowed) {
    const Mode from = t.first;
    const Mode to = t.second;
    g_context = std::string("allowed ") + name(from) + "->" + name(to);
    Rig r;
    r.connect();
    r.go_to(from);
    const uint32_t start = r.now;
    const uint32_t duration = expected_duration(from, to);
    CHECK_EQ(PostureStateMachine::transition_duration_ms(from, to), duration);
    CHECK_EQ(r.request(to), RequestResult::kAccepted);
    CHECK(r.sm.in_transition());
    CHECK_EQ(r.sm.transition_target(), to);
    CHECK_EQ(r.sm.reported_mode(), from);  // Old mode until the transition completes.
    r.run_until(start, duration - 1);
    CHECK_EQ(r.sm.reported_mode(), from);
    CHECK(r.sm.in_transition());
    r.run(1);
    CHECK_EQ(r.sm.reported_mode(), to);
    CHECK(!r.sm.in_transition());
    CHECK_EQ(r.sm.rejected_requests(), 0);
  }
}

void test_illegal_and_same_mode_requests() {
  for (Mode from : kAllModes) {
    for (Mode to : kAllModes) {
      if (to == Mode::kTorqueOff) continue;  // Allowed from anywhere; tested separately.
      if (kAllowed.count({from, to}) > 0) continue;
      g_context = std::string("rejected ") + name(from) + "->" + name(to);
      Rig r;
      r.connect();
      r.go_to(from);
      const uint32_t before = r.sm.rejected_requests();
      if (to == from) {
        CHECK_EQ(r.request(to), RequestResult::kNoChange);
        CHECK_EQ(r.sm.rejected_requests(), before);
      } else {
        CHECK_EQ(r.request(to), RequestResult::kRejectedIllegal);
        CHECK_EQ(r.sm.rejected_requests(), before + 1);
      }
      CHECK_EQ(r.sm.reported_mode(), from);
      CHECK(!r.sm.in_transition());
      r.run(2000);
      CHECK_EQ(r.sm.reported_mode(), from);
    }
  }
}

void test_invalid_mode_values() {
  g_context = "invalid values";
  Rig r;
  r.connect();
  const uint8_t invalid[] = {4, 5, 6, 7, 8, 10, 0x7F, 0xFF};
  uint32_t expected = 0;
  for (uint8_t v : invalid) {
    CHECK_EQ(r.sm.request_mode(v, r.now), RequestResult::kRejectedInvalid);
    CHECK_EQ(r.sm.rejected_requests(), ++expected);
  }
  CHECK_EQ(r.sm.reported_mode(), Mode::kLie);
}

void test_requests_during_transition_rejected() {
  g_context = "busy";
  Rig r;
  r.connect();
  CHECK_EQ(r.request(Mode::kStand), RequestResult::kAccepted);
  r.run(800);
  CHECK_EQ(r.request(Mode::kSit), RequestResult::kRejectedBusy);
  CHECK_EQ(r.request(Mode::kStand), RequestResult::kRejectedBusy);  // Even the target.
  CHECK_EQ(r.request(Mode::kLie), RequestResult::kRejectedBusy);    // Even the reported mode.
  CHECK_EQ(r.sm.rejected_requests(), 3);
  CHECK_EQ(r.sm.transition_target(), Mode::kStand);
  r.run(800);
  CHECK_EQ(r.sm.reported_mode(), Mode::kStand);
}

void test_request_at_transition_end_without_tick() {
  // advance() completes an elapsed transition before handling a request, independent of ticks.
  g_context = "request at transition end";
  PostureStateMachine sm;
  sm.on_heartbeat(0);
  CHECK_EQ(sm.request_mode(static_cast<uint8_t>(Mode::kSit), 0), RequestResult::kAccepted);
  sm.on_heartbeat(400);
  sm.on_heartbeat(800);
  CHECK_EQ(sm.request_mode(static_cast<uint8_t>(Mode::kStand), 999), RequestResult::kRejectedBusy);
  CHECK_EQ(sm.request_mode(static_cast<uint8_t>(Mode::kStand), 1000), RequestResult::kAccepted);
  CHECK_EQ(sm.reported_mode(), Mode::kSit);
}

void test_torque_off_from_anywhere() {
  for (Mode from : kAllModes) {
    g_context = std::string("torque_off from ") + name(from);
    Rig r;
    r.connect();
    r.go_to(from);
    const RequestResult expected =
        from == Mode::kTorqueOff ? RequestResult::kNoChange : RequestResult::kAccepted;
    CHECK_EQ(r.request(Mode::kTorqueOff), expected);
    CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);  // Immediate.
    CHECK(!r.sm.in_transition());
    CHECK_EQ(r.sm.rejected_requests(), 0);
  }
  // During every kind of transition.
  for (const auto& t : kAllowed) {
    g_context = std::string("torque_off during ") + name(t.first) + "->" + name(t.second);
    Rig r;
    r.connect();
    r.go_to(t.first);
    CHECK_EQ(r.request(t.second), RequestResult::kAccepted);
    r.run(300);
    CHECK_EQ(r.request(Mode::kTorqueOff), RequestResult::kAccepted);
    CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);
    CHECK(!r.sm.in_transition());
    r.run(3000);
    CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);  // The aborted transition never completes.
  }
}

// ---------------------------------------------------------------------------------------------
// Heartbeat loss
// ---------------------------------------------------------------------------------------------

void test_loss_detection_boundary() {
  g_context = "loss boundary";
  Rig r;
  r.connect();
  r.run(1000);
  const uint32_t loss = r.stop_heartbeats();
  r.run_until(loss, static_cast<uint32_t>(-1));  // loss - 1 ms
  CHECK(!r.sm.fault_latched());
  CHECK_EQ(r.sm.fault_bits(), 0);
  r.run(1);
  CHECK(r.sm.fault_latched());
  CHECK_EQ(r.sm.fault_bits(), hg::kFaultHeartbeatLost);
}

// Loss from a stable mode that must end in TORQUE_OFF 5 s after the loss.
void check_loss_sit_then_torque_off(Mode from) {
  g_context = std::string("loss from ") + name(from);
  Rig r;
  r.connect();
  r.go_to(from);
  r.run(200);
  const uint32_t loss = r.stop_heartbeats();
  r.run_until(loss, 0);
  CHECK(r.sm.fault_latched());
  CHECK(r.sm.torque_off_pending());
  if (from == Mode::kSit) {
    CHECK(!r.sm.in_transition());
    CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
  } else {
    CHECK(r.sm.in_transition());
    CHECK_EQ(r.sm.transition_target(), Mode::kSit);
    CHECK_EQ(r.sm.reported_mode(), from);
    r.run_until(loss, 999);
    CHECK_EQ(r.sm.reported_mode(), from);
    r.run(1);
    CHECK_EQ(r.sm.reported_mode(), Mode::kSit);  // Forced SIT takes 1.0 s.
  }
  r.run_until(loss, 4999);
  CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
  r.run(1);
  CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);  // Exactly 5 s after the loss.
  CHECK(!r.sm.torque_off_pending());
  CHECK(r.sm.fault_latched());
  r.run(10000);
  CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);
}

void test_loss_from_stable_modes() {
  check_loss_sit_then_torque_off(Mode::kStand);
  check_loss_sit_then_torque_off(Mode::kWalk);
  check_loss_sit_then_torque_off(Mode::kSit);

  for (Mode stay : {Mode::kLie, Mode::kTorqueOff}) {
    g_context = std::string("loss from ") + name(stay);
    Rig r;
    r.connect();
    r.go_to(stay);
    const uint32_t loss = r.stop_heartbeats();
    r.run_until(loss, 0);
    CHECK(r.sm.fault_latched());
    CHECK(!r.sm.torque_off_pending());
    CHECK(!r.sm.in_transition());
    r.run(20000);
    CHECK_EQ(r.sm.reported_mode(), stay);
  }
}

// Loss during a transition is decided by the transition's target.
void test_loss_during_transitions() {
  for (const auto& t : kAllowed) {
    const Mode from = t.first;
    const Mode to = t.second;
    g_context = std::string("loss during ") + name(from) + "->" + name(to);
    Rig r;
    r.connect();
    r.go_to(from);
    const uint32_t start = r.now;
    CHECK_EQ(r.request(to), RequestResult::kAccepted);
    const uint32_t loss = r.stop_heartbeats();  // Detected 450..500 ms into the transition.
    r.run_until(loss, 0);
    CHECK(r.sm.fault_latched());
    CHECK(r.sm.in_transition());

    if (to == Mode::kLie) {
      // Finish moving to LIE on the original schedule; no TORQUE_OFF.
      CHECK_EQ(r.sm.transition_target(), Mode::kLie);
      CHECK(!r.sm.torque_off_pending());
      r.run_until(start, expected_duration(from, to) - 1);
      CHECK_EQ(r.sm.reported_mode(), from);
      r.run(1);
      CHECK_EQ(r.sm.reported_mode(), Mode::kLie);
      r.run(20000);
      CHECK_EQ(r.sm.reported_mode(), Mode::kLie);
    } else if (to == Mode::kSit) {
      // Finish moving to SIT on the original schedule; TORQUE_OFF 5 s after the loss.
      CHECK_EQ(r.sm.transition_target(), Mode::kSit);
      CHECK(r.sm.torque_off_pending());
      r.run_until(start, expected_duration(from, to));
      CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
      r.run_until(loss, 4999);
      CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
      r.run(1);
      CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);
    } else {
      // Heading to STAND or WALK: abort into SIT (1.0 s from the loss); TORQUE_OFF at +5 s.
      CHECK_EQ(r.sm.transition_target(), Mode::kSit);
      CHECK(r.sm.torque_off_pending());
      CHECK_EQ(r.sm.reported_mode(), from);
      r.run_until(loss, 999);
      CHECK_EQ(r.sm.reported_mode(), from);
      r.run(1);
      CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
      r.run_until(loss, 4999);
      CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
      r.run(1);
      CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);
    }
  }
}

void test_loss_revealed_by_late_heartbeat() {
  // No tick between two heartbeats 600 ms apart: the late heartbeat must not hide the loss.
  g_context = "late heartbeat";
  PostureStateMachine sm;
  sm.on_heartbeat(0);
  CHECK_EQ(sm.request_mode(static_cast<uint8_t>(Mode::kStand), 0), RequestResult::kAccepted);
  for (uint32_t t = 50; t <= 1600; t += 50) sm.on_heartbeat(t);
  sm.tick(1600);
  CHECK_EQ(sm.reported_mode(), Mode::kStand);
  sm.on_heartbeat(2200);  // 600 ms gap.
  CHECK(sm.fault_latched());
  CHECK(sm.in_transition());
  CHECK_EQ(sm.transition_target(), Mode::kSit);
  CHECK(sm.torque_off_pending());
}

// ---------------------------------------------------------------------------------------------
// Latch and CLEAR_FAULT
// ---------------------------------------------------------------------------------------------

void test_latch_survives_resumed_heartbeats() {
  g_context = "latch";
  Rig r;
  r.connect();
  r.go_to(Mode::kStand);
  const uint32_t loss = r.stop_heartbeats();
  r.run_until(loss, 300);
  r.resume_heartbeats();
  r.run(500);
  CHECK(r.sm.heartbeat_healthy(r.now));
  CHECK(r.sm.fault_latched());
  CHECK_EQ(r.sm.fault_bits(), hg::kFaultHeartbeatLost);
  const uint32_t before = r.sm.rejected_requests();
  CHECK_EQ(r.request(Mode::kLie), RequestResult::kRejectedFault);
  CHECK_EQ(r.request(Mode::kStand), RequestResult::kRejectedFault);
  CHECK_EQ(r.sm.rejected_requests(), before + 2);

  // Heartbeats resumed but no CLEAR_FAULT: TORQUE_OFF still happens 5 s after the loss.
  r.run_until(loss, 4999);
  CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
  r.run(1);
  CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);
  CHECK(r.sm.fault_latched());

  // From TORQUE_OFF, LIE is still rejected until the fault is cleared.
  CHECK_EQ(r.request(Mode::kLie), RequestResult::kRejectedFault);
  CHECK_EQ(r.sm.clear_fault(r.now), ClearResult::kCleared);
  CHECK_EQ(r.sm.fault_bits(), 0);
  CHECK_EQ(r.request(Mode::kLie), RequestResult::kAccepted);
  r.run(1000);
  CHECK_EQ(r.sm.reported_mode(), Mode::kLie);
}

void test_clear_fault_cancels_pending_torque_off() {
  g_context = "clear cancels torque_off";
  Rig r;
  r.connect();
  r.go_to(Mode::kWalk);
  const uint32_t loss = r.stop_heartbeats();
  r.run_until(loss, 2000);
  r.resume_heartbeats();
  CHECK(r.sm.torque_off_pending());
  CHECK_EQ(r.sm.clear_fault(r.now), ClearResult::kCleared);
  CHECK(!r.sm.fault_latched());
  CHECK(!r.sm.torque_off_pending());
  CHECK_EQ(r.sm.fault_bits(), 0);
  r.run_until(loss, 10000);
  CHECK_EQ(r.sm.reported_mode(), Mode::kSit);                   // Never torqued off.
  CHECK_EQ(r.request(Mode::kStand), RequestResult::kAccepted);  // Requests accepted again.
}

void test_clear_fault_rejected_while_link_unhealthy() {
  g_context = "clear while lost";
  Rig r;
  r.connect();
  r.go_to(Mode::kSit);
  const uint32_t loss = r.stop_heartbeats();
  r.run_until(loss, 100);
  CHECK_EQ(r.sm.clear_fault(r.now), ClearResult::kRejectedLinkUnhealthy);
  CHECK_EQ(r.sm.rejected_clears(), 1);
  CHECK(r.sm.fault_latched());
  CHECK(r.sm.torque_off_pending());
  r.run_until(loss, 5000);
  CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);
}

void test_clear_fault_when_nothing_latched() {
  g_context = "clear not latched";
  Rig r;
  r.connect();
  r.go_to(Mode::kStand);
  CHECK_EQ(r.sm.clear_fault(r.now), ClearResult::kNotLatched);
  CHECK_EQ(r.sm.rejected_clears(), 0);
  CHECK_EQ(r.sm.reported_mode(), Mode::kStand);
  CHECK_EQ(r.sm.fault_bits(), 0);
}

void test_repeated_loss_does_not_postpone_torque_off() {
  g_context = "repeated loss";
  Rig r;
  r.connect();
  r.go_to(Mode::kStand);
  const uint32_t first_loss = r.stop_heartbeats();
  r.run_until(first_loss, 1000);
  r.resume_heartbeats();
  r.run(500);
  const uint32_t second_loss = r.stop_heartbeats();
  r.run_until(second_loss, 0);
  CHECK(r.sm.fault_latched());
  r.run_until(first_loss, 4999);
  CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
  r.run(1);
  CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);  // Still 5 s after the first loss.
}

void test_torque_off_request_while_latched() {
  g_context = "torque_off while latched";
  Rig r;
  r.connect();
  r.go_to(Mode::kStand);
  const uint32_t loss = r.stop_heartbeats();
  r.run_until(loss, 300);  // Forced SIT in progress.
  CHECK(r.sm.in_transition());
  CHECK_EQ(r.request(Mode::kTorqueOff), RequestResult::kAccepted);
  CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);
  CHECK(!r.sm.in_transition());
  CHECK(!r.sm.torque_off_pending());
  CHECK(r.sm.fault_latched());
}

void test_millis_wraparound() {
  g_context = "wraparound";
  Rig r(0xFFFFFFFFu - 3000u);
  r.connect();
  r.go_to(Mode::kStand);  // Completes before the wrap.
  const uint32_t loss = r.stop_heartbeats();
  r.run_until(loss, 0);
  CHECK(r.sm.fault_latched());
  r.run_until(loss, 999);
  CHECK_EQ(r.sm.reported_mode(), Mode::kStand);
  r.run(1);
  CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
  r.run_until(loss, 4999);  // Crosses the 32-bit wrap.
  CHECK(r.now < 10000u);
  CHECK_EQ(r.sm.reported_mode(), Mode::kSit);
  r.run(1);
  CHECK_EQ(r.sm.reported_mode(), Mode::kTorqueOff);
}

}  // namespace

int main() {
  test_boot_state();
  test_torque_off_before_first_heartbeat();
  test_is_allowed_matches_spec();
  test_allowed_transitions_and_timing();
  test_illegal_and_same_mode_requests();
  test_invalid_mode_values();
  test_requests_during_transition_rejected();
  test_request_at_transition_end_without_tick();
  test_torque_off_from_anywhere();
  test_loss_detection_boundary();
  test_loss_from_stable_modes();
  test_loss_during_transitions();
  test_loss_revealed_by_late_heartbeat();
  test_latch_survives_resumed_heartbeats();
  test_clear_fault_cancels_pending_torque_off();
  test_clear_fault_rejected_while_link_unhealthy();
  test_clear_fault_when_nothing_latched();
  test_repeated_loss_does_not_postpone_torque_off();
  test_torque_off_request_while_latched();
  test_millis_wraparound();

  std::printf("posture tests: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
