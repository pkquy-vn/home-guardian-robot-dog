// SPDX-License-Identifier: Apache-2.0
//
// Home Guardian real-time controller firmware (Teensy 4.1).
//
// Implemented so far:
//   - HEARTBEAT in (uint32 sequence), MODE in (uint8 mode), CLEAR_FAULT in (empty).
//   - Posture mode state machine with heartbeat watchdog and heartbeat-loss response
//     (posture.h; HEARTBEAT_LOST latches, see docs/decisions/0002-heartbeat-loss-latches.md).
//     No servos yet: timed transitions stand in for joint interpolation.
//   - 200 Hz control tick: an IntervalTimer sets a flag, the work runs in loop().
//   - STATUS out at 10 Hz (uint8 mode, uint8 gait phase, uint16 fault bits).
//   - On-board LED: 1 Hz blink while the link is OK and no fault is set, 5 Hz otherwise.
// All other message types are parsed and ignored.
//
// The loop never blocks: no delay(), no waiting for the host, and STATUS is skipped rather than
// queued when the link cannot take it.

#include <Arduino.h>

#include "posture.h"
#include "protocol.h"

// Serial port used for the Pi link. `Serial` is USB; change to `Serial7` (pins 28/29) for the
// hardware UART. See docs/decisions/0001-usb-serial-link.md.
#define LINK Serial

namespace {

constexpr uint32_t kLinkBaud = 115200;            // Ignored by USB serial; used by hardware UARTs.
constexpr uint32_t kControlPeriodUs = 5000;       // 200 Hz control tick.
constexpr uint32_t kStatusPeriodMs = 100;         // 10 Hz
constexpr uint32_t kLedTogglePeriodOkMs = 500;    // 1 Hz blink
constexpr uint32_t kLedTogglePeriodLostMs = 100;  // 5 Hz blink
constexpr int kMaxBytesPerLoop = 256;             // Bound the time spent reading per loop.

hg::FrameParser parser;
hg::PostureStateMachine posture;

IntervalTimer control_timer;
volatile bool control_tick_due = false;
bool control_timer_running = false;
uint32_t last_fallback_tick_ms = 0;

uint32_t last_status_ms = 0;
uint32_t last_led_toggle_ms = 0;
bool led_on = false;

void on_control_timer() { control_tick_due = true; }

void handle_frame() {
  switch (static_cast<hg::MsgType>(parser.type())) {
    case hg::MsgType::kHeartbeat: {
      uint32_t sequence = 0;
      if (hg::unpack_heartbeat(parser.payload(), parser.length(), &sequence)) {
        // The sequence number is not checked yet; any well-formed heartbeat feeds the watchdog.
        posture.on_heartbeat(millis());
      }
      break;
    }
    case hg::MsgType::kMode: {
      uint8_t mode = 0;
      if (hg::unpack_mode(parser.payload(), parser.length(), &mode)) {
        posture.request_mode(mode, millis());  // Rejections are counted by the state machine.
      }
      break;
    }
    case hg::MsgType::kClearFault:
      if (parser.length() == hg::kClearFaultPayloadSize) posture.clear_fault(millis());
      break;
    default:
      // Not implemented yet.
      break;
  }
}

void read_link() {
  for (int i = 0; i < kMaxBytesPerLoop && LINK.available() > 0; ++i) {
    const int c = LINK.read();
    if (c < 0) break;
    if (parser.feed(static_cast<uint8_t>(c))) {
      do {
        handle_frame();
      } while (parser.poll());
    }
  }
}

void run_control_tick() {
  bool due = false;
  if (control_timer_running) {
    if (control_tick_due) {
      control_tick_due = false;  // Ticks are not counted: timing comes from millis().
      due = true;
    }
  } else if (millis() - last_fallback_tick_ms >= kControlPeriodUs / 1000) {
    // IntervalTimer unavailable: tick from millis() so the loss response still runs.
    last_fallback_tick_ms = millis();
    due = true;
  }
  if (due) posture.tick(millis());
}

void send_status(uint32_t now) {
  if (now - last_status_ms < kStatusPeriodMs) return;
  last_status_ms = now;

  hg::StatusPayload status;
  status.mode = static_cast<uint8_t>(posture.reported_mode());
  status.gait_phase = 0;  // Placeholder until gaits are implemented.
  status.fault_bits = posture.fault_bits();

  uint8_t payload[hg::kStatusPayloadSize];
  hg::pack_status(status, payload);

  uint8_t frame[hg::kMaxFrameSize];
  const size_t n =
      hg::encode_frame(hg::MsgType::kStatus, payload, sizeof(payload), frame, sizeof(frame));
  if (n == 0) return;
  // Skip this STATUS rather than block if the host is not draining the port.
  if (LINK.availableForWrite() < static_cast<int>(n)) return;
  LINK.write(frame, n);
}

void update_led(uint32_t now) {
  const uint32_t period = posture.fault_bits() != 0 ? kLedTogglePeriodLostMs : kLedTogglePeriodOkMs;
  if (now - last_led_toggle_ms < period) return;
  last_led_toggle_ms = now;
  led_on = !led_on;
  digitalWriteFast(LED_BUILTIN, led_on ? HIGH : LOW);
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, LOW);
  LINK.begin(kLinkBaud);  // Does not wait for the host to open the port.
  control_timer_running = control_timer.begin(on_control_timer, kControlPeriodUs);
}

void loop() {
  read_link();
  run_control_tick();
  const uint32_t now = millis();
  send_status(now);
  update_led(now);
}
