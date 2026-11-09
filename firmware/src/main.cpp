// SPDX-License-Identifier: Apache-2.0
//
// Home Guardian real-time controller firmware (Teensy 4.1).
//
// Implemented so far:
//   - HEARTBEAT in (uint32 sequence) with a 500 ms watchdog. Fault bit HEARTBEAT_LOST is set at
//     boot and whenever no valid heartbeat has arrived for 500 ms; it clears when heartbeats
//     resume.
//   - STATUS out at 10 Hz (uint8 mode, uint8 gait phase, uint16 fault bits).
//   - On-board LED: 1 Hz blink while the link is OK, 5 Hz while the heartbeat is lost.
// All other message types are parsed and ignored.
//
// The loop never blocks: no delay(), no waiting for the host, and STATUS is skipped rather than
// queued when the link cannot take it.

#include <Arduino.h>

#include "protocol.h"

// Serial port used for the Pi link. `Serial` is USB; change to `Serial7` (pins 28/29) for the
// hardware UART. See docs/decisions/0001-usb-serial-link.md.
#define LINK Serial

namespace {

constexpr uint32_t kLinkBaud = 115200;  // Ignored by USB serial; used by hardware UARTs.
constexpr uint32_t kHeartbeatTimeoutMs = 500;
constexpr uint32_t kStatusPeriodMs = 100;         // 10 Hz
constexpr uint32_t kLedTogglePeriodOkMs = 500;    // 1 Hz blink
constexpr uint32_t kLedTogglePeriodLostMs = 100;  // 5 Hz blink
constexpr int kMaxBytesPerLoop = 256;             // Bound the time spent reading per loop.

hg::FrameParser parser;

bool heartbeat_received = false;
uint32_t last_heartbeat_ms = 0;

uint16_t fault_bits = hg::kFaultHeartbeatLost;  // Set at boot until the first heartbeat.
uint8_t mode = 0;                               // Placeholder until MODE is implemented.
uint8_t gait_phase = 0;                         // Placeholder until gaits are implemented.

uint32_t last_status_ms = 0;
uint32_t last_led_toggle_ms = 0;
bool led_on = false;

void handle_frame() {
  switch (static_cast<hg::MsgType>(parser.type())) {
    case hg::MsgType::kHeartbeat: {
      uint32_t sequence = 0;
      if (hg::unpack_heartbeat(parser.payload(), parser.length(), &sequence)) {
        // The sequence number is not checked yet; any well-formed heartbeat feeds the watchdog.
        heartbeat_received = true;
        last_heartbeat_ms = millis();
      }
      break;
    }
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

void update_watchdog(uint32_t now) {
  const bool lost = !heartbeat_received || (now - last_heartbeat_ms) >= kHeartbeatTimeoutMs;
  if (lost) {
    fault_bits |= hg::kFaultHeartbeatLost;
  } else {
    fault_bits &= static_cast<uint16_t>(~hg::kFaultHeartbeatLost);
  }
}

void send_status(uint32_t now) {
  if (now - last_status_ms < kStatusPeriodMs) return;
  last_status_ms = now;

  hg::StatusPayload status;
  status.mode = mode;
  status.gait_phase = gait_phase;
  status.fault_bits = fault_bits;

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
  const uint32_t period =
      (fault_bits & hg::kFaultHeartbeatLost) ? kLedTogglePeriodLostMs : kLedTogglePeriodOkMs;
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
}

void loop() {
  read_link();
  const uint32_t now = millis();
  update_watchdog(now);
  send_status(now);
  update_led(now);
}
