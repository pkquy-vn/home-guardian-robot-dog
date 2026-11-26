// SPDX-License-Identifier: Apache-2.0
//
// Home Guardian real-time controller firmware (Teensy 4.1).
//
// Implemented so far:
//   - Pi link in: HEARTBEAT, MODE, CLEAR_FAULT, CALIBRATION, JOINT_TARGET, SPEED_LIMIT. Every
//     command except HEARTBEAT is answered with an ACK.
//   - Posture mode state machine with heartbeat watchdog and heartbeat-loss response
//     (posture.h; HEARTBEAT_LOST latches, see docs/decisions/0002-heartbeat-loss-latches.md).
//   - Joint-level postures (kinematics.h, joint_motion.h): during a transition all eight joint
//     angles are interpolated to the target pose with a minimum-jerk profile.
//   - Servo bus (servo_manager.h): Feetech STS3250 servos, IDs 1..8, on Serial1 half-duplex.
//   - Servo motion and safety (joint_control.h, docs/servo-motion.md): joints that are present
//     and calibrated follow the posture or a test target, rate-limited and clamped. Everything
//     is limp at boot; torque needs an explicit SIT, STAND, WALK or JOINT_TARGET. Servo faults
//     (temperature, current, missing, torque lost) latch and remove torque.
//   - Joint calibration in the Teensy's EEPROM (calibration.h). A servo's own EEPROM is never
//     written.
//   - 200 Hz control tick: an IntervalTimer sets a flag, the work runs in loop().
//   - Pi link out: STATUS 10 Hz, JOINT_STATE 50 Hz (measured angles where a servo is present
//     and calibrated), SERVO_STATE 10 Hz per group of four servos, JOINT_STATUS 10 Hz, FAULT
//     when a servo fault latches and once per second while it stays latched.
//   - On-board LED: 1 Hz blink while the link is OK and no fault is set, 5 Hz otherwise.
// All other message types are parsed and ignored.
//
// The loop never blocks: no delay(), no waiting for the host or for a servo, and outgoing
// messages are skipped rather than queued when the link cannot take them. The one exception is
// saving the calibration (a flash write of a few milliseconds), which only happens on a
// CALIBRATION command and only while every servo is limp.

#include <Arduino.h>
#include <avr/eeprom.h>

#include <cstring>

#include "calibration.h"
#include "joint_control.h"
#include "joint_motion.h"
#include "posture.h"
#include "protocol.h"
#include "servo_manager.h"

// Serial port used for the Pi link. `Serial` is USB; change to `Serial7` (pins 28/29) for the
// hardware UART. See docs/decisions/0001-usb-serial-link.md.
#define LINK Serial

// Serial port of the servo bus: half-duplex, data on the TX pin only (Serial1: pin 1), with a
// 1 kOhm pull-up to 3.3 V.
#define SERVO_SERIAL Serial1

namespace {

constexpr uint32_t kLinkBaud = 115200;            // Ignored by USB serial; used by hardware UARTs.
constexpr uint32_t kControlPeriodUs = 5000;       // 200 Hz control tick.
constexpr uint32_t kStatusPeriodMs = 100;         // 10 Hz
constexpr uint32_t kJointStatePeriodMs = 20;      // 50 Hz
constexpr uint32_t kServoBaud = 1000000;          // Feetech STS default.
constexpr uint32_t kServoStatePeriodMs = 50;      // Two groups alternate: 10 Hz per group.
constexpr uint32_t kJointStatusPeriodMs = 100;    // 10 Hz
constexpr uint32_t kFaultPeriodMs = 1000;         // FAULT repeat while a servo fault is latched.
constexpr uint32_t kLedTogglePeriodOkMs = 500;    // 1 Hz blink
constexpr uint32_t kLedTogglePeriodLostMs = 100;  // 5 Hz blink
constexpr int kMaxBytesPerLoop = 256;             // Bound the time spent reading per loop.

// The control tick must match the period the motion rate limit is computed for.
static_assert(kControlPeriodUs == static_cast<uint32_t>(hg::kControlPeriodS * 1e6f + 0.5f),
              "kControlPeriodUs and hg::kControlPeriodS disagree");

// Where the calibration record lives in the Teensy's emulated EEPROM.
constexpr uintptr_t kCalibrationEepromAddress = 0;

// Extra buffer memory for the servo port. A goal sync write to 8 servos is 72 bytes and the
// port's own transmit buffer is 64, so without this the write would never fit and never be
// sent. Sized in uint16_t so it is large enough whichever element size the core is built with.
constexpr size_t kServoBufferExtra = 128;
uint16_t servo_tx_memory[kServoBufferExtra];
uint16_t servo_rx_memory[kServoBufferExtra];

hg::FrameParser parser;
hg::PostureStateMachine posture;
hg::JointMotion joint_motion;
uint32_t last_joint_state_ms = 0;

hg::ServoManager servos;
hg::JointController joints;
uint32_t last_servo_state_ms = 0;
uint8_t next_servo_state_group = 0;
uint32_t last_joint_status_ms = 0;
uint32_t last_fault_ms = 0;
uint16_t last_fault_bits_sent = 0;

IntervalTimer control_timer;
volatile bool control_tick_due = false;
bool control_timer_running = false;
uint32_t last_fallback_tick_ms = 0;

uint32_t last_status_ms = 0;
uint32_t last_led_toggle_ms = 0;
bool led_on = false;

void on_control_timer() { control_tick_due = true; }

// ---------------------------------------------------------------------------------------------
// Calibration storage (the Teensy's EEPROM; never a servo's)
// ---------------------------------------------------------------------------------------------

bool load_calibration(hg::JointCalibration (&calibration)[hg::kNumJoints]) {
  uint8_t record[hg::kCalibrationRecordSize];
  eeprom_read_block(record, reinterpret_cast<const void*>(kCalibrationEepromAddress),
                    sizeof(record));
  return hg::deserialize_calibration(record, calibration);
}

// Writes the calibration and reads it back. Takes a few milliseconds, part of it with interrupts
// off (flash write), so it is only called while every servo is limp. Bytes that did not change
// are not rewritten.
bool save_calibration() {
  hg::JointCalibration calibration[hg::kNumJoints];
  for (size_t i = 0; i < hg::kNumJoints; ++i) calibration[i] = joints.calibration(i);
  uint8_t record[hg::kCalibrationRecordSize];
  hg::serialize_calibration(calibration, record);
  eeprom_write_block(record, reinterpret_cast<void*>(kCalibrationEepromAddress), sizeof(record));

  uint8_t stored[hg::kCalibrationRecordSize];
  eeprom_read_block(stored, reinterpret_cast<const void*>(kCalibrationEepromAddress),
                    sizeof(stored));
  return std::memcmp(record, stored, sizeof(record)) == 0;
}

// ---------------------------------------------------------------------------------------------
// Pi link: outgoing
// ---------------------------------------------------------------------------------------------

// Sends one frame, or skips it rather than block if the host is not draining the port.
void send_frame(hg::MsgType type, const uint8_t* payload, size_t length) {
  uint8_t frame[hg::kMaxFrameSize];
  const size_t n = hg::encode_frame(type, payload, length, frame, sizeof(frame));
  if (n == 0) return;
  if (LINK.availableForWrite() < static_cast<int>(n)) return;
  LINK.write(frame, n);
}

int16_t to_centidegrees(float deg) {
  const float cdeg = deg * 100.0f;
  if (!(cdeg > -32768.0f)) return -32768;  // Also catches NaN.
  if (cdeg >= 32767.0f) return 32767;
  return static_cast<int16_t>(cdeg < 0.0f ? cdeg - 0.5f : cdeg + 0.5f);  // Round to nearest.
}

int16_t saturate_i16(int32_t value) {
  if (value > INT16_MAX) return INT16_MAX;
  if (value < INT16_MIN) return INT16_MIN;
  return static_cast<int16_t>(value);
}

void send_ack(const hg::AckPayload& ack) {
  uint8_t payload[hg::kAckPayloadSize];
  hg::pack_ack(ack, payload);
  send_frame(hg::MsgType::kAck, payload, sizeof(payload));
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
  send_frame(hg::MsgType::kStatus, payload, sizeof(payload));
}

// True if JOINT_STATE carries measured values for this joint: its servo answers and the joint
// is calibrated.
bool joint_is_measured(size_t joint) {
  return servos.samples()[joint].present && joints.calibration(joint).valid;
}

void send_joint_state(uint32_t now) {
  if (now - last_joint_state_ms < kJointStatePeriodMs) return;
  last_joint_state_ms = now;

  hg::JointStateEntry entries[hg::kNumJoints];
  const hg::RobotPose& pose = joint_motion.commanded();
  for (size_t i = 0; i < hg::kNumJoints; ++i) {
    if (joint_is_measured(i)) {
      const hg::ServoSample& sample = servos.samples()[i];
      entries[i].position_cdeg =
          to_centidegrees(hg::position_to_deg(joints.calibration(i), i, sample.position));
      entries[i].current = saturate_i16(hg::sts::current_to_milliamps(sample.current));
      entries[i].temperature =
          static_cast<int8_t>(sample.temperature > 127 ? 127 : sample.temperature);
    } else {
      // No servo, or not calibrated: the commanded posture angle; current and temperature 0.
      const hg::LegAngles& leg = pose.legs[i / 2];
      entries[i].position_cdeg = to_centidegrees(hg::is_knee_joint(i) ? leg.knee_deg : leg.hip_deg);
    }
  }
  uint8_t payload[hg::kJointStatePayloadSize];
  hg::pack_joint_state(entries, payload);
  send_frame(hg::MsgType::kJointState, payload, sizeof(payload));
}

void send_servo_state(uint32_t now) {
  if (now - last_servo_state_ms < kServoStatePeriodMs) return;
  last_servo_state_ms = now;

  // Servo IDs 1-4 and 5-8 alternate, so each group is sent at 10 Hz.
  const size_t first_index = next_servo_state_group * hg::kServoStateGroupSize;
  next_servo_state_group ^= 1;
  hg::ServoStateEntry group[hg::kServoStateGroupSize];
  for (size_t i = 0; i < hg::kServoStateGroupSize; ++i) group[i] = servos.servo(first_index + i);

  uint8_t payload[hg::kServoStatePayloadSize];
  hg::pack_servo_state(static_cast<uint8_t>(first_index + 1), group, payload);
  send_frame(hg::MsgType::kServoState, payload, sizeof(payload));
}

void send_joint_status(uint32_t now) {
  if (now - last_joint_status_ms < kJointStatusPeriodMs) return;
  last_joint_status_ms = now;

  hg::JointStatusEntry entries[hg::kNumJoints];
  for (size_t i = 0; i < hg::kNumJoints; ++i) {
    const hg::JointCalibration& calibration = joints.calibration(i);
    const hg::ServoSample& sample = servos.samples()[i];
    uint8_t flags = 0;
    if (calibration.valid) flags |= hg::kJointCalibrated;
    if (calibration.direction < 0) flags |= hg::kJointDirectionNegative;
    if (sample.present && sample.torque_enabled) flags |= hg::kJointTorqueOn;
    if (joints.driven(i)) flags |= hg::kJointDriven;
    if (joints.has_test_target(i)) flags |= hg::kJointTestTarget;
    if (joint_is_measured(i)) flags |= hg::kJointMeasured;
    entries[i].flags = flags;
    entries[i].block = static_cast<uint8_t>(joints.block(i));
    entries[i].reference = calibration.reference;
    entries[i].target_cdeg = to_centidegrees(joints.target_deg(i));
  }
  const uint16_t speed_limit = static_cast<uint16_t>(joints.speed_limit_deg_s() * 10.0f + 0.5f);
  uint8_t payload[hg::kJointStatusPayloadSize];
  hg::pack_joint_status(speed_limit, entries, payload);
  send_frame(hg::MsgType::kJointStatus, payload, sizeof(payload));
}

// One FAULT per latched servo fault: at once when the set of faults changes, then every second.
void send_faults(uint32_t now) {
  const uint16_t bits = joints.fault_bits();
  if (bits == 0) {
    last_fault_bits_sent = 0;
    return;
  }
  if (bits == last_fault_bits_sent && now - last_fault_ms < kFaultPeriodMs) return;
  last_fault_bits_sent = bits;
  last_fault_ms = now;
  for (unsigned bit = 0; bit < 16; ++bit) {
    hg::FaultPayload cause;
    if (!joints.fault_cause(static_cast<uint16_t>(1u << bit), &cause)) continue;
    uint8_t payload[hg::kFaultPayloadSize];
    hg::pack_fault(cause, payload);
    send_frame(hg::MsgType::kFault, payload, sizeof(payload));
  }
}

// ---------------------------------------------------------------------------------------------
// Pi link: incoming
// ---------------------------------------------------------------------------------------------

hg::AckResult ack_for(hg::RequestResult result) {
  switch (result) {
    case hg::RequestResult::kAccepted:
      return hg::AckResult::kOk;
    case hg::RequestResult::kNoChange:
      return hg::AckResult::kNoChange;
    case hg::RequestResult::kRejectedInvalid:
      return hg::AckResult::kBadRequest;
    case hg::RequestResult::kRejectedFault:
      return hg::AckResult::kFaultLatched;
    case hg::RequestResult::kRejectedBusy:
      return hg::AckResult::kBusy;
    case hg::RequestResult::kRejectedIllegal:
      return hg::AckResult::kNotAllowed;
  }
  return hg::AckResult::kBadRequest;
}

void handle_mode(uint32_t now, hg::AckPayload* ack) {
  uint8_t mode = 0;
  if (!hg::unpack_mode(parser.payload(), parser.length(), &mode)) return;
  ack->value = mode;
  const hg::RequestResult result = posture.request_mode(mode, now);
  ack->result = static_cast<uint8_t>(ack_for(result));
  // An accepted request, or a repeat of the mode already held, is the explicit command that
  // gives the joints torque (SIT, STAND, WALK) or takes it away (TORQUE_OFF).
  if (result == hg::RequestResult::kAccepted || result == hg::RequestResult::kNoChange) {
    joints.on_mode_command(static_cast<hg::Mode>(mode), now);
  }
}

void handle_clear_fault(uint32_t now, hg::AckPayload* ack) {
  if (parser.length() != hg::kClearFaultPayloadSize) return;
  const hg::ClearResult result = posture.clear_fault(now);
  if (result == hg::ClearResult::kRejectedLinkUnhealthy) {
    ack->result = static_cast<uint8_t>(hg::AckResult::kLinkUnhealthy);
    return;
  }
  const bool had_servo_fault = joints.clear_faults();
  posture.set_external_faults(joints.fault_bits(), now);
  const bool cleared = result == hg::ClearResult::kCleared || had_servo_fault;
  ack->result = static_cast<uint8_t>(cleared ? hg::AckResult::kOk : hg::AckResult::kNoChange);
}

void handle_calibration(hg::AckPayload* ack) {
  hg::CalibrationPayload request;
  if (!hg::unpack_calibration(parser.payload(), parser.length(), &request)) return;
  ack->joint = request.joint;
  int16_t reference = 0;
  hg::AckResult result = joints.calibrate(request, &reference);
  ack->value = reference;
  if (result == hg::AckResult::kOk || result == hg::AckResult::kRangeError) {
    // The calibration changed. calibrate() only allows that while every servo is limp.
    if (!save_calibration()) result = hg::AckResult::kStorageError;
  }
  ack->result = static_cast<uint8_t>(result);
}

void handle_joint_target(uint32_t now, hg::AckPayload* ack) {
  uint8_t joint = 0;
  int16_t angle_cdeg = 0;
  if (!hg::unpack_joint_target(parser.payload(), parser.length(), &joint, &angle_cdeg)) return;
  if (joint >= hg::kNumJoints) return;
  ack->joint = joint;
  if (posture.fault_bits() != 0) {
    ack->result = static_cast<uint8_t>(hg::AckResult::kFaultLatched);
    return;
  }
  if (posture.reported_mode() == hg::Mode::kTorqueOff) {
    ack->result = static_cast<uint8_t>(hg::AckResult::kNotAllowed);  // Request LIE first.
    return;
  }
  float clamped_deg = 0.0f;
  const hg::AckResult result =
      joints.set_test_target(joint, static_cast<float>(angle_cdeg) * 0.01f, now, &clamped_deg);
  ack->value = to_centidegrees(clamped_deg);
  ack->result = static_cast<uint8_t>(result);
}

void handle_speed_limit(hg::AckPayload* ack) {
  uint16_t speed_ddeg_s = 0;
  if (!hg::unpack_speed_limit(parser.payload(), parser.length(), &speed_ddeg_s)) return;
  ack->value = static_cast<int16_t>(joints.set_speed_limit(speed_ddeg_s));
  ack->result = static_cast<uint8_t>(hg::AckResult::kOk);
}

void handle_frame() {
  const uint32_t now = millis();
  const hg::MsgType type = static_cast<hg::MsgType>(parser.type());
  if (type == hg::MsgType::kHeartbeat) {
    uint32_t sequence = 0;
    if (hg::unpack_heartbeat(parser.payload(), parser.length(), &sequence)) {
      // The sequence number is not checked yet; any well-formed heartbeat feeds the watchdog.
      posture.on_heartbeat(now);
    }
    return;
  }

  // Commands: each handler leaves kBadRequest in place when the payload is malformed.
  hg::AckPayload ack;
  ack.command = parser.type();
  ack.result = static_cast<uint8_t>(hg::AckResult::kBadRequest);
  switch (type) {
    case hg::MsgType::kMode:
      handle_mode(now, &ack);
      break;
    case hg::MsgType::kClearFault:
      handle_clear_fault(now, &ack);
      break;
    case hg::MsgType::kCalibration:
      handle_calibration(&ack);
      break;
    case hg::MsgType::kJointTarget:
      handle_joint_target(now, &ack);
      break;
    case hg::MsgType::kSpeedLimit:
      handle_speed_limit(&ack);
      break;
    default:
      return;  // Not implemented: ignored, no ACK.
  }
  send_ack(ack);
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

// ---------------------------------------------------------------------------------------------
// Control tick
// ---------------------------------------------------------------------------------------------

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
  if (due) {
    const uint32_t now = millis();
    posture.tick(now);
    joint_motion.update(posture, now);
    // Servo safety checks and the joint commands for this tick; servo faults then go to the
    // posture state machine, which answers them (TORQUE_OFF or LIE) and refuses requests.
    joints.update(now, posture, joint_motion.commanded());
    posture.set_external_faults(joints.fault_bits(), now);
    // Hands this tick's servo writes to the bus and returns at once.
    servos.on_control_tick(joints.plan(), now, micros());
  }
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
  LINK.begin(kLinkBaud);                       // Does not wait for the host to open the port.
  joint_motion.init(posture.reported_mode());  // Boot pose: LIE.

  // Buffer memory must be added before the port is opened.
  SERVO_SERIAL.addMemoryForRead(servo_rx_memory, kServoBufferExtra);
  SERVO_SERIAL.addMemoryForWrite(servo_tx_memory, kServoBufferExtra);
  SERVO_SERIAL.begin(kServoBaud, SERIAL_HALF_DUPLEX);
  servos.begin(SERVO_SERIAL, kServoBaud);
  // The one switch that lets the firmware write to the servos (torque enable and goals only).
  servos.set_writes_enabled(true);

  // A missing or damaged calibration record leaves every joint uncalibrated: no torque.
  hg::JointCalibration calibration[hg::kNumJoints];
  if (!load_calibration(calibration)) {
    for (size_t i = 0; i < hg::kNumJoints; ++i) {
      calibration[i] = hg::JointCalibration();
      calibration[i].direction = hg::default_direction(i);
    }
  }
  joints.begin(servos.samples(), calibration);

  control_timer_running = control_timer.begin(on_control_timer, kControlPeriodUs);
}

void loop() {
  read_link();
  servos.poll(millis(), micros());  // Advances the servo bus; never waits.
  run_control_tick();
  const uint32_t now = millis();
  send_status(now);
  send_joint_state(now);
  send_servo_state(now);
  send_joint_status(now);
  send_faults(now);
  update_led(now);
}
