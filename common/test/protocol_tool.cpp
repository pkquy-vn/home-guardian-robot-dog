// SPDX-License-Identifier: Apache-2.0
//
// Host-side command-line wrapper around common/include/protocol.h, driven by test_protocol.py.
// Bytes are passed and printed as lowercase hex strings ("" for no bytes).
//
//   protocol_tool crc <hex>                      -> crc as 4 hex digits
//   protocol_tool encode <type> <payload-hex>    -> frame hex (exit 1 if rejected)
//   protocol_tool parse                          -> reads stream hex from stdin, prints
//                                                   "frame <type> <payload-hex>" per frame, then
//                                                   "counters <ok> <crc> <length> <dropped>"
//   protocol_tool pack-heartbeat <sequence>      -> payload hex
//   protocol_tool unpack-heartbeat <payload-hex> -> sequence, or "invalid"
//   protocol_tool pack-status <mode> <gait> <faults>  -> payload hex
//   protocol_tool unpack-status <payload-hex>    -> "<mode> <gait> <faults>", or "invalid"
//   protocol_tool pack-mode <mode>               -> payload hex
//   protocol_tool unpack-mode <payload-hex>      -> mode, or "invalid"
//   protocol_tool valid-modes                    -> every value 0..255 accepted by is_valid_mode
//   protocol_tool pack-joint-state <24 numbers>  -> payload hex (position, current, temp) x 8
//   protocol_tool unpack-joint-state <hex>       -> 24 numbers, or "invalid"
//
// Numeric arguments accept decimal or 0x-prefixed hex.

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "protocol.h"

namespace {

bool parse_hex(const std::string& text, std::vector<uint8_t>* out) {
  out->clear();
  std::string digits;
  for (char c : text) {
    if (c != ' ' && c != '\n' && c != '\r' && c != '\t') digits.push_back(c);
  }
  if (digits.size() % 2 != 0) return false;
  for (size_t i = 0; i < digits.size(); i += 2) {
    char* end = nullptr;
    const std::string byte = digits.substr(i, 2);
    const long value = std::strtol(byte.c_str(), &end, 16);
    if (end != byte.c_str() + 2) return false;
    out->push_back(static_cast<uint8_t>(value));
  }
  return true;
}

std::string to_hex(const uint8_t* data, size_t len) {
  static const char kDigits[] = "0123456789abcdef";
  std::string s;
  for (size_t i = 0; i < len; ++i) {
    s.push_back(kDigits[data[i] >> 4]);
    s.push_back(kDigits[data[i] & 0x0F]);
  }
  return s;
}

unsigned long parse_number(const char* text) { return std::strtoul(text, nullptr, 0); }

int usage() {
  std::fprintf(stderr, "usage: protocol_tool <command> [args...] (see source)\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) return usage();
  const std::string cmd = argv[1];
  std::vector<uint8_t> bytes;

  if (cmd == "crc" && argc == 3) {
    if (!parse_hex(argv[2], &bytes)) return usage();
    std::printf("%04x\n", hg::crc16_ccitt_false(bytes.data(), bytes.size()));
    return 0;
  }

  if (cmd == "encode" && argc == 4) {
    if (!parse_hex(argv[3], &bytes)) return usage();
    uint8_t frame[hg::kMaxFrameSize + 16];
    const size_t n = hg::encode_frame(static_cast<uint8_t>(parse_number(argv[2])),
                                      bytes.empty() ? nullptr : bytes.data(), bytes.size(), frame,
                                      sizeof(frame));
    if (n == 0) {
      std::printf("rejected\n");
      return 1;
    }
    std::printf("%s\n", to_hex(frame, n).c_str());
    return 0;
  }

  if (cmd == "parse" && argc == 2) {
    const std::string input((std::istreambuf_iterator<char>(std::cin)),
                            std::istreambuf_iterator<char>());
    if (!parse_hex(input, &bytes)) return usage();
    hg::FrameParser parser;
    for (uint8_t b : bytes) {
      if (parser.feed(b)) {
        do {
          std::printf("frame %02x %s\n", parser.type(),
                      to_hex(parser.payload(), parser.length()).c_str());
        } while (parser.poll());
      }
    }
    std::printf("counters %lu %lu %lu %lu\n", static_cast<unsigned long>(parser.frames_ok()),
                static_cast<unsigned long>(parser.crc_errors()),
                static_cast<unsigned long>(parser.length_errors()),
                static_cast<unsigned long>(parser.dropped_bytes()));
    return 0;
  }

  if (cmd == "pack-heartbeat" && argc == 3) {
    uint8_t payload[hg::kHeartbeatPayloadSize];
    hg::pack_heartbeat(static_cast<uint32_t>(parse_number(argv[2])), payload);
    std::printf("%s\n", to_hex(payload, sizeof(payload)).c_str());
    return 0;
  }

  if (cmd == "unpack-heartbeat" && argc == 3) {
    if (!parse_hex(argv[2], &bytes)) return usage();
    uint32_t sequence = 0;
    if (!hg::unpack_heartbeat(bytes.data(), bytes.size(), &sequence)) {
      std::printf("invalid\n");
      return 0;
    }
    std::printf("%lu\n", static_cast<unsigned long>(sequence));
    return 0;
  }

  if (cmd == "pack-status" && argc == 5) {
    hg::StatusPayload status;
    status.mode = static_cast<uint8_t>(parse_number(argv[2]));
    status.gait_phase = static_cast<uint8_t>(parse_number(argv[3]));
    status.fault_bits = static_cast<uint16_t>(parse_number(argv[4]));
    uint8_t payload[hg::kStatusPayloadSize];
    hg::pack_status(status, payload);
    std::printf("%s\n", to_hex(payload, sizeof(payload)).c_str());
    return 0;
  }

  if (cmd == "unpack-status" && argc == 3) {
    if (!parse_hex(argv[2], &bytes)) return usage();
    hg::StatusPayload status;
    if (!hg::unpack_status(bytes.data(), bytes.size(), &status)) {
      std::printf("invalid\n");
      return 0;
    }
    std::printf("%u %u %u\n", status.mode, status.gait_phase, status.fault_bits);
    return 0;
  }

  if (cmd == "pack-mode" && argc == 3) {
    uint8_t payload[hg::kModePayloadSize];
    hg::pack_mode(static_cast<uint8_t>(parse_number(argv[2])), payload);
    std::printf("%s\n", to_hex(payload, sizeof(payload)).c_str());
    return 0;
  }

  if (cmd == "unpack-mode" && argc == 3) {
    if (!parse_hex(argv[2], &bytes)) return usage();
    uint8_t mode = 0;
    if (!hg::unpack_mode(bytes.data(), bytes.size(), &mode)) {
      std::printf("invalid\n");
      return 0;
    }
    std::printf("%u\n", mode);
    return 0;
  }

  if (cmd == "pack-joint-state" && argc == 2 + 3 * static_cast<int>(hg::kNumJoints)) {
    hg::JointStateEntry joints[hg::kNumJoints];
    for (size_t i = 0; i < hg::kNumJoints; ++i) {
      joints[i].position_cdeg = static_cast<int16_t>(std::strtol(argv[2 + 3 * i], nullptr, 0));
      joints[i].current = static_cast<int16_t>(std::strtol(argv[3 + 3 * i], nullptr, 0));
      joints[i].temperature = static_cast<int8_t>(std::strtol(argv[4 + 3 * i], nullptr, 0));
    }
    uint8_t payload[hg::kJointStatePayloadSize];
    hg::pack_joint_state(joints, payload);
    std::printf("%s\n", to_hex(payload, sizeof(payload)).c_str());
    return 0;
  }

  if (cmd == "unpack-joint-state" && argc == 3) {
    if (!parse_hex(argv[2], &bytes)) return usage();
    hg::JointStateEntry joints[hg::kNumJoints];
    if (!hg::unpack_joint_state(bytes.data(), bytes.size(), joints)) {
      std::printf("invalid\n");
      return 0;
    }
    for (const hg::JointStateEntry& j : joints) {
      std::printf("%d %d %d ", j.position_cdeg, j.current, j.temperature);
    }
    std::printf("\n");
    return 0;
  }

  if (cmd == "valid-modes" && argc == 2) {
    for (unsigned v = 0; v <= 0xFF; ++v) {
      if (hg::is_valid_mode(static_cast<uint8_t>(v))) std::printf("%u\n", v);
    }
    return 0;
  }

  return usage();
}
