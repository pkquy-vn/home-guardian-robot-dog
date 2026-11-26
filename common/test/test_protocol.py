#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Host-side cross-check of the two protocol implementations.

Compiles common/test/protocol_tool.cpp (a thin CLI over common/include/protocol.h) with g++ and
checks it against the Python codec in ros2_ws/src/hg_teensy_bridge/hg_teensy_bridge/protocol.py
(used by the teensy_bridge node and tools/test_link.py), in both directions.

Run from anywhere:
    python3 common/test/test_protocol.py

Needs g++ with C++17 and Python 3.9+; no third-party packages. Set CXX to use another compiler.
"""

from __future__ import annotations

import importlib.util
import os
import random
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
INCLUDE_DIR = REPO_ROOT / "common" / "include"
TOOL_SOURCE = Path(__file__).resolve().parent / "protocol_tool.cpp"
PY_CODEC = REPO_ROOT / "ros2_ws" / "src" / "hg_teensy_bridge" / "hg_teensy_bridge" / "protocol.py"

CXX = os.environ.get("CXX", "g++")
CXX_FLAGS = [
    "-std=c++17",
    "-O1",
    "-g",
    "-Wall",
    "-Wextra",
    "-Wpedantic",
    "-Wconversion",
    "-Wsign-conversion",
    "-Wshadow",
    "-Werror",
]
SANITIZER_FLAGS = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"]


def load_python_protocol():
    spec = importlib.util.spec_from_file_location("hg_protocol", PY_CODEC)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


py = load_python_protocol()
_tmpdir: tempfile.TemporaryDirectory | None = None
_tool: Path | None = None


def setUpModule() -> None:  # noqa: N802 (unittest hook name)
    global _tmpdir, _tool
    if shutil.which(CXX) is None:
        raise unittest.SkipTest(f"{CXX} not found")
    _tmpdir = tempfile.TemporaryDirectory(prefix="hg_protocol_test_")
    _tool = Path(_tmpdir.name) / "protocol_tool"
    base = [CXX, *CXX_FLAGS, f"-I{INCLUDE_DIR}", str(TOOL_SOURCE), "-o", str(_tool)]
    # Prefer a sanitizer build; fall back if the toolchain or platform does not support it.
    result = subprocess.run(base[:1] + SANITIZER_FLAGS + base[1:], capture_output=True, text=True)
    if result.returncode != 0:
        result = subprocess.run(base, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"compiling {TOOL_SOURCE.name} failed:\n{result.stderr}")


def tearDownModule() -> None:  # noqa: N802
    if _tmpdir is not None:
        _tmpdir.cleanup()


def cpp(*args: str, stdin: str | None = None, check: bool = True) -> str:
    result = subprocess.run(
        [str(_tool), *args], input=stdin, capture_output=True, text=True, check=False
    )
    if check and result.returncode != 0:
        raise AssertionError(f"protocol_tool {args[0]} failed: {result.stdout}{result.stderr}")
    return result.stdout.strip()


def cpp_encode(msg_type: int, payload: bytes) -> bytes:
    return bytes.fromhex(cpp("encode", str(msg_type), payload.hex()))


def cpp_parse(stream: bytes) -> tuple[list[tuple[int, bytes]], tuple[int, int, int, int]]:
    frames = []
    counters = None
    for line in cpp("parse", stdin=stream.hex()).splitlines():
        fields = line.split(" ")
        if fields[0] == "frame":
            frames.append((int(fields[1], 16), bytes.fromhex(fields[2] if len(fields) > 2 else "")))
        elif fields[0] == "counters":
            counters = tuple(int(x) for x in fields[1:])
    assert counters is not None
    return frames, counters


def py_parse(stream: bytes) -> tuple[list[tuple[int, bytes]], tuple[int, int, int, int]]:
    parser = py.FrameParser()
    frames = parser.feed(stream)
    counters = (parser.frames_ok, parser.crc_errors, parser.length_errors, parser.dropped_bytes)
    return frames, counters


def flip_bit(frame: bytes, index: int, bit: int = 0) -> bytes:
    data = bytearray(frame)
    data[index] ^= 1 << bit
    return bytes(data)


# Frames used by several tests: varied types and lengths, including 0xA5 inside a payload.
SAMPLE_FRAMES = [
    (py.HEARTBEAT, py.pack_heartbeat(0)),
    (py.HEARTBEAT, py.pack_heartbeat(0x12345678)),
    (py.STATUS, py.pack_status(1, 2, 0x0001)),
    (py.CLEAR_FAULT, b""),
    (py.MODE, b"\x03"),
    (py.CMD_VEL, bytes([0xA5, 0x00, 0xA5, 0xFF, 0x10])),
    (py.JOINT_STATE, bytes(range(64))),
    (py.FAULT, bytes([0xA5] * 64)),
]


class CrcTest(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(py.crc16_ccitt_false(b"123456789"), 0x29B1)
        self.assertEqual(cpp("crc", b"123456789".hex()), "29b1")

    def test_empty_input_is_init_value(self):
        self.assertEqual(py.crc16_ccitt_false(b""), 0xFFFF)
        self.assertEqual(cpp("crc", ""), "ffff")


class EncodeTest(unittest.TestCase):
    def test_both_sides_encode_identically(self):
        for msg_type, payload in SAMPLE_FRAMES:
            with self.subTest(msg_type=msg_type, length=len(payload)):
                self.assertEqual(cpp_encode(msg_type, payload), py.encode_frame(msg_type, payload))

    def test_layout_and_little_endian_crc(self):
        frame = py.encode_frame(py.HEARTBEAT, py.pack_heartbeat(1))
        self.assertEqual(frame[:3], bytes([0xA5, 0x01, 0x04]))
        self.assertEqual(frame[3:7], bytes([0x01, 0x00, 0x00, 0x00]))
        crc = py.crc16_ccitt_false(frame[1:7])
        self.assertEqual(frame[7:], bytes([crc & 0xFF, crc >> 8]))

    def test_oversize_payload_rejected(self):
        self.assertEqual(cpp("encode", "1", bytes(65).hex(), check=False), "rejected")
        with self.assertRaises(ValueError):
            py.encode_frame(py.HEARTBEAT, bytes(65))


class PayloadTest(unittest.TestCase):
    def test_heartbeat_little_endian(self):
        self.assertEqual(py.pack_heartbeat(0x12345678), bytes.fromhex("78563412"))
        self.assertEqual(cpp("pack-heartbeat", "0x12345678"), "78563412")
        self.assertEqual(cpp("unpack-heartbeat", "78563412"), str(0x12345678))
        self.assertEqual(py.unpack_heartbeat(bytes.fromhex("78563412")), 0x12345678)

    def test_status_little_endian(self):
        self.assertEqual(py.pack_status(3, 7, 0x8001), bytes.fromhex("03070180"))
        self.assertEqual(cpp("pack-status", "3", "7", "0x8001"), "03070180")
        self.assertEqual(cpp("unpack-status", "03070180"), f"3 7 {0x8001}")
        self.assertEqual(py.unpack_status(bytes.fromhex("03070180")), (3, 7, 0x8001))

    def test_wrong_payload_size_rejected(self):
        self.assertEqual(cpp("unpack-status", "030701"), "invalid")
        self.assertEqual(cpp("unpack-heartbeat", "7856341200"), "invalid")
        self.assertIsNone(py.unpack_status(b"\x03\x07\x01"))
        self.assertIsNone(py.unpack_heartbeat(b"\x00" * 5))

    def test_mode_payload(self):
        for mode in (0, 1, 2, 3, 9, 7, 255):
            with self.subTest(mode=mode):
                self.assertEqual(cpp("pack-mode", str(mode)), py.pack_mode(mode).hex())
                self.assertEqual(cpp("unpack-mode", f"{mode:02x}"), str(mode))
                self.assertEqual(py.unpack_mode(bytes([mode])), mode)
        self.assertEqual(cpp("unpack-mode", ""), "invalid")
        self.assertEqual(cpp("unpack-mode", "0200"), "invalid")
        self.assertIsNone(py.unpack_mode(b""))
        self.assertIsNone(py.unpack_mode(b"\x02\x00"))
        with self.assertRaises(ValueError):
            py.pack_mode(256)

    def test_mode_values_agree(self):
        cpp_modes = [int(v) for v in cpp("valid-modes").split()]
        self.assertEqual(cpp_modes, sorted(py.MODE_NAMES))
        self.assertEqual(
            (py.MODE_LIE, py.MODE_SIT, py.MODE_STAND, py.MODE_WALK, py.MODE_TORQUE_OFF),
            (0, 1, 2, 3, 9),
        )

    def test_mode_and_clear_fault_frames(self):
        cases = [(py.MODE, py.pack_mode(m)) for m in sorted(py.MODE_NAMES)]
        cases.append((py.CLEAR_FAULT, b""))
        for msg_type, payload in cases:
            with self.subTest(msg_type=msg_type, payload=payload.hex()):
                frame = py.encode_frame(msg_type, payload)
                self.assertEqual(cpp_encode(msg_type, payload), frame)
                self.assertEqual(cpp_parse(frame)[0], [(msg_type, payload)])
        crc = py.crc16_ccitt_false(bytes([0x08, 0x00]))
        self.assertEqual(py.encode_frame(py.CLEAR_FAULT), bytes([0xA5, 0x08, 0x00, crc & 0xFF, crc >> 8]))

    def test_joint_state_payload(self):
        joints = [
            (4123, 0, 0),
            (10557, 0, 0),
            (-4500, -1, -1),
            (15000, 32767, 127),
            (-32768, -32768, -128),
            (32767, 1234, 45),
            (0, -2, 3),
            (-1, 0, 0),
        ]
        payload = py.pack_joint_state(joints)
        self.assertEqual(len(payload), py.JOINT_STATE_PAYLOAD_SIZE)
        self.assertEqual(len(payload), 40)
        args = [str(v) for joint in joints for v in joint]
        self.assertEqual(cpp("pack-joint-state", *args), payload.hex())
        self.assertEqual(cpp("unpack-joint-state", payload.hex()).split(), args)
        self.assertEqual(py.unpack_joint_state(payload), joints)
        frame = py.encode_frame(py.JOINT_STATE, payload)
        self.assertEqual(cpp_encode(py.JOINT_STATE, payload), frame)
        self.assertEqual(cpp("unpack-joint-state", payload[:-1].hex()), "invalid")
        self.assertIsNone(py.unpack_joint_state(payload + b"\x00"))
        with self.assertRaises(ValueError):
            py.pack_joint_state(joints[:7])

    def test_servo_state_payload(self):
        servos = [
            (True, 2048, 0, 0, 0, 121, 31, 0),
            (True, -32767, -1200, -1000, -3, 255, 255, 0x3F),
            (False, 0, 0, 0, 0, 0, 0, 0),
            (True, 32767, 32767, 1000, 461, 74, 70, 0x01),
        ]
        for first_id in (1, 5):
            payload = py.pack_servo_state(first_id, servos)
            self.assertEqual(len(payload), 49)
            self.assertEqual(len(payload), py.SERVO_STATE_PAYLOAD_SIZE)
            args = [str(first_id)] + [str(int(v)) for servo in servos for v in servo]
            self.assertEqual(cpp("pack-servo-state", *args), payload.hex())
            self.assertEqual(cpp("unpack-servo-state", payload.hex()).split(), args)
            self.assertEqual(py.unpack_servo_state(payload), (first_id, servos))
            frame = py.encode_frame(py.SERVO_STATE, payload)
            self.assertEqual(cpp_encode(py.SERVO_STATE, payload), frame)
        self.assertEqual(py.SERVO_STATE, 0x18)
        payload = py.pack_servo_state(1, servos)
        # First entry on the wire: flags, then little-endian int16 fields, then three bytes.
        self.assertEqual(payload[:13].hex(), "01" + "01" + "0008" + "0000" * 3 + "791f00")
        bad_first_ids = (bytes([0]) + payload[1:], bytes([2]) + payload[1:])
        for bad in (payload[:-1], payload + b"\x00", *bad_first_ids):
            self.assertEqual(cpp("unpack-servo-state", bad.hex()), "invalid")
            self.assertIsNone(py.unpack_servo_state(bad))
        with self.assertRaises(ValueError):
            py.pack_servo_state(3, servos)

    def check_payload(self, name, py_pack, py_unpack, values, wire_hex, msg_type, size):
        """One pack/unpack case: both sides agree with each other and with the wire bytes."""
        payload = py_pack(*values)
        args = [str(v) for v in values]
        self.assertEqual(payload.hex(), wire_hex)
        self.assertEqual(len(payload), size)
        self.assertEqual(cpp(f"pack-{name}", *args), wire_hex)
        self.assertEqual(cpp(f"unpack-{name}", wire_hex).split(), args)
        self.assertEqual(py_unpack(payload), tuple(values))
        self.assertEqual(cpp_encode(msg_type, payload), py.encode_frame(msg_type, payload))
        for bad in (payload[:-1], payload + b"\x00"):
            self.assertEqual(cpp(f"unpack-{name}", bad.hex()), "invalid")
            self.assertIsNone(py_unpack(bad))

    def test_calibration_payload(self):
        self.assertEqual((py.CALIBRATION, py.CALIBRATION_PAYLOAD_SIZE), (0x09, 5))
        self.assertEqual((py.CAL_CAPTURE, py.CAL_SET_DIRECTION, py.CAL_RESTORE), (1, 2, 3))
        cases = [
            ((py.CAL_CAPTURE, 1, 1, 0), "0101010000"),
            ((py.CAL_SET_DIRECTION, 7, -1, 0), "0207ff0000"),
            ((py.CAL_RESTORE, 0, -1, 2047), "0300ffff07"),
            ((py.CAL_RESTORE, 6, 1, -2), "030601feff"),
        ]
        for values, wire_hex in cases:
            with self.subTest(values=values):
                self.check_payload(
                    "calibration", py.pack_calibration, py.unpack_calibration, values, wire_hex,
                    py.CALIBRATION, 5,
                )

    def test_joint_target_payload(self):
        self.assertEqual((py.JOINT_TARGET, py.JOINT_TARGET_PAYLOAD_SIZE), (0x0A, 3))
        cases = [((1, 10000), "011027"), ((0, -4500), "006cee"), ((7, 0), "070000")]
        for values, wire_hex in cases:
            with self.subTest(values=values):
                self.check_payload(
                    "joint-target", py.pack_joint_target, py.unpack_joint_target, values, wire_hex,
                    py.JOINT_TARGET, 3,
                )

    def test_speed_limit_payload(self):
        self.assertEqual((py.SPEED_LIMIT, py.SPEED_LIMIT_PAYLOAD_SIZE), (0x0B, 2))
        for speed, wire_hex in [(600, "5802"), (0, "0000"), (65535, "ffff")]:
            with self.subTest(speed=speed):
                self.assertEqual(py.pack_speed_limit(speed).hex(), wire_hex)
                self.assertEqual(cpp("pack-speed-limit", str(speed)), wire_hex)
                self.assertEqual(cpp("unpack-speed-limit", wire_hex), str(speed))
                self.assertEqual(py.unpack_speed_limit(bytes.fromhex(wire_hex)), speed)
        self.assertEqual(cpp("unpack-speed-limit", "58"), "invalid")
        self.assertIsNone(py.unpack_speed_limit(b"\x58\x02\x00"))

    def test_fault_payload(self):
        self.assertEqual((py.FAULT, py.FAULT_PAYLOAD_SIZE), (0x17, 5))
        cases = [
            ((py.FAULT_SERVO_OVERCURRENT, 1, 3900), "0800013c0f"),
            ((py.FAULT_SERVO_OVERHEAT, 0, 71), "0400004700"),
            ((py.FAULT_SERVO_MISSING, py.NO_JOINT, -1), "1000ffffff"),
        ]
        for values, wire_hex in cases:
            with self.subTest(values=values):
                self.check_payload(
                    "fault", py.pack_fault, py.unpack_fault, values, wire_hex, py.FAULT, 5
                )
        self.assertEqual(
            (py.FAULT_HEARTBEAT_LOST, py.FAULT_SERVO_HOT, py.FAULT_SERVO_TORQUE_LOST), (1, 2, 32)
        )

    def test_joint_status_payload(self):
        joints = [
            (py.JOINT_CALIBRATED | py.JOINT_TORQUE_ON | py.JOINT_DRIVEN | py.JOINT_MEASURED, 0,
             2047, 0),
            (py.JOINT_CALIBRATED | py.JOINT_DIRECTION_NEGATIVE | py.JOINT_TEST_TARGET, 3,
             1024, 10000),
            (0, 1, 0, 604),
            (0, 2, -1, -4500),
            (255, 255, 32767, 32767),
            (1, 5, -32768, -32768),
            (0, 4, 4095, 15000),
            (py.JOINT_DIRECTION_NEGATIVE, 1, 0, 3522),
        ]
        payload = py.pack_joint_status(600, joints)
        self.assertEqual((py.JOINT_STATUS, len(payload), py.JOINT_STATUS_PAYLOAD_SIZE),
                         (0x19, 50, 50))
        # Speed limit, then the first entry: flags, block, little-endian reference and target.
        self.assertEqual(payload[:14].hex(), "5802" + "2d00ff070000" + "1303" + "0004" + "1027")
        args = ["600"] + [str(v) for joint in joints for v in joint]
        self.assertEqual(cpp("pack-joint-status", *args), payload.hex())
        self.assertEqual(cpp("unpack-joint-status", payload.hex()).split(), args)
        self.assertEqual(py.unpack_joint_status(payload), (600, joints))
        self.assertEqual(cpp_encode(py.JOINT_STATUS, payload),
                         py.encode_frame(py.JOINT_STATUS, payload))
        self.assertEqual(cpp("unpack-joint-status", payload[:-1].hex()), "invalid")
        self.assertIsNone(py.unpack_joint_status(payload + b"\x00"))
        with self.assertRaises(ValueError):
            py.pack_joint_status(600, joints[:7])

    def test_ack_payload(self):
        self.assertEqual((py.ACK, py.ACK_PAYLOAD_SIZE), (0x1A, 5))
        cases = [
            ((py.MODE, py.ACK_OK, py.NO_JOINT, 2), "0400ff0200"),
            ((py.CALIBRATION, py.ACK_RANGE_ERROR, 1, 2047), "090b01ff07"),
            ((py.JOINT_TARGET, 8, 0, -4500), "0a08006cee"),
            ((py.SPEED_LIMIT, py.ACK_OK, py.NO_JOINT, 600), "0b00ff5802"),
        ]
        for values, wire_hex in cases:
            with self.subTest(values=values):
                self.check_payload("ack", py.pack_ack, py.unpack_ack, values, wire_hex, py.ACK, 5)

    def test_fault_bit_names(self):
        self.assertEqual(py.decode_fault_bits(0), "none")
        self.assertEqual(py.decode_fault_bits(0x0001), "HEARTBEAT_LOST")
        self.assertEqual(py.decode_fault_bits(0x8001), "HEARTBEAT_LOST,BIT15")


class CrossParseTest(unittest.TestCase):
    def test_python_frames_parse_in_cpp(self):
        stream = b"".join(py.encode_frame(t, p) for t, p in SAMPLE_FRAMES)
        frames, counters = cpp_parse(stream)
        self.assertEqual(frames, SAMPLE_FRAMES)
        self.assertEqual(counters, (len(SAMPLE_FRAMES), 0, 0, 0))

    def test_cpp_frames_parse_in_python(self):
        stream = b"".join(cpp_encode(t, p) for t, p in SAMPLE_FRAMES)
        frames, counters = py_parse(stream)
        self.assertEqual(frames, SAMPLE_FRAMES)
        self.assertEqual(counters, (len(SAMPLE_FRAMES), 0, 0, 0))


class ErrorHandlingTest(unittest.TestCase):
    """Each case runs through both parsers, with input built by the other side's encoder."""

    def check_both(self, build, expected_frames, crc_errors=None, length_errors=None):
        cases = {
            "cpp parses python": (cpp_parse, build(py.encode_frame)),
            "python parses cpp": (py_parse, build(cpp_encode)),
        }
        for name, (parse, stream) in cases.items():
            with self.subTest(name):
                frames, (ok, crc, length, _dropped) = parse(stream)
                self.assertEqual(frames, expected_frames)
                self.assertEqual(ok, len(expected_frames))
                if crc_errors is not None:
                    self.assertEqual(crc, crc_errors)
                if length_errors is not None:
                    self.assertEqual(length, length_errors)

    def test_corrupted_payload_rejected_and_counted(self):
        a = (py.HEARTBEAT, py.pack_heartbeat(1))
        b = (py.HEARTBEAT, py.pack_heartbeat(2))
        c = (py.HEARTBEAT, py.pack_heartbeat(3))
        self.check_both(
            lambda enc: enc(*a) + flip_bit(enc(*b), 4) + enc(*c), [a, c], crc_errors=1
        )

    def test_corrupted_crc_rejected_and_counted(self):
        a = (py.STATUS, py.pack_status(0, 0, 1))
        b = (py.STATUS, py.pack_status(0, 0, 0))
        self.check_both(lambda enc: flip_bit(enc(*a), 7, 3) + enc(*b), [b], crc_errors=1)

    def test_corrupted_length_resynchronizes(self):
        # Length 4 -> 0: the shortened candidate fails its CRC and the parser resynchronizes.
        a = (py.HEARTBEAT, py.pack_heartbeat(7))
        b = (py.HEARTBEAT, py.pack_heartbeat(8))
        self.check_both(lambda enc: flip_bit(enc(*a), 2, 2) + enc(*b), [b], crc_errors=1)

    def test_recovers_after_junk(self):
        junk = bytes([0x00, 0xFF, 0x13, 0xA5, 0x37, 0xA5, 0xA5, 0x02, 0x10, 0x55])
        self.check_both(
            lambda enc: junk + b"".join(enc(t, p) for t, p in SAMPLE_FRAMES), SAMPLE_FRAMES
        )

    def test_false_start_does_not_swallow_real_frames(self):
        # A stray 0xA5 that claims a 64-byte payload spans the next several real frames; when its
        # CRC fails, the parser must rescan and still deliver every real frame.
        frames = [(py.HEARTBEAT, py.pack_heartbeat(n)) for n in range(10)]
        self.check_both(
            lambda enc: bytes([0xA5, 0x01, 0x40]) + b"".join(enc(t, p) for t, p in frames),
            frames,
            crc_errors=1,
        )

    def test_oversize_length_rejected_and_counted(self):
        a = (py.HEARTBEAT, py.pack_heartbeat(42))
        self.check_both(
            lambda enc: bytes([0xA5, 0x01, 0x41]) + enc(*a), [a], crc_errors=0, length_errors=1
        )

    def test_truncated_frame_at_end_is_held(self):
        a = (py.HEARTBEAT, py.pack_heartbeat(5))
        self.check_both(lambda enc: enc(*a) + enc(*a)[:-1], [a], crc_errors=0, length_errors=0)


class RandomizedStreamTest(unittest.TestCase):
    def test_parsers_agree_on_noisy_streams(self):
        rng = random.Random(0x5EED)
        stream = bytearray()
        intact = []
        for _ in range(400):
            if rng.random() < 0.2:  # Junk, rich in start bytes.
                junk_len = rng.randrange(1, 12)
                stream += bytes(rng.choice([0xA5, rng.randrange(256)]) for _ in range(junk_len))
            msg_type = rng.randrange(256)
            payload = bytes(rng.randrange(256) for _ in range(rng.randrange(py.MAX_PAYLOAD + 1)))
            frame = py.encode_frame(msg_type, payload)
            if rng.random() < 0.15:
                frame = flip_bit(frame, rng.randrange(1, len(frame)), rng.randrange(8))
            else:
                intact.append((msg_type, payload))
            stream += frame

        cpp_frames, cpp_counters = cpp_parse(bytes(stream))
        py_frames, py_counters = py_parse(bytes(stream))
        self.assertEqual(cpp_frames, py_frames)
        self.assertEqual(cpp_counters, py_counters)
        self.assertGreater(py_counters[1], 0, "expected some CRC errors in a noisy stream")

        # Every frame that was sent intact must be delivered, in order.
        remaining = iter(py_frames)
        for frame in intact:
            self.assertIn(frame, remaining)


if __name__ == "__main__":
    unittest.main(verbosity=2)
