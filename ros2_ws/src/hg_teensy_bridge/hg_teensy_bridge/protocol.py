# SPDX-License-Identifier: Apache-2.0
"""Home Guardian Pi <-> Teensy serial protocol: the single Python implementation.

Mirrors common/include/protocol.h. Used by the teensy_bridge node and by tools/test_link.py;
common/test/test_protocol.py checks it against the C++ header. Keep both in sync.

Pure Python, standard library only (no ROS or pyserial imports).
"""

from __future__ import annotations

import struct
from typing import Optional, Sequence

START_BYTE = 0xA5
MAX_PAYLOAD = 64
HEADER_SIZE = 3  # start, type, length
CRC_SIZE = 2
MAX_FRAME_SIZE = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE

CRC_INIT = 0xFFFF
CRC_POLY = 0x1021

# Pi -> Teensy
HEARTBEAT = 0x01
CMD_VEL = 0x02
BODY_POSE = 0x03
MODE = 0x04
LED = 0x05
IR_LED = 0x06
CONFIG = 0x07
CLEAR_FAULT = 0x08
# Teensy -> Pi
JOINT_STATE = 0x11
IMU = 0x12
ODOM = 0x13
CONTACT = 0x14
BATTERY = 0x15
STATUS = 0x16
FAULT = 0x17

# Posture modes: requested with MODE, reported in STATUS.
MODE_LIE = 0
MODE_SIT = 1
MODE_STAND = 2
MODE_WALK = 3
MODE_TORQUE_OFF = 9
MODE_NAMES = {
    MODE_LIE: "LIE",
    MODE_SIT: "SIT",
    MODE_STAND: "STAND",
    MODE_WALK: "WALK",
    MODE_TORQUE_OFF: "TORQUE_OFF",
}

FAULT_BIT_NAMES = {0: "HEARTBEAT_LOST"}

HEARTBEAT_PAYLOAD_SIZE = 4
MODE_PAYLOAD_SIZE = 1
CLEAR_FAULT_PAYLOAD_SIZE = 0
STATUS_PAYLOAD_SIZE = 4

# JOINT_STATE: 8 x (int16 position in 0.01 deg, int16 current, int8 temperature), in this order.
JOINT_ORDER = (
    "FL hip",
    "FL knee",
    "FR hip",
    "FR knee",
    "RL hip",
    "RL knee",
    "RR hip",
    "RR knee",
)
NUM_JOINTS = len(JOINT_ORDER)
JOINT_STATE_ENTRY_FORMAT = "<hhb"
JOINT_STATE_PAYLOAD_SIZE = NUM_JOINTS * struct.calcsize(JOINT_STATE_ENTRY_FORMAT)  # 40


def crc16_ccitt_false(data: bytes, crc: int = CRC_INIT) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, payload: bytes = b"") -> bytes:
    """Returns one encoded frame. Raises ValueError for an invalid type or oversize payload."""
    if not 0 <= msg_type <= 0xFF:
        raise ValueError(f"message type out of range: {msg_type}")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([msg_type, len(payload)]) + bytes(payload)
    return bytes([START_BYTE]) + body + struct.pack("<H", crc16_ccitt_false(body))


class FrameParser:
    """Stream frame parser with the same resynchronization rules as hg::FrameParser.

    A frame with a bad CRC or an oversize length is rejected and counted, and the rejected bytes
    are rescanned for the next start byte so that a real frame starting inside them is not lost.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.length_errors = 0
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        """Adds received bytes and returns the valid frames completed by them as (type, payload)."""
        frames = []
        for byte in data:
            if not self._buf and byte != START_BYTE:
                self.dropped_bytes += 1
                continue
            self._buf.append(byte)
            frame = self._poll()
            while frame is not None:
                frames.append(frame)
                frame = self._poll()
        return frames

    def _poll(self) -> Optional[tuple[int, bytes]]:
        buf = self._buf
        while len(buf) >= HEADER_SIZE:
            length = buf[2]
            if length > MAX_PAYLOAD:
                self.length_errors += 1
                self._consume(1)
                continue
            frame_size = HEADER_SIZE + length + CRC_SIZE
            if len(buf) < frame_size:
                return None
            received = buf[HEADER_SIZE + length] | (buf[HEADER_SIZE + length + 1] << 8)
            if received != crc16_ccitt_false(buf[1 : HEADER_SIZE + length]):
                self.crc_errors += 1
                self._consume(1)
                continue
            frame = (buf[1], bytes(buf[HEADER_SIZE : HEADER_SIZE + length]))
            self.frames_ok += 1
            self._consume(frame_size)
            return frame
        return None

    def _consume(self, n: int) -> None:
        """Removes n bytes from the front of the buffer, then skips to the next start byte."""
        buf = self._buf
        nxt = n
        while nxt < len(buf) and buf[nxt] != START_BYTE:
            nxt += 1
        self.dropped_bytes += nxt - n
        del buf[:nxt]


def pack_heartbeat(sequence: int) -> bytes:
    return struct.pack("<I", sequence & 0xFFFFFFFF)


def unpack_heartbeat(payload: bytes) -> Optional[int]:
    if len(payload) != HEARTBEAT_PAYLOAD_SIZE:
        return None
    return struct.unpack("<I", payload)[0]


def pack_mode(mode: int) -> bytes:
    """MODE payload: uint8 requested mode. Raises ValueError outside 0..255."""
    if not 0 <= mode <= 0xFF:
        raise ValueError(f"mode out of range: {mode}")
    return struct.pack("<B", mode)


def unpack_mode(payload: bytes) -> Optional[int]:
    if len(payload) != MODE_PAYLOAD_SIZE:
        return None
    return payload[0]


def mode_name(mode: int) -> str:
    return MODE_NAMES.get(mode, f"UNKNOWN({mode})")


def pack_status(mode: int, gait_phase: int, fault_bits: int) -> bytes:
    return struct.pack("<BBH", mode, gait_phase, fault_bits)


def unpack_status(payload: bytes) -> Optional[tuple[int, int, int]]:
    """Returns (mode, gait_phase, fault_bits), or None if the payload size is wrong."""
    if len(payload) != STATUS_PAYLOAD_SIZE:
        return None
    return struct.unpack("<BBH", payload)


def pack_joint_state(joints: Sequence[tuple[int, int, int]]) -> bytes:
    """JOINT_STATE payload from 8 (position_cdeg, current, temperature) tuples."""
    if len(joints) != NUM_JOINTS:
        raise ValueError(f"expected {NUM_JOINTS} joints, got {len(joints)}")
    return b"".join(struct.pack(JOINT_STATE_ENTRY_FORMAT, *joint) for joint in joints)


def unpack_joint_state(payload: bytes) -> Optional[list[tuple[int, int, int]]]:
    """Returns 8 (position_cdeg, current, temperature) tuples, or None if the size is wrong."""
    if len(payload) != JOINT_STATE_PAYLOAD_SIZE:
        return None
    return list(struct.iter_unpack(JOINT_STATE_ENTRY_FORMAT, payload))


def decode_fault_bits(bits: int) -> str:
    if bits == 0:
        return "none"
    names = []
    for bit in range(16):
        if bits & (1 << bit):
            names.append(FAULT_BIT_NAMES.get(bit, f"BIT{bit}"))
    return ",".join(names)
