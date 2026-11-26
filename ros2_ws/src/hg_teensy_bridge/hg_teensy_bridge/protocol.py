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
CALIBRATION = 0x09
JOINT_TARGET = 0x0A
SPEED_LIMIT = 0x0B
# Teensy -> Pi
JOINT_STATE = 0x11
IMU = 0x12
ODOM = 0x13
CONTACT = 0x14
BATTERY = 0x15
STATUS = 0x16
FAULT = 0x17
SERVO_STATE = 0x18
JOINT_STATUS = 0x19
ACK = 0x1A

# Commands answered with an ACK, by message type.
COMMAND_NAMES = {
    MODE: "MODE",
    CLEAR_FAULT: "CLEAR_FAULT",
    CALIBRATION: "CALIBRATION",
    JOINT_TARGET: "JOINT_TARGET",
    SPEED_LIMIT: "SPEED_LIMIT",
}

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

# Fault bits in STATUS. Every fault latches until CLEAR_FAULT.
FAULT_HEARTBEAT_LOST = 1 << 0
FAULT_SERVO_HOT = 1 << 1  # >= 65 C: went to LIE
FAULT_SERVO_OVERHEAT = 1 << 2  # >= 70 C: torque off
FAULT_SERVO_OVERCURRENT = 1 << 3  # torque off
FAULT_SERVO_MISSING = 1 << 4  # a driven servo stopped answering: torque off
FAULT_SERVO_TORQUE_LOST = 1 << 5  # a servo did not take or keep torque: torque off
FAULT_BIT_NAMES = {
    0: "HEARTBEAT_LOST",
    1: "SERVO_HOT",
    2: "SERVO_OVERHEAT",
    3: "SERVO_OVERCURRENT",
    4: "SERVO_MISSING",
    5: "SERVO_TORQUE_LOST",
}
NO_JOINT = 0xFF  # "no joint" in FAULT and ACK

HEARTBEAT_PAYLOAD_SIZE = 4
MODE_PAYLOAD_SIZE = 1
CLEAR_FAULT_PAYLOAD_SIZE = 0
STATUS_PAYLOAD_SIZE = 4

# JOINT_STATE: 8 x (int16 position in 0.01 deg, int16 current in mA, int8 temperature in deg C),
# in this order. Measured values for a joint whose servo is present and calibrated (JOINT_STATUS
# flag JOINT_MEASURED); otherwise the commanded posture angle, with current and temperature 0.
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
# The same joints as identifiers, for command-line tools and files.
JOINT_KEYS = tuple(name.lower().replace(" ", "_") for name in JOINT_ORDER)
JOINT_STATE_ENTRY_FORMAT = "<hhb"
JOINT_STATE_PAYLOAD_SIZE = NUM_JOINTS * struct.calcsize(JOINT_STATE_ENTRY_FORMAT)  # 40

# SERVO_STATE: telemetry of the bus servos, IDs 1..8 (ID = JOINT_ORDER index + 1), four servos per
# message: uint8 first servo ID (1 or 5), then 4 x (uint8 flags, int16 position, int16 speed,
# int16 load, int16 current, uint8 voltage, uint8 temperature, uint8 status).
NUM_SERVOS = NUM_JOINTS
SERVO_STATE_GROUP_SIZE = 4
SERVO_STATE_ENTRY_FORMAT = "<BhhhhBBB"
SERVO_STATE_ENTRY_SIZE = struct.calcsize(SERVO_STATE_ENTRY_FORMAT)  # 12
SERVO_STATE_PAYLOAD_SIZE = 1 + SERVO_STATE_GROUP_SIZE * SERVO_STATE_ENTRY_SIZE  # 49
SERVO_FLAG_PRESENT = 0x01

# Units of the SERVO_STATE fields. Keep in sync with firmware/src/servo_registers.h.
SERVO_COUNTS_PER_REV = 4096  # position: counts, raw (no calibration)
SERVO_LOAD_PERCENT_PER_COUNT = 0.1
SERVO_VOLTS_PER_COUNT = 0.1
SERVO_MILLIAMPS_PER_COUNT = 6.5  # approximate: to be checked against a meter
# Status byte bits, from Feetech's STS memory table (to be confirmed for the STS3250).
SERVO_STATUS_BIT_NAMES = {
    0: "VOLTAGE",
    1: "SENSOR",
    2: "TEMPERATURE",
    3: "CURRENT",
    4: "ANGLE",
    5: "OVERLOAD",
}

# CALIBRATION: uint8 operation, uint8 joint, int8 direction, int16 reference (servo counts).
CAL_CAPTURE = 1  # store the servo's current position as the reference pose
CAL_SET_DIRECTION = 2  # set the direction sign (+1 or -1)
CAL_RESTORE = 3  # set direction and reference from a saved copy
CALIBRATION_FORMAT = "<BBbh"
CALIBRATION_PAYLOAD_SIZE = struct.calcsize(CALIBRATION_FORMAT)  # 5

# JOINT_TARGET: uint8 joint, int16 angle in 0.01 deg. SPEED_LIMIT: uint16 in 0.1 deg/s.
JOINT_TARGET_FORMAT = "<Bh"
JOINT_TARGET_PAYLOAD_SIZE = struct.calcsize(JOINT_TARGET_FORMAT)  # 3
SPEED_LIMIT_FORMAT = "<H"
SPEED_LIMIT_PAYLOAD_SIZE = struct.calcsize(SPEED_LIMIT_FORMAT)  # 2

# FAULT: uint16 fault (one fault bit), uint8 joint (or NO_JOINT), int16 value (deg C or mA).
FAULT_FORMAT = "<HBh"
FAULT_PAYLOAD_SIZE = struct.calcsize(FAULT_FORMAT)  # 5

# JOINT_STATUS: uint16 speed limit in 0.1 deg/s, then 8 x (uint8 flags, uint8 block,
# int16 reference in servo counts, int16 target in 0.01 deg).
JOINT_STATUS_ENTRY_FORMAT = "<BBhh"
JOINT_STATUS_ENTRY_SIZE = struct.calcsize(JOINT_STATUS_ENTRY_FORMAT)  # 6
JOINT_STATUS_PAYLOAD_SIZE = 2 + NUM_JOINTS * JOINT_STATUS_ENTRY_SIZE  # 50
JOINT_CALIBRATED = 1 << 0
JOINT_DIRECTION_NEGATIVE = 1 << 1
JOINT_TORQUE_ON = 1 << 2  # the servo reports torque on
JOINT_DRIVEN = 1 << 3  # the firmware is driving this joint
JOINT_TEST_TARGET = 1 << 4  # a JOINT_TARGET test target is active
JOINT_MEASURED = 1 << 5  # JOINT_STATE carries measured values for this joint
# block: why the joint cannot be given torque right now.
JOINT_BLOCK_NAMES = {
    0: "none",
    1: "servo not present",
    2: "not calibrated",
    3: "joint range does not fit the servo range: flip the direction or remount the horn",
    4: "servo is not in position mode",
    5: "measured angle is far outside the joint limits: check the calibration",
}

# ACK: uint8 command (message type), uint8 result, uint8 joint (or NO_JOINT), int16 value.
ACK_FORMAT = "<BBBh"
ACK_PAYLOAD_SIZE = struct.calcsize(ACK_FORMAT)  # 5
ACK_OK = 0
ACK_NO_CHANGE = 1
ACK_RANGE_ERROR = 11
ACK_RESULT_NAMES = {
    0: "OK",
    1: "no change",
    2: "bad request",
    3: "refused: a fault is latched, clear it first",
    4: "refused: a posture transition is in progress",
    5: "refused: not allowed in the current mode",
    6: "refused: a servo has torque (calibration only changes while everything is limp)",
    7: "refused: the servo is not present",
    8: "refused: the joint is not calibrated",
    9: "refused: no healthy heartbeat",
    10: "calibration is in use but could not be saved to the Teensy's EEPROM",
    11: "joint range does not fit the servo range: flip the direction or remount the horn",
    12: "refused: the servo is not in position mode",
    13: "refused: measured angle is far outside the joint limits, check the calibration",
}


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


def is_valid_servo_group(first_id: int) -> bool:
    """True for the first servo ID of a SERVO_STATE group: 1 or 5."""
    return (
        1 <= first_id <= NUM_SERVOS - SERVO_STATE_GROUP_SIZE + 1
        and (first_id - 1) % SERVO_STATE_GROUP_SIZE == 0
    )


def pack_servo_state(first_id: int, servos: Sequence[tuple]) -> bytes:
    """SERVO_STATE payload from the group's first servo ID and 4 tuples of
    (present, position, speed, load, current, voltage, temperature, status)."""
    if not is_valid_servo_group(first_id):
        raise ValueError(f"invalid first servo ID: {first_id}")
    if len(servos) != SERVO_STATE_GROUP_SIZE:
        raise ValueError(f"expected {SERVO_STATE_GROUP_SIZE} servos, got {len(servos)}")
    payload = bytes([first_id])
    for present, *values in servos:
        flags = SERVO_FLAG_PRESENT if present else 0
        payload += struct.pack(SERVO_STATE_ENTRY_FORMAT, flags, *values)
    return payload


def unpack_servo_state(payload: bytes) -> Optional[tuple[int, list[tuple]]]:
    """Returns (first servo ID, 4 tuples of (present, position, speed, load, current, voltage,
    temperature, status)), or None if the size or the first servo ID is wrong."""
    if len(payload) != SERVO_STATE_PAYLOAD_SIZE or not is_valid_servo_group(payload[0]):
        return None
    servos = []
    for flags, *values in struct.iter_unpack(SERVO_STATE_ENTRY_FORMAT, payload[1:]):
        servos.append((bool(flags & SERVO_FLAG_PRESENT), *values))
    return payload[0], servos


def decode_servo_status(status: int) -> str:
    if status == 0:
        return "none"
    names = []
    for bit in range(8):
        if status & (1 << bit):
            names.append(SERVO_STATUS_BIT_NAMES.get(bit, f"BIT{bit}"))
    return ",".join(names)


def decode_fault_bits(bits: int) -> str:
    if bits == 0:
        return "none"
    names = []
    for bit in range(16):
        if bits & (1 << bit):
            names.append(FAULT_BIT_NAMES.get(bit, f"BIT{bit}"))
    return ",".join(names)


def pack_calibration(operation: int, joint: int, direction: int = 1, reference: int = 0) -> bytes:
    return struct.pack(CALIBRATION_FORMAT, operation, joint, direction, reference)


def unpack_calibration(payload: bytes) -> Optional[tuple[int, int, int, int]]:
    """Returns (operation, joint, direction, reference), or None if the size is wrong."""
    if len(payload) != CALIBRATION_PAYLOAD_SIZE:
        return None
    return struct.unpack(CALIBRATION_FORMAT, payload)


def pack_joint_target(joint: int, angle_cdeg: int) -> bytes:
    return struct.pack(JOINT_TARGET_FORMAT, joint, angle_cdeg)


def unpack_joint_target(payload: bytes) -> Optional[tuple[int, int]]:
    """Returns (joint, angle_cdeg), or None if the size is wrong."""
    if len(payload) != JOINT_TARGET_PAYLOAD_SIZE:
        return None
    return struct.unpack(JOINT_TARGET_FORMAT, payload)


def pack_speed_limit(speed_ddeg_s: int) -> bytes:
    return struct.pack(SPEED_LIMIT_FORMAT, speed_ddeg_s)


def unpack_speed_limit(payload: bytes) -> Optional[int]:
    if len(payload) != SPEED_LIMIT_PAYLOAD_SIZE:
        return None
    return struct.unpack(SPEED_LIMIT_FORMAT, payload)[0]


def pack_fault(fault: int, joint: int, value: int) -> bytes:
    return struct.pack(FAULT_FORMAT, fault, joint, value)


def unpack_fault(payload: bytes) -> Optional[tuple[int, int, int]]:
    """Returns (fault bit, joint, value), or None if the size is wrong."""
    if len(payload) != FAULT_PAYLOAD_SIZE:
        return None
    return struct.unpack(FAULT_FORMAT, payload)


def pack_joint_status(
    speed_limit_ddeg_s: int, joints: Sequence[tuple[int, int, int, int]]
) -> bytes:
    """JOINT_STATUS payload from the speed limit and 8 (flags, block, reference, target_cdeg)."""
    if len(joints) != NUM_JOINTS:
        raise ValueError(f"expected {NUM_JOINTS} joints, got {len(joints)}")
    payload = struct.pack("<H", speed_limit_ddeg_s)
    return payload + b"".join(struct.pack(JOINT_STATUS_ENTRY_FORMAT, *joint) for joint in joints)


def unpack_joint_status(payload: bytes) -> Optional[tuple[int, list[tuple[int, int, int, int]]]]:
    """Returns (speed limit in 0.1 deg/s, 8 x (flags, block, reference, target_cdeg)), or None
    if the size is wrong."""
    if len(payload) != JOINT_STATUS_PAYLOAD_SIZE:
        return None
    speed_limit = struct.unpack("<H", payload[:2])[0]
    return speed_limit, list(struct.iter_unpack(JOINT_STATUS_ENTRY_FORMAT, payload[2:]))


def pack_ack(command: int, result: int, joint: int, value: int) -> bytes:
    return struct.pack(ACK_FORMAT, command, result, joint, value)


def unpack_ack(payload: bytes) -> Optional[tuple[int, int, int, int]]:
    """Returns (command, result, joint, value), or None if the size is wrong."""
    if len(payload) != ACK_PAYLOAD_SIZE:
        return None
    return struct.unpack(ACK_FORMAT, payload)


def joint_label(joint: int) -> str:
    return JOINT_ORDER[joint] if 0 <= joint < NUM_JOINTS else "-"


def ack_result_name(result: int) -> str:
    return ACK_RESULT_NAMES.get(result, f"result {result}")
