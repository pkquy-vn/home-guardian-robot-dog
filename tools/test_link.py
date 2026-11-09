#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Pi <-> Teensy serial link test.

Sends HEARTBEAT at 20 Hz, parses STATUS frames from the Teensy and prints one line per second:
heartbeats sent, STATUS frames per second, mode, decoded fault bits and the CRC error count.

Usage:
    python3 tools/test_link.py [PORT] [--baud BAUD]

PORT defaults to /dev/ttyACM0. Stop with Ctrl+C. Requires pyserial.

The protocol code below mirrors common/include/protocol.h and is imported by
common/test/test_protocol.py, which checks that both implementations agree. Keep them in sync.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Optional

# --------------------------------------------------------------------------------------------
# Protocol (mirror of common/include/protocol.h)
# --------------------------------------------------------------------------------------------

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

FAULT_BIT_NAMES = {0: "HEARTBEAT_LOST"}

HEARTBEAT_PAYLOAD_SIZE = 4
STATUS_PAYLOAD_SIZE = 4


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


def pack_status(mode: int, gait_phase: int, fault_bits: int) -> bytes:
    return struct.pack("<BBH", mode, gait_phase, fault_bits)


def unpack_status(payload: bytes) -> Optional[tuple[int, int, int]]:
    """Returns (mode, gait_phase, fault_bits), or None if the payload size is wrong."""
    if len(payload) != STATUS_PAYLOAD_SIZE:
        return None
    return struct.unpack("<BBH", payload)


def decode_fault_bits(bits: int) -> str:
    if bits == 0:
        return "none"
    names = []
    for bit in range(16):
        if bits & (1 << bit):
            names.append(FAULT_BIT_NAMES.get(bit, f"BIT{bit}"))
    return ",".join(names)


# --------------------------------------------------------------------------------------------
# Link test
# --------------------------------------------------------------------------------------------

HEARTBEAT_RATE_HZ = 20.0
REPORT_PERIOD_S = 1.0


def parse_args(argv: Optional[list[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0", help="default: /dev/ttyACM0")
    parser.add_argument(
        "--baud", type=int, default=115200, help="ignored by USB serial (default: 115200)"
    )
    return parser.parse_args(argv)


def run(port_name: str, baud: int) -> int:
    try:
        import serial  # pyserial; imported here so the protocol code above has no dependencies
    except ImportError:
        print("pyserial is required: sudo apt install python3-serial", file=sys.stderr)
        return 2

    try:
        port = serial.Serial(port_name, baud, timeout=0.005)
    except serial.SerialException as exc:
        print(f"cannot open {port_name}: {exc}", file=sys.stderr)
        return 1

    parser = FrameParser()
    hb_period = 1.0 / HEARTBEAT_RATE_HZ
    sequence = 0
    heartbeats_sent = 0
    status_in_window = 0
    last_status: Optional[tuple[int, int, int]] = None

    start = time.monotonic()
    next_heartbeat = start
    window_start = start
    next_report = start + REPORT_PERIOD_S

    print(f"{port_name}: sending HEARTBEAT at {HEARTBEAT_RATE_HZ:.0f} Hz; Ctrl+C to stop")
    try:
        with port:
            while True:
                now = time.monotonic()
                if now >= next_heartbeat:
                    port.write(encode_frame(HEARTBEAT, pack_heartbeat(sequence)))
                    sequence = (sequence + 1) & 0xFFFFFFFF
                    heartbeats_sent += 1
                    next_heartbeat += hb_period
                    if next_heartbeat < now:  # Fell behind: resume the schedule, don't burst.
                        next_heartbeat = now + hb_period

                for msg_type, payload in parser.feed(port.read(max(1, port.in_waiting))):
                    if msg_type == STATUS:
                        status = unpack_status(payload)
                        if status is not None:
                            last_status = status
                            status_in_window += 1

                if now >= next_report:
                    rate = status_in_window / max(now - window_start, 1e-6)
                    if last_status is None:
                        mode_text, fault_text = "-", "- (no STATUS yet)"
                    else:
                        mode, _gait_phase, faults = last_status
                        mode_text = str(mode)
                        fault_text = f"0x{faults:04X} ({decode_fault_bits(faults)})"
                    print(
                        f"[{now - start:7.1f}s] hb_sent={heartbeats_sent} "
                        f"status={rate:4.1f}/s mode={mode_text} faults={fault_text} "
                        f"crc_errors={parser.crc_errors}",
                        flush=True,
                    )
                    status_in_window = 0
                    window_start = now
                    next_report += REPORT_PERIOD_S
                    if next_report < now:
                        next_report = now + REPORT_PERIOD_S
    except KeyboardInterrupt:
        print(
            f"\nstopped: hb_sent={heartbeats_sent} frames_ok={parser.frames_ok} "
            f"crc_errors={parser.crc_errors} length_errors={parser.length_errors}"
        )
        return 0
    except serial.SerialException as exc:
        print(f"serial error on {port_name}: {exc}", file=sys.stderr)
        return 1


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    return run(args.port, args.baud)


if __name__ == "__main__":
    sys.exit(main())
