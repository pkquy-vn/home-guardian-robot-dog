#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Pi <-> Teensy serial link test.

Sends HEARTBEAT at 20 Hz, parses STATUS frames from the Teensy and prints one line per second:
heartbeats sent, STATUS frames per second, mode, decoded fault bits and the CRC error count.

Usage:
    python3 tools/test_link.py [PORT] [--baud BAUD]

PORT defaults to /dev/ttyACM0. Stop with Ctrl+C. Requires pyserial.

The protocol code comes from ros2_ws/src/hg_teensy_bridge/hg_teensy_bridge/protocol.py, the
single Python implementation shared with the teensy_bridge ROS 2 node.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

# The single Python implementation of the protocol lives in the hg_teensy_bridge ROS 2 package.
# Import it from the source tree so this script works without building or sourcing ROS.
_BRIDGE_PKG = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "hg_teensy_bridge"
sys.path.insert(0, str(_BRIDGE_PKG))
from hg_teensy_bridge.protocol import (  # noqa: E402
    HEARTBEAT,
    STATUS,
    FrameParser,
    decode_fault_bits,
    encode_frame,
    mode_name,
    pack_heartbeat,
    unpack_status,
)

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
        import serial  # pyserial; imported here so --help works without it
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
                        mode_text = f"{mode} ({mode_name(mode)})"
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
