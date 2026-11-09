# CLAUDE.md

Guidance for Claude Code (and anyone else) working in this repository.

## Project context

Home Guardian is a custom 8-DOF quadruped robot for autonomous indoor patrol.

- **Raspberry Pi 5 (8 GB):** Ubuntu Server 24.04, ROS 2 Jazzy. High-level software. The ROS 2
  packages (`hg_*`) are planned but not written yet; `ros2_ws/src/` is empty.
- **Teensy 4.1:** bare-metal Arduino-framework firmware built with PlatformIO
  (`firmware/`). Real-time controller. Connected to the Pi over USB as `/dev/ttyACM0`.
- **Link:** framed binary serial protocol, `0xA5 | type | length | payload | CRC-16/CCITT-FALSE`
  (little-endian). Defined in `common/include/protocol.h`, mirrored in Python in
  `tools/test_link.py`. USB serial for now; hardware UART deferred
  (`docs/decisions/0001-usb-serial-link.md`).
- Implemented message types: HEARTBEAT (Pi → Teensy) and STATUS (Teensy → Pi). All other IDs are
  reserved without a payload definition.
- **The repository is public.**

## Build and test

Run from the repository root.

```bash
python3 common/test/test_protocol.py        # host protocol test: g++ vs Python, no hardware
pio run -d firmware                         # build the firmware
clang-format --dry-run --Werror common/include/*.h common/test/*.cpp firmware/src/*.cpp
```

Need the hardware and the owner's permission:

```bash
pio run -d firmware -t upload               # upload firmware: ASK FIRST
python3 tools/test_link.py /dev/ttyACM0     # live link test against the Teensy
```

ROS 2 (once packages exist): `source /opt/ros/jazzy/setup.bash`, then `colcon build` and
`colcon test` in `ros2_ws/`.

CI (`.github/workflows/firmware.yml`) runs the host protocol test and `pio run` on every push and
pull request.

## Conventions

- **Commits:** Conventional Commits (`feat`, `fix`, `docs`, `test`, `ci`, `chore`, `refactor`),
  with a scope where it helps: `firmware`, `protocol`, `tools`, `ros2`, `docs`.
- **C++:** formatted with the root `.clang-format` (Google style, 100 columns). Naming follows
  `protocol.h`: types `PascalCase`, functions and variables `snake_case`, constants
  `kPascalCase`, everything shared in namespace `hg`.
- **Python:** 4-space indent, 100 columns. Tools use the standard library plus pyserial only.
- **Protocol changes:** `protocol.h` is the source of truth. Any change must be mirrored in
  `tools/test_link.py`, covered in `common/test/test_protocol.py`, and reflected in the message
  table in `README.md`. Multi-byte fields are little-endian; payloads are at most 64 bytes.
- **Firmware:** the main loop must never block (no `delay()`, no `while (!Serial)`). The serial
  port for the link is chosen by the `LINK` macro in `firmware/src/main.cpp`.
- **Decisions:** record significant decisions as ADRs in `docs/decisions/NNNN-title.md`.
- **Docs:** keep them factual. Do not describe planned features as done.
- **Secrets and personal data:** never commit tokens, credentials, Wi-Fi settings, SSH keys,
  `.env` files, face-enrollment images, maps, event databases or recordings. `.gitignore` covers
  these; do not force-add them. Use placeholders (`<user>`, `<hostname>`, `<pi-ip>`) in docs.

## Hardware cautions

- **Teensy 4.1 I/O is 3.3 V only and not 5 V tolerant.** Never connect a 5 V signal to a Teensy
  pin.
- **Never touch Raspberry Pi header pins 2 and 4.** They are 5 V; shorting them to a GPIO pin can
  destroy the Pi.
- **Do not upload firmware to the Teensy without asking the owner first.**
- Ask before running `sudo` or installing system packages on the Pi.
- Only one program can use `/dev/ttyACM0` at a time. Keep ModemManager disabled, because it
  probes new serial devices.
