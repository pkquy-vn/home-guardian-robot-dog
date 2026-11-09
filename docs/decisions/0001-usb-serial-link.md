# ADR 0001: Use USB serial for the Pi–Teensy link during development; hardware UART deferred

- **Status:** Accepted
- **Date:** 2026-11-09

## Context

The Raspberry Pi 5 and the Teensy 4.1 exchange framed binary messages
(`common/include/protocol.h`). The protocol does not depend on the transport, and there are two
physical options:

- **USB serial.** The Teensy's native USB port appears on the Pi as `/dev/ttyACM0`. The same
  cable is already used to upload firmware.
- **Hardware UART.** Teensy `Serial7` (pin 28 = RX7, pin 29 = TX7) wired to the Pi's UART0 on
  GPIO 14/15. This keeps the link independent of USB enumeration and leaves the Teensy's USB port
  free for uploads and debug output.

The Pi's 40-pin GPIO header is covered by the M.2 HAT+ (NVMe SSD). GPIO 14/15 cannot be reached
until a taller stacking header is fitted, and none is fitted yet.

## Decision

Use **USB serial** for the Pi–Teensy link during development. Defer the hardware UART until the
GPIO header is reachable.

The firmware selects the port with a single macro in `firmware/src/main.cpp`:

```cpp
#define LINK Serial
```

## Consequences

- The link works now, with no extra wiring.
- USB serial ignores the configured baud rate.
- The link depends on USB enumeration. The device name can change (for example to
  `/dev/ttyACM1`) if the Teensy re-enumerates, and ModemManager must stay disabled so that it
  does not probe the port.
- Firmware upload and the link share one port: stop the link test (or a future bridge node)
  before uploading.
- If the Teensy is ever powered from the robot's own 5 V supply while also connected to the Pi
  over USB, check PJRC's guidance on separating VIN and VUSB, so that the robot's supply does not
  back-feed the Pi's USB port.

## Switching to the hardware UART

Power everything off while wiring.

1. Fit a taller stacking header so the GPIO header is reachable above the M.2 HAT+.
2. Wire the link. Both sides use 3.3 V logic, so no level shifter is needed:

   | Teensy 4.1 | Raspberry Pi 5 header |
   | --- | --- |
   | pin 29 (TX7) | GPIO 15 / RXD (physical pin 10) |
   | pin 28 (RX7) | GPIO 14 / TXD (physical pin 8) |
   | GND | GND (for example physical pin 6) |

   Never connect anything to Pi header pins 2 or 4 (5 V). Teensy pins are not 5 V tolerant.
3. On the Pi, confirm that `dtparam=uart0=on` is in `/boot/firmware/config.txt`, then find the
   device for GPIO 14/15 (expected to be `/dev/ttyAMA0` on a Pi 5; check with
   `ls -l /dev/ttyAMA* /dev/serial*`). Make sure no serial console or login prompt uses it: look
   for `console=serial0` or `console=ttyAMA0` in `/boot/firmware/cmdline.txt` and check
   `systemctl status serial-getty@ttyAMA0.service`.
4. In `firmware/src/main.cpp`, change the link macro and, if needed, `kLinkBaud`
   (115200 by default):

   ```cpp
   #define LINK Serial7
   ```

   Build and upload over USB as usual.
5. Test the link at the same baud rate:

   ```bash
   python3 tools/test_link.py /dev/ttyAMA0 --baud 115200
   ```

6. Record the switch in a new ADR that supersedes this one.

To go back to USB, set `#define LINK Serial`, upload, and use `/dev/ttyACM0`.
