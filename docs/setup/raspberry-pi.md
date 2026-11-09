# Raspberry Pi 5 setup

How the Home Guardian Raspberry Pi 5 (8 GB) was set up, so the setup can be reproduced.

> **Verification status.** Every item below is **reported by the owner, not verified from this
> session**: the document was written from the owner's description in a cloud session that had no
> access to the Pi. Each section has a read-only command to check it on the Pi. Commands
> marked "reproduce" change the system; review them before running.

Placeholders: `<user>` is the login user, `<hostname>` the Pi's hostname and `<pi-ip>` its IP
address. Do not commit real values; the repository is public.

## Summary

| # | Item | Status |
| --- | --- | --- |
| 1 | Ubuntu Server 24.04 LTS (arm64) on NVMe | Reported by the owner, not verified from this session |
| 2 | `noble-updates` and `noble-backports` added to apt sources | Reported by the owner, not verified from this session |
| 3 | ROS 2 Jazzy `ros-base` and `ros-dev-tools` | Reported by the owner, not verified from this session |
| 4 | Release-upgrade prompt set to `never` | Reported by the owner, not verified from this session |
| 5 | `config.txt`: `dtparam=uart0=on`, `dtparam=i2c_arm=on`, `usb_max_current_enable=1` | Reported by the owner, not verified from this session |
| 6 | User in groups `dialout` and `i2c` | Reported by the owner, not verified from this session |
| 7 | PlatformIO Core | Reported by the owner, not verified from this session |
| 8 | PJRC `00-teensy.rules` and PlatformIO udev rules | Reported by the owner, not verified from this session |
| 9 | ModemManager disabled | Reported by the owner, not verified from this session |

## 1. Ubuntu Server 24.04 on NVMe

**Status:** reported by the owner, not verified from this session.

Ubuntu Server 24.04 LTS (arm64) for Raspberry Pi, installed on an NVMe SSD in the M.2 HAT+. How
the image was written and how NVMe boot was configured were not recorded.

Check:

```bash
lsb_release -ds
```

```bash
findmnt -no SOURCE /
```

Expected: `Ubuntu 24.04.x LTS`, and a root device under `/dev/nvme0n1`.

## 2. apt sources: `noble-updates` and `noble-backports`

**Status:** reported by the owner, not verified from this session.

`/etc/apt/sources.list.d/ubuntu.sources` was missing the `noble-updates` and `noble-backports`
suites, which broke the ROS 2 installation. They were added to the `Suites:` line. On an arm64
Ubuntu image the main entry looks like this (illustrative; keep your mirror URI and the separate
`noble-security` entry as they are):

```
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: noble noble-updates noble-backports
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
```

Reproduce: edit the file with `sudoedit /etc/apt/sources.list.d/ubuntu.sources`, then:

```bash
sudo apt update
```

Check:

```bash
grep -n '^Suites:' /etc/apt/sources.list.d/ubuntu.sources
```

## 3. ROS 2 Jazzy

**Status:** reported by the owner, not verified from this session.

ROS 2 Jazzy requires Ubuntu 24.04 (Noble). Add the ROS 2 apt repository as described in the
official guide,
[Ubuntu (deb packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
then install `ros-base` and the development tools:

```bash
sudo apt install ros-jazzy-ros-base ros-dev-tools
```

To use ROS 2 in a shell:

```bash
source /opt/ros/jazzy/setup.bash
```

Check:

```bash
dpkg-query -W ros-jazzy-ros-base ros-dev-tools
```

## 4. Release upgrades disabled

**Status:** reported by the owner, not verified from this session.

ROS 2 Jazzy is built for Ubuntu 24.04, so the release-upgrade prompt is set to `never` to avoid
an accidental upgrade to a newer Ubuntu release. In `/etc/update-manager/release-upgrades`:

```
Prompt=never
```

Reproduce:

```bash
sudo sed -i 's/^Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades
```

Check:

```bash
grep '^Prompt=' /etc/update-manager/release-upgrades
```

## 5. Firmware configuration (`config.txt`)

**Status:** reported by the owner, not verified from this session.

Added to `/boot/firmware/config.txt` (reboot afterwards):

```
dtparam=uart0=on
dtparam=i2c_arm=on
usb_max_current_enable=1
```

- `dtparam=uart0=on` enables UART0 on GPIO 14/15, intended for the deferred hardware-UART link
  to the Teensy (see [ADR 0001](../decisions/0001-usb-serial-link.md)). The link currently runs
  over USB.
- `dtparam=i2c_arm=on` enables the I2C bus on GPIO 2/3 (`/dev/i2c-1`).
- `usb_max_current_enable=1` lifts the Pi 5's USB current limit (600 mA total) when the power
  supply does not identify itself as a 5 A supply. Only use it with a supply that can actually
  deliver the extra current.

Check:

```bash
grep -nE '^(dtparam=uart0|dtparam=i2c_arm|usb_max_current_enable)' /boot/firmware/config.txt
```

## 6. User groups

**Status:** reported by the owner, not verified from this session.

The login user is in `dialout` (serial ports such as `/dev/ttyACM0`) and `i2c` (`/dev/i2c-*`).

Reproduce (log out and back in afterwards):

```bash
sudo usermod -aG dialout,i2c <user>
```

Check:

```bash
id -nG <user>
```

## 7. PlatformIO Core

**Status:** reported by the owner, not verified from this session.

PlatformIO Core provides `pio`, used to build and upload the Teensy firmware. The install method
used on this Pi was not recorded; the
[official installation guide](https://docs.platformio.org/en/latest/core/installation/index.html)
covers the installer script and the shell commands.

Check:

```bash
pio --version
```

## 8. udev rules for the Teensy

**Status:** reported by the owner, not verified from this session.

Two rule files let a normal user access and upload to the Teensy:

- PJRC's `00-teensy.rules` from <https://www.pjrc.com/teensy/00-teensy.rules>
- PlatformIO's `99-platformio-udev.rules`, see the
  [PlatformIO udev rules FAQ](https://docs.platformio.org/en/latest/core/installation/udev-rules.html)

Reproduce:

```bash
sudo curl -fsSL -o /etc/udev/rules.d/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
```

```bash
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
```

```bash
sudo udevadm control --reload-rules
```

```bash
sudo udevadm trigger
```

Check:

```bash
ls -l /etc/udev/rules.d/ | grep -Ei 'teensy|platformio'
```

## 9. ModemManager disabled

**Status:** reported by the owner, not verified from this session.

ModemManager probes newly attached USB serial devices such as `/dev/ttyACM0` looking for modems,
which can interfere with the Teensy link and firmware uploads.

Reproduce:

```bash
sudo systemctl disable --now ModemManager
```

Check:

```bash
systemctl is-enabled ModemManager
```

```bash
systemctl is-active ModemManager
```

Expected: `disabled` (or `masked`, or "not found" if the package is removed) and `inactive`.
