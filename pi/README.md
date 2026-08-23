# pi/ — Raspberry Pi 5 bridge tooling

Python side of the [planned RPi5 controller](../docs/superpowers/specs/2026-08-04-rpi5-main-controller-design.md):
the Pi drives two ESP32 links over USB serial while an Xbox gamepad pairs to the Pi.

**Not used by the delivered term prototype** (that runs base ↔ arm over ESP-NOW Wi-Fi
with no Pi); kept here for the planned migration.

| Module | What it is |
|---|---|
| `bridge.py` | fixed-rate control loop over both ESP serial links (`--dry-run` logs only, never commands motion) |
| `link.py` | one asynchronous serial link to one ESP |
| `mecanum.py` | byte-for-byte Python port of [`lib/drive/MecanumDrive.cpp`](../lib/drive/MecanumDrive.cpp) so sign fixes apply to both |
| `gamepad.py` / `drive_gamepad.py` | Xbox controller via Linux evdev + the drive loop around it |
| `fake_esp.py` | fake ESP-BASE on a virtual serial port — test the bridge with no hardware |

Setup a fresh Pi with [`tools/pi_bootstrap.sh`](../tools/pi_bootstrap.sh); find it on the
network with [`tools/find_pi.sh`](../tools/find_pi.sh); runbook: [`docs/rpi5-setup.md`](../docs/rpi5-setup.md).
