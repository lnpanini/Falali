# ESP-ARM — arm subsystem firmware

Runs on the **arm ESP32-S3**. Receives commands from the base over ESP-NOW and
drives the two travel axes, the four flipper servos and the clamp signal.

## Authorship

**Written by Kai Xiang and Heng Li, EPD Batch of 2028.**

Vendored into this repository from
[`sks0826/Arm_code`](https://github.com/sks0826/Arm_code) at commit
`ca81eef096dc06a8c228da2e69a983bfa2abafc4`, plus the local fixes recorded in
[`LOCAL-CHANGES.md`](LOCAL-CHANGES.md).

It was a git submodule until 2026-08-14. That was changed because a submodule is
a poor fit for a handover: the fixes below live only in this working tree, so
anyone cloning TrolleyBot would have got firmware that does not talk to the base,
with nothing on screen to say why. Vendoring makes one clone give one working
pair of boards.

A predecessor exists — `chuahengli/30.007-EDI-ESP32-Arm-System`, which spoke
single-character commands over a three-wire UART. **It is superseded.** The base
no longer has a UART link and its GPIO1/2 are free. Do not flash it expecting the
gamepad to reach the arm.

## Building and flashing

```bash
cd arm && pio run -e esp32s3 -t upload
```

Its serial console is **UART0 on GPIO43/44 at 115200** — the DevKitC's `UART`
port, not the one marked `USB`. `platformio.ini` leaves `build_flags` empty, so
`ARDUINO_USB_CDC_ON_BOOT` defaults to 0 and no CDC object is linked into the
binary at all. Plugging into the native USB port gives a port that enumerates,
stays silent, and ignores everything typed at it.

## The interface the base depends on

Four commands over ESP-NOW, **WiFi channel 1**, plain text, NUL-terminated:

| command | effect |
|---|---|
| `grab` | extend X → flip X servos 170°→80° → retract X, then the same for Y |
| `release` | as `grab`, but servos return 80°→170° |
| `home_setup` | all four servos home, then retract X, then retract Y |
| `estop` | stop everything, reply `stopped` |

Replies: `done_grab`, `done_release`, `done_home`, `busy`, `stopped`,
`failed:<reason>`, `rejected:<reason>`.

**These strings are an interface.** The base sends exactly these; renaming one
breaks the pair silently, because an unrecognised command is answered
`rejected:unknown_command` and nothing moves.

The serial console carries a much larger command set (`xext`, `xret`, `yext`,
`yret`, `mstop`, `fx`, `fy`, `ex`, `ey`, `clamp`, `limit`, …). Since the
fall-through fix below, all of it is reachable over ESP-NOW too, which is how the
base's LT/RT + D-pad manual controls work.

## Before you change anything here

Read [`../docs/gotchas.md`](../docs/gotchas.md). Several behaviours in this
firmware are surprising in ways that cost days to rediscover — an out-of-range
ToF reading counting as "clear air", a receive buffer that holds exactly one
command, and extend checking a different limit switch from retract.
