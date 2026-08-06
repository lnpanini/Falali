"""A fake ESP-BASE on a virtual serial port, for testing the bridge with no hardware.

Speaks exactly what src/main.cpp + SerialTelemetry actually put on the wire:
  - accepts bare line commands: DOCK ABORT UNCLAMP STATUS PING RESUME
  - emits '#'-prefixed log lines
  - emits the JSON status frame every 200 ms (cfg::kTelemetryPeriodMs)
  - reimplements LinkWatchdog's semantics, so the Pi-side code is exercised
    against the same latch/timeout behaviour the firmware will show

    python -m pi.fake_esp                 # prints the port to point the bridge at
    python -m pi.fake_esp --die-after 3   # go silent after 3 s, to test link loss

This is a test double, not a simulator: it does not model motors, ToF or odometry.
It exists so `bridge.py` can be debugged on a laptop at 3am without a robot.
"""

from __future__ import annotations

import argparse
import json
import os
import pty
import time

TELEM_PERIOD_S = 0.200  # cfg::kTelemetryPeriodMs
LINK_TIMEOUT_S = 0.100  # cfg::makeLinkConfig().timeout_ms


class FakeEsp:
    def __init__(self, die_after: float | None = None) -> None:
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.boot = time.monotonic()
        self.die_after = die_after
        # LinkWatchdog state, mirrored from lib/domain/LinkWatchdog.h
        self.health = "NEVER_SEEN"
        self.last_feed: float | None = None
        self.rx = b""

    # -- wire helpers ----------------------------------------------------
    def _write(self, text: str) -> None:
        try:
            os.write(self.master, (text + "\r\n").encode())
        except OSError:
            pass

    def log(self, msg: str) -> None:
        self._write(f"# {msg}")

    def publish(self) -> None:
        self._write(
            json.dumps(
                {
                    "state": "IDLE",
                    "link": self.health,
                    "corners": [False, False, False, False],
                    "x_mm": 0.0,
                    "y_mm": 0.0,
                    "theta": 0.0,
                    "confirmed": False,
                    "motor_alarm": False,
                    "overcurrent": False,
                    "estop": False,
                },
                separators=(",", ":"),
            )
        )

    # -- watchdog, mirroring the firmware --------------------------------
    def feed(self, now: float) -> None:
        self.last_feed = now
        if self.health == "NEVER_SEEN":
            self.health = "OK"

    def tick_watchdog(self, now: float) -> None:
        if self.health != "OK" or self.last_feed is None:
            return
        if now - self.last_feed >= LINK_TIMEOUT_S:
            self.health = "LOST"
            self.log("LINK LOST - braking and latching. Send RESUME to recover.")

    def resume(self, now: float) -> bool:
        if self.health != "LOST" or self.last_feed is None:
            return False
        if now - self.last_feed >= LINK_TIMEOUT_S:
            return False
        self.health = "OK"
        return True

    # -- main loop -------------------------------------------------------
    def handle(self, line: str, now: float) -> None:
        cmd = line.strip().upper()
        if not cmd:
            return
        if cmd not in ("DOCK", "ABORT", "UNCLAMP", "STATUS", "PING", "RESUME"):
            self._write(f"# unknown command: {cmd}")
            return
        self.feed(now)  # any well-formed frame feeds the watchdog
        if cmd == "STATUS":
            self.publish()
        elif cmd == "RESUME":
            if self.resume(now):
                self.log("link resumed - state machine reset to IDLE, re-issue DOCK")
            else:
                self.log("RESUME refused - link not healthy")

    def run(self) -> None:
        self.log("TrolleyBot ready. Commands: DOCK ABORT UNCLAMP STATUS PING RESUME")
        self.log("drivetrain disabled until the control link is up (send PING)")
        next_pub = time.monotonic()
        while True:
            now = time.monotonic()
            if self.die_after is not None and now - self.boot > self.die_after:
                self.log("(fake esp going silent)")
                time.sleep(60)

            try:
                chunk = os.read(self.master, 4096)
                if chunk:
                    self.rx += chunk
                    while b"\n" in self.rx:
                        raw, self.rx = self.rx.split(b"\n", 1)
                        self.handle(raw.decode(errors="replace"), now)
            except BlockingIOError:
                pass
            except OSError:
                break

            self.tick_watchdog(now)
            if now >= next_pub:
                next_pub = now + TELEM_PERIOD_S
                self.publish()
            time.sleep(0.005)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--die-after", type=float, default=None,
                    help="go silent after N seconds, to exercise link-loss handling")
    args = ap.parse_args()
    esp = FakeEsp(die_after=args.die_after)
    print(f"fake ESP-BASE on {esp.port}", flush=True)
    print(f"  TB_BASE_DEV={esp.port} python -m pi.bridge", flush=True)
    esp.run()


if __name__ == "__main__":
    main()
