"""TrolleyBot Pi 5 bridge -- fixed-rate control loop over two ESP links.

    python -m pi.bridge            # normal run
    python -m pi.bridge --dry-run  # log only, never command motion

STAGE 1 OF THE MIGRATION.
This deliberately does NOT dock, mix mecanum, or fuse odometry yet. It exists to
prove the two things that everything else stands on:

  1. Both links stay up, and we can measure how late they are.
  2. When a link goes down, the wheels stop -- driven from BOTH ends.

Read `docs/superpowers/specs/2026-08-04-rpi5-main-controller-design.md` before
extending this. The migration order there is not advisory; each step is the test
harness for the next one.

THE SHAPE OF THE LOOP
---------------------
One coroutine ticks at a fixed 50 Hz. Two more coroutines just drain their serial
ports into `LinkState`. The tick reads whatever arrived since last time -- it never
waits for a frame. That is what makes a late ESP a degraded reading rather than a
stalled control loop, and it is the single most important structural decision here.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import os
import time

from .link import EspLink

# --- Timing -----------------------------------------------------------------
TICK_HZ = 50
TICK_S = 1.0 / TICK_HZ

# If an ESP has not spoken for this long, treat it as gone. Must be comfortably
# shorter than the ESP-side watchdog so the Pi gives up FIRST and commands a stop,
# rather than being overruled by the firmware reflex. Belt and braces, in that order.
LINK_TIMEOUT_MS = 100.0

# Stage 1 only: the stock firmware publishes status at 5 Hz, so the Pi must allow
# ~3 missed publishes before calling the link dead. This is a property of the
# current telemetry rate, not a safety parameter -- the ESP-side 100 ms watchdog
# is what actually protects the robot.
STAGE1_RX_TIMEOUT_MS = 700.0

DEV_BASE = os.environ.get("TB_BASE_DEV", "/dev/ttyTB_BASE")
DEV_ARM = os.environ.get("TB_ARM_DEV", "/dev/ttyTB_ARM")

# Migration stage, per docs/superpowers/specs/2026-08-04-rpi5-main-controller-design.md.
#   1 = heartbeat only. Firmware still owns the docking loop; the Pi proves the
#       link watchdog works and nothing else. ESP-ARM is not split out yet.
#   4+ = structured JSON frames, Pi owns wheel setpoints.
STAGE = int(os.environ.get("TB_STAGE", "1"))

# Stage 1 talks to the existing firmware, which is 115200 (monitor_speed in
# platformio.ini). Raise to 921600 when ESP-BASE switches to JSON frames.
BAUD = 115200 if STAGE == 1 else 921600

log = logging.getLogger("trolleybot")


def try_realtime_priority() -> bool:
    """Ask the kernel for SCHED_FIFO. Best-effort; failure is fine.

    Needs `setcap cap_sys_nice+ep` on the interpreter (see docs/rpi5-setup.md).
    Priority 10 is low enough to stay well clear of kernel threads -- taking a high
    RT priority on a Pi is a good way to lock yourself out of your own robot.
    """
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
        return True
    except (PermissionError, OSError, AttributeError):
        return False


class Bridge:
    def __init__(self, dry_run: bool = False) -> None:
        self.base = EspLink("base", DEV_BASE, BAUD)
        self.arm = EspLink("arm", DEV_ARM, BAUD)
        self.dry_run = dry_run
        self._tick = 0
        # Set once either link times out. Latched: recovering the cable does not
        # silently resume motion -- a human decides that.
        self._safe_latched = False
        self._last_link_seen = "?"

    # -- safety ------------------------------------------------------------
    def links_healthy(self) -> bool:
        # In stage 1 ESP-ARM does not exist yet, so it is not part of the verdict.
        # NOTE: the ESP publishes status at 5 Hz (kTelemetryPeriodMs = 200), so the
        # Pi's own view of link age cannot be tighter than that until ESP-BASE
        # streams at the control rate. The authoritative 100 ms watchdog is the one
        # ON THE ESP -- this check is only the Pi noticing too.
        if STAGE == 1:
            return self.base.state.age_ms() < STAGE1_RX_TIMEOUT_MS
        return (
            self.base.state.age_ms() < LINK_TIMEOUT_MS
            and self.arm.state.age_ms() < LINK_TIMEOUT_MS
        )

    def command_stop(self) -> None:
        """Brake and drop enable on both ESPs. Safe to call repeatedly."""
        if STAGE == 1:
            # Do NOT send ABORT here. On the firmware side Command::Abort runs
            # DockingStateMachine::handleCommand, which does brake(false) +
            # enable(true) + clearEstopLatch() -- it RE-ENABLES the drivetrain.
            # Using it as a "stop" would mean the Pi's stop command starts the
            # motors, and (before the fix in main.cpp) also re-fed the watchdog
            # so it could never trip.
            #
            # Stopping is the ESP's job: withhold heartbeats and its own 100 ms
            # watchdog brakes and latches. Silence is the stop signal.
            return
        self.base.send({"cmd": "DRIVE", "w": [0.0, 0.0, 0.0, 0.0], "en": False, "brk": True})
        self.arm.send({"cmd": "STOP"})

    def heartbeat(self) -> None:
        """Prove to ESP-BASE that we are still alive. This is the whole of stage 1."""
        self.base.send_line("PING")

    # -- the tick ----------------------------------------------------------
    def tick(self) -> None:
        self._tick += 1
        base_frame = self.base.state.last_frame

        if STAGE == 1:
            self._tick_stage1(base_frame)
            return

        # Hardware faults reported from below always win.
        estop = bool(base_frame.get("estop", False))
        alarm = bool(base_frame.get("alarm", False))

        if not self.links_healthy():
            if not self._safe_latched:
                log.error(
                    "LINK LOST -- base %.0f ms, arm %.0f ms. Braking and latching.",
                    self.base.state.age_ms(),
                    self.arm.state.age_ms(),
                )
                self._safe_latched = True
            self.command_stop()
            return

        if estop or alarm:
            if not self._safe_latched:
                log.error("FAULT -- estop=%s motor_alarm=%s. Braking and latching.", estop, alarm)
                self._safe_latched = True
            self.command_stop()
            return

        if self._safe_latched or self.dry_run:
            # Latched, or explicitly told not to drive: heartbeat only. The ESP
            # watchdog stays fed (so it does not latch on its own) but we command
            # nothing. Clearing the latch is a deliberate operator action -- add
            # that when you add an operator interface.
            self.command_stop()
            return

        # ---------------------------------------------------------------
        # STAGE 4+ GOES HERE: estimator update, docking state machine, mecanum
        # mixing -> four wheel setpoints. Until then we command zero motion.
        #
        #   pose = self.estimator.update(base_frame, dt)
        #   vx, vy, omega = self.sequencer.update(pose, base_frame["tof"])
        #   wheels = mecanum_mix(vx, vy, omega)
        #   self.base.send({"cmd": "DRIVE", "w": wheels, "en": True, "brk": False})
        # ---------------------------------------------------------------
        self.base.send({"cmd": "DRIVE", "w": [0.0, 0.0, 0.0, 0.0], "en": True, "brk": False})

        if self._tick % TICK_HZ == 0:  # once a second
            log.info(
                "base rtt=%.1fms age=%.0fms err=%d | arm rtt=%.1fms age=%.0fms err=%d | enc=%s",
                self.base.state.last_rtt_ms,
                self.base.state.age_ms(),
                self.base.state.parse_errors,
                self.arm.state.last_rtt_ms,
                self.arm.state.age_ms(),
                self.arm.state.parse_errors,
                base_frame.get("enc"),
            )

    def _tick_stage1(self, base_frame: dict) -> None:
        """Heartbeat only. Prove the watchdog works; command no motion.

        The test this exists for: run it, watch `link` report OK, then pull the
        USB cable and confirm the wheels brake within 100 ms -- driven by the ESP,
        with this process no longer able to influence anything.
        """
        self.heartbeat()

        link = base_frame.get("link", "?")
        if link != self._last_link_seen:
            log.warning("ESP link state: %s -> %s", self._last_link_seen, link)
            self._last_link_seen = link

        # Surface firmware log lines as they arrive.
        while self.base.log_lines:
            log.info("esp: %s", self.base.log_lines.popleft())

        if self._tick % TICK_HZ == 0:
            log.info(
                "link=%s age=%.0fms state=%s estop=%s parse_err=%d",
                link,
                self.base.state.age_ms(),
                base_frame.get("state", "?"),
                base_frame.get("estop"),
                self.base.state.parse_errors,
            )

    async def run_loop(self) -> None:
        """Fixed-rate tick with drift correction.

        Scheduling against an absolute deadline rather than `sleep(TICK_S)` stops
        per-tick overshoot from accumulating -- otherwise a loop that runs 1 ms late
        each tick is a full period behind after 20 ticks.
        """
        next_deadline = time.monotonic()
        while True:
            self.tick()
            next_deadline += TICK_S
            slack = next_deadline - time.monotonic()
            if slack < 0:
                # We overran. Log it and resync rather than trying to catch up --
                # a burst of back-to-back ticks is worse than a skipped one.
                log.warning("tick overrun by %.1f ms", -slack * 1000.0)
                next_deadline = time.monotonic()
                slack = 0.0
            await asyncio.sleep(slack)

    async def main(self) -> None:
        # ESP-ARM is not split out until migration step 6; in stage 1 only the base
        # link must open.
        links = [self.base] if STAGE == 1 else [self.base, self.arm]
        for lnk in links:
            try:
                await lnk.connect()
                log.info("connected %s @ %d baud", lnk.state.name, BAUD)
            except Exception as exc:
                log.error("could not open %s: %s", lnk.state.name, exc)
                raise SystemExit(1)

        tasks = [asyncio.create_task(lnk.read_forever()) for lnk in links]
        tasks.append(asyncio.create_task(self.run_loop()))
        try:
            await asyncio.gather(*tasks)
        finally:
            # Whatever went wrong, leave the robot braked before we exit.
            self.command_stop()
            await asyncio.sleep(0.05)  # let the last frame drain
            for t in tasks:
                t.cancel()
            self.base.close()
            self.arm.close()


def cli() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry-run", action="store_true", help="log only, never command motion")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
    )
    log.info("realtime priority: %s", "granted" if try_realtime_priority() else "not available")

    bridge = Bridge(dry_run=args.dry_run)
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(bridge.main())
    log.info("stopped")


if __name__ == "__main__":
    cli()
