"""Xbox controller on the Pi, read through Linux evdev.

Replaces the Bluepad32 path from bench_ble/ — same sticks, same feel, but the
gamepad now pairs to the Pi (the brain) instead of to the ESP. The deadzone and
axis convention are carried over from bench_ble/main/sketch.cpp unchanged, so the
robot handles exactly as it did on the ESP build:

    DEADZONE = 0.12          absorbs stick drift (~-27 at rest)
    vx = -axisY              stick up    = forward
    vy =  axisX              stick right = strafe right
    w  =  axisRX             right stick right = rotate CW

Note vy/omega SIGNS: mecanum.mix() takes vy positive = strafe LEFT and omega
positive = yaw CCW, so this module negates on the way out. Keeping the raw stick
convention identical to the ESP build makes the two comparable; keeping mix()
identical to MecanumDrive.cpp makes the Pi and firmware agree. The negation is
where the two conventions meet.

    sudo apt install python3-evdev      (or: pip install evdev)
"""

from __future__ import annotations

import time
from dataclasses import dataclass

try:
    from evdev import InputDevice, ecodes, list_devices
except ImportError:  # keeps the module importable on the Mac for testing
    InputDevice = None  # type: ignore
    ecodes = None       # type: ignore
    list_devices = None  # type: ignore

DEADZONE = 0.12          # from bench_ble/main/sketch.cpp — do not change casually

# A Bluetooth pad going out of range, or having its battery pulled, does NOT
# raise from evdev until BlueZ tears the input node down -- seconds later. Until
# then the last stick values sit in the buffer and would keep being mixed and
# sent at 50 Hz, at whatever deflection they were last at. The ESP's own watchdog
# does NOT catch this: frames ARE arriving, they are just stale.
#
# A healthy pad emits events at least every few tens of ms, so silence this long
# means it is gone.
STALE_S = 0.25


@dataclass
class Command:
    vx: float = 0.0        # forward +
    vy: float = 0.0        # strafe LEFT + (mecanum.mix convention)
    omega: float = 0.0     # yaw CCW +
    boost: bool = False    # right trigger / RB — hold for full speed
    estop: bool = False    # B button
    connected: bool = False


def axis_norm(raw: int, lo: int, hi: int) -> float:
    """Scale a raw axis to [-1, 1] and apply the deadzone.

    Rescales the region OUTSIDE the deadzone back to full range, so the stick
    still reaches 1.0 at the mechanical limit. A plain cutoff would leave you
    unable to reach full speed and would jump discontinuously at the edge.
    """
    mid = (lo + hi) / 2.0
    span = (hi - lo) / 2.0
    v = (raw - mid) / span if span else 0.0
    v = max(-1.0, min(1.0, v))
    if abs(v) < DEADZONE:
        return 0.0
    return (v - (DEADZONE if v > 0 else -DEADZONE)) / (1.0 - DEADZONE)


class Gamepad:
    """Non-blocking evdev reader. Call poll() from your control loop."""

    def __init__(self, device_path: str | None = None) -> None:
        if InputDevice is None:
            raise RuntimeError("evdev not installed — sudo apt install python3-evdev")
        self.path = device_path or self._find()
        self.dev = InputDevice(self.path)
        self.dev.grab()          # stop keystrokes leaking to the Pi's console
        self.name = self.dev.name
        self._abs = {}           # code -> (lo, hi)
        for code, info in self.dev.capabilities().get(ecodes.EV_ABS, []):
            self._abs[code] = (info.min, info.max)
        self._raw = {}
        self._btn = {}
        self.last_event = time.monotonic()

    @staticmethod
    def _find() -> str:
        """Pick the first device that looks like a gamepad (has ABS_X and BTN_A)."""
        for path in sorted(list_devices()):
            d = InputDevice(path)
            caps = d.capabilities()
            has_stick = ecodes.ABS_X in [c for c, _ in caps.get(ecodes.EV_ABS, [])]
            has_a = ecodes.BTN_A in caps.get(ecodes.EV_KEY, [])
            if has_stick and has_a:
                return path
        raise RuntimeError("no gamepad found — is it paired and connected?")

    def poll(self) -> Command:
        """Drain pending events and return the current command. Never blocks."""
        try:
            while True:
                ev = self.dev.read_one()
                if ev is None:
                    break
                if ev.type == ecodes.EV_ABS:
                    self._raw[ev.code] = ev.value
                    self.last_event = time.monotonic()
                elif ev.type == ecodes.EV_KEY:
                    self._btn[ev.code] = ev.value
                    self.last_event = time.monotonic()
        except OSError:
            return Command(connected=False)   # controller went away mid-read

        def ax(code: int) -> float:
            if code not in self._raw or code not in self._abs:
                return 0.0
            lo, hi = self._abs[code]
            return axis_norm(self._raw[code], lo, hi)

        # Stick convention from the ESP build; negated where mix() differs.
        vx = -ax(ecodes.ABS_Y)          # up = forward
        vy = -ax(ecodes.ABS_X)          # stick right -> strafe right = vy negative
        omega = -ax(ecodes.ABS_RX)      # stick right -> rotate CW  = omega negative

        # Treat silence as a disconnect, not as "hold the last command".
        fresh = (time.monotonic() - self.last_event) < STALE_S
        if not fresh:
            return Command(connected=False)

        return Command(
            vx=vx, vy=vy, omega=omega,
            boost=bool(self._btn.get(ecodes.BTN_TR, 0)),
            estop=bool(self._btn.get(ecodes.BTN_B, 0)),
            connected=True,
        )

    def close(self) -> None:
        try:
            self.dev.ungrab()
        except Exception:
            pass


if __name__ == "__main__":
    # Live axis dump — use this to confirm directions before driving anything.
    g = Gamepad()
    print(f"reading {g.name} on {g.path}   (Ctrl-C to stop)")
    print("push the LEFT stick UP: vx should go POSITIVE")
    print("push the LEFT stick RIGHT: vy should go NEGATIVE (strafe right)")
    print("push the RIGHT stick RIGHT: omega should go NEGATIVE (rotate CW)")
    try:
        while True:
            c = g.poll()
            print(f"\rvx {c.vx:+.2f}  vy {c.vy:+.2f}  omega {c.omega:+.2f}"
                  f"  boost {int(c.boost)}  estop {int(c.estop)}   ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        g.close()
        print()
