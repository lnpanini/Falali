"""Mecanum mixing — a faithful port of lib/drive/MecanumDrive.cpp.

Kept byte-for-byte equivalent to the C++ on purpose. The signs in that file are
marked "to be validated on hardware", so when you flip one during bring-up you
must flip it in BOTH places or the Pi and the ESP will disagree about which way
the robot goes. There is a test below that guards the shape of the mapping.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Wheels:
    """Normalised wheel commands, each in [-1, 1]. Order matches fal::Corner."""

    fl: float
    fr: float
    rl: float
    rr: float

    def scaled(self, limit: float = 1.0) -> "Wheels":
        """Uniformly scale all four. Use to cap top speed without distorting the
        mix — scaling one wheel alone would change the direction of travel."""
        return Wheels(self.fl * limit, self.fr * limit, self.rl * limit, self.rr * limit)

    def as_setpoints(self) -> tuple[int, int, int, int]:
        """To the ESP's -1000..1000 integer wire format."""
        return tuple(int(round(max(-1.0, min(1.0, w)) * 1000))
                     for w in (self.fl, self.fr, self.rl, self.rr))


def mix(vx: float, vy: float, omega: float) -> Wheels:
    """Body-frame command -> four wheel commands.

    vx    forward (+) / reverse (-)
    vy    strafe left (+) / right (-)
    omega yaw CCW (+) / CW (-)

    Mirrors MecanumDrive::move():

        wfl = vx - vy - omega
        wfr = vx + vy + omega
        wrl = vx + vy - omega
        wrr = vx - vy + omega

    then normalises so the largest-magnitude wheel is at most 1.0. That
    normalisation matters: without it, a diagonal command saturates two wheels
    while the others keep their value, and the robot curves instead of going
    where you asked.
    """
    wfl = vx - vy - omega
    wfr = vx + vy + omega
    wrl = vx + vy - omega
    wrr = vx - vy + omega

    peak = max(abs(wfl), abs(wfr), abs(wrl), abs(wrr))
    m = (1.0 / peak) if peak > 1.0 else 1.0
    return Wheels(wfl * m, wfr * m, wrl * m, wrr * m)


if __name__ == "__main__":
    # Shape checks — these encode the sign convention. If bring-up flips a sign
    # in MecanumDrive.cpp, one of these will fail and remind you to flip it here.
    def close(a, b, eps=1e-9):
        return abs(a - b) < eps

    w = mix(1, 0, 0)
    assert all(close(x, 1.0) for x in (w.fl, w.fr, w.rl, w.rr)), "forward: all equal"

    w = mix(0, 1, 0)
    assert close(w.fl, -1) and close(w.fr, 1) and close(w.rl, 1) and close(w.rr, -1), \
        "strafe left: FL/RR negative, FR/RL positive"

    w = mix(0, 0, 1)
    assert close(w.fl, -1) and close(w.fr, 1) and close(w.rl, -1) and close(w.rr, 1), \
        "yaw CCW: left side back, right side forward"

    w = mix(1, 1, 0)          # diagonal — would be 2.0 unnormalised
    assert close(max(abs(w.fl), abs(w.fr), abs(w.rl), abs(w.rr)), 1.0), "normalised"
    assert close(w.fr, 1.0) and close(w.rl, 1.0) and close(w.fl, 0.0) and close(w.rr, 0.0)

    assert mix(1, 0, 0).as_setpoints() == (1000, 1000, 1000, 1000)
    assert mix(0, 0, 0).as_setpoints() == (0, 0, 0, 0)
    assert mix(1, 0, 0).scaled(0.3).as_setpoints() == (300, 300, 300, 300)

    print("mecanum mixing: all shape checks pass")
