"""Drive the rig from an Xbox controller paired to the Pi.

    python3 -m pi.drive_gamepad                     # autodetect gamepad + serial
    python3 -m pi.drive_gamepad --port /dev/ttyACM0 --limit 0.25

    gamepad --BT--> Pi --USB--> ESP32-S3 --> 4x BLD-120A

This is the first thing in the project where the Pi is genuinely the brain: it
reads the sticks, does the mecanum mixing (pi/mecanum.py, a port of the tested
MecanumDrive.cpp), and sends four wheel setpoints. The ESP is dumb I/O.

SAFETY, in layers
-----------------
  * --limit caps every wheel. STARTS AT 0.25. Do not raise it until you have
    driven at 0.25 and are happy with the directions.
  * B button = immediate zero.
  * Losing the gamepad, or Ctrl-C, sends zeros then an 'x' E-STOP.
  * The ESP brakes by itself if no frame arrives for 300 ms, so a Python crash
    or a yanked USB cable stops the robot without needing this script's help.
"""

from __future__ import annotations

import argparse
import glob
import sys
import time

import serial  # pyserial

from .gamepad import Gamepad
from .mecanum import mix

RATE_HZ = 50
BAUD = 115200


def find_port() -> str | None:
    for pat in ("/dev/ttyTB_BASE", "/dev/ttyACM*", "/dev/ttyUSB*",
                "/dev/cu.usbmodem*", "/dev/cu.wchusbserial*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None, help="serial device (default: autodetect)")
    ap.add_argument("--limit", type=float, default=0.25,
                    help="cap on every wheel, 0..1 (default 0.25 — raise slowly)")
    ap.add_argument("--boost", type=float, default=1.0,
                    help="limit used while the right bumper is held (default 1.0)")
    args = ap.parse_args()

    port_name = args.port or find_port()
    if not port_name:
        print("no serial port found — is the ESP plugged into the Pi?")
        return 1

    try:
        gp = Gamepad()
    except RuntimeError as e:
        print(f"gamepad: {e}")
        print("pair it first:  bluetoothctl -> scan on -> pair/trust/connect <MAC>")
        return 1

    port = serial.Serial(port_name, BAUD, timeout=0.1)
    time.sleep(0.4)                 # let the ESP finish resetting on DTR
    port.reset_input_buffer()

    print(f"gamepad : {gp.name}")
    print(f"serial  : {port_name} @ {BAUD}")
    print(f"limit   : {args.limit:.2f}  (boost {args.boost:.2f} on right bumper)")
    print("left stick = drive/strafe, right stick = rotate, B = stop, Ctrl-C = quit\n")

    period = 1.0 / RATE_HZ
    next_t = time.monotonic()
    last_print = 0.0
    try:
        while True:
            c = gp.poll()
            if not c.connected:
                print("\ngamepad disconnected — stopping")
                break

            if c.estop:
                vx = vy = omega = 0.0
            else:
                vx, vy, omega = c.vx, c.vy, c.omega

            limit = args.boost if c.boost else args.limit
            sp = mix(vx, vy, omega).scaled(limit).as_setpoints()
            port.write(f"W {sp[0]} {sp[1]} {sp[2]} {sp[3]}\n".encode())

            now = time.monotonic()
            if now - last_print > 0.1:
                last_print = now
                print(f"\rvx{vx:+.2f} vy{vy:+.2f} w{omega:+.2f} -> "
                      f"FL{sp[0]:+5d} FR{sp[1]:+5d} RL{sp[2]:+5d} RR{sp[3]:+5d}"
                      f"{'  BOOST' if c.boost else '       '}", end="", flush=True)

            next_t += period
            slack = next_t - time.monotonic()
            if slack < 0:
                next_t = time.monotonic()   # overran; resync rather than catch up
            else:
                time.sleep(slack)
    except KeyboardInterrupt:
        pass
    finally:
        # Zeros first, then E-STOP. Even if this fails the ESP's own 300 ms
        # watchdog brakes the robot, but do not rely on that when you can help it.
        try:
            for _ in range(3):
                port.write(b"W 0 0 0 0\n")
                port.flush()
                time.sleep(0.02)
            port.write(b"x")
            port.flush()
        except Exception:
            pass
        gp.close()
        port.close()
        print("\nstopped (zeros sent, E-STOP issued)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
