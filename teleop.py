#!/usr/bin/env python
"""Live keyboard teleop for the TrolleyBot bench sketch (src/bench_check.cpp).

Replaces `pio device monitor`, which throws `tcsetattr: Invalid argument` on this
host's CH343 driver. Sends single keypresses straight to the ESP32 and prints the
serial stream (ToF readings + state) in the background.

    /opt/anaconda3/bin/python teleop.py        # anaconda python has pyserial
    # or any python with:  pip install pyserial

Keys (handled by the firmware):
    w/a/s/d move   q/e rotate   space stop   +/- speed   1-4 wheel test
    g start align   x abort align   t toggle auto-trigger   ? help
Local key:
    Q (capital)    quit teleop
"""
import argparse
import csv
import datetime
import glob
import os
import re
import sys
import threading
import time
import serial  # pyserial

BAUD = 115200

# Status lines from src/bench_s3_motor.cpp look like:
#   EN:on  BRK:off DIR:fwd  duty 1228/4095 (30%)  ~0.99 V at SV
STATUS_RE = re.compile(
    r"EN:(\w+)\s+BRK:(\w+)\s+DIR:(\w+)\s+duty\s+(\d+)/(\d+)\s+\((\d+)%\)\s+~([\d.]+)\s*V"
)


def find_port():
    # macOS first, then Linux (so this also runs on the Pi).
    for pat in ("/dev/cu.wchusbserial*", "/dev/cu.usbserial*", "/dev/cu.usbmodem*",
                "/dev/ttyUSB*", "/dev/ttyACM*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def reader(port, stop, writer=None, raw=None, t0=0.0):
    """Print the serial stream; parse status lines into CSV if logging."""
    while not stop.is_set():
        try:
            line = port.readline()
        except Exception:
            break
        if not line:
            continue
        text = line.decode(errors="replace").rstrip()
        # \r keeps the line readable while the user is also typing.
        sys.stdout.write("\r" + text + "\n")
        sys.stdout.flush()

        if raw is not None:
            raw.write(f"{time.time() - t0:8.3f}  {text}\n")
            raw.flush()

        if writer is not None:
            m = STATUS_RE.search(text)
            if m:
                en, brk, dirn, duty, full, pct, volts = m.groups()
                writer.writerow({
                    "t_s": round(time.time() - t0, 3),
                    "enabled": en == "on",
                    "brake": brk == "on",
                    "direction": dirn,
                    "duty": int(duty),
                    "duty_full": int(full),
                    "duty_pct": int(pct),
                    "sv_volts_est": float(volts),
                    # Filled in by hand after the run — the bench has no RPM
                    # feedback until the encoders are wired.
                    "measured_rpm": "",
                    "note": "",
                })


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--log", nargs="?", const="auto", default=None, metavar="PREFIX",
                    help="record the session. Writes PREFIX.csv (parsed status) and "
                         "PREFIX.log (raw stream). Omit the value to auto-name by date.")
    ap.add_argument("--port", default=None, help="serial device (default: autodetect)")
    args = ap.parse_args()

    port_name = args.port or find_port()
    if not port_name:
        print("No serial port found (cu.wchusbserial* / ttyUSB* / ttyACM*) — is the ESP plugged in?")
        return 1
    try:
        port = serial.Serial(port_name, BAUD, timeout=0.2)
    except Exception as e:
        print(f"Could not open {port_name}: {e}")
        return 1

    print(f"connected {port_name} @ {BAUD}")
    print("  w/a/s/d move | q/e rotate | space stop | g align | x abort | Q quit")

    writer = csv_f = raw_f = None
    if args.log:
        prefix = args.log
        if prefix == "auto":
            os.makedirs("bench_logs", exist_ok=True)
            prefix = os.path.join(
                "bench_logs",
                "bench_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        csv_f = open(prefix + ".csv", "w", newline="")
        raw_f = open(prefix + ".log", "w")
        writer = csv.DictWriter(csv_f, fieldnames=[
            "t_s", "enabled", "brake", "direction", "duty", "duty_full",
            "duty_pct", "sv_volts_est", "measured_rpm", "note"])
        writer.writeheader()
        print(f"  logging -> {prefix}.csv (parsed) and {prefix}.log (raw)")
        print("  NOTE: measured_rpm is left blank — fill it in as you sweep.")

    stop = threading.Event()
    t0 = time.time()
    threading.Thread(target=reader, args=(port, stop, writer, raw_f, t0),
                     daemon=True).start()

    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # deliver keys immediately, no Enter needed
        while True:
            ch = sys.stdin.read(1)
            if ch == "Q":  # capital Q quits teleop (lowercase q = rotate, sent to firmware)
                break
            port.write(ch.encode())
            port.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        stop.set()
        # Leave the rig safe on the way out, whatever happened.
        try:
            port.write(b"x")
            port.flush()
            time.sleep(0.1)
        except Exception:
            pass
        port.close()
        for f in (csv_f, raw_f):
            if f:
                f.close()
        print("\nbye (sent 'x' E-STOP on exit)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
