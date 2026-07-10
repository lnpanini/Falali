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
import glob
import sys
import threading
import serial  # pyserial

BAUD = 115200


def find_port():
    for pat in ("/dev/cu.wchusbserial*", "/dev/cu.usbserial*", "/dev/cu.usbmodem*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def reader(port, stop):
    while not stop.is_set():
        try:
            line = port.readline()
        except Exception:
            break
        if line:
            # \r keeps the line readable while the user is also typing.
            sys.stdout.write("\r" + line.decode(errors="replace").rstrip() + "\n")
            sys.stdout.flush()


def main():
    port_name = find_port()
    if not port_name:
        print("No /dev/cu.wchusbserial* (or usbserial/usbmodem) port found — is the ESP32 plugged in?")
        return 1
    try:
        port = serial.Serial(port_name, BAUD, timeout=0.2)
    except Exception as e:
        print(f"Could not open {port_name}: {e}")
        return 1

    print(f"connected {port_name} @ {BAUD}")
    print("  w/a/s/d move | q/e rotate | space stop | g align | x abort | Q quit")

    stop = threading.Event()
    threading.Thread(target=reader, args=(port, stop), daemon=True).start()

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
        port.close()
        print("\nbye")
    return 0


if __name__ == "__main__":
    sys.exit(main())
