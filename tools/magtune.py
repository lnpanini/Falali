#!/usr/bin/env python3
"""Live AS5600 magnet-airgap tuner.

Polls the bench firmware's `m` command and renders AGC as a bar so the magnet
can be positioned by eye. AGC is the sensor's own GAIN, so it reads the airgap
BACKWARDS: high gain = straining to see a distant magnet.

Full scale is supply dependent -- 0-255 at 5V but only 0-128 at 3.3V. The
breakout is on 3V3, so 128 is the RAIL, not the middle. Target ~64.
"""
import re
import sys
import time

import serial

PORT = "/dev/cu.wchusbserial110"
BAUD = 115200
FS = 128          # AGC full scale at 3.3V
TARGET = FS // 2  # 64

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 3600.0

AGC_RE = re.compile(r"AGC:(\d+)/(\d+)\s+magnitude:(\d+)")
DET_RE = re.compile(r"magnet:\s*(\w+)")

s = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.3)
s.reset_input_buffer()

print(f"Polling AS5600 for {secs:.0f}s — move the magnet and watch AGC.")
print(f"Target AGC ~{TARGET}/{FS}.  HIGH = too FAR (high gain).  LOW = too CLOSE.")
print("-" * 78)

print("Ctrl-C when the magnet is seated.\n")

t0 = time.time()
next_poll = 0.0
buf = ""
last = None

try:
  while time.time() - t0 < secs:
      now = time.time()
      if now >= next_poll:
          next_poll = now + 0.4
          s.write(b"m\n")

      chunk = s.read(4096).decode("utf-8", "replace")
      if not chunk:
          continue
      buf += chunk
      while "\n" in buf:
          line, buf = buf.split("\n", 1)
          mm = AGC_RE.search(line)
          if not mm:
              continue
          agc, fs, mag = int(mm.group(1)), int(mm.group(2)), int(mm.group(3))
          det = DET_RE.search(line)
          detected = det.group(1) if det else "?"

          # Bar with the target marked, so "move toward the middle" is literal.
          width = 40
          pos = min(int(agc / fs * width), width - 1)
          tgt = int(TARGET / fs * width)
          cells = []
          for i in range(width):
              if i == pos:
                  cells.append("#")
              elif i == tgt:
                  cells.append("|")
              else:
                  cells.append("-")
          bar = "".join(cells)

          if agc >= fs - 1:
              verdict = "RAILED - far too far"
          elif agc <= 8:
              verdict = "RAILED - far too close"
          elif agc > TARGET + 24:
              verdict = "too far  -> move CLOSER"
          elif agc < TARGET - 24:
              verdict = "too close -> move AWAY"
          else:
              verdict = "GOOD"

          stamp = f"AGC {agc:3d}/{fs} [{bar}] mag {mag:5d}  {detected:8s} {verdict}"
          if stamp != last:
              print(stamp)
              last = stamp

except KeyboardInterrupt:
    print()

s.close()
print("-" * 78)
print("done. '|' marks the target; move the magnet until '#' sits on it.")
