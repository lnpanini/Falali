#!/usr/bin/env python3
"""Production test loop for manufactured Wheel Drive PCBs.

Runs the env:pcb_rl bring-up firmware against one board after another, records
what you saw, and writes a CSV so the result survives the bench session.

    pio run -e pcb_rl -t upload        # once, onto the module you'll swap around
    python3 tools/board_test.py

Per board it:
  1. waits for the USB port to appear,
  2. reads the ROM boot mode and FAILS EARLY on 0x3 (DOWNLOAD),
  3. runs the RL self-test, streaming output live so you can watch the wheel,
  4. asks what actually happened and appends it to tools/board_test_log.csv.

WHY THE BOOT-MODE CHECK COMES FIRST
-----------------------------------
boot:0x3 means GPIO0 was low at reset, so the app never runs and the motor can
never move. Without this check that failure looks identical to a dead driver
channel, and you go hunting the wrong subsystem. On 2026-08-06 the cause was a
missing ground between the ESP and the motor PSU -- a wiring gap presenting as a
strapping fault. See docs/hardware-architecture.md.

DTR IS NOT A MODEM LINE HERE
----------------------------
On the ESP32-S3's USB-Serial-JTAG, asserting DTR is esptool's "set IO0" step and
forces the chip into download mode. Every port here is opened with DTR and RTS
held LOW. Do not "helpfully" set dtr=True.
"""
import csv
import glob
import json
import os
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  pip install pyserial --break-system-packages")

BAUD = 115200
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "board_test_log.csv")
SELFTEST_S = 26          # the 't' sequence takes ~16 s; leave headroom
BOOT_WAIT_S = 5


def find_port(timeout=90):
    """Block until a board enumerates. Returns the device path."""
    print("  waiting for a board to appear...", end="", flush=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.wchusbserial*"))
        if ports:
            print(f" {ports[0]}")
            return ports[0]
        print(".", end="", flush=True)
        time.sleep(0.5)
    print()
    return None


def open_port(path):
    p = serial.Serial()
    p.port, p.baudrate, p.timeout = path, BAUD, 0.2
    p.dtr = False          # DTR HIGH == IO0 == download mode. Keep it low.
    p.rts = False
    p.open()
    p.dtr = False
    p.rts = False
    return p


def pump(p, seconds, echo=True):
    """Read for `seconds`, printing as it arrives so the operator can watch."""
    buf = b""
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = p.read(4096)
        if chunk:
            buf += chunk
            if echo:
                sys.stdout.write(chunk.decode("utf-8", "replace"))
                sys.stdout.flush()
    return buf.decode("utf-8", "replace")


GOLDEN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "golden_signature.json")


def parse_scan(text):
    """Pull the scan table out of the firmware's output.

    Rows look like:   FL.SV    42    1  0  ok/floating
    """
    rows = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        name, gpio, up, dn = parts[0], parts[1], parts[2], parts[3]
        if "." not in name or not gpio.isdigit() or up not in "01" or dn not in "01":
            continue
        bridged = " ".join(w for w in parts[4:] if "." in w)
        rows[name] = {"gpio": int(gpio), "up": int(up), "dn": int(dn), "bridged": bridged}
    return rows


def save_golden(scan):
    # Guard the guard: never let an empty or partial signature become the
    # reference every other board is judged against.
    if len(scan) < 16:
        raise ValueError(f"refusing to save a {len(scan)}-row golden signature; "
                         "expected all 16 wheel lines")
    with open(GOLDEN, "w") as f:
        json.dump(scan, f, indent=2, sort_keys=True)


def compare_golden(scan):
    """Return a list of human-readable differences, or None if no baseline yet.

    Absolute up/dn values cannot be judged on their own -- a populated adapter
    loads the pin enough to look 'held low' on a perfectly good board. What IS
    meaningful is DIFFERING FROM A BOARD KNOWN TO WORK. So the first board
    becomes the reference and every later one is judged against it.
    """
    if not os.path.exists(GOLDEN) or not scan:
        return None
    with open(GOLDEN) as f:
        golden = json.load(f)
    diffs = []
    for name, g in sorted(golden.items()):
        cur = scan.get(name)
        if cur is None:
            diffs.append(f"{name}: missing from this board's scan")
            continue
        if (cur["up"], cur["dn"]) != (g["up"], g["dn"]):
            diffs.append(f"{name} (GPIO{g['gpio']}): up/dn {cur['up']}/{cur['dn']}"
                         f" vs golden {g['up']}/{g['dn']}")
        if cur["bridged"] != g["bridged"]:
            diffs.append(f"{name} (GPIO{g['gpio']}): bridged to "
                         f"'{cur['bridged'] or 'nothing'}' vs golden "
                         f"'{g['bridged'] or 'nothing'}'")
    for name in sorted(set(scan) - set(golden)):
        diffs.append(f"{name}: present here but not in golden")
    return diffs


def boot_mode(text):
    for line in text.splitlines():
        if line.startswith("rst:") and "boot:" in line:
            return line.split("boot:")[1].strip()
    return None


def wedged_prompt(what):
    """The ESP32-S3's HWCDC can lock up if the sketch ever blocks on a full TX
    buffer with no host attached. Only a physical replug clears it. Say so
    plainly rather than letting it masquerade as a board or firmware fault --
    that mistake burned an evening on 2026-08-06.
    """
    print(f"\n  *** BOARD IS SILENT ({what}).")
    print("  *** This is almost always a wedged USB session, not a bad board.")
    ans = input("  *** Unplug the USB, plug it back in, then press Enter "
                "(or 'q' to give up) > ").strip().lower()
    return ans != "q"


def test_one(board_id, _attempt=1):
    path = find_port()
    if not path:
        return dict(board=board_id, boot="NO PORT", result="FAIL", note="never enumerated")

    p = open_port(path)                       # opening resets the chip
    print(f"  reading boot mode ({BOOT_WAIT_S}s)...")
    banner = pump(p, BOOT_WAIT_S, echo=False)
    mode = boot_mode(banner) or "(no ROM banner)"
    print(f"  boot mode: {mode}")

    if not banner.strip():
        p.close()
        if _attempt < 3 and wedged_prompt("nothing at all on boot"):
            return test_one(board_id, _attempt + 1)
        return dict(board=board_id, boot="SILENT", result="ERROR",
                    note="no serial output; USB session wedged")

    if mode.startswith("0x3"):
        p.close()
        print("\n  *** FAIL: chip booted into DOWNLOAD mode -- the app never ran.")
        print("  *** GPIO0 is low at reset. Check the ESP<->PSU ground first,")
        print("  *** then GPIO0 continuity to GND with the board unpowered.")
        return dict(board=board_id, boot=mode, result="FAIL", note="boot 0x3, GPIO0 low")

    print("\n  --- signal scan: 16 wheel lines (no motor needed) ---")
    p.reset_input_buffer()
    p.write(b"s")
    p.flush()
    scan_txt = pump(p, 12)
    scan = parse_scan(scan_txt)

    # An empty scan is a TEST FAILURE, not a clean board and not a missing
    # baseline. Saving it as golden would write an empty reference that silently
    # matches every future board -- a test that always passes is worse than no
    # test, because you trust it.
    if not scan:
        p.close()
        if not scan_txt.strip():
            # Silence here means the chip stopped talking mid-test, which is the
            # wedge again -- NOT a missing 's' command. Retry rather than
            # reporting a phantom firmware problem.
            if _attempt < 3 and wedged_prompt("went quiet during the scan"):
                return test_one(board_id, _attempt + 1)
            return dict(board=board_id, boot=mode, result="ERROR",
                        note="went silent during scan; USB session wedged")
        print(f"\n  *** SCAN RETURNED UNPARSEABLE OUTPUT ({len(scan_txt)} bytes).")
        print("  *** Firmware may predate 's':  pio run -e pcb_rl -t upload")
        print("  --- raw response ---")
        print("  " + scan_txt.strip()[:400].replace("\n", "\n  "))
        return dict(board=board_id, boot=mode, result="ERROR",
                    note="signal scan output did not parse")

    diffs = compare_golden(scan)
    if diffs is None:
        save_golden(scan)
        print(f"\n  (no golden signature yet -- saved this board as the reference:")
        print(f"   {GOLDEN}. Delete it to re-baseline.)")
    elif diffs:
        print("\n  *** SCAN DIFFERS FROM GOLDEN BOARD:")
        for d in diffs:
            print(f"      {d}")
    else:
        print("\n  scan matches the golden board.")

    # The two phases want OPPOSITE power states: the scan drives control lines
    # individually and must not have 24 V behind them, while the motion test
    # obviously needs the supply up. Stop and say so rather than assuming.
    print("\n  Scan done. NOW SWITCH THE 24 V SUPPLY ON.")
    ans = input("  Wheels off the ground and supply on? [y] to run the motion test, "
                "[s] to skip > ").strip().lower()
    if ans != "y":
        return dict(board=board_id, boot=mode, result="SCAN-ONLY",
                    note="motion test skipped; " + (
                        "scan clean" if diffs == [] else
                        "scan is the new golden" if diffs is None else
                        f"{len(diffs)} scan diff(s)"))

    print("\n  --- self-test: WATCH THE WHEEL ---")
    p.reset_input_buffer()
    p.write(b"t")
    p.flush()
    pump(p, SELFTEST_S)
    p.write(b" ")                             # explicit stop on the way out
    p.flush()
    time.sleep(0.5)
    p.read(4096)
    p.close()

    print()
    ans = ""
    while ans not in ("b", "f", "r", "n"):
        ans = input("  Wheel turned:  [b]oth  [f]orward only  [r]everse only  [n]othing > ").strip().lower()
    note = {"b": "both directions",
            "f": "forward only -- needs invert",
            "r": "reverse only -- needs invert",
            "n": "no motion -- check RV pot fully CCW and 24 V at the driver"}[ans]
    return dict(board=board_id, boot=mode,
                result="PASS" if ans == "b" else "FAIL", note=note)


def main():
    rows = []
    print("Wheel Drive PCB test loop.  Ctrl-C to stop.")
    print("Wheels OFF THE GROUND. 24 V on, RV pot fully anticlockwise.\n")
    try:
        while True:
            board_id = input("Board ID (blank to quit) > ").strip()
            if not board_id:
                break
            print(f"\n=== board {board_id} ===")
            row = test_one(board_id)
            row["time"] = datetime.now().isoformat(timespec="seconds")
            rows.append(row)
            print(f"  --> {row['result']}: {row['note']}\n")
            print("  Swap the next board in, then enter its ID.\n")
    except KeyboardInterrupt:
        print("\ninterrupted")

    if not rows:
        return
    new = not os.path.exists(LOG)
    with open(LOG, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["time", "board", "boot", "result", "note"])
        if new:
            w.writeheader()
        w.writerows(rows)

    npass = sum(r["result"] == "PASS" for r in rows)
    print(f"\n{npass}/{len(rows)} passed. Appended to {LOG}")
    for r in rows:
        print(f"  {r['result']:4}  board {r['board']:<10} {r['note']}")


if __name__ == "__main__":
    main()
