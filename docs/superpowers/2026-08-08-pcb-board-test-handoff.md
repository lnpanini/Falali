# Handoff — Wheel Drive PCB board testing (2026-08-08)

Pick up here. Everything below is committed and builds.

## Where things stand

The RL corner of the fabricated Wheel Drive PCB is working: motor spins both
directions under `env:pcb_rl`, driven from the pin map in `include/pins.h`.
A production test loop exists for qualifying the rest of the manufactured boards.

**Immediate next step:** the ESP needs one more flash of `env:pcb_rl` (the
`setTxTimeoutMs(0)` fix). At handoff the chip was wedged and refusing to sync;
a USB replug clears it, then:

```
pio run -e pcb_rl -t upload
python3 tools/board_test.py
```

## The tooling

| File | What it does |
|---|---|
| `src/bench_pcb_rl.cpp` | RL bring-up + full-board signal scan. Reads `pins.h`, never a hand-typed table. |
| `tools/board_test.py` | Per-board loop: boot check, signal scan vs golden, motion test, CSV log. |
| `tools/golden_signature.json` | 16-line reference signature. Delete to re-baseline. |
| `tools/board_test_log.csv` | Per-board results. |

Firmware commands: `t` self-test, `s` signal scan, `w` meter walk, `e/+/-/d/b`
manual, space to stop, `?` help.

### Why the golden-signature approach

Absolute pin readings cannot be judged on their own. A populated transistor
adapter loads the pin — its ~100k base pulldown against the ESP's ~45k internal
pull-up divides to ~2.3 V, below the S3's input-high threshold — so a perfectly
good adapter channel reads "held low". The first board scanned becomes the
reference and every later board is diffed against it, which cancels the loading.

Current golden shows RL.F/R, RL.EN, RL.BRK loaded (RL is the only corner with
adapters fitted) and the other 13 lines floating. Fit FL's adapters and those
rows shift the same way — the scan doubles as a check of which channels are
populated.

### Scan limits

- Sees the **ESP side only**: pin, trace, adapter input. Not the adapter output
  or the header wiring to the driver.
- Detects shorts and bridges. **Cannot detect an open trace** — the chip cannot
  sense a wire going nowhere. Opens need `w` plus a meter at the header.
- `s` pulses 3 ms per pin so it is safe either way, but `w` holds each line
  asserted for 4 s. Run `w` with 24 V OFF.

## Results so far

| Board | Result |
|---|---|
| 1 | PASS — both directions |
| 2 | FAIL — reverse only |
| 3 | FAIL — no motion |
| 4 | FAIL, then PASS on retest |

Treat the FAILs as provisional. Several were recorded while the wedge bug was
active, and "reverse only" may not be a fault at all — see below.

## Open items

- [ ] **"Reverse only" may be a false failure.** `bench_pcb_rl.cpp` has NO invert
      table; it drives F/R raw. RL is a left-hand corner, and the bench
      calibration found left motors mounted mirrored (`g_bld_invert =
      {true,false,true,false}`). A board turning "reverse only" may be behaving
      identically to board 1. Re-test boards 2 and 4 before scrapping anything.
- [ ] **Re-test every board** once the `setTxTimeoutMs(0)` firmware is flashed.
- [ ] **GPIO0 on the PCB** — root-caused to a missing ESP↔PSU ground, fixed by
      wiring it. Confirm it stays fixed across boards.
- [ ] **FR and RL pin order** — `bench_bld_drive.h` and `pins.h` disagree: the
      four roles are in opposite order for those two corners. The breadboard rigs
      must never be pointed at the fabricated PCB. Reconcile or delete one.
- [ ] **`Serial.setTxTimeoutMs(0)` belongs in `main.cpp` too.** A Pi that stops
      draining the link must not be able to stall the ESP's control loop — same
      principle as `LinkWatchdog`, one layer down. Not yet done.
- [ ] Extend the motion test beyond RL once FL/FR/RR adapters are fitted.
- [ ] `max_lin_mm_s = 300` in `config.h` is still wrong by several times.

## Traps, in one place

1. **Open serial ports with DTR and RTS LOW.** DTR is the IO0 strap on
   USB-Serial-JTAG; asserting it forces download mode. A probe that sets
   `dtr=True` wedges the board and then reports it as silent.
2. **A silent board is usually a wedged USB session, not a fault.** Unplug and
   replug. `tools/board_test.py` now detects this and retries.
3. **`boot:0x3` means the app never ran** — GPIO0 low at reset. No amount of
   motor debugging will help until it reads `boot:0x8`.
4. **`digitalWrite` before `pinMode` is rejected** on Arduino-ESP32 3.x. Boot
   safety comes from the adapter polarity (latch resets LOW = released), not
   from write ordering. See the warning in `lib/hal_esp32/Bld120aMotor.h`.

Full detail on 1–3 in `docs/hardware-architecture.md`, "Bring-up gotchas".
