# Falali — hardware documentation

Handover documentation for **SOAR**. The firmware side is documented in
`docs/gotchas.md` and `docs/hardware-architecture.md`; this folder is the
hardware.

Falali is an ESP32-S3 under-ride docking AGV: a mecanum base that crabs
sideways under an 860 × 1260 mm trolley, aligns on four corner ToF sensors, and
hands off to an arm subsystem that clamps it.

---

## Start here

| If you want to… | Read |
|---|---|
| **Switch it on without breaking it** | [`startup.md`](startup.md) |
| Know what a part is, or what's worth salvaging | [`components.md`](components.md) |
| Find which GPIO a connector pin goes to | [`boards.md`](boards.md) |
| Order more boards, or change one | [`fabrication.md`](fabrication.md) |
| Avoid a trap that already cost someone days | [`../gotchas.md`](../gotchas.md) |

**If you read only one thing, read `startup.md` §1.** The big red mushroom
button is wired as an *enable*, not an emergency stop — pressing it turns the arm
rail **on**. That is the single most dangerous surprise in this machine.

---

## How these documents are written

Every claim carries a tag, because treating a good guess as a fact is how the
next person loses a day:

- **[VERIFIED]** — read from the KiCad netlist or firmware source, or measured on
  hardware, and quoted here. Build on it.
- **[OBSERVED]** — this happened on the robot and the evidence is quoted. The
  cause may still be open.
- **[HYPOTHESIS]** — a model that fits the evidence and has not been tested
  against an alternative. It may be wrong. Where one is load-bearing, the
  experiment that would kill it is written alongside.

Gaps are listed explicitly at the end of `startup.md` and `components.md` rather
than left silent, so an absence is never mistaken for a completed survey.

**The KiCad netlists are the authority for anything electrical.** `boards.md` was
built by parsing `*.net` directly, not by reading a rendered schematic. Where
`include/pins.h` or the older docs disagree with the netlist, the disagreement is
called out in place.

---

## What's in the repo

| Path | |
|---|---|
| `KiCad/Wheel Drive PCB/` | Base board — ESP, 4 driver outputs, all base sensors |
| `KiCad/Driver PCB/` | 3.3 V → open-collector adapter (**build 4**) |
| `KiCad/Arm Subsystem PCB/` | Arm board — by **Kai Xiang and Heng Li, EPD Batch of 2028** |
| `bench_ble/` | Base firmware — ESP-IDF + Bluepad32, gamepad, docking |
| `arm/` | Arm firmware — vendored, same authors |
| `lib/domain/` | Pure docking logic, 77 host tests |
| `include/pins.h` | Base GPIO map, netlist-derived |
| `docs/BLD-120-English-version.pdf`, `docs/SYS-BLD-120A-manual.pdf`, `docs/BLDC-BLD120A-bldc-motor-controller-specs.pdf` | Driver manuals. They **contradict each other**, and the BLD-120A-specific spec sheet is the tie-breaker — see `components.md` §2. The spec sheet is image-only; reading it needs `pdftoppm` (poppler) |
| `docs/hardware/img/motor-80-flange-spec-table.jpeg` | Motor datasheet — the source for every figure in `components.md` §2 |

---

## Known open items, carried over deliberately

These are recorded rather than quietly left. Full detail in `gotchas.md`.

| | Status |
|---|---|
| **X-axis extend does not stop at its limit switch** | Open. The netlist has now **eliminated the leading hypothesis** — SW3 *is* correctly wired to GPIO42. See `boards.md` §3 for what survives and the 30-second test that settles it |
| **Arm PCB servo headers are dead in firmware** | The board wires S1–S4 to GPIO; the firmware drives a PCA9685 over I²C that has no home on the board. `boards.md` §3 |
| **The red button is an enable, not an E-stop** | The 12 V rail sits on the switch's normally-open contact; the part has an unused NC contact. `startup.md` §1 |
| **The current sensors are not wired up** | Fitted inline on the motor negative, but their outputs go nowhere — so the base has **no drivetrain fault detection at all**. `components.md` §4 |
| **The ACC/DEC trim has never been checked** | A third, previously undocumented driver pot that ramps every speed change over **0.3–15 s**. Could invalidate the odometry and look exactly like stiction. `gotchas.md` §19 |
| **SV is driven at 3.3 V, spec is 5 V** | May be capping top speed at ~70 %. One level-shifted wheel would settle it. `components.md` §2 |
| **Battery is undocumented** | Custom-built: no chemistry, cell count, BMS rating or charge profile exists anywhere. `components.md` §1 |
| **Two custom KiCad libraries missing** | Boards can be re-ordered as-is, but not cleanly re-synced. `fabrication.md` §3 |
| **Encoders abandoned** | No wheel-speed feedback — but **the motors have unused hall sensors**, landing on the drivers' own screw terminals. `components.md` §2 |
| **Gearbox ratio is inherited, not verified** | Documented everywhere as 15:1; the purchase listing groups the motor as "ratio 10–18". The speed ceiling depends on it. `components.md` §2 |

---

## Credits

Base hardware, base firmware, docking logic and these documents: **Bryan**.
Arm Subsystem PCB and `arm/` firmware: **Kai Xiang and Heng Li, EPD Batch of
2028**.
