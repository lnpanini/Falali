# Re-ordering, modifying and repairing the boards

Written for someone who can solder but has never opened KiCad.

---

## 1. What all these files are

A KiCad project is two drawings plus a pile of machine-readable exports.

| File | What it is | Do you need it? |
|---|---|---|
| `*.kicad_sch` | **Schematic** — what is connected to what. No physical positions | To understand the circuit |
| `*.kicad_pcb` | **Layout** — where the copper physically goes | To change the board |
| `*.net` | **Netlist** — a plain-text dump of every connection. This is what `boards.md` was built from | To check something without opening KiCad |
| `*.kicad_pro` / `.kicad_prl` | Project settings / local UI state | Ignore |
| `*.gbr`, `*.gbl`, `*.gtl`, `*.gto`… | **Gerbers** — one file per manufacturing layer | To order boards |
| `*.drl` | **Drill file** — hole positions and sizes | To order boards |
| `*Fab Output*.zip` | Gerbers + drill, zipped and ready to upload | **This is the one you send a factory** |

The `.net` file is genuinely readable in a text editor. If you only want to know
which GPIO a connector pin goes to, open it and search — you do not need KiCad
installed at all.

---

## 2. Just re-order a board, unchanged

**You do not need KiCad for this.** Upload the fab zip and pick options.

| Board | Zip to upload | Size |
|---|---|---|
| Wheel Drive | `KiCad/Wheel Drive PCB/Wheel Drive Fab Output/` *(zip the folder)* | 130 × 105 mm |
| Driver ×4 | `KiCad/Driver PCB/Driver_PCB_fabrication.zip` | 42.8 × 33.8 mm |
| Arm Subsystem | `KiCad/Arm Subsystem PCB/Arm Fab Output.zip` | 110 × 110 mm |

Order settings that match what was built: **2 layers, 1.6 mm thickness, 1 oz
copper, HASL finish.** Nothing here needs impedance control, blind vias or any
other paid option.

**Order at least 5 Driver PCBs** even though the robot needs 4 — they are the
cheapest board and the most likely to be damaged, and minimum order is usually 5
anyway.

---

## 3. Opening the projects

Install **KiCad 9.0.6 or newer** (these were made in 9.0.6). Open the
`.kicad_pro` file.

### What will and will not work

The schematic caches every symbol it uses inside the `.kicad_sch`, and the layout
stores complete footprint definitions inside the `.kicad_pcb`. So:

✅ **Opening, reading, printing and re-exporting gerbers all work perfectly**
with no libraries installed.

⚠️ **Three libraries are referenced but are not in this repo**, and you will hit
them the moment you try to *add* one of those parts or run **Tools → Update PCB
from Schematic**:

| Library | Contains | Getting it back |
|---|---|---|
| `PCM_Espressif` | `ESP32-S3-DevKitC` symbol | Install from KiCad's **Plugin and Content Manager** — it is Espressif's official library. Easy |
| `My_30.007_Library` | `VL53L0X_breakout_(Kuriosity)` symbol | **Custom.** Recreate, or ask the original authors |
| `HengLi's Footprint Library` | `ESP32-S3-DevKitC-N16R8` footprint | **Custom** (Heng Li). Recreate, or ask |

**[VERIFIED]** — no `.pretty` folder, `fp-lib-table` or `sym-lib-table` exists
anywhere in this repo.

**What this means practically:** the boards can be reproduced indefinitely and
small layout edits are possible. Schematic→layout re-sync is unavailable until
the two custom libraries are rebuilt. For scale: the DevKitC footprint is two
1×22 headers 22.86 mm apart, so recreating it is an afternoon's work rather than
a project.

---

## 4. Making a change

The loop is always the same:

1. Edit the **schematic** (`.kicad_sch`).
2. **Tools → Update PCB from Schematic** — pushes new connections into the
   layout as unrouted "ratsnest" lines. *(This is the step that needs the
   libraries above.)*
3. Move footprints, route the new tracks in the **layout**.
4. **Inspect → Design Rules Checker (DRC)** — must come back clean.
5. **File → Fabrication Outputs → Gerbers**, and separately **Drill Files**.
6. Zip them. That is your new order.
7. **Re-export the netlist too** (`File → Export → Netlist`) and commit it —
   `boards.md` and `pins.h` are both derived from it, and a stale netlist is how
   this project got a GPIO map that drove the I²C bus as motor enables.

> **If you change a GPIO, you have changed a contract.** `include/pins.h` mirrors
> the netlist and `boards.md` documents it. Update both in the same commit as the
> board change, or the next person will trust the wrong one. The header at the
> top of `pins.h` says "if the board is respun, re-extract rather than
> hand-editing" — follow that.

---

## 5. Assembly notes

### Order of soldering

Shortest parts first, so the board sits flat on the bench:

1. Resistors (Driver PCB)
2. Transistors — **watch the orientation**, see below
3. Pin headers
4. Terminal blocks
5. Female sockets for the DevKit **last**

### The one orientation that will bite you

The Driver PCB's transistor footprint is **TO-92 Inline Wide, E-B-C**, and the
silkscreen prints `E`, `B`, `C` next to the pads. Match the flat face of the
part to the flat side on the silk.

> ⚠️ **2N3904 is E-B-C. BC547 is C-B-E.** Identical packages, mirrored pinouts.
> The board expects a 2N3904. If all you have is BC547, it works only rotated
> 180° — which means the legs must be crossed, not just the body turned. Pick one
> type and stay with it across all four boards. Getting this wrong produces a
> driver that sits happily idle showing a **green** LED, because from its point
> of view it was simply told to stop — see the diagnostic table in `boards.md` §2.

### Socket the DevKits

Both boards take 2× 1×22 female headers for the ESP module. **Do not solder the
DevKit down.** It is the part most likely to need swapping and the one most
easily destroyed during removal.

### Do not fit the encoder connectors

J1/J2, J10/J11, J13/J15, J16/J18 on the Wheel Drive PCB are for the abandoned
AS5600 encoders. Leaving them unpopulated saves 28 solder joints and removes any
chance of something being plugged into GPIO1/2/3/14 by accident. `boards.md` §1.

---

## 6. Repair

### Intermittent signal — clean before you desolder

**[OBSERVED — Bryan]** A control line that works, stops, and works again is
usually **flux residue between the traces**, not a failed component. Old
no-clean flux absorbs moisture and oxidises, and at the microamp currents these
adapters run at, the leakage across a 200 µm gap is enough to matter.

**Isopropyl alcohol, a stiff brush, scrub between the traces, dry thoroughly.**
Do this first, every time. It is the cheaper hypothesis and it is usually the
right one.

### Localise a dead motor channel in one minute

Two measurements on the Driver PCB, both safe (microamps throughout):

| Measurement | Healthy | Faulty |
|---|---|---|
| base → COM, GPIO driven high | **~0.7 V** | 1.678 V — junction not conducting |
| collector → COM, enabled vs disabled | large change | 1.069 → 0.945 V — barely moves |

A base that will not clamp at ~0.7 V means the base–emitter path to COM is
broken. A collector that does not move means the transistor is not switching.

### Swap-test before you theorise

All four Driver PCBs are identical and all four `J4`–`J7` outputs are identical
in form. Move a suspect board to a known-good wheel's output: if the fault
follows the board it is the board; if it stays with the wheel it is the driver,
the motor, or the loom. **This is faster than any measurement** and it needs no
equipment.

---

## 7. Before you power a board you just built

- [ ] **Continuity: every ground pin to every other ground pin.**
- [ ] **No continuity between 3V3 and GND.** A short here with the DevKit fitted
      kills the DevKit's regulator.
- [ ] Transistor orientation matches the `E`/`B`/`C` silkscreen.
- [ ] DevKit sockets are the right way round — pin 1 is the **square pad**.
- [ ] Wheel Drive **J3 wired to the motor supply ground** before anything is
      switched on (`startup.md` §2).

Note there is **no power input** on either ESP board — `3V3` is generated by the
DevKit and flows *outward* to the sensors. So the first power-on is simply
plugging the DevKit's USB in, and the 3V3-to-GND short check above is your only
chance to catch a fault before the DevKit's regulator finds it for you. Do it
with a meter, on the empty socket, before the module goes in.
