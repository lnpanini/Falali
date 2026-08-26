# Bench BLE Gamepad Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive the mecanum bench car with an Xbox BLE controller (proportional analog), keeping the existing serial WASD jog and auto-align intact via one shared logic core.

**Architecture:** The bench drivetrain + ToF + auto-align logic moves into a header-only shared core (`src/bench_core.h`, with the pure mix math in Arduino-free `src/bench_mix.h`). Two thin frontends include it: the existing serial build (`src/bench_check.cpp`, `[env:bench]`) and a **separate** Bluepad32 project (`bench_ble/`, an ESP-IDF+Arduino-component project from Bluepad32's official template) whose `arduino_main.cpp` reads the Xbox pad.

**Tech Stack:** ESP32-S3-WROOM-1 N16R8 · PlatformIO · Arduino-ESP32 (serial build) · Bluepad32 on ESP-IDF+Arduino via the `pioarduino` platform (gamepad build) · Pololu VL53L0X · Unity (host tests).

## Global Constraints

- **C++17** everywhere: `-std=gnu++17` (both builds); enables `inline` variables in the shared header.
- **Board:** `esp32-s3-devkitc-1`, `board_build.arduino.memory_type = qio_opi` (N16R8 octal PSRAM; GPIO26–37 reserved).
- **Serial = native USB CDC** on the serial build: `-DARDUINO_USB_CDC_ON_BOOT=1 -DARDUINO_USB_MODE=1`.
- **Every `Serial` write is guarded with `if (Serial) { … }`** — the board runs untethered; an unguarded CDC write with no host can stall the control loop.
- **Motor rail (L298N VS) ≤ ~7.5 V** for the 6 V motors (12 V over-currents the L298N and can kill the ESP).
- **Controller:** Xbox Wireless **model 1914** (Series X/S), **firmware ≥ v5.15**, BLE-only. Pair = hold Pair button ~3 s.
- **Shared header rule:** `bench_core.h` defines state as C++17 `inline` — safe, but still include it from **one** frontend `.cpp` per project.
- **Do not modify** `lib/` (production firmware) or the auto-align *algorithm* — only relocate it.

---

### Task 1: Pure mecanum mix + host test

Extract the mix math into an Arduino-free header and prove the normalization with a host test before any hardware or refactor.

**Files:**
- Create: `src/bench_mix.h`
- Create: `test/test_bench_mix/test_bench_mix.cpp`
- Modify: `platformio.ini` (`[env:native]` — add `-I src` so the test finds the header)

**Interfaces:**
- Produces: `void mixWheels(float vx, float vy, float w, float out[4])` — writes normalized wheel commands in `[-1,1]` for `{FL,FR,RL,RR}` (O-config), normalized by max magnitude so blends don't clip.

- [ ] **Step 1: Write the failing test**

`test/test_bench_mix/test_bench_mix.cpp`:
```cpp
#include <unity.h>
#include "bench_mix.h"

void test_forward_all_equal() {
  float o[4]; mixWheels(1.0f, 0.0f, 0.0f, o);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, o[0]); TEST_ASSERT_EQUAL_FLOAT(1.0f, o[1]);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, o[2]); TEST_ASSERT_EQUAL_FLOAT(1.0f, o[3]);
}
void test_rotate_cw_signs() {              // O-config: FL+ FR- RL+ RR-
  float o[4]; mixWheels(0.0f, 0.0f, 1.0f, o);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, o[0]); TEST_ASSERT_EQUAL_FLOAT(-1.0f, o[1]);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, o[2]); TEST_ASSERT_EQUAL_FLOAT(-1.0f, o[3]);
}
void test_blend_is_normalized() {          // fwd + CW: raw fl=2 -> divide by 2
  float o[4]; mixWheels(1.0f, 0.0f, 1.0f, o);
  for (int i = 0; i < 4; ++i) { TEST_ASSERT_TRUE(o[i] <= 1.0f && o[i] >= -1.0f); }
  TEST_ASSERT_EQUAL_FLOAT(1.0f, o[0]); TEST_ASSERT_EQUAL_FLOAT(0.0f, o[1]);
}
void test_deadcenter_is_zero() {
  float o[4]; mixWheels(0.0f, 0.0f, 0.0f, o);
  for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_FLOAT(0.0f, o[i]);
}
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_forward_all_equal); RUN_TEST(test_rotate_cw_signs);
  RUN_TEST(test_blend_is_normalized); RUN_TEST(test_deadcenter_is_zero);
  return UNITY_END();
}
```

- [ ] **Step 2: Add the `-I src` include path to the native env**

In `platformio.ini`, under `[env:native]` `build_flags`, add `-I src`:
```ini
build_flags =
    -std=gnu++17
    -Wall
    -Wextra
    -I src
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `pio test -e native -f test_bench_mix`
Expected: FAIL — `bench_mix.h` not found / `mixWheels` undefined.

- [ ] **Step 4: Implement `src/bench_mix.h`**

```cpp
#pragma once
#include <algorithm>
#include <cmath>

// Pure mecanum mix (O-config rollers), Arduino-free so it is host-testable.
// vx,vy,w in [-1,1]; writes normalized wheel commands in [-1,1] to out[FL,FR,RL,RR].
// Normalize by the max magnitude so blended translate+rotate scales together (no clip).
inline void mixWheels(float vx, float vy, float w, float out[4]) {
  const float fl = vx + vy + w;
  const float fr = vx - vy - w;
  const float rl = vx - vy + w;
  const float rr = vx + vy - w;
  const float m = std::max(1.0f, std::max(std::max(std::fabs(fl), std::fabs(fr)),
                                          std::max(std::fabs(rl), std::fabs(rr))));
  out[0] = fl / m; out[1] = fr / m; out[2] = rl / m; out[3] = rr / m;
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `pio test -e native -f test_bench_mix`
Expected: PASS — 4 tests.

- [ ] **Step 6: Commit**

```bash
git add src/bench_mix.h test/test_bench_mix/test_bench_mix.cpp platformio.ini
git commit -m "feat(bench): pure host-tested mecanum mix (mixWheels)"
```

---

### Task 2: Shared core header + serial frontend refactor

Move the drivetrain, ToF, presence, and auto-align state machine out of `bench_check.cpp` into `src/bench_core.h`, exposing a small intent API; rewrite `bench_check.cpp` as a thin serial frontend. The deliverable's test is a **hardware regression**: the serial build must still drive via WASD and auto-align exactly as before.

**Files:**
- Create: `src/bench_core.h`
- Modify: `src/bench_check.cpp` (becomes the serial frontend)
- Reference (unchanged): `platformio.ini` `[env:bench]`

**Interfaces:**
- Consumes: `mixWheels` (Task 1).
- Produces (from `bench_core.h`):
  - `void coreSetup();` — init motors + I2C + ToF; prints banner (guarded).
  - `void coreUpdate(float vx, float vy, float w, bool alignStart, bool alignAbort);` — one iteration: applies the proportional manual intent in MANUAL, runs the auto-align state machine, prints status (guarded). `vx,vy,w ∈ [-1,1]`; `alignStart/alignAbort` are one-shot edges.
  - Calibration helpers for the serial CAL keys: `void coreSpinWheel(uint8_t i);` `void coreFlipWheel(uint8_t i);` `void corePrintInvert();` `void coreStop();` `void coreSpeedStep(int delta);`
  - `int g_speed;` (inline, manual max duty).

- [ ] **Step 1: Create `src/bench_core.h` by relocating the tested blocks**

Create `bench_core.h` and move these blocks **verbatim (no logic change)** out of the current `src/bench_check.cpp` into it, marking file-scope variables and functions `inline`:
- Pins: `MotorPins`, `MOTOR[4]`, `WHEEL[4]` (`src/bench_check.cpp:28-37`).
- Alignment tunables (`:40-57`) and add `static const int MANUAL_FULL_DUTY = 255;`.
- ToF: `tofA/tofB/okA/okB/useMux`, `muxSelect`, `i2cPresent`, `i2cScan`, `initOne`, `bringUpToF`, `readOne` (`:59-136`).
- Presence: `struct Presence`, `presA/presB` (`:138-152`).
- Motors: `g_speed`, `g_invert[4]`, `g_sel`, `wheel`, `stopAll` (`:154-183`).
- Mode/odometry/state-machine helpers: `enum class Mode`, `mode`, timers, `prev_any`, `auto_trigger`, odometry (`odom_x`,`x_far`,`odoReset`,`odoStep`), `modeName`, `setMode`, `startOrient`, `abortToManual` (`:196-249`).

Header preamble and includes:
```cpp
#pragma once
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>
#include "bench_mix.h"
```

- [ ] **Step 2: Add the proportional mix + integer wrapper to `bench_core.h`**

Replace the old integer-only `driveMix` with:
```cpp
inline void driveMixF(float vx, float vy, float w, int spd) {
  float o[4];
  mixWheels(vx, vy, w, o);
  for (uint8_t i = 0; i < 4; ++i) wheel(i, (int)lroundf(o[i] * spd));
}
// Integer wrapper preserves every existing state-machine call site unchanged.
inline void driveMix(int vx, int vy, int w, int spd) {
  driveMixF((float)vx, (float)vy, (float)w, spd);
}
```

- [ ] **Step 3: Add `coreSetup()` to `bench_core.h`**

Relocate the body of the old `setup()` (motor pinModes + `stopAll` + `Wire.begin`/`setClock` + `i2cScan` + `bringUpToF`), guarding the banner print:
```cpp
inline void coreSetup() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(MOTOR[i].in1, OUTPUT); pinMode(MOTOR[i].in2, OUTPUT); pinMode(MOTOR[i].en, OUTPUT);
  }
  stopAll();
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  if (Serial) i2cScan();
  bringUpToF();
}
```
(Also move `PIN_SDA/PIN_SCL/PIN_XSHUT_A/PIN_XSHUT_B/MUX_ADDR` consts into the header.)

- [ ] **Step 4: Add `coreUpdate()` to `bench_core.h`**

The old `loop()` body becomes `coreUpdate`, with the passed intent replacing "set-and-hold from serial". Show the new top + MANUAL case; the `Orient`/`CenterSeek`/`CenterReturn`/`Centered` cases and the edge-event/heartbeat prints move **verbatim from the old `loop()` (`:394-490`)**, except every `Serial.print*` is wrapped in `if (Serial)`:
```cpp
inline void coreUpdate(float vx, float vy, float w, bool alignStart, bool alignAbort) {
  if (alignAbort) abortToManual("cmd");
  const bool manualActive = (vx != 0.0f || vy != 0.0f || w != 0.0f);
  if (manualActive && mode != Mode::Manual) { setMode(Mode::Manual); }

  bool vA = false, vB = false;
  const uint16_t mmA = readOne(tofA, okA, 0, &vA);
  const uint16_t mmB = readOne(tofB, okB, 1, &vB);
  presA.update(mmA, vA); presB.update(mmB, vB);
  const bool a = presA.present, b = presB.present, any = a || b;

  if (alignStart && mode == Mode::Manual) startOrient("cmd");
  if (mode == Mode::Manual && auto_trigger && any && !prev_any) startOrient("edge sensed");
  prev_any = any;

  switch (mode) {
    case Mode::Manual:
      driveMixF(vx, vy, w, g_speed);   // proportional; zero intent => stop
      break;
    // ... Orient / CenterSeek / CenterReturn / Centered: move verbatim from old loop() ...
  }
  // ... edge-event prints + 400 ms heartbeat: move verbatim, each Serial.* wrapped in if (Serial) ...
}
```

- [ ] **Step 5: Add the calibration helpers to `bench_core.h`**

```cpp
inline void coreStop() { setMode(Mode::Manual); stopAll(); }
inline void coreSpeedStep(int delta) { g_speed = constrain(g_speed + delta, 0, 255); }
inline void coreSpinWheel(uint8_t i) { g_sel = i; setMode(Mode::Manual); stopAll(); wheel(i, g_speed); }
inline void coreFlipWheel(uint8_t i) { if (i < 4) { g_invert[i] = !g_invert[i]; wheel(i, g_speed); } }
inline void corePrintInvert() {
  if (Serial) Serial.printf("> invert {FL,FR,RL,RR}={%d,%d,%d,%d}\n",
                            g_invert[0], g_invert[1], g_invert[2], g_invert[3]);
}
```

- [ ] **Step 6: Rewrite `src/bench_check.cpp` as the serial frontend**

Replace the whole file with an include of the core + a serial keymap that maintains a set-and-hold intent and forwards edges. Full file:
```cpp
// Serial WASD frontend over the shared bench core (src/bench_core.h).
//   pio run -e bench -t upload   then   python teleop.py
#include "bench_core.h"

static float g_vx = 0, g_vy = 0, g_w = 0;   // set-and-hold manual intent

static void handleSerial(bool &alignStart, bool &alignAbort) {
  while (Serial.available()) {
    const char c = Serial.read();
    switch (c) {
      case 'w': g_vx = 1;  g_vy = 0;  g_w = 0; break;
      case 's': g_vx = -1; g_vy = 0;  g_w = 0; break;
      case 'a': g_vx = 0;  g_vy = -1; g_w = 0; break;
      case 'd': g_vx = 0;  g_vy = 1;  g_w = 0; break;
      case 'q': g_vx = 0;  g_vy = 0;  g_w = -1; break;
      case 'e': g_vx = 0;  g_vy = 0;  g_w = 1; break;
      case ' ': g_vx = g_vy = g_w = 0; coreStop(); break;
      case 'g': alignStart = true; break;
      case 'x': g_vx = g_vy = g_w = 0; alignAbort = true; break;
      case 't': /* toggle auto-trigger */ extern bool auto_trigger; auto_trigger = !auto_trigger; break;
      case '+': coreSpeedStep(+20); break;
      case '-': coreSpeedStep(-20); break;
      case '1': case '2': case '3': case '4': coreSpinWheel(c - '1'); break;
      case 'f': if (g_sel >= 0) coreFlipWheel(g_sel); break;
      case 'p': corePrintInvert(); break;
      default: break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  if (Serial) Serial.println("\n# Falali bench (serial WASD + auto-align)");
  coreSetup();
}

void loop() {
  bool alignStart = false, alignAbort = false;
  handleSerial(alignStart, alignAbort);
  coreUpdate(g_vx, g_vy, g_w, alignStart, alignAbort);
}
```
(Note: `auto_trigger` and `g_sel` are `inline` in the core; the `extern` line references the core's `auto_trigger`. If cleaner, add a `coreToggleAuto()` helper instead.)

- [ ] **Step 7: Build the serial firmware**

Run: `pio run -e bench`
Expected: SUCCESS, links clean.

- [ ] **Step 8: Hardware regression — flash & drive**

Motor rail at ~7.5 V. Run: `pio run -e bench -t upload`, then `python teleop.py`.
Expected: WASD moves correct directions; `q/e` rotate; `+/-` change speed; `1-4/f/p` calibrate; holding a board over a sensor auto-triggers ORIENT → CENTER_X; `x` aborts. Identical to pre-refactor behavior.

- [ ] **Step 9: Commit**

```bash
git add src/bench_core.h src/bench_check.cpp
git commit -m "refactor(bench): shared header-only core + thin serial frontend"
```

---

### Task 3: Stand up the Bluepad32 project & pair the Xbox pad

Prove the hardest external dependency — the ESP-IDF+Arduino+Bluepad32 toolchain and BLE pairing — in isolation, with the template's stock example, before any of our logic.

**Files:**
- Create: `bench_ble/` (from the template) with `bench_ble/platformio.ini`, `bench_ble/src/arduino_main.cpp` (stock), `bench_ble/SETUP.md`
- Create: `.gitignore` entries for `bench_ble/.git`, `bench_ble/components/`, `bench_ble/.pio/`, `bench_ble/sdkconfig.*` build artifacts
- Create: `bench_ble/SETUP.md` documenting the clone/init commands

- [ ] **Step 1: Clone the official template into `bench_ble/`**

Run:
```bash
git clone --recursive https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template bench_ble
rm -rf bench_ble/.git          # de-nest: make it part of the Falali repo
```

- [ ] **Step 2: Record the setup in `bench_ble/SETUP.md`**

```markdown
# bench_ble setup
Generated from https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template
(ESP-IDF v5.4.2, pioarduino platform). `components/` (arduino, bluepad32, btstack)
are vendored by the template and gitignored — re-fetch with:
    git clone --recursive <template-url> tmp && rsync -a tmp/components/ components/ && rm -rf tmp
Build/flash: `pio run -e esp32-s3-devkitc-1 -t upload`
```

- [ ] **Step 3: Gitignore the vendored bulk**

Append to the repo root `.gitignore`:
```
bench_ble/.pio/
bench_ble/components/
bench_ble/sdkconfig.esp32-s3-devkitc-1
bench_ble/dependencies.lock
```

- [ ] **Step 4: Select the S3 board & build the stock example**

Confirm `bench_ble/platformio.ini` has an `[env:esp32-s3-devkitc-1]` section (it does per the template). Then run:
```bash
cd bench_ble && pio run -e esp32-s3-devkitc-1
```
Expected: SUCCESS (first run downloads the pioarduino toolchain — slow).

- [ ] **Step 5: Flash & pair the Xbox pad**

Run: `cd bench_ble && pio run -e esp32-s3-devkitc-1 -t upload`, then monitor the native USB port with `python ../teleop.py` (or `pio device monitor`). Turn on the Xbox 1914 pad and hold Pair ~3 s.
Expected: a `CALLBACK: Controller is connected` line, then button/axis dumps as you press buttons. If it never connects, verify controller firmware ≥ v5.15.

- [ ] **Step 6: Commit**

```bash
git add bench_ble/platformio.ini bench_ble/src/arduino_main.cpp bench_ble/SETUP.md .gitignore
git commit -m "feat(bench_ble): Bluepad32 ESP-IDF project scaffolding; Xbox pad pairs"
```

---

### Task 4: Gamepad frontend — proportional drive via the shared core

Wire the shared core into the Bluepad32 project and drive the car with the left/right sticks.

**Files:**
- Modify: `bench_ble/platformio.ini` (add VL53L0X lib + include path to `../src`)
- Modify: `bench_ble/src/arduino_main.cpp` (replace stock body with the gamepad frontend)

**Interfaces:**
- Consumes: `coreSetup`, `coreUpdate`, `g_speed` from `../src/bench_core.h`; Bluepad32 `BP32`, `ControllerPtr`.

- [ ] **Step 1: Add VL53L0X + the shared-core include path**

In `bench_ble/platformio.ini` under `[env:esp32-s3-devkitc-1]`:
```ini
build_flags =
    -std=gnu++17
    -I ${PROJECT_DIR}/../src        ; find bench_core.h / bench_mix.h
lib_deps =
    pololu/VL53L0X
```
(If the ESP-IDF component build rejects `lib_deps`, add VL53L0X under `bench_ble/components/` as an Arduino library component instead — note this in `SETUP.md`.)

- [ ] **Step 2: Write the gamepad frontend `bench_ble/src/arduino_main.cpp`**

```cpp
// Xbox BLE gamepad frontend over the shared bench core.
#include <Arduino.h>
#include <Bluepad32.h>
#include "bench_core.h"

static ControllerPtr g_ctl = nullptr;

static void onConnect(ControllerPtr c)    { if (!g_ctl) g_ctl = c; }
static void onDisconnect(ControllerPtr c) { if (g_ctl == c) g_ctl = nullptr; }

static const int   AXIS_MAX   = 512;    // Bluepad32 sticks: -512..511
static const float DEADZONE   = 0.12f;  // fraction of full scale

// Map a raw axis to [-1,1] with deadzone; small values snap to 0.
static float axisNorm(int raw) {
  float v = (float)raw / AXIS_MAX;
  if (v > 1.f) v = 1.f; else if (v < -1.f) v = -1.f;
  if (fabsf(v) < DEADZONE) return 0.f;
  return (v - (v > 0 ? DEADZONE : -DEADZONE)) / (1.f - DEADZONE);
}

void setup() {
  Serial.begin(115200);
  BP32.setup(&onConnect, &onDisconnect);
  BP32.forgetBluetoothKeys();     // optional: clean pairing during bring-up
  coreSetup();
}

void loop() {
  BP32.update();
  float vx = 0, vy = 0, w = 0;
  bool alignStart = false, alignAbort = false;
  if (g_ctl && g_ctl->isConnected() && g_ctl->isGamepad()) {
    vx = -axisNorm(g_ctl->axisY());   // up = forward
    vy =  axisNorm(g_ctl->axisX());   // right = strafe right
    w  =  axisNorm(g_ctl->axisRX());  // right stick X = rotate CW
    alignStart = g_ctl->a();
    alignAbort = g_ctl->b();
  }
  coreUpdate(vx, vy, w, alignStart, alignAbort);
  delay(5);
}
```

- [ ] **Step 3: Build**

Run: `cd bench_ble && pio run -e esp32-s3-devkitc-1`
Expected: SUCCESS; `bench_core.h` + VL53L0X resolve.

- [ ] **Step 4: Hardware check — proportional drive**

Motor rail ~7.5 V. Flash (`-t upload`), pair the pad.
Expected: left stick drives fwd/back/strafe on the correct wheels; partial stick = visibly slower than full; right stick X rotates the correct way. Fix any reversed sign by negating that axis in Step 2 (`vx/vy/w`) and note it.

- [ ] **Step 5: Commit**

```bash
git add bench_ble/platformio.ini bench_ble/src/arduino_main.cpp
git commit -m "feat(bench_ble): proportional Xbox-stick drive via shared core"
```

---

### Task 5: Fail-safe + align buttons

Make the wireless link fail **stopped**, and wire the align buttons — the safety-critical deliverable.

**Files:**
- Modify: `bench_ble/src/arduino_main.cpp`

**Interfaces:**
- Consumes: `coreStop` (add to `bench_core.h` if not already), `millis()`.

- [ ] **Step 1: Add a stale-input watchdog + disconnect stop**

Update `loop()` and the disconnect handler in `bench_ble/src/arduino_main.cpp`:
```cpp
static uint32_t g_last_input_ms = 0;
static const uint32_t INPUT_WATCHDOG_MS = 300;

static void onDisconnect(ControllerPtr c) {
  if (g_ctl == c) { g_ctl = nullptr; coreStop(); }   // fail stopped
}

// inside loop(), after BP32.update():
if (g_ctl && g_ctl->isConnected()) {
  g_last_input_ms = millis();
} else if (millis() - g_last_input_ms > INPUT_WATCHDOG_MS) {
  vx = vy = w = 0;          // no fresh pad data -> command stop
}
```

- [ ] **Step 2: Build**

Run: `cd bench_ble && pio run -e esp32-s3-devkitc-1`
Expected: SUCCESS.

- [ ] **Step 3: Hardware check — fail-safe**

Flash, pair, drive forward, then: (a) **power the controller off mid-drive** → motors stop within ~300 ms; (b) walk the pad out of BLE range → motors stop; (c) press **A** → ORIENT starts; **B** → aborts to MANUAL; (d) **unplug USB while driving** → loop keeps running smoothly (guarded serial writes; no stall).

- [ ] **Step 4: Commit**

```bash
git add bench_ble/src/arduino_main.cpp
git commit -m "feat(bench_ble): link-loss fail-safe (disconnect + 300ms watchdog) and align buttons"
```

---

## Verification (whole plan)

- `pio test -e native -f test_bench_mix` — mix math passes (Task 1).
- `pio run -e bench -t upload` — serial WASD + auto-align unchanged (Task 2, regression).
- `cd bench_ble && pio run -e esp32-s3-devkitc-1 -t upload` — gamepad drive, align buttons, and fail-safe all verified on hardware (Tasks 3–5).
- Both builds share `src/bench_core.h` + `src/bench_mix.h` — no logic duplication.

## Self-review notes

- **Spec coverage:** §3 architecture → Tasks 2–4 (shared core + two frontends); §4 proportional map → Tasks 1,4; §4.1 driveMixF → Tasks 1,2; §5 coexistence/`if(Serial)` → Task 2; §6 fail-safe → Task 5; §8 verification → per-task HIL steps + final section.
- **Sign conventions** are explicitly flagged as verify-and-fix on hardware (Task 4 Step 4), matching the spec's verify-on-hardware note.
- **Known risk:** if the ESP-IDF component build won't take `lib_deps` for VL53L0X, fall back to a component (Task 4 Step 1 note).
