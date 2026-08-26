// Tunable parameters for the firmware. Kept out of the pure domain so the domain
// stays generic; main.cpp uses these to build the domain config structs.
#pragma once

#include <cstdint>

#include "CornerEdgeDetector.h"
#include "DeadReckonOdometry.h"
#include "DockingStateMachine.h"
#include "LinkWatchdog.h"

namespace cfg {

// --- Loop timing ---
constexpr uint32_t kControlPeriodMs = 20;     // 50 Hz control tick
constexpr uint32_t kTelemetryPeriodMs = 200;  // 5 Hz status publish

// --- Alignment sensor layout: 4 corner ToF, XSHUT re-addressing (NOT a mux) ---
// Zone index order MUST match Corner: FL=0, FR=1, RL=2, RR=3.
//
// The mux constants that used to live here described the ENCODER branches, not
// the ToF -- pointing the ToF driver at them was a real bug, fixed 2026-08-11
// (see the comment on g_tof in src/main.cpp). They are gone with the encoders;
// pins::kMuxAddr remains only for the bring-up bus scan.
constexpr uint8_t kNumZones = 4;

// --- Clamp safety ---
constexpr float kClampStallAmps = 4.0f;  // over-current -> fault (calibrate on bench)

// Per-wheel over-current trip, from the ACS758 -> ADS1115 chain.
//
// The BLD-120A is rated 8 A continuous, 30 A instantaneous (<3 s). 10 A sits
// above any legitimate running current -- the bench measured roughly 0.2 A per
// wheel unloaded -- while staying well under the driver's own instantaneous
// rating, so this trips on something genuinely wrong rather than on a hill.
//
// *** NOT CALIBRATED UNDER LOAD. *** Measured only with wheels off the ground
// on 2026-08-11. Re-check with a loaded trolley before trusting it to protect
// anything, and remember the sensor cannot tell a stall from a heavy load.
constexpr float kWheelStallAmps = 10.0f;

// PER-CORNER ToF OFFSET, in mm, added to every valid reading. FL, FR, RL, RR.
//
// Regenerate with the View button on the gamepad (bench_ble), which averages 16
// frames and prints a replacement for this line.
//
// DERIVED 2026-08-13 from steady all-four-present frames under the trolley:
// FL 78.7, FR 125.1, RL 103.7, RR 102.0. Referenced to FL, so all four now
// report ~78 for the same gap.
//
// These stopped being cosmetic the moment the bands had to separate a real
// return from an edge artefact. Uncorrected, legitimate readings spanned 74-138
// while artefacts at the edge ran 131-296 -- overlapping, so no threshold could
// split them. Corrected, real coverage collapses to ~70-88 and the nearest
// artefact sits at ~108, which a ceiling can get between.
//
// Re-capture with the View button if the brackets are ever disturbed; these came
// off logged frames rather than the averaging routine, so treat them as good to
// a couple of mm, not better.
constexpr int16_t kTofOffsetMm[4] = {0, -46, -25, -23};

// Corner edge detection: the solid board sits within this height band above the
// up-facing sensors. Tune on the bench.
inline fal::CornerConfig makeCornerConfig() {
  fal::CornerConfig c;

  // BANDS SIZED FROM MEASUREMENT, NOT FROM CAUTION. The old 20-400 window was
  // picked before the gap was known, and is wide enough to accept things that
  // are not the trolley -- the arm structure, a low ceiling, a wall on approach.
  //
  // CEILINGS SET BY THE EDGE ARTEFACT, NOT BY THE TROLLEY.
  //
  // A sensor straddling the far edge returns unstable partial-target values --
  // 131, 202, 219, 230, 296 mm logged 60 mm past the edge on 2026-08-13, while
  // the sensors still under the trolley held steady to +/-5 mm. Those artefacts
  // all sat inside the previous 300 mm release ceiling, so every one reset the
  // release counter and the corner clung on long after it had physically
  // cleared. That, not debounce or frame rate, is what made exits feel slow.
  //
  // With kTofOffsetMm applied the two populations finally separate: real
  // coverage lands near 78 (70-88 including the trolley rocking), the nearest
  // artefact near 108, the rest 179 and above.
  //
  // Do not raise these back for safety. A wider band is not more tolerant here,
  // it is just slower to admit the sensor has left the trolley.
  //
  // *** THE TWO CEILINGS ARE EQUAL ON PURPOSE. DO NOT SPLIT THEM. ***
  //
  // The trolley edge is a vertical face, and the sensor's outermost rays graze
  // it from outside. A ray at the 12.5 deg cone edge reaches lateral offset d at
  // slant range d/sin(12.5) -- so the ceiling is not really a distance test, it
  // is a "how far past the edge am I" test:
  //
  //     ceiling 300 -> corner holds until 65 mm past the edge
  //     ceiling 130 -> corner holds until 28 mm past
  //
  // The same grazing rays see the wall BEFORE the sensor reaches the edge, so
  // entry is early by the same amount the exit is late -- and the bisection
  // cancels it exactly, but ONLY while the two ceilings match. A hysteresis gap
  // of 130/160 would make the exit threshold looser than the entry one and put
  // a ~3 mm bias straight into the midpoint.
  //
  // Hysteresis is kept on the FLOOR, where it costs nothing: readings climb at
  // an edge crossing, they do not fall, so the floor never governs a crossing.
  c.assert_min_mm = 45;
  c.assert_max_mm = 130;
  c.band_min_mm = 30;
  c.band_max_mm = 130;

  // 3 each way rather than 2. At the measured 6.4 ms/frame and 0.10 command that
  // is ~2.7 mm of edge lag, which front_offset_mm compensates -- cheap insurance
  // now that the margin is +/-205 mm instead of +/-40.
  c.debounce = 3;
  c.release_debounce = 3;
  return c;
}

// PHYSICAL GEOMETRY, measured on the assembled base 2026-08-13.
//
// The AMR enters the trolley through its long (1260 mm) side, so 860 mm lies
// across the entry and 1260 mm along it. Robot sensor rectangle: 780 mm
// nose-to-tail by 450 mm side-to-side.
//
// *** THE DOCKING SEQUENCE CRABS IN SIDEWAYS. MANUAL DRIVING DOES NOT. ***
// See lib/domain/DockFrame.h -- the sequence runs in a frame rotated 90 degrees
// from the robot, so its travel axis is the robot's STRAFE axis. That is what
// makes the margins liveable:
//
//   approach     span along travel   trolley   margin about centre
//   nose-first        780              860        +/-  40 mm
//   sideways          450              860        +/- 205 mm   <- what we use
//
//   and laterally     780             1260        +/- 240 mm
constexpr float kTrolleyDepthMm = 860.0f;    // across the entry = docking travel axis
constexpr float kTrolleyWidthMm = 1260.0f;   // along the entry  = docking lateral axis
constexpr float kSensorSpanXMm = 780.0f;     // robot front sensor line -> rear
constexpr float kSensorSpanYMm = 450.0f;     // robot left sensor line -> right

// Which way the robot crabs to get under. If Approach times out having never
// seen an edge, this is almost certainly the thing to flip -- it is the only
// setting whose failure mode is "drove confidently away from the trolley".
constexpr bool kDockStrafeRight = true;

// HOW MUCH LONGER THE MEASURED SPAN IS THAN THE TROLLEY.
//
// The trolley edge is a vertical face and the VL53L0X has a ~25 deg cone, so its
// outermost rays graze that face from outside. A ray at the cone edge reaches
// lateral offset d at slant range d/sin(12.5 deg) -- meaning a corner lights up
// before the sensor reaches the edge and holds after it passes, by the same
// distance at each end:
//
//     overshoot per edge = assert_max_mm * sin(12.5 deg) = 130 * 0.2164 ~ 28 mm
//
// Both marks therefore sit OUTSIDE the true edges, so the midpoint is untouched
// (which is the whole reason the two band ceilings are kept equal) but the SPAN
// reads ~56 mm long. Calibrating max_lin_mm_s against a bare 860 would blame the
// drivetrain for the optics and bias the constant ~6% low.
constexpr float kEdgeOverreachMm = 28.1f;                       // per edge
constexpr float kExpectedSpanMm = kTrolleyDepthMm + 2.0f * kEdgeOverreachMm;  // ~916

// Dead-reckoning calibration: platform speed / yaw rate at full command.
inline fal::OdometryCal makeOdometryCal() {
  fal::OdometryCal c;
  // WAS 300, WHICH WAS A GUESS AND WRONG BY ~4.6x.
  //
  // Two independent measurements, and they disagree by 20%:
  //   (a) 3000 mm in 9.05 s at command 0.20  ->  1657 mm/s
  //   (b) CenterX measured a 186.4 mm span (at the old 300) across a trolley
  //       known to be 860 mm deep  ->  300 x 860/186.4 = 1384 mm/s
  //
  // (b) is used. It is measured through the same sensors, at the same speeds,
  // over the same motion the docking sequence actually performs, so it absorbs
  // edge-detection lag and any skew instead of pretending they do not exist.
  // (a) is a stopwatch over a run that included acceleration from rest, which
  // biases it -- though not in the direction that would close the gap, so the
  // discrepancy is real and unexplained. Suspect skew on entry.
  //
  // This constant now MATTERS, where before it cancelled. The bisection midpoint
  // is immune to a scale error (both marks scale together), but front_offset_mm
  // and centre_tol_mm below are REAL millimetres subtracted from an odometry-
  // space position -- a 20% scale error moves the park point by ~70 mm, which is
  // most of the +/-40 mm budget. bench_ble prints the implied value from every
  // CenterX span; refine it there.
  // REVISED 2026-08-13 from two consecutive sideways docks that agreed to 0.5%:
  // spans of 1028.0 and 1032.8, measured at 1384.
  //
  // Compared against kExpectedSpanMm (~916), NOT the bare 860 -- the cone grazes
  // the trolley's vertical edge face and pushes both marks ~28 mm outside the
  // true edges. Charging that to the drivetrain would bias this constant ~6% low.
  //
  //     1384 x 916.2 / 1030.4 = 1231
  //
  // Two independent runs agreeing beats the earlier nose-first estimate and the
  // stopwatch, which never had a confirmed speed setting behind it anyway.
  c.max_lin_mm_s = 1231.0f;
  // STILL UNMEASURED. Only Orient commands omega, and it exits on booleans
  // rather than on angle, so an error here mostly shows up as theta drift
  // rotating the subsequent x/y integration. Measure it before trusting a dock
  // that had to rotate far.
  c.max_ang_rad_s = 1.5f;
  return c;
}

// Docking sequence tuning.
inline fal::DockingConfig makeDockConfig() {
  fal::DockingConfig c;

  // *** front_offset_mm IS NEGATIVE, AND THAT IS NOT A TYPO. ***
  //
  // Both travel-axis edges are found by the LEADING sensor pair, so the raw
  // bisection centres those two sensors on the trolley -- leaving the robot half
  // a sensor span short. Crabbing in, the leading pair is the robot's right-hand
  // side, half a span is 450/2 = 225, and the trolley spans +/-430:
  //
  //   leading sensors cross the near edge (-430) at robot travel-x = -655
  //   leading sensors cross the far  edge (+430) at robot travel-x = +205
  //   midpoint of those robot positions            = -225
  //
  // and the code computes x_target = midpoint - front_offset_mm, so reaching
  // x = 0 needs front_offset_mm = -225. A POSITIVE 225 would drive it 450 mm
  // the wrong way.
  //
  // Confirmed against hardware in the NOSE-FIRST geometry, where the same
  // argument gives -390: with front_offset_mm = 0 the machine backtracked 430 mm
  // and pushed the trailing sensors 350 mm clear of the trolley, exactly as
  // observed on 2026-08-13. Same error, larger lever arm.
  //
  // The +7 dials out edge-detection lag. Both marks are stamped late by the same
  // distance (frame x debounce x speed), so the midpoint inherits that bias
  // whole -- unlike the lateral phase, whose opposed sweeps cancel it. With
  // continuous ranging at ~25 ms, debounce 2 and centre_speed 0.10 that is
  // that. It only holds while approach_speed == centre_speed, hence they match.
  //
  // *** kTofFramePeriodS IS AN ESTIMATE. *** Press L3 on the gamepad; the stream
  // prints the measured us/frame. Continuous ranging should give ~25 ms; if it
  // still reads ~93 ms then startContinuous() did not take and this term is 4x
  // too small.
  // Computed at the END of this function, once centre_speed is set -- the lag
  // term is proportional to it, and hard-coding the speed here is how the two
  // silently drift apart.

  // *** THE BASE ALIGNS ON ONE AXIS ONLY. ***
  //
  // Once the base is under the trolley, the trolley's own wheels block travel
  // along the robot's fore-aft axis -- which is the docking frame's LATERAL
  // axis, since the sequence crabs in. CenterY would command a motion the
  // chassis physically cannot make, burn its 25 s timeout and fault.
  //
  // So CenterX (the crab axis, robot left-right) completes, Confirm checks all
  // four corners, and the remaining axis is handed to the ARM, which has its own
  // sensors and its own travel. Splitting alignment across the two subsystems by
  // which one can actually move in that direction.
  //
  // Unused while this is false -- kept correct in case the geometry changes.
  // Structurally zero anyway: the lateral phase finds its two edges with
  // OPPOSITE sensor lines, so any sensor offset cancels in the midpoint.
  c.centre_lateral = false;
  c.side_offset_mm = 0.0f;

  // *** 0.10 IS A FLOOR SET BY BREAK-AWAY, NOT BY PRECISION. ***
  //
  // 0.05 was tried on 2026-08-13 and the base did not move at all: the console
  // showed duty 205 on all four wheels -- the command arriving correctly -- while
  // the BLD-120A drivers never turned. The lowest speed ever verified to move
  // the assembled base is the gamepad's own bottom limit, 0.10, and the 5%
  // break-away figure in the bench sketch was measured UNLOADED, on a rig
  // without the arm on top.
  //
  // Do not lower these to buy precision. Below break-away the sequence does not
  // run slowly, it does not run at all, and it looks from the console exactly
  // like the docking command was ignored.
  //
  // EQUAL, ON PURPOSE. Different speeds would give the two travel-axis marks
  // different lags, and the front_offset_mm compensation assumes one uniform
  // bias. 0.10 x 1384 = ~138 mm/s, ~10 mm of edge lag against +/-205 mm of
  // margin -- affordable now that the sequence crabs in.
  c.approach_speed = 0.10f;
  c.centre_speed = 0.10f;
  c.rotate_speed = 0.10f;

  // FLIP THIS IF ORIENT PIVOTS ENDLESSLY. It cannot be derived from the corner
  // layout alone, and DockFrame.h's axis swap is a reflection, which inverts it.
  c.rotate_dir = 1.0f;

  // 5 mm, per the arm having its own fine-alignment stage downstream: this only
  // has to land inside that mechanism's capture range, not solve the problem.
  c.centre_tol_mm = 5.0f;

  // Slower speeds need longer deadlines. Approach must cover the run-in, and
  // CenterX must traverse the full 860 mm depth at ~138 mm/s (~6.2 s) before it
  // even starts driving back.
  c.approach_timeout_ms = 20000;
  c.center_timeout_ms = 25000;

  // CENTRE ON OPPOSED EDGES. Bryan's idea, 2026-08-13, and it removes the worst
  // dependence in the whole sequence.
  //
  // The leading pair used to mark BOTH edges, so the midpoint landed half a
  // sensor span short and front_offset_mm = -225 added it back. But -225 is real
  // millimetres subtracted from an odometry-space position: it is only right
  // while max_lin_mm_s is right, and that constant has been estimated anywhere
  // from 1155 to 1657 tonight. A 6% error there moved the park point ~13 mm.
  //
  // Marking the near edge with the TRAILING pair and the far edge with the
  // LEADING pair gives two marks symmetric about the platform centre, so the
  // midpoint IS the centre. No geometry term, and both marks scale with the
  // odometry constant exactly as the current position does -- so the park point
  // is now INDEPENDENT of max_lin_mm_s.
  c.centre_opposed_pairs = true;

  // All that is left to compensate is debounce latency, which does NOT cancel:
  // both marks are stamped late in the direction of travel, so the midpoint
  // inherits the lag whole. At the measured 6.4 ms/frame, debounce 3 and 0.10
  // command that is ~2.7 mm.
  //
  // Optical overreach is absent from this term on purpose. The cone grazes the
  // vertical edge face and makes one mark early and the other late by the same
  // distance, so it cancels in the midpoint -- under either method, and only
  // while makeCornerConfig()'s two band ceilings stay equal.
  constexpr float kTofFramePeriodS = 0.0064f;
  const float debounce_samples = static_cast<float>(makeCornerConfig().debounce);
  c.front_offset_mm = kTofFramePeriodS * debounce_samples *
                      (c.centre_speed * makeOdometryCal().max_lin_mm_s);  // ~2.7
  return c;
}

// Pi 5 control-link watchdog. 100 ms == 5 missed control ticks: long enough to
// ride out ordinary Linux scheduling jitter, short enough that the platform
// travels only ~30 mm at full speed before the brakes go on.
inline fal::LinkConfig makeLinkConfig() {
  fal::LinkConfig c;
  c.timeout_ms = 100;
  return c;
}

} // namespace cfg
