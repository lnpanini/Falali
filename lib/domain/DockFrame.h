// Rotates the docking sequence 90 degrees so the robot CRABS IN SIDEWAYS while
// manual driving stays nose-first.
//
// WHY THIS EXISTS RATHER THAN A SECOND STATE MACHINE
// --------------------------------------------------
// DockingStateMachine never refers to a physical corner. It only knows
//   * a LEADING pair, p[0] and p[1], which crosses the near and far edges, and
//   * two SIDE pairs, p[0]&&p[2] and p[1]&&p[3], for the lateral bisection
// and it only ever commands vx (advance) and vy (traverse) in its own frame.
// Nothing in it assumes "front" means the nose of the robot.
//
// So a sideways approach is a change of coordinates, not a change of logic:
// permute which sensors it calls the leading pair, and swap which axis its
// commands drive. Both happen here, at the boundary, and the domain is
// untouched -- which is also why all 70 host tests still cover it.
//
// THE GEOMETRY THIS BUYS
// ----------------------
// The trolley is 860 mm across the entry and 1260 mm along it; the robot's
// sensor rectangle is 780 mm nose-to-tail by 450 mm side-to-side.
//
//   approach     span along travel   trolley   margin about centre
//   nose-first        780              860        +/-  40 mm
//   sideways          450              860        +/- 205 mm
//
// Five times the room. The tight nose-first budget was what made frame rate and
// edge lag critical; crabbing in makes both comfortable.
#pragma once

#include "types.h"

namespace fal {

// Sensor index the docking frame's slot i should read from, for a robot that
// approaches by strafing RIGHT.
//
// Robot frame is +x forward, +y strafe (see DriveCommand); corners sit at
// (+-390, +-225). Rotating so the direction of travel is the strafe axis puts
// the RIGHT-hand pair in the lead:
//
//   dock slot 0 (leading, side A)  <- FR
//   dock slot 1 (leading, side B)  <- RR
//   dock slot 2 (trailing, side A) <- FL
//   dock slot 3 (trailing, side B) <- RL
//
// Check the side pairs come out right: the machine's "left" is slots 0 and 2 =
// FR and FL, the robot's FRONT pair -- one side of the rotated frame, as needed.
// Its "right" is slots 1 and 3 = RR and RL, the robot's REAR pair.
constexpr size_t kSidewaysCornerMap[kNumCorners] = {1, 3, 0, 2};

// Mirror image, for a robot that goes under from its other side. Slots 0/1 swap
// with 2/3 -- the LEFT pair leads instead.
constexpr size_t kSidewaysLeftCornerMap[kNumCorners] = {0, 2, 1, 3};

inline void toDockFrame(const bool robot[kNumCorners], bool dock[kNumCorners],
                        bool strafe_right = true) {
  const size_t* m = strafe_right ? kSidewaysCornerMap : kSidewaysLeftCornerMap;
  for (size_t i = 0; i < kNumCorners; ++i) dock[i] = robot[m[i]];
}

// The matching command rotation. The machine's advance axis becomes the robot's
// strafe, and its traverse axis becomes the robot's fore-aft.
//
// MUST be applied together with toDockFrame and the same strafe_right, or the
// machine drives one way and reads edges from the other -- which does not fail
// loudly, it just bisects nonsense.
//
// omega passes through: yaw is the same rotation in either frame. Note that the
// axis swap is a reflection, so Orient's rotate-toward-the-lagging-corner sign
// may need inverting; it was already marked "verify on hardware".
inline DriveCommand toRobotFrame(const DriveCommand& c, bool strafe_right = true) {
  DriveCommand r;
  r.vx = c.vy;
  r.vy = strafe_right ? c.vx : -c.vx;
  r.omega = c.omega;
  return r;
}

}  // namespace fal
