# Local changes to `sks0826/Arm_code`

Everything here differs from upstream commit `ca81eef`. Three changes, all in
service of the base being able to drive this firmware over ESP-NOW.

**None of it has been upstreamed.** If Kai Xiang or Heng Li merge these, re-vendor
from their commit and delete this file rather than keeping two copies in step.
`docs/arm-espnow-jog.patch` is the same diff in applicable form.

---

## 1. `mstop` above the busy check — `src/main.cpp`

```cpp
  if (command == "estop") { emergencyStopEverything(true); return; }
+ if (command == "mstop") { stopTravelMotors(); ... sendWirelessStatus("stopped"); return; }

  if (isBusyForWirelessCommand()) { sendWirelessStatus("busy"); return; }
```

A running jog sets `xJog.active` / `yJog.active`, which makes
`isBusyForWirelessCommand()` true. A stop arriving from the remote would
therefore be answered `busy` and refused, and the jog would run its full 3 s
timeout regardless — silently turning a held-to-move control into a fixed burst.

`estop` was already hoisted for the same reason. `mstop` needs it for the same
reason.

## 2. Unknown wireless commands fall through to the serial handler

```cpp
  } else {
-   sendWirelessStatus("rejected:unknown_command");
+   processCommand(command);
  }
```

`processCommand()` already implements the manual jogs (`xext`/`xret`/`yext`/
`yret`), the per-axis flips (`fx`/`fy`/`ex`/`ey`) and everything else this
firmware can do. They were unreachable from the radio only because the two
handlers are separate functions — the logic existed, the door didn't.

One line exposes the lot, and the next command added to the serial set works
remotely for free instead of needing a matching string compare.

**It does mean `clamp`, `limit` and the raw servo sweeps are now reachable over
the air.** Fine between two paired boards on a bench. Reconsider if this ever
runs somewhere a stray packet would matter — the receiver already filters on
sender MAC, so the exposure is bounded by that.

## 3. The servo flip cache is invalidated when we lose control

`src/MotorController.{h,cpp}` — added `flipStateKnown_`.

`xAxisFlipped_` / `yAxisFlipped_` record what was last **commanded**, and the
setters skip the move when the request already matches:

```cpp
  if (yAxisFlipped_ == flipped) {
    return true;            // reports SUCCESS without moving
  }
```

That is a useful shortcut until the servos stop obeying. `stopAll()` cuts PWM on
all four channels and an aborted ramp leaves them part-way; neither reset the
flags. So after any E-stop the cache still claimed a position the arms were no
longer holding, the next flip saw its request already satisfied, returned success
without moving, and the cycle went straight to retract.

Observed on hardware 2026-08-14 as *"the Y flippers didn't come up after
extending, but it still retracted when the limit switch was hit."* The retract
worked because nothing was wrong with the retract.

Reachable without an E-stop too: flip a pair by hand, then run `grab`, and grab's
flip is skipped.

Fixed with a **validity flag** rather than by resetting the position — resetting
has to pick a value, and either choice is wrong half the time. Claiming "home"
after PWM is cut just moves the silent no-op onto the next *home* command
instead. The cache is now used only when it is both matching **and** known good:
`stopAll()` clears it, `begin()` sets it after driving all four to home, and a
completed ramp sets it.

---

## Known upstream issues NOT fixed here

These are real and were left alone because they are the authors' to decide on.
Both are described in full in [`../docs/gotchas.md`](../docs/gotchas.md).

- **An out-of-range ToF reading counts as "clear air."** `TOF_OUT_OF_RANGE_MM`
  is 8191 and the air test is `> 200`, so a disconnected, failed or
  pointed-at-nothing sensor reports the exact condition that lets the arm stop
  extending. The sensor's failure mode is indistinguishable from its all-clear.
- **Extend trusts one limit switch; retract trusts three.** `isXExtendBlocked`
  reads `X_ARM_MAX` (GPIO42) alone, while `isXRetractBlocked` reads
  `X_ARM_MIN` ∥ `LIMIT_X1` ∥ `LIMIT_X2`. Extend has no redundancy, and a working
  retract stop tells you nothing about whether extend has one.
