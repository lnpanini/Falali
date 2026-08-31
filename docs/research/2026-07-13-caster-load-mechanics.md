# Moving a 4-Swivel-Caster Trolley: Mechanics & Best Practice

**Research report — 2026-07-13**
**Context:** Falali is a small ESP32-S3 under-ride AMR (4-wheel mecanum, bench rig on L298N, ~300 mm/s class) that drives beneath a textile trolley on **four swivel casters**, clamps its underside, and moves it. Target load: **up to 200 kg**, indoor flat floor. This report answers "what is the best way to move it" from both a **physics** and a **control-strategy** angle.

**How to read the confidence tags.** Every quantitative claim below is tagged:
- **[V]** — verified by 3-vote adversarial check against the primary source (survived refutation).
- **[S]** — single-source / sourced-but-unverified: the claim comes from a real fetched source but its independent verification did not complete (the research run hit a session limit mid-verify). Treat as credible-but-uncorroborated.
- **[E]** — engineering reasoning / worked calculation (two independent LLM analyses via Gemini + local arithmetic), not an empirical measurement.
- **[O]** — observed on the assembled robot, or read from the project's own
  requirements. Added 2026-08-31; **outranks [E].**

---

> # CORRECTION 2026-08-31 — built on inputs the project never specified
>
> This report concludes that a horizontal-only clamp *"will spin its wheels and go
> nowhere."* That does not describe this machine, and the reason is the inputs,
> not the arithmetic.
>
> | | Assumed here | Specified by the project **[O]** |
> |---|---|---|
> | Robot mass | **5–15 kg** | **≥ 25 kg** (delivered ~30 kg) |
> | Load to move | **200 kg** | **100 kg** |
>
> A **4× error in the traction ratio**, and the ratio is the whole argument.
> Source: `docs/term-paper-falali-amr.pdf`, Table 1.
>
> **The project had already answered the traction constraint — with mass.** Table 1
> specifies *"AMR total mass ≥ 25 kg — Drive wheels maintain grip without slipping
> under load."* That requirement **is** the mitigation, chosen instead of a lifting
> clamp. `docs/pdr-falali-amr.pdf` lists *"uses the trolley's own caster wheels and
> the AMR never lifts the trolley"* under **Innovation In Our Project**.
>
> So the recommendation below — transfer weight or lift — proposes an approach the
> project deliberately rejected, for a problem it had already solved another way.
>
> **[O] Outcome:** the delivered robot passed its 100 kg drive test clamping
> horizontally, transferring no weight, on dry concrete — and went past it,
> moving **~150 kg** (three adults on a trolley platform, mass estimated rather
> than weighed). The requirement was exceeded by roughly 1.5×, in the regime this
> report said was impossible.
>
> The original text is left intact below. A report that quietly agrees with events
> it got wrong teaches nothing, and this failure mode — a model whose assumed
> range never overlapped the specification — is worth leaving visible.

## TL;DR — the best way to move it

1. **The binding constraint is your robot's own wheel traction, not motor torque.** [E] **This part held up, and the project acted on it — by specifying a ≥ 25 kg robot.** A light mecanum robot (5–15 kg) simply cannot put enough force into the floor to start a 200 kg trolley rolling. Motors that spin the wheels are trivial; the wheels *slip* long before the trolley moves.

2. ~~**Therefore the clamp must have a vertical load path — it must transfer trolley weight onto the robot's wheels — not just grip horizontally.**~~ **[SUPERSEDED — see the correction above.]** [E] Transferring ~**20–30 % of the trolley's weight (≈40–60 kg)** onto the robot is the minimum to move it reliably; **fully lifting the casters off the floor** is the robust industrial answer. ~~A horizontal-only clamp will spin its wheels and go nowhere.~~ **[O] The delivered horizontal-only clamp passed the 100 kg drive test.** The conclusion followed from a 5–15 kg robot against 200 kg; against ~30 kg and 100 kg it does not.

3. **Lifting solves two problems at once.** Adding weight to the robot fixes traction *and* — because unloaded swivel casters rotate freely to a home/trailing position — it eliminates the brutal "caster-flip" torque you'd otherwise fight on every direction change. [S, patent US 12,472,772]

4. **If you cannot lift, then: keep casters pre-aligned, drive in arcs (never rotate-in-place under load), pivot about the load's geometric centre, and cap acceleration.** These mitigations reduce — but do not remove — the traction problem. [E]

5. **A rigid centered under-ride is directionally stable** (no hitch to jackknife), but an all-swivel load has essentially zero lateral constraint, so it drifts sideways under any side force (slope, bump) up to the robot's lateral traction limit — which for mecanum wheels is its *weakest* axis. Closed-loop heading correction is required. [E, S]

---

## Part A — Physics

### A1. Rolling resistance: how much force just to keep it rolling

The governing model (manufacturer-standard) is a lever-arm relation:

> **F = f · W / R**, where *f* = coefficient of rolling friction (in length units), *W* = load on the wheel, *R* = wheel radius. Resistance scales **linearly with load** and **inversely with wheel radius** — bigger wheels roll easier. **[V]** (Hamilton Caster White Paper No. 11)

For traction budgeting it's easier to use the dimensionless **C_rr = push force / vertical load**. Measured/spec values for industrial casters:

| Wheel tread | C_rr (steady-state) | Source |
|---|---|---|
| Forged steel | 0.01–0.015 | CasterHQ [S] |
| Phenolic | 0.02–0.03 | CasterHQ [S] |
| **Hard 95A polyurethane** (common) | **0.025–0.035** | CasterHQ [S] |
| Nylon on concrete | 0.03–0.04 | Bulldog [S] |
| Soft 70A rubber | 0.06–0.08 | CasterHQ [S] |
| AMR-grade PU (target) | **≤ 0.018–0.02** | JessCaster [S] |

Material ranking is corroborated: **softer/rubber ≈ 30–45 % higher rolling resistance than hard 95A PU**, and rubber has the highest coefficient, hard materials the lowest [V for the qualitative ranking, S for the 30–45 % figure]. **Nylon < PU < rubber** for push force at equal load [S, Blickle].

**Real-world anchor:** a ~227 kg cart (near our 200 kg case) measured **~200 N** sustained push on standard 4″ 80A plain-bearing casters (C_rr ≈ 0.09), dropping to **~107 N** on 6″ 95A sealed-bearing ergonomic casters (C_rr ≈ 0.048). **[S, CasterHQ]** Small, cheap, soft wheels roughly **double** the force budget versus large hard sealed-bearing wheels.

**Planning band for a 200 kg trolley (W ≈ 1962 N):** sustained rolling force ≈ **40–120 N** depending entirely on wheel size/material/bearing. Use **~60 N (C_rr ≈ 0.03)** as a central estimate, ~120 N as a pessimistic (small soft wheels) case.

### A2. Breakaway and the "caster flip" — the peaks that actually break traction

- **Breakaway (starting) force ≈ 2–2.5× sustained rolling force**, dropping to sustained once moving. **[V, Hamilton]** (Bulldog says 2–3× [S].) So at C_rr 0.03 the ~60 N cruise becomes **~120–150 N to break away.**
- **Misaligned casters make it far worse.** If casters point across the intended travel direction (their natural state after stopping/reversing), they must *swivel-scrub* to reorient. The worst case is casters perpendicular to travel [S, IEEE 7158298]. Real automotive-plant carts needed **>267 N initial push** with poor swivel casters, reduced to <200 N with better wheels [S, CasterConcepts]. That's **~10–13 % of load weight** as a startup spike — i.e. **~150–260 N for our 200 kg trolley.** [S + E]
- **Swivel offset (trail) is the key geometric lever:** larger swivel offset *reduces* the force to reorient a caster (τ = r × F, bigger moment arm) [S, Darcor]. You don't control the trolley's casters, but it explains why the flip penalty varies trolley-to-trolley, and it was "larger for bigger casters" in reversed-start tests [S, ScienceDirect S0003687002000029].

**Takeaway:** the number that matters for "can it even start" is not the ~60 N cruise but the **~120–260 N startup peak**.

### A3. Rotating the load in place

Turning a 4-swivel-caster body about its centroid forces every caster to swivel-scrub simultaneously. **[E]** Estimated static swivel torque is **~10–15 N·m per caster** at ~500 N caster load (200 kg / 4), i.e. **~40–60 N·m** total just to reorient all four from rest [E, Gemini]. A small mecanum robot cannot deliver that through its wheels without slipping. **Consequence: rotate-in-place under load is effectively off the table** unless the casters are unloaded (lifted) or pre-aligned.

Industry corroboration of the general principle: providing a **defined axis of rotation (a fixed, non-swivel wheel) reduces the effort of 90° turns** versus all-swivel [S, ScienceDirect]. An all-swivel load is rated **5/5 manoeuvrability but only 2/5 straight-line tracking** [S, Blickle] — great in theory, squirrelly in practice.

### A4. Traction budget of the mecanum pusher — the crux

A pushing robot can only exert as much force as friction between **its own wheels and the floor** allows: **F_push ≤ μ_eff · N_robot.**

**Mecanum penalty.** Mecanum wheels drive through 45° free-spinning rollers, so they have **inherently less traction/pushing force than standard wheels with identical motors, weight and tread — the deficit is geometric, not compound** [S, Chief Delphi / ERAU thesis]. The 45° geometry raises the floor reaction by 1/cos(45°) and contact is intermittent (vertical micro-vibration as rollers hand off) [S]. Measured PU mecanum-roller kinetic friction: **0.65 on concrete**, 0.40 aluminium, 0.35 painted, 0.30 wet, 0.21 wood [S, ERAU]. After the geometric penalty and duty-cycle losses, a **realistic effective μ_eff ≈ 0.3–0.5 on concrete** [E], lower still when strafing (the weak axis).

**Worked traction budget** (g = 9.81 m/s², trolley 200 kg → 1962 N):

*Force the robot can deliver (no weight transfer):*

| Robot mass | N_robot | μ_eff 0.3 | μ_eff 0.4 | μ_eff 0.5 |
|---|---|---|---|---|
| 10 kg | 98 N | 29 N | **39 N** | 49 N |
| 15 kg | 147 N | 44 N | 59 N | **74 N** |
| **25 kg — the requirement [O]** | 245 N | 74 N | 98 N | **123 N** |
| **~30 kg — as delivered [O]** | **294 N** | 88 N | 118 N | **147 N** |

Neither of the bottom two rows existed when this was written; the table stopped
at 15 kg. On **dry concrete** the measured PU mecanum-roller μ is 0.65, giving
μ_eff ≈ 0.5 after the geometric penalty — so the delivered robot sits in the
**147 N** cell.

*Force required:*
- Sustained cruise: **~60 N** (C_rr 0.03), up to ~120 N (soft wheels)
- Breakaway, aligned: **~120–150 N**
- Startup, misaligned casters: **~150–260 N**
- Plus acceleration to 0.3 m/s: 31 N (gentle, 0.15 m/s²) to 63 N (brisk, 0.3 m/s²) for the 210 kg system

*Same requirements scaled to the **100 kg** the project actually specified (×0.5):*

| | @ 200 kg (modelled here) | @ 100 kg (**requirement**) | @ ~150 kg (**observed**) |
|---|---|---|---|
| Sustained cruise | ~60 N | **~30 N** | ~45 N |
| Breakaway, aligned | 120–150 N | **60–75 N** | 90–113 N |
| Startup, misaligned | 150–260 N | **75–130 N** | 113–195 N |

**147 N available against a 60–75 N aligned breakaway — roughly 2× at the
requirement**, and still 1.3–1.6× at the ~150 kg actually moved. Above the
misaligned band at 100 kg; inside it at 150 kg, so a stall on badly flipped
castors is plausible at the heavier load. Independently, the project's own worst-case castor
figure is μ·m·g = 0.15 × 100 × 9.81 = **147 N**, level with available traction —
presumably why *"moves under all 4 castor orientations"* was written as its own
acceptance criterion rather than assumed. It passed.

**Floor sensitivity [E].** μ_eff is the whole ballgame, and concrete is the best
case:

| Surface | μ_eff | Force @ 30 kg | vs 60–75 N aligned / 75–130 N misaligned |
|---|---|---|---|
| Dry concrete | 0.50 | **147 N** | clears both — **the tested case** |
| Painted | 0.35 | 103 N | clears aligned; inside the misaligned band |
| Wet | 0.30 | 88 N | clears aligned; likely fails misaligned |

**Verdict [E], as originally written:** the best a plausibly-light robot delivers (~74 N) is **below even the aligned breakaway force (~120 N)**, and far below the misaligned startup peak. **A light mecanum robot cannot start a 200 kg caster trolley without weight transfer.**

> **Revised verdict [O], 2026-08-31.** True *as scoped* — for a 5–15 kg robot
> against 200 kg. Neither figure is this project's. Against the specified
> **≥ 25 kg robot and 100 kg load**, the delivered ~30 kg machine supplies ~147 N
> on dry concrete, **passed its 100 kg drive test with a horizontal-only clamp,
> and moved ~150 kg** — 1.5× the requirement. The 200 kg case modelled here was
> never a requirement, remains unproven, and on anything worse than concrete is
> unlikely. Sustained cruise *might* be marginally feasible on excellent wheels once moving, but you can never get it moving. This matches both independent Gemini analyses and the real-world push-force data in A1–A2.

**Weight transfer fixes it.** Let *f* = fraction of trolley weight carried by the robot's wheels. Then N_robot = (m_robot + f·200)·g and the caster load (hence caster resistance) drops to (1−f)·200. Solving the startup inequality [E, Gemini]:

- Clean floor (μ_eff 0.40): **f ≥ ~16 %** (~32 kg onto the robot)
- Dusty floor (μ_eff 0.30, higher caster friction): **f ≥ ~25 %** (~50 kg)

So **transfer ~20–30 % of trolley weight (≈40–60 kg)** as the practical minimum, with margin. **Full lift** (casters clear the floor, robot carries all ~210 kg) gives a traction ceiling of μ·210·g ≈ 620–1030 N — vastly more than any requirement — and is why high-volume under-cart AMRs lift.

### A5. Dynamics & safety

- **Braking authority = robot traction only.** Free-rolling casters can't help stop the load. Without weight transfer, max decel ≈ F_available/m ≈ 74 N / 210 kg ≈ **0.35 m/s²**; stopping distance from 0.3 m/s ≈ v²/2a ≈ **~13 cm.** With weight transfer, braking authority scales up with N_robot. **This bounds safe speed near the dock and near people.** [E]
- **Acceleration must be capped** so the startup force stays inside the traction envelope; sudden torque spikes throw mecanum wheels out of their friction cone and into slip (losing odometry). [E]
- **Momentum is real:** 200 kg at 0.3 m/s carries ~9 J and ~60 kg·m/s — an unclamped or compliant coupling would let the load shift on stop. A rigid vertical load path keeps robot+load decelerating as one body. [E]

---

## Part B — Control & manoeuvring strategy

### B1. Push vs pull vs rigid centered under-ride

The classic "**towing is passively stable, pushing jackknifes**" rule **applies only to articulated (hitch/pivot) couplings**, where the extra joint DOF amplifies angular error. **Falali's rigid clamp removes that joint** — robot + trolley become a single 3-DOF rigid body, so geometric jackknifing is mathematically eliminated. **[E, both Gemini analyses agree]** A pivoted push or pull of an all-swivel trolley, by contrast, is unstable/drifts badly because the casters supply no lateral constraint (it behaves "like a sliding block on ice").

**Centered under-ride is the best mount point:** it puts the robot directly under the load's mass, maximising the normal force (and weight-transfer leverage) on the drive wheels, and makes caster loads symmetric about the drive centre. An end-mounted (front/rear) clamp leaves the robot's wheels lightly loaded → slip. **[E]** Industry echo: MasterMover's MasterHandler slides a **stabilising leg beneath the load to create a single rigid control point** specifically to kill swivel-caster drift [S].

### B2. Where to put the rotation centre (ICR)

- **Pure rotation:** place the ICR at the **geometric centre of the caster layout** (≈ robot drive centre). All four casters then see equal-magnitude velocity and reorient symmetrically, so their resistances form a balanced couple instead of shoving the robot sideways. **[E]**
- **Cornering:** keep the ICR **away from any single caster's pivot axis.** A caster whose pivot velocity → 0 cannot roll to align; it becomes a static-scrub **friction anchor** that forces the other wheels to slip. Constrain minimum turn radius so every caster keeps a pivot speed above a small threshold (~0.05 m/s). **[E]**

### B3. Arc turns vs rotate-then-drive

**Prefer slow arc (continuous-curvature) turns; avoid rotate-in-place under load.** [E] A *rolling* caster steers by rolling into the new heading (relaxation-length behaviour) at roughly **1/10th** the force of a *stationary* caster scrubbing to reorient. Rotate-in-place from a stop demands the full ~40–60 N·m static swivel torque (A3) at once → guaranteed mecanum slip and ruined odometry. Arc turns spread caster reorientation over distance while wheels stay rolling and inside their traction envelope. Practically: **forbid zero-radius turns while loaded; use clothoid/spline paths.**

### B4. Caster pre-alignment — the "back-up" trick

Before a long straight move from rest, **jog backward ~2× the caster trail (≈50–100 mm)** to flip all casters into the trailing/aligned position pointing along the intended travel. Startup resistance then collapses from the ~150–260 N misaligned spike to near pure rolling resistance (~0.01–0.02 of load). **[E]** This matters because at t=0 the robot faces both load inertia *and* misaligned-caster drag, with asymmetric caster orientations inducing a parasitic yaw that veers the robot before the control loop reacts. **Note:** if the design lifts the casters clear, this becomes moot — unloaded casters self-home (see B5/patent).

### B5. Lateral drift & industry practice

- All-swivel loads **drift laterally** under any side force and are hard to track straight (2/5 straight-line rating) [S, Blickle; MasterMover calls drift *the* characteristic failure mode of swivel-caster loads [S]]. Correcting it needs **closed-loop heading/position feedback** (odometry + IMU, ideally a floor/marker reference), not open-loop dead reckoning — the mecanum lateral axis is too weak and too slip-prone to reject disturbances passively.
- **The dominant industry pattern for under-cart robots is to LIFT.** US Patent 12,472,772 (Kiva-lineage) notes that (a) random resting caster orientations can physically **collide with the robot tunnelling under the cart**, and (b) **lifting the cart unloads the swivel casters so they rotate to defined home positions** — turning the loaded caster-flip problem into a non-problem. [S] This is the single most important strategic datum in the report: it's why Amazon/Kiva-style movers lift rather than push.
- Manual/powered cart-mover practice (MasterMover) for *steerable* control favours **2 fixed + 2 swivel** casters (a defined rolling axis) and flags all-swivel carts as hard to steer [S] — not an option we can impose on the trolley, but it reinforces why the robot must supply the missing constraint itself (via rigid clamp + feedback).

---

## Part C — Implications for Falali

**Headline for the hardware team (the `IClamp` subsystem, teammate-owned):**
The clamp is not just a horizontal grip — **it must establish a vertical load path that transfers trolley weight onto the robot's wheels.** Two viable options, in order of robustness:

1. **Full lift (recommended if structurally feasible).** Jack the trolley a few mm until its casters clear the floor. Robot carries ~210 kg → traction is a non-issue; casters unload and self-orient (no flip, no scrub, no pre-align dance). Cost: robot wheels/motors/frame and ride-height must handle full load; needs vertical clearance above the casters' unloaded droop.
2. **Partial weight transfer (~40–60 kg, ≈20–30 %).** Compliant/spring or cam clamp that presses the trolley frame down onto the robot. Lighter robot; casters still roll and still partially flip, so you *also* need the software mitigations below. Center mount essential.

**Anti-pattern to avoid:** a purely horizontal clamp with no vertical load path. Physics says it will spin its wheels and fail to move 200 kg. The current design's zeroed offset placeholders (`DockingStateMachine.h`) and horizontal-clamp framing should be revisited against this.

**Also flag (both analyses raised it):** 4 casters + 4 robot wheels is an **over-constrained 8-point support**. On any floor unevenness the robot's mecanum wheels can unload and lose traction. Build **vertical compliance** (suspension or sprung clamp) into the load path so the drive wheels stay pressed to the floor.

**Candidate control-software rules (to validate on the bench, feed into the drive/authority controller):**
- **Speed:** the existing `max_lin_mm_s = 300` is fine as a ceiling, but derate near the dock/humans so stopping distance (traction-limited, ~13 cm ungeared) stays safe. Braking authority scales with weight transfer.
- **Acceleration:** cap accel/decel (start ~0.15 m/s²) to stay inside the mecanum traction cone; ramp, don't step.
- **Turning:** **forbid rotate-in-place while loaded**; use arc/continuous-curvature turns with a minimum radius; place the pure-yaw ICR at the caster-layout centroid.
- **Startup:** implement the **back-up pre-align jog** (~50–100 mm) before long strokes *if* the design does not fully lift; unnecessary if it lifts.
- **Feedback:** closed-loop heading/position correction (odometry + IMU) to reject lateral drift; consider wheel-slip detection (compare commanded vs measured motion) to back off throttle when casters spike.
- **Note on docking (pre-clamp):** the traction problem only bites *after* clamping. During docking (ORIENT / CENTER_X strafe) the robot is unloaded and free, so the mecanum weak-lateral-axis concern is minor there — but that same weak axis is exactly why a *loaded* strafe should be avoided.

---

## Part D — Unknowns that need bench measurement

The single largest caveat: **the repo documents none of the trolley's physical properties.** Every number above is a literature/AMR-industry estimate, not a measurement of *your* trolley. Measure these before trusting the budget:

1. **Actual loaded trolley mass** (spec says "up to 200 kg" — confirm the real distribution and worst case).
2. **Caster wheel diameter, tread material, bearing type** — these swing C_rr by ~4× (0.02 → 0.09) and set the breakaway/flip peaks.
3. **Floor material & condition** (concrete, epoxy, tile, dust/wet) — sets μ_eff for the robot (0.65 dry concrete down to 0.30 wet) and caster resistance.
4. ~~**Does the clamp lift, and by how much?**~~ **ANSWERED [O]: it does not, by design.** It transfers no weight. The robot is in neither modelled regime but a third this report did not consider — *heavy enough that it does not need to* — which is what the ≥ 25 kg requirement bought.
5. **Robot mass and mecanum μ_eff on the real floor** — partially answered. Mass is **~30 kg [O]** against a ≥ 25 kg requirement, and 100 kg moved on dry concrete (~150 kg observed, estimated), consistent with
   μ_eff ≈ 0.5. **Still unmeasured:** a spring-scale pull test for true push force, the misaligned-castor breakaway, and behaviour on any surface other than dry concrete.

A 30-minute bench session with a luggage/fish spring scale (push the trolley; pull-test the robot) will replace most of the [S]/[E] estimates here with hard numbers specific to your hardware.

---

## Sources

**Verified (3-vote) — primary:**
- Hamilton Caster, *White Paper No. 11: Rolling Resistance and Industrial Wheels* (Lippert & Spektor) — rolling-resistance formula, PU 85A f=0.047″, breakaway 2–2.5×, worked 4800 lb example. https://www.hamiltoncaster.com/Portals/0/blog/White%20Paper%20Rolling%20Resistance.pdf
- Blickle caster engineering guide — five factors governing resistance; material ranking. https://www.blickle.com/guide/basic-information-definitions (and /guide/optimal-product, /guide/manouevrability)

**Single-source / sourced (verification incomplete):**
- CasterHQ — rolling-resistance coefficient tables; 227 kg cart push-force data. https://casterhq.com/blogs/engineering-specifications/rolling-resistance ; https://casterhq.com/blogs/caster-university/caster-ergonomics-push-pull-force
- Caster Concepts — automotive-plant breakaway push-force case study (>267 N → <200 N). https://www.casterconcepts.com/case-study/case-study-swivel-on-swivel-casters-with-twergo-wheels-help-automotive-plant-stay-up-to-speed/
- Darcor — swivel offset / torque relation. https://darcor.com/2016/01/26/science-caster-wheels-impact-workplace-ergonomics/
- Bulldog Castors — breakaway 2–3×; nylon-on-concrete C_rr. https://www.bulldogcastors.co.uk/blog/castor-wheels-roll-resistance/
- JessCaster — AMR-grade caster C_rr ≤0.02 target. https://jesscaster.com/amr-casters-specification-guide/
- IEEE IEEM 2015 (doc 7158298) — push-force vs caster orientation, four-wheeled carts. https://ieeexplore.ieee.org/document/7158298
- ScienceDirect S0003687002000029 — reversed-caster start penalty; fixed-wheel rotation axis eases 90° turns.
- PMC4672999 — push/pull force linear in load; harder wheels/floors ease pushing. https://pmc.ncbi.nlm.nih.gov/articles/PMC4672999/
- ERAU thesis (edt 1249) — measured mecanum-roller μ by surface; mecanum traction deficit. https://commons.erau.edu/cgi/viewcontent.cgi?article=1249&context=edt
- Chief Delphi — mecanum force-vector analysis (why mecanum < standard wheel traction). https://www.chiefdelphi.com/t/paper-mecanum-wheel-force-vector-analysis/168773
- MasterMover — swivel-caster drift; MasterHandler stabilising-leg control point. https://www.mastermover.com/news/masterhandler-launch ; https://www.mastermover.com/electric-tug-essentials/load-moving-castors
- US Patent 12,472,772 — under-cart robot: lifting unloads casters to home positions; caster-collision-during-tunnel problem. https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12472772

**Engineering analysis:** two independent Gemini (Antigravity `agy`) second-opinion passes on (1) the traction budget and (2) stability/manoeuvring, reconciled against the sources above and local arithmetic. Both converged on the "traction is the constraint → must transfer weight / lift" conclusion.

---

*Method: deep-research fan-out (5 angles, 21 sources, 102 extracted claims, 25 sent to 3-vote adversarial verification → 5 confirmed, 0 refuted, 20 verification-incomplete due to a mid-run session limit) + 2 Gemini engineering second opinions + local traction-budget arithmetic. Confidence tags above reflect this; the [S] claims are credible but not independently corroborated in this run.*
