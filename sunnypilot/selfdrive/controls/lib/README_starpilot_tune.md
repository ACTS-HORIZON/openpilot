# StarPilot lateral tune — constant reference & provenance

## Provenance

`latcontrol_torque_starpilot.py` and `latcontrol_vehicle_tunes.py` are a port of
firestar5683/StarPilot, branch `Dom` (commit `f5a5b9e1`, "lat control refactor"),
reduced to a single tune seeded from StarPilot's **2023 Ioniq 6** entry. The tune is
registered as an additional "Torque Control Version" and applies **unconditionally**
when selected — there is no per-car fingerprint gate.

**Concern:** the heavy LEFT/RIGHT asymmetry in the seed values (e.g. unwind taper
3.18 vs 8.20, plus a right-turn-only feedforward boost at highway speed) was almost
certainly hand-tuned around a steering-rack bias specific to that individual Ioniq 6.
It is not an E-GMP platform property and should not be assumed appropriate for a
2023 Genesis GV60 Performance AWD.

## Current state (GV60 refit baseline)

Two changes were made relative to the original port:

1. **Discontinuity fix.** `_starpilot_side_value()` used to hard-switch between the
   LEFT and RIGHT constants on the sign of `desired_lateral_accel`. On a straight
   road, desired lateral accel crosses zero constantly (lane-keeping noise, camber),
   so every crossing instantly swapped entire gain profiles — the likely source of
   the random highway pulls. It now sigmoid-blends between the sides over
   `STARPILOT_SIDE_BLEND_WIDTH` (0.05 m/s²), consistent with the sigmoid smoothing
   used everywhere else in the file. With the logistic blend, ~90% of the transition
   completes within about ±0.11 m/s², so lane-keeping noise produces a gradual morph
   while a deliberate turn (≳0.2 m/s²) sees essentially the pure side value. At
   exactly zero the blend returns the LEFT/RIGHT midpoint.

2. **Neutralization.** Every *magnitude* constant (gain, boost, taper maximum,
   assist torque) was reset to its per-use neutral value, so the whole shaping layer
   is a no-op pending a deliberate GV60 re-tune. *Shape* constants (breakpoints,
   band edges, sigmoid widths, divisors) keep their archived values: they only
   position curves whose magnitudes are now zero, and every `*_WIDTH` / divisor
   constant is a denominator that must stay > 0. The original values are archived
   verbatim in a comment block in `latcontrol_vehicle_tunes.py` and listed per
   constant below.

**Verified** (sweep over v_ego 0–40 m/s, lateral accel ±3 m/s², jerk ±2.5 m/s³,
angles ±540°, torque ±1, ~8,800 combinations): with the neutralized constants,
`get_starpilot_ff_scale`, `get_starpilot_friction_scale`,
`get_starpilot_center_taper_scale`, `get_starpilot_directional_taper_scale`,
`get_starpilot_output_taper_scale`, `get_starpilot_highway_output_taper_scale` and
`get_starpilot_highway_transition_output_taper_scale` all return exactly `1.0`;
`get_starpilot_friction_threshold` returns exactly the stock speed-scaled curve from
`get_friction_threshold`; and `get_starpilot_low_speed_angle_assist_torque` returns
the input torque unchanged.

### What is still active after neutralization

The controller (`latcontrol_torque_starpilot.py`) itself is untouched. In
particular:

- `STARPILOT_LAT_ACCEL_FACTOR_MULT = 1.22` (controller file) still multiplies the
  car's `latAccelFactor`, including live-tuned values. `>1` means the controller
  assumes the rack is more effective, i.e. ~18% **less** feedforward torque per
  requested lateral accel, with the PID making up the difference. This is an
  Ioniq 6 seed value; set it to `1.0` if you want a fully neutral baseline.
- `STARPILOT_FF_MASTER_GAIN` and `STARPILOT_FRICTION_MASTER_GAIN` (controller file)
  are already `1.0`.
- `get_friction_threshold()`'s speed-scaled curve (0.16 → 0.27 over 1 → 75 mph)
  is now the effective friction threshold. Upstream sunnypilot's torque controller
  uses a fixed 0.3, so friction compensation engages over a somewhat narrower error
  band than stock at most speeds.
- `STARPILOT_LOW_SPEED_PID_RESET_SPEED` (see Misc below) still governs PID resets.
- The controller's own PID gains, jerk filtering, unwind detection, and integrator
  handling are StarPilot's structure and were never part of the constants block.

### Known remaining sharp edges (inert today, relevant when re-tuning)

- The **crawl turn-in boost** and **high-speed right turn-in boost** sit behind hard
  `desired_lateral_accel * desired_lateral_jerk > 0` branch gates in
  `get_starpilot_ff_scale`. With the archived values these stepped the FF scale by
  up to ~0.018 across a zero crossing (~0.015 at 30 m/s from the right-turn boost —
  plausibly a second contributor to the highway pulls, independent of the
  side-value bug). With the boosts at 0 the branches contribute nothing. If you
  re-enable either boost, consider gating it with a sigmoid on `accel * jerk`
  instead.
- `get_starpilot_ff_scale` and `get_starpilot_directional_taper_scale` early-return
  `1.0` at exactly `desired_lateral_accel == 0.0`. Harmless with neutral constants
  (the full expression also evaluates to 1.0), but with non-neutral constants the
  exact-zero sample can differ slightly from the limit of neighboring values.

## Sign convention

Positive `desired_lateral_accel` = **left** turn (the controller comment notes
"left is positive in this convention"), so `*_LEFT` constants shape left turns and
`*_RIGHT` right turns, blended near zero by `_starpilot_side_value`.

---

# Constant reference

Columns: **Archived** = original Ioniq 6 seed value; **Live** = current neutralized
value in code. "Increasing it…" describes the practical effect *when the group is
active* (i.e. once its magnitude constant is non-zero again).

## Directional left/right blending

Used by `_starpilot_side_value()`, which feeds every `*_LEFT`/`*_RIGHT` selection in
`get_starpilot_ff_scale`, `get_starpilot_friction_threshold`,
`get_starpilot_friction_scale`, and `get_starpilot_directional_taper_scale`.

| Constant | Unit | Archived | Live | Increasing it… |
|---|---|---|---|---|
| `STARPILOT_SIDE_BLEND_WIDTH` | m/s² | — (new) | 0.05 | Widens the band around zero lateral accel over which LEFT and RIGHT values morph into each other. Larger = softer, slower transition (more asymmetry dilution near center); smaller = closer to the old hard switch. Must stay > 0. |

## Feedforward shaping

Used by `get_starpilot_ff_scale`, which multiplies the feedforward lateral-accel
request (`1.0` = stock FF). The "extra FF band" below is a bump of height
`FF_GAIN` between `FF_ONSET` and `FF_CUTOFF`, further scaled up during turn-in and
down during unwind.

| Constant | Unit | Archived | Live | Increasing it… |
|---|---|---|---|---|
| `STARPILOT_FF_GAIN_LEFT` | unitless fraction | 0.045 | **0.0** | More extra FF (stronger, more eager steering) in the mid-lateral-accel band for left turns. |
| `STARPILOT_FF_GAIN_RIGHT` | unitless fraction | 0.015 | **0.0** | Same for right turns. |
| `STARPILOT_FF_ONSET` | m/s² | 0.10 | 0.10 (kept) | Moves the start of the extra-FF band to harder turns; below it the boost fades out. |
| `STARPILOT_FF_ONSET_WIDTH` | m/s² | 0.04 | 0.04 (kept) | Softens the onset edge (more gradual engagement). Must stay > 0. |
| `STARPILOT_FF_CUTOFF` | m/s² | 0.48 | 0.48 (kept) | Extends the extra-FF band into harder corners before it fades. |
| `STARPILOT_FF_CUTOFF_WIDTH` | m/s² | 0.12 | 0.12 (kept) | Softens the fade-out edge. Must stay > 0. |
| `STARPILOT_TURN_IN_BOOST_LEFT` | unitless | 1.64 | **0.0** | Amplifies the extra-FF band while *turning in* (phase > 0) to the left; effect strongest at low speed (scaled by the low-speed factor). More aggressive initial turn-in. |
| `STARPILOT_TURN_IN_BOOST_RIGHT` | unitless | 2.10 | **0.0** | Same for right turn-in. |
| `STARPILOT_UNWIND_TAPER_LEFT` | unitless | 3.18 | **0.0** | Cuts the extra-FF band while *unwinding* (phase < 0) out of a left turn (clamped so it can zero the band but not push it negative). Only affects the extra band, not base FF. |
| `STARPILOT_UNWIND_TAPER_RIGHT` | unitless | 8.20 | **0.0** | Same for right-turn unwind. The 8.20-vs-3.18 asymmetry is donor-car rack-bias compensation. |

## Crawl / high-speed turn-in feedforward boosts

Used by `get_starpilot_ff_scale`; both are additive bumps to the FF scale, gated by
the hard `accel · jerk > 0` branch (see "Known remaining sharp edges").

| Constant | Unit | Archived | Live | Increasing it… |
|---|---|---|---|---|
| `STARPILOT_CRAWL_TURN_IN_FF_BOOST_LEFT` | unitless fraction | 0.18 | **0.0** | More FF while initiating a left turn at crawl/parking speeds. |
| `STARPILOT_CRAWL_TURN_IN_FF_BOOST_RIGHT` | unitless fraction | 0.24 | **0.0** | Same, right turns. |
| `STARPILOT_CRAWL_TURN_IN_FF_SPEED` | m/s | 5.3 | 5.3 (kept) | Raises the speed below which the crawl boost applies. |
| `STARPILOT_CRAWL_TURN_IN_FF_SPEED_WIDTH` | m/s | 1.0 | 1.0 (kept) | Softens the speed edge. Must stay > 0. |
| `STARPILOT_CRAWL_TURN_IN_FF_LAT` | m/s² | 0.06 | 0.06 (kept) | Raises the minimum lateral accel before the crawl boost engages. |
| `STARPILOT_CRAWL_TURN_IN_FF_LAT_WIDTH` | m/s² | 0.035 | 0.035 (kept) | Softens that edge. Must stay > 0. |
| `STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_BOOST` | unitless fraction | 0.10 | **0.0** | More FF when initiating gentle **right** curves at highway speed (0.06–0.22 m/s² band above ~18 m/s). Right-only: pure donor-car bias compensation — treat with suspicion on the GV60. |
| `STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED` | m/s | 18.0 | 18.0 (kept) | Raises the speed above which it engages. |
| `STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED_WIDTH` | m/s | 2.5 | 2.5 (kept) | Softens the speed edge. Must stay > 0. |
| `STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_START` | m/s² | 0.06 | 0.06 (kept) | Moves the lower edge of the lateral-accel band. |
| `STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_END` | m/s² | 0.22 | 0.22 (kept) | Moves the upper edge of the band. |
| `STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_WIDTH` | m/s² | 0.035 | 0.035 (kept) | Softens both band edges. Must stay > 0. |

## Turn-in/unwind phase machinery (shared)

`_starpilot_transition_phase` = `tanh(accel · jerk / PHASE_SCALE)` classifies
turn-in (+) vs unwind (−); `_starpilot_low_speed_factor` =
`1 / (1 + (v / TRANSITION_SPEED)²)` weights effects toward low speed;
`_starpilot_transition_envelope` gates the friction shaping. Used (indirectly) by
`get_starpilot_ff_scale`, `get_starpilot_friction_threshold`,
`get_starpilot_friction_scale`, `get_starpilot_directional_taper_scale`.

| Constant | Unit | Archived | Live | Increasing it… |
|---|---|---|---|---|
| `STARPILOT_TRANSITION_SPEED` | m/s | 10.0 | 10.0 (kept) | Keeps the low-speed-weighted effects (turn-in boost, part of unwind taper, friction envelope) strong up to higher speeds. Divisor — must stay > 0. |
| `STARPILOT_PHASE_SCALE` | (m/s²)·(m/s³) | 0.10 | 0.10 (kept) | Softens the turn-in/unwind classification (tanh saturates later); decreasing makes it snappier/more binary. Divisor — must stay > 0. |
| `STARPILOT_FRICTION_LAT_RISE` | m/s² | 0.20 | 0.20 (kept) | Requires more lateral accel before the friction-shaping envelope reaches full strength. Divisor — must stay > 0. |
| `STARPILOT_FRICTION_JERK_RISE` | m/s³ | 0.24 | 0.24 (kept) | Same, for lateral jerk. Divisor — must stay > 0. |

## Friction shaping

`get_starpilot_friction_threshold` returns the error band fed to `get_friction`
(bigger threshold = friction/stiction torque spreads over a wider error range);
its directional scaling is clamped to ±18% of base. `get_starpilot_friction_scale`
multiplies the friction magnitude, clamped to [0.82, 1.08].

| Constant | Unit | Archived | Live | Increasing it… |
|---|---|---|---|---|
| `STARPILOT_BASE_FRICTION_THRESHOLD` | m/s² (lat-accel error) | 0.36 | **0.0** | Raises the floor of the friction threshold (`max(stock curve, this)`). At 0 the stock speed-scaled curve (0.16–0.27) applies unmodified. |
| `STARPILOT_FRICTION_MULT` | unitless | 0.928 | **1.0** | Scales overall friction (stiction-compensation) torque; >1 = more, <1 = less (then clamped 0.82–1.08). Neutral is 1.0, and 1.0 survives the clamp. |
| `STARPILOT_TURN_IN_THRESHOLD_REDUCTION_LEFT` | unitless | 0.78 | **0.0** | Shrinks the friction threshold during left turn-in → friction help kicks in sooner while initiating. |
| `STARPILOT_TURN_IN_THRESHOLD_REDUCTION_RIGHT` | unitless | 1.42 | **0.0** | Same, right turn-in. |
| `STARPILOT_UNWIND_THRESHOLD_INCREASE_LEFT` | unitless | 3.90 | **0.0** | Grows the threshold during left-turn unwind → less friction interference while straightening. |
| `STARPILOT_UNWIND_THRESHOLD_INCREASE_RIGHT` | unitless | 10.20 | **0.0** | Same, right-turn unwind (heavily asymmetric in the seed). |
| `STARPILOT_TURN_IN_FRICTION_BOOST_LEFT` | unitless | 0.44 | **0.0** | More friction torque during left turn-in (crisper initial bite). |
| `STARPILOT_TURN_IN_FRICTION_BOOST_RIGHT` | unitless | 0.94 | **0.0** | Same, right turn-in. |
| `STARPILOT_UNWIND_FRICTION_REDUCTION_LEFT` | unitless | 3.55 | **0.0** | Less friction torque during left-turn unwind (clamped at the 0.82 floor). |
| `STARPILOT_UNWIND_FRICTION_REDUCTION_RIGHT` | unitless | 9.10 | **0.0** | Same, right-turn unwind. |

## Center / highway tapers

All of these *reduce* torque near center (small desired lateral accel) at various
speed bands. `get_starpilot_center_taper_scale` multiplies the FF term only (the
controller also divides the friction threshold by it and blends the friction scale
with it); the two "output taper" functions multiply the **final output torque**
(FF + PID), so they soften everything.

Used by: `get_starpilot_center_taper_scale` (first three sub-groups),
`get_starpilot_highway_output_taper_scale`,
`get_starpilot_highway_transition_output_taper_scale`, and
`get_starpilot_output_taper_scale` (last sub-group; **not called by the current
controller**).

| Constant | Unit | Archived | Live | Increasing it… |
|---|---|---|---|---|
| `STARPILOT_CENTER_TAPER_MAX` | fraction | 0.082 | **0.0** | More FF reduction when nearly straight above ~18 m/s → lighter, lazier on-center feel at speed. |
| `STARPILOT_CENTER_TAPER_LAT` | m/s² | 0.24 | 0.24 (kept) | Widens what counts as "near center" (taper reaches into gentler curves). |
| `STARPILOT_CENTER_TAPER_LAT_WIDTH` | m/s² | 0.025 | 0.025 (kept) | Softens that edge. Must stay > 0. |
| `STARPILOT_CENTER_TAPER_SPEED` | m/s | 18.0 | 18.0 (kept) | Raises the speed where this taper engages. |
| `STARPILOT_CENTER_TAPER_SPEED_WIDTH` | m/s | 2.5 | 2.5 (kept) | Softens the speed edge. Must stay > 0. |
| `STARPILOT_HIGHWAY_CENTER_TAPER_MAX` | fraction | 0.046 | **0.0** | Additional near-center FF reduction above ~24.5 m/s in a tighter center band (≤ ~0.10 m/s²). |
| `STARPILOT_HIGHWAY_CENTER_TAPER_LAT` | m/s² | 0.10 | 0.10 (kept) | Widens its center band. |
| `STARPILOT_HIGHWAY_CENTER_TAPER_LAT_WIDTH` | m/s² | 0.035 | 0.035 (kept) | Softens that edge. Must stay > 0. |
| `STARPILOT_HIGHWAY_CENTER_TAPER_SPEED` | m/s | 24.5 | 24.5 (kept) | Raises its engagement speed. |
| `STARPILOT_HIGHWAY_CENTER_TAPER_SPEED_WIDTH` | m/s | 1.8 | 1.8 (kept) | Softens the speed edge. Must stay > 0. |
| `STARPILOT_LOW_MID_CENTER_TAPER_MAX` | fraction | 0.088 | **0.0** | Near-center FF reduction inside the 8.5–16.5 m/s band (city/arterial speeds). |
| `STARPILOT_LOW_MID_CENTER_TAPER_LAT` | m/s² | 0.28 | 0.28 (kept) | Widens its center band. |
| `STARPILOT_LOW_MID_CENTER_TAPER_LAT_WIDTH` | m/s² | 0.06 | 0.06 (kept) | Softens that edge. Must stay > 0. |
| `STARPILOT_LOW_MID_CENTER_TAPER_SPEED_MIN` | m/s | 8.5 | 8.5 (kept) | Raises the band's lower speed edge. |
| `STARPILOT_LOW_MID_CENTER_TAPER_SPEED_MAX` | m/s | 16.5 | 16.5 (kept) | Raises the band's upper speed edge. |
| `STARPILOT_LOW_MID_CENTER_TAPER_SPEED_WIDTH` | m/s | 1.5 | 1.5 (kept) | Softens both speed edges. Must stay > 0. |

The three center reductions above are summed and capped at 0.12 total (hardcoded in
`get_starpilot_center_taper_scale`).

| Constant | Unit | Archived | Live | Increasing it… |
|---|---|---|---|---|
| `STARPILOT_HIGHWAY_OUTPUT_TAPER_MAX` | fraction | 0.10 | **0.0** | More reduction of **total output torque** (FF *and* PID) when near-center above ~23.5 m/s → softer overall highway centering. |
| `STARPILOT_HIGHWAY_OUTPUT_TAPER_LAT` | m/s² | 0.14 | 0.14 (kept) | Widens its near-center band. |
| `STARPILOT_HIGHWAY_OUTPUT_TAPER_LAT_WIDTH` | m/s² | 0.04 | 0.04 (kept) | Softens that edge. Must stay > 0. |
| `STARPILOT_HIGHWAY_OUTPUT_TAPER_SPEED` | m/s | 23.5 | 23.5 (kept) | Raises the engagement speed. **Shared** by the transition taper below. |
| `STARPILOT_HIGHWAY_OUTPUT_TAPER_SPEED_WIDTH` | m/s | 2.0 | 2.0 (kept) | Softens the speed edge (also shared). Must stay > 0. |
| `STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_MAX` | fraction | 0.18 | **0.0** | More output-torque reduction during high-jerk maneuvers (lane changes, curve entry) at highway speed → damps twitchiness but weakens transitions. |
| `STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT` | m/s² | 1.05 | 1.05 (kept) | Raises the lateral accel below which the transition taper can apply. |
| `STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT_WIDTH` | m/s² | 0.22 | 0.22 (kept) | Softens that edge. Must stay > 0. |
| `STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK` | m/s³ | 0.24 | 0.24 (kept) | Raises the jerk needed before the transition taper engages. |
| `STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK_WIDTH` | m/s³ | 0.14 | 0.14 (kept) | Softens the jerk edge. Must stay > 0. |
| `STARPILOT_OUTPUT_TAPER_SPEED` | m/s | 8.5 | 8.5 (kept) | (Unused function) engagement speed of `get_starpilot_output_taper_scale`. |
| `STARPILOT_OUTPUT_TAPER_SPEED_WIDTH` | m/s | 2.5 | 2.5 (kept) | (Unused function) speed edge softness. Must stay > 0. |
| `STARPILOT_OUTPUT_CENTER_TAPER_BLEND` | unitless 0–1 | 0.90 | 0.90 (kept) | (Unused function) fraction of the center taper passed through to output torque. Inert while the center tapers are neutral. |
| `STARPILOT_OUTPUT_DIRECTIONAL_TAPER_BLEND` | unitless 0–1 | 0.97 | 0.97 (kept) | (Unused function) same for the directional taper. |

## Directional / left-right asymmetry taper

`get_starpilot_directional_taper_scale` multiplies the FF request (via
`get_starpilot_ff_scale`) with per-direction reductions: a "base" reduction in a
mid-lateral-accel band (0.19–0.90 m/s²), a "heavy" variant above ~0.90 m/s², extra
reduction during unwind, and low-speed relief that restores strength in tight
low-speed turns. **This group carries most of the donor car's left/right bias and
most of its unwind character** — if steering unwind feels lazy after
neutralization, `*_UNWIND_*` here (reducing held FF while straightening, letting
the wheel return faster) is where the archived tune got its strong unwind.

| Constant | Unit | Archived | Live | Increasing it… |
|---|---|---|---|---|
| `STARPILOT_DIRECTIONAL_TAPER_LAT_START` | m/s² | 0.19 | 0.19 (kept) | Moves the start of the mid band to harder turns. |
| `STARPILOT_DIRECTIONAL_TAPER_LAT_END` | m/s² | 0.90 | 0.90 (kept) | Extends the mid band into harder turns. |
| `STARPILOT_DIRECTIONAL_TAPER_LAT_WIDTH` | m/s² | 0.06 | 0.06 (kept) | Softens both band edges. Must stay > 0. |
| `STARPILOT_DIRECTIONAL_TAPER_BASE_LEFT` | fraction | 0.11 | **0.0** | Steady FF reduction in moderate left turns → weaker steering there. |
| `STARPILOT_DIRECTIONAL_TAPER_BASE_RIGHT` | fraction | 0.45 | **0.0** | Same for right turns (the seed's 0.45 is a large rack-bias correction). |
| `STARPILOT_DIRECTIONAL_TAPER_UNWIND_LEFT` | fraction | 2.15 | **0.0** | Much less held FF while unwinding out of a left turn (above the jerk onset) → stronger/faster wheel return. |
| `STARPILOT_DIRECTIONAL_TAPER_UNWIND_RIGHT` | fraction | 4.25 | **0.0** | Same, right turns. |
| `STARPILOT_DIRECTIONAL_TAPER_FLOOR_LEFT` | scale floor | 0.48 | **0.0** | Raises the lowest value the taper may drop the FF scale to in left turns (limits total reduction). With reductions at 0 the floor is irrelevant. |
| `STARPILOT_DIRECTIONAL_TAPER_FLOOR_RIGHT` | scale floor | 0.52 | **0.0** | Same, right turns. |
| `STARPILOT_DIRECTIONAL_TAPER_UNWIND_FLOOR_LEFT` | scale floor delta | 0.16 | **0.0** | Lowers that floor during unwind (allows deeper reduction) in left turns. |
| `STARPILOT_DIRECTIONAL_TAPER_UNWIND_FLOOR_RIGHT` | scale floor delta | 0.04 | **0.0** | Same, right turns. |
| `STARPILOT_DIRECTIONAL_TAPER_JERK_ONSET` | m/s³ | 0.60 | 0.60 (kept) | Requires more jerk before the unwind reduction activates. |
| `STARPILOT_DIRECTIONAL_TAPER_JERK_WIDTH` | m/s³ | 0.14 | 0.14 (kept) | Softens the jerk edge. Must stay > 0. |
| `STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF` | unitless 0–1 | 0.98 | **0.0** | Cancels more of the *base* directional reduction in tight turns below ~11 m/s (1.0 = fully restored steering strength at low speed). |
| `STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED` | m/s | 11.2 | 11.2 (kept) | Raises the speed below which relief applies. |
| `STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED_WIDTH` | m/s | 1.5 | 1.5 (kept) | Softens the speed edge. Must stay > 0. |
| `STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT` | m/s² | 0.10 | 0.10 (kept) | Raises the lateral accel needed to count as a "tight turn" for relief. |
| `STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT_WIDTH` | m/s² | 0.06 | 0.06 (kept) | Softens that edge. Must stay > 0. |
| `STARPILOT_HEAVY_DIRECTIONAL_TAPER_LAT_START` | m/s² | 0.90 | 0.90 (kept) | Moves where the heavy-turn variant takes over. |
| `STARPILOT_HEAVY_DIRECTIONAL_TAPER_LAT_WIDTH` | m/s² | 0.18 | 0.18 (kept) | Softens that handover. Must stay > 0. |
| `STARPILOT_HEAVY_DIRECTIONAL_TAPER_BASE_LEFT` | fraction | 0.06 | **0.0** | Steady FF reduction in hard left turns. |
| `STARPILOT_HEAVY_DIRECTIONAL_TAPER_BASE_RIGHT` | fraction | 0.17 | **0.0** | Same, hard right turns. |
| `STARPILOT_HEAVY_DIRECTIONAL_TAPER_UNWIND_LEFT` | fraction | 0.78 | **0.0** | Extra reduction unwinding out of hard left turns. |
| `STARPILOT_HEAVY_DIRECTIONAL_TAPER_UNWIND_RIGHT` | fraction | 1.10 | **0.0** | Same, hard right turns. |

## Low-speed angle assist

`get_starpilot_low_speed_angle_assist_torque` rewrites the final output torque at
crawl speeds. Two paths: **wind assist** (steering still needs to move further
toward the desired angle — adds torque in the direction that closes the angle
error) and **unwind assist** (wheel is wound past ~10° and needs to come back).
Setting either `*_MAX_TORQUE` to 0 makes its entire path return the input torque
unchanged, so all other parameters in the path are inert. Torque values are in
normalized EPS output units (−1…1); angles in degrees at the steering wheel.

| Constant | Unit | Archived | Live | Increasing it… |
|---|---|---|---|---|
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_MAX_TORQUE` | torque (−1…1) | 0.46 | **0.0** | Stronger push toward the desired angle at crawl speed (parking, creeping turns). 0 disables the wind-assist path entirely. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_SPEED` | m/s | 3.25 | 3.25 (kept) | Raises the speed below which wind assist engages. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_SPEED_WIDTH` | m/s | 0.45 | 0.45 (kept) | Softens the speed edge. Must stay > 0. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_ERROR` | deg | 1.9 | 1.9 (kept) | Requires a larger angle error before assist ramps in. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_ERROR_WIDTH` | deg | 1.20 | 1.20 (kept) | Softens the error edge. Must stay > 0. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE` | deg | 5.5 | 5.5 (kept) | Requires a larger desired angle before assist ramps in (keeps it out of near-straight driving). |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE_WIDTH` | deg | 2.4 | 2.4 (kept) | Softens that edge. Must stay > 0. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_START` | unitless ratio | 0.66 | 0.66 (kept) | Lets assist stay strong until the wheel has tracked closer to the target (actual/desired ratio) before tapering. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_WIDTH` | unitless ratio | 0.12 | 0.12 (kept) | Softens the tracking taper. Must stay > 0. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_FLOOR` | unitless 0–1 | 0.26 | 0.26 (kept) | Raises the minimum assist fraction that survives the tracking taper. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_ADD_BP` | \|torque\| breakpoints | [0, 0.35, 0.65, 1.0] | kept | Breakpoints of existing output torque for the add-scale curve below. |
| `STARPILOT_LOW_SPEED_ANGLE_ASSIST_ADD_V` | unitless scale | [1.0, 1.0, 0.88, 0.08] | kept | How much of the assist is added when it points the same way as existing output — backs off to 8% near full torque to avoid saturation. |
| `STARPILOT_LOW_SPEED_UNWIND_ASSIST_MAX_TORQUE` | torque (−1…1) | 0.30 | **0.0** | Stronger return-to-center helper at crawl speed. 0 disables the unwind-assist path entirely. |
| `STARPILOT_LOW_SPEED_UNWIND_ASSIST_SPEED` | m/s | 3.35 | 3.35 (kept) | Raises the speed below which unwind assist engages. |
| `STARPILOT_LOW_SPEED_UNWIND_ASSIST_SPEED_WIDTH` | m/s | 0.50 | 0.50 (kept) | Softens the speed edge. Must stay > 0. |
| `STARPILOT_LOW_SPEED_UNWIND_ASSIST_ERROR` | deg | 1.6 | 1.6 (kept) | Requires a larger angle error before it ramps in. |
| `STARPILOT_LOW_SPEED_UNWIND_ASSIST_ERROR_WIDTH` | deg | 0.95 | 0.95 (kept) | Softens the error edge. Must stay > 0. |
| `STARPILOT_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE` | deg | 10.5 | 10.5 (kept) | Requires the wheel to be wound further before return help applies. |
| `STARPILOT_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE_WIDTH` | deg | 4.0 | 4.0 (kept) | Softens that edge. Must stay > 0. |
| `STARPILOT_LOW_SPEED_UNWIND_ASSIST_BLEND` | unitless 0–1 | 0.52 | 0.52 (kept) | Passes more of the unwind assist through when it points the same way as the existing output torque. Inert while `MAX_TORQUE` is 0. |

## Misc / controller plumbing

| Constant | Unit | Archived | Live | Notes |
|---|---|---|---|---|
| `STARPILOT_LOW_SPEED_PID_RESET_SPEED` | m/s | 0.1 mph ≈ 0.045 | unchanged (kept functional) | Used by the controller's `__init__`: caps the speed below which the PID resets and the integrator freezes (`min(max(minSteerSpeed, 0.3), this)`). A control-flow constant of the ported controller, not a tune magnitude — there is no meaningful "off" value, so it was deliberately left active. |
| `STARPILOT_BASE_LAT_ACCEL_FACTOR_MULT` | unitless | 1.22 | **1.0** | **Unused** in this file — the live copy is `STARPILOT_LAT_ACCEL_FACTOR_MULT` in `latcontrol_torque_starpilot.py`, which is still active at 1.22 (see "What is still active"). Set to multiplicative neutral here to avoid confusion. |

## Controller-side `STARPILOT_*` constants (latcontrol_torque_starpilot.py, not neutralized)

| Constant | Unit | Value | Effect |
|---|---|---|---|
| `STARPILOT_LAT_ACCEL_FACTOR_MULT` | unitless | 1.22 (**active**, Ioniq 6 seed) | Multiplies the car's `latAccelFactor` (incl. live-tuned values): >1 assumes a more effective rack → less feedforward torque per requested lateral accel, PID compensates. Set to 1.0 for a fully neutral baseline. |
| `STARPILOT_FF_MASTER_GAIN` | unitless | 1.0 (neutral) | Global feedforward scale; <1 softens, >1 sharpens everything. |
| `STARPILOT_FRICTION_MASTER_GAIN` | unitless | 1.0 (neutral) | Global friction-response scale. |

## GV60 controller (Torque Control Version 3.0, `latcontrol_torque_gv60.py`)

A bespoke controller for the 2023 Genesis GV60 Performance AWD, built around the
July 2026 rlog regression (81 routes, ~6.3M frames; `laf_from_rlogs.py` +
`lag_and_viz.py`). It keeps the StarPilot port's phase machinery (delay-compensated
setpoint buffer, jerk FF + low-pass, derivative-on-measurement damping, integrator
release decay, unwind detection, low-speed PID reset) and replaces the vehicle model:

- **Torque conversion** — `output_torque = output_lataccel / k(v)` with the measured
  gain table `LAF_SPEEDS`/`LAF_GAINS` (k ≈ 0.55 + 0.085·v, floored at 1.0 below
  5 m/s). PID limits track `±steer_max · k(v)` every cycle. The live torque
  learner's scalar `latAccelFactor` is ignored for conversion (its speed-averaged
  value is the fiction this controller removes); `latAccelOffset` is still applied
  live and the learner's friction is logged but not applied.
- **Friction** — hysteresis-model compensation `±FRICTION_TORQUE ·
  tanh(d(desired_torque)/dt / rate_scale)` in torque space, from the measured
  half-width 0.078. The error-based `get_friction` term remains behind
  `USE_ERROR_FRICTION` for A/B.
- **Delay** — setpoint buffer indexed with a speed-interpolated delay
  (`DELAY_SPEEDS`/`DELAY_VALUES`, ~330 ms low speed → ~120 ms highway; pooled
  ~168 ms), falling back to the live `lat_delay` when disabled. A slew-aware FF
  lead (`LEAD_S(v) · d(ff_torque)/dt`, ≤10 m/s only, clamped ±0.15) counters the
  measured amplitude-dependent EPS torque-rate lag; single-constant disable for A/B.
- **KP rescale** — the inherited low-speed KP schedule is divided by the ratio of
  old effective gain (`KP_old / 3.15`, the learner's typical latAccelFactor) to the
  new `KP / k(v)`, so the initial torque-loop gain matches known-stable behavior.

Registered as `"GV60": 3.0` in `latcontrol_torque_versions.json` and dispatched in
`controlsd_ext.initialize_lateral_control`. Applies unconditionally when selected —
no fingerprint gate. Commit series is separable for road testing: (a) gain curve +
limits, (b) friction hysteresis, (c) speed-interp delay + slew lead, (d) KP rescale.
