<!-- SUMMARY -->
> **Scope:** Architecture and design decisions from the last 90 days.
> **Key constraint:** Entries older than 90 days should be moved to decisions-archive.md.

---

# Recent Decisions

## Exclusive-pair clamp measurement — telemetry, no mitigation needed
**Date:** 2026-04-28
**Status:** Decided

**Context**
`mpc/constraints.py:clamp_exclusive` enforces non-convex mutual exclusivity by zeroing the smaller member of each exclusive pair after the QP returns. The prior `mpc-solver.md` warned about "saw-tooth oscillation when the QP plans to use both actuators". This worry had never been measured; the team would not commit to a fix without data.

**Method**
`scripts/measure_clamp_oscillation.py` monkey-patches `clamp_exclusive` to record (pre, post) for every solve across three 12hr `--fast` sims (`hvacd_full`, `ac_dehum`, `ac_minimal`). Raw per-tick CSVs and a summary table land in `docs/tasks/02-results/`.

**Findings**
| Profile | Pair channels in registry | Clamp activity |
|---|---|---|
| `hvacd_full` | None — uses `cool_sp` / `heat_sp` / `dehum_sp` setpoint mode | Constraint inactive (1399 solves, 0 fires) |
| `ac_minimal` | None — only `u_cool` | Constraint inactive (1392 solves, 0 fires) |
| `ac_dehum` | `u_heat`/`u_cool` and `u_hum`/`u_dehum` | Heavy |

For `ac_dehum` (1392 solves over 12h):

| Pair | Fires (% of solves) | Winner-flips A↔B | Worst 60s window |
|---|---|---|---|
| `u_heat__u_cool` | 1084 (78%) | 47 (~4/hr) | 2 fires |
| `u_hum__u_dehum` | 1080 (78%) | 4 (~0.3/hr) | 2 fires |

**Diagnostic on the heat/cool pair:**
- Median pre-clamp gap `|a-b|/max(a,b)` = **0.93** — when the clamp fires, the larger value is typically ~14× the smaller. The QP runs ~95/5 with a tiny tail, not 51/49 tie-breaking.
- At the 51 flip moments, median gap = **0.74**. **Zero** flips happen within a 5% close-band where hysteresis would engage.
- The flips are **genuine regime shifts** (cool 60→5 while heat 4→25), not numerical noise. The MPC's existing structure — re-linearization every solve, `R` on `‖Δu‖²`, 1 Hz re-clamp — already gives stable winner selection.

**Decision**
**No mitigation. Telemetry only.** The "sawtooth" the docs warned about doesn't exist in measurement, and the QP cost structure already produces stable winners. Adding hysteresis would solve a problem that doesn't appear; it would also introduce a silent failure mode where stale-winner holds could mask real control degradation in future configurations.

What ships:
- Per-pair `fires` / `flips` / `winner` counters via `clamp_metrics()`.
- Counters surfaced in the per-tick `info` dict under `clamp_metrics`, so production dashboards can watch for real-world deviations from the measured baseline.
- The measurement script and captured CSVs in `docs/tasks/02-results/` as a re-runnable regression baseline.
- `docs/mpc-solver.md` corrected: the "sawtooth" framing was intuition, not measurement.

Skipped (data-driven negatives):
- ❌ MIQP — 80–400ms on Pi 5 vs. 1Hz target. Not feasible.
- ❌ Switching-cost q-bias on the QP — overkill, would risk OSQP PSD invariant.
- ❌ Hard hysteresis lockout — risks trapping the wrong actuator during load steps.
- ❌ Soft hysteresis (5% close-band) — first prototyped, then removed: re-measurement showed flip count unchanged (47 → 51, within noise). The flips happen far outside the close band. Worse, hysteresis introduces a silent failure mode (stale-winner holds don't show up in `flips` or `fires` telemetry) — exactly the kind of debt this hardening pass is supposed to remove.

**Trade-offs**
- ✅ No-op for `hvacd_full` and `ac_minimal` (no exclusive pairs in those registries)
- ✅ Per-pair `fires`/`flips` counters surfaced in the per-tick `info` dict — load-bearing observability, not preemptive control
- ✅ No new knobs, no new failure modes; controller behaviour is bit-for-bit unchanged
- ⚠️ If a future configuration genuinely produces tie-breaking sawtooth, the counters will reveal it and a targeted mitigation can be designed against real data
- ❌ Rejected: removing the clamp constraint entirely. The QP wanting both is correct *intent* but the hardware is exclusive — the clamp is still required.

**Implications**
- `mpc/constraints.py:ConstraintBuilder.__init__` accepts `clamp_config` with `hysteresis_margin` (default 0.0 → legacy behaviour).
- `clamp_metrics()` exposes per-pair `fires`/`flips`/`winner`; `SolverProcess` puts the dict into the per-tick `info` under the `clamp_metrics` key.
- `tests/test_clamp_oscillation.py` pins legacy and hysteresis behaviour, counter invariants, and the no-pair-no-op profile.
- `scripts/measure_clamp_oscillation.py` and `docs/tasks/02-results/` document the numbers; future regressions can re-run the script.

---

## Irrigation as feedforward disturbance, not MPC control variable
**Date:** 2026-03-06
**Status:** Decided

**Context**
Irrigation timing is an agronomic decision (trigger on theta, fixed shot volume) — not something the MPC should optimize continuously.

**Decision**
IrrigationScheduler is a separate state machine that sets `disturbance_schedule.irr_rate`. MPC treats irrigation as an unmeasured disturbance (d[5]=0 in forecast). EKF observes theta changes in real-time.

**Trade-offs**
- ✅ Clean separation of agronomic and control domains
- ✅ Scheduler can be substrate-aware without complicating QP
- ⚠️ MPC cannot anticipate theta changes (relies on EKF to catch up)
- ❌ Rejected: irrigation as MPC actuator — too many constraints, wrong optimization objective

**Implications**
IrrigationPlugin still exists in registry for profiles that list it, but the channel is locked to 0.0 when scheduler is active.

---

## Cuticular conductance for overnight dryback
**Date:** 2026-03-06
**Status:** Decided

**Context**
Without cuticular conductance, theta flatlined at night (stomata fully closed). Real plants lose water through cuticle even at night.

**Decision**
Added cuticular conductance term to transpiration model — provides realistic overnight dryback.

**Trade-offs**
- ✅ Realistic theta dynamics during lights-off
- ✅ Better EKF estimation of theta overnight
- ⚠️ Small additional computation per timestep

---

## Piecewise linearization of B matrix at each solve
**Date:** 2026-03-07
**Status:** Decided

**Context**
Fixed linearization point caused MPC to make poor decisions when operating far from the linearization point (especially during light transitions).

**Decision**
B matrix is re-linearized around the current operating point at each MPC solve. Verified accurate within 0.01% of nonlinear ground truth.

**Trade-offs**
- ✅ Much better tracking during transients
- ✅ Negligible compute cost (Jacobian evaluation is cheap)
- ⚠️ None significant

---

## CO2 hysteresis scheduler instead of continuous MPC
**Date:** 2026-03-07
**Status:** Decided

**Context**
CO2 injection is binary (valve open/closed) — continuous MPC control of a binary actuator causes rapid cycling.

**Decision**
Hysteresis-based CO2 scheduler: inject when CO2 drops below (target - deadband), stop when above (target + overshoot). Only active during lights-on.

**Trade-offs**
- ✅ Stable on/off behavior, no rapid cycling
- ✅ Simple, predictable control
- ⚠️ Not optimal in the MPC sense — but CO2 dynamics are slow enough that hysteresis works well
- ❌ Rejected: continuous MPC — binary valve makes continuous control meaningless

---

## Outdoor weather presets for simulation
**Date:** 2026-03-07
**Status:** Decided

**Context**
Testing robustness to different outdoor conditions required manually editing config files each time.

**Decision**
Added named weather presets (e.g., hot_humid, cold_dry) selectable via CLI `--weather` flag and API.

**Trade-offs**
- ✅ Easy reproducible testing of outdoor condition sensitivity
- ✅ Accessible from both CLI and web dashboard

---

## Leaf temperature deficit advisory
**Date:** 2026-03-17
**Status:** Decided

**Context**
T_leaf is a first-class state in the 7-state ODE, but nothing was watching the gap between T_air and T_leaf. A large deficit (T_air − T_leaf > 3°C) signals excessive transpiration or poor canopy airflow; a reversed deficit (T_leaf > T_air) signals stomatal closure from drought/salt stress or a direct radiant heat source. Both are actionable grower warnings that existing env_breach advisories don't cover.

**Decision**
Added `TYPE_LEAF_TEMP_DEFICIT` advisory type. `_check_leaf_temp_deficit()` in `advisory/engine.py` evaluates x_hat each cycle and computes the mean deficit over the MPC trajectory. Three severity tiers: info (>3°C), warning (>5°C or reversed), critical (>7°C). Template in `advisory/templates.py` provides direction-specific cause text and recommendations.

**Trade-offs**
- ✅ Catches transpiration stress and airflow problems invisible to VPD/RH channels
- ✅ Reversed deficit (leaf > air) catches stomatal closure — not surfaced anywhere else
- ✅ Trajectory mean gives persistence signal — avoids single-step spikes triggering warnings
- ⚠️ T_leaf is an EKF estimate (MLX90614 sensor) — advisory confidence tied to sensor accuracy
- ❌ Rejected: using trajectory forward-scan like env_breach — deficit doesn't have a setpoint to breach against, so threshold-on-current-state is more appropriate

**Implications**
`current_value` on the advisory is the instantaneous deficit; `predicted_value` is the trajectory mean. Cooldown is 300s (shared default). Advisory auto-resolves when deficit returns to 0–3°C.

---

## Suppress CO2 advisories during lights-off
**Date:** 2026-03-07
**Status:** Decided

**Context**
CO2 injection is disabled at night, so CO2 naturally drifts to ambient (~420 ppm). Advisory engine was generating false breach warnings.

**Decision**
Skip CO2 breach/trend detection when lights are off. Resolve any active CO2 advisories from the lights-on period.

**Trade-offs**
- ✅ Eliminates false positive CO2 advisories at night
- ⚠️ Genuine CO2 issues during lights-off won't be flagged (acceptable — injection is off anyway)

---

## RH reference derived from VPD target
**Date:** 2026-03-04
**Status:** Decided

**Context**
Hardcoded RH targets conflicted with VPD targets at different temperatures. VPD is the biologically meaningful metric.

**Decision**
RH reference is computed from VPD target and leaf-air temperature offset. VPD target is the primary setpoint; RH follows.

**Trade-offs**
- ✅ Physically consistent — RH and VPD targets can't contradict
- ✅ Simplifies phase config (only need to specify VPD target)
- ⚠️ RH target shifts with temperature changes (by design)

---

## HVACD dehumidification formula fix (room volume denominator)
**Date:** 2026-02-27
**Status:** Decided

**Context**
HVACDModel's `_setpoint_mode()` used `rho * airflow_m3s * 1000` as denominator for w_removal. Correct denominator is `rho * V` (room air mass). Airflow mass is ~5x smaller → dehumidification was ~5x too weak.

**Decision**
HVACDPlugin.compute_physics() recomputes w_removal using room volume. HVACD capacity bumped from 8→15 L/hr.

**Trade-offs**
- ✅ Correct physics, proper dehumidification capacity
- ✅ Broke the RH-T coupling trap (high RH preventing cooling)

---

## Integral action dt_solve correction
**Date:** 2026-02-27
**Status:** Decided

**Context**
`dt_solve` was `step_s * mpc_interval` (30×30=900) — 30x too large. Caused bang-bang oscillation from integral windup.

**Decision**
Corrected to `float(self._mpc_interval)` — the actual seconds between solves. Gain=0.003, max=4.0°C.

**Trade-offs**
- ✅ Smooth integral action, no bang-bang
- ✅ ~2°C bias after 5 min of persistent 2°C error (appropriate)

---

## HVACD quadratic changeover deadband and sub-1% effort suppression
**Date:** 2026-03-17
**Status:** Decided

**Context**
Two separate UI/control issues: (1) HVACD mode indicator flickered between heating/cooling when T_air ≈ T_sp because any tiny T error triggered a full proportional response. (2) Real Desert Aire units have a ~0.5°C changeover deadband — the model had no equivalent.

**Decision**
Added a 0.5°C quadratic ramp in `HVACDModel` — sensible output scales as `min(ΔT / 0.5°C, 1.0)` within the deadband, preventing abrupt mode switching near equilibrium. Added sub-1% effort clamp in `HVACDPlugin` — cooling/heating fractions below 1% are zeroed before the UI effort dict is built.

**Trade-offs**
- ✅ Eliminates mode indicator flickering near changeover
- ✅ Better matches real Desert Aire CM3550 behavior
- ⚠️ Slight reduction in control authority right at setpoint (intentional deadband)
- ❌ Rejected: cross-penalty in QP cost — makes P indefinite

**Implications**
The deadband ramp is in `core/hvacd.py:HVACDModel._setpoint_mode()`. The sub-1% suppression is in `core/actuators/hvacd_plugin.py:HVACDPlugin.compute_physics()`.

---

## T_out and w_out added to solver telemetry and WebSocket broadcast
**Date:** 2026-03-17
**Status:** Decided

**Context**
The web dashboard had no visibility into outdoor conditions (temperature and humidity ratio) driving the room's thermal load. Growers couldn't distinguish indoor HVAC issues from outdoor weather load.

**Decision**
`solver_process.py` now includes `T_out` and `w_out` (from disturbance vector d[1], d[2]) in `solve_info`. `WebSocketBroadcaster` forwards these in the `cortex_status` message. Equipment manifest now includes `plugin` field per channel for frontend mode grouping.

**Trade-offs**
- ✅ Dashboard can display current outdoor conditions
- ✅ Plugin field enables mode-aware UI grouping (e.g., show HVACD cooling vs. heating state)
- ⚠️ Slightly larger WebSocket payload per tick (two floats + string per channel)