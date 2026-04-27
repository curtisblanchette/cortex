<!-- SUMMARY -->
> **Scope:** Architecture and design decisions from the last 90 days.
> **Key constraint:** Entries older than 90 days should be moved to decisions-archive.md.

---

# Recent Decisions

## Integral action diagnosis — names what it is actually compensating for
**Date:** 2026-04-27
**Status:** Decided

**Context**
The previous decision note ("Integral action dt_solve correction", 2026-02-27) framed the reference-bias integral as compensation for "linearization mismatch". Subsequent piecewise-linearization work (2026-03-07) verified the B matrix is accurate within 0.01% of the nonlinear ground truth over the control horizon. Both can't be true — if linearization is that accurate, a ±4°C integral has no linearization error to absorb. A 3-angle review (thermo, MPC, code archaeology) was run to find what the integral is actually doing.

**Decision**
The integral is *not* a linearization patch. It absorbs persistent bias from slow unmodeled dynamics that the in-horizon QP plan cannot resolve:

| Mechanism | Magnitude | Mechanism |
|-----------|-----------|-----------|
| Wall thermal mass beyond horizon | ~0.5–1.0°C | τ_wall = C_wall/UA_wall ≈ 10,000s vs. control horizon 450s — the QP literally cannot see the wall reach steady state in its plan. |
| Outdoor-weather persistence-forecast bias | ~0.5–1.5°C | `mpc/forecast.py` holds T_out / w_out constant over the horizon. Any systematic bias (e.g., AC delivers air colder than the model claims) appears as a constant disturbance the QP cannot anticipate. |
| HVAC supply-air lag | ~0.3–0.6°C | τ_hvac = 120s, persistent at high duty cycle. |
| Affine VPD/RH linearization (exponential SVP over 15 steps) | ~0.2–0.4°C | Output linearization compounds because saturation vapor pressure is exponential in T. |

The deeper structural reason: `mpc/solver.py` penalizes `||u - u_prev||²` (deviation from previous), not `||u||²`. With R on Δu, the QP has no incentive to drive u toward a specific value at steady state — tracking error is set by the Q/R balance, not the dynamics. Bias absorption has to happen somewhere, and today it happens in this integral.

**Trade-offs**
- ✅ The current behaviour is preserved (defaults match the prior hardcoded `gain=0.003`, `t_clamp=4.0`, `vpd_clamp=0.3`).
- ✅ Constants moved to `config/room.yaml` under `room.integral_action` — tunable per room.
- ✅ Saturation tracking + new `TYPE_INTEGRAL_SATURATED` advisory surface the case where the bias has pegged at clamp (signalling either equipment under-capacity or a model-reality gap that exceeds the bias headroom).
- ✅ Per-tick `info` dict now includes `t_integral`, `vpd_integral`, and saturation flags so dashboards can monitor it.
- ⚠️ The integral still applies to the *reference*, not the state estimate. The principled fix is an augmented-state output-disturbance observer with the EKF estimating a constant bias term per channel. Queued, not done — needs the regression suite (Task 5) to land safely.
- ❌ Rejected: removing the integral. Without it, persistent steady-state error of 1.5–3.0°C would degrade compliance and stress the actuators. The integral is doing real work.

**Implications**
- `control/solver_process.py` reads the integral config from `room.integral_action`, falls back to legacy defaults when absent.
- `advisory/templates.py:integral_saturated` produces grower-facing copy that names likely physical causes ("outdoor load exceeds capacity", "wall thermal mass mistuned").
- `tests/test_integral_action.py` pins the math, the configurability, and the saturation tracking — first regression test on the controller path.
- Follow-up task: replace the reference-bias integral with an augmented-state disturbance observer once Task 5's regression suite gives us a safety net.

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