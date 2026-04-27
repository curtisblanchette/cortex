<!-- SUMMARY -->
> **Scope:** OSQP QP formulation, exclusive pair constraints, reference generation, forecast system.
> **Key constraint:** OSQP requires upper-triangular P with positive definiteness — cross-penalty (bilinear) violates PSD requirement.

---

# MPC Solver

## QP Formulation

- Prediction horizon Np=60, Control horizon Nc=15
- Dynamic n_u from ActuatorRegistry
- R weight penalizes `||u - u_prev||²` (deviation from previous), NOT `||u||²`
- OSQP settings: upper-triangular P, regularization for positive definiteness, max_iter ~4000

## Piecewise Linearization

B matrix is re-linearized at each solve around current operating point. Verified accurate within 0.01% of nonlinear ground truth over 15 steps.

## Exclusive Pair Constraints

Linear QP constraint `u_a + u_b <= max_sum` + post-solve `clamp_exclusive()` zeros the smaller actuator.

Active pairs (auto-detected from registry):
- `(u_hum, u_dehum, 100)`
- `(u_heat, u_cool, 100)`

**Why not cross-penalty?** Bilinear u_a*u_b in P makes P indefinite → OSQP rejects it. True mutual exclusion is non-convex; linear constraint + clamp is the best convex approximation.

**Measured behaviour** (12hr `ac_dehum` sim, see `docs/tasks/02-results/clamp_summary.md`):
- The QP plans to use both pair members on **78% of solves** (linearization split — R penalises Δu, no penalty on splitting).
- The post-clamp winner stays stable for long runs; not solve-by-solve sawtooth.
- Heat/cool pair: ~4 winner-flips/hr, hum/dehum: ~0.3/hr (dehum dominates).
- HVACD profiles (`hvacd_full`) **never trigger the clamp** — they expose `cool_sp`/`heat_sp`/`dehum_sp`, not the standalone `u_cool`/`u_heat`/`u_dehum` channels the pairs key off.

**Mitigation:** Soft hysteresis in `clamp_exclusive` keeps the previous winner when the two pre-clamp values are within `room.clamp_hysteresis.hysteresis_margin` of each other (default 5%). Suppresses regime-shift flips without touching the QP.

## Predictive Disturbance Forecast

`mpc/forecast.py` uses:
- **Light:** Scheduled forecast via `_light_heat()` — MPC sees transitions 30+ min ahead for pre-cooling/pre-heating
- **Outdoor weather:** Persistence forecast (hold current) with configurable presets
- **Irrigation:** d[5]=0 in forecast (scheduler-driven, not predicted)

## CO2 Scheduler

Hysteresis-based on/off control (not continuous MPC):
- Deadband: 75 ppm, overshoot: 75 ppm
- Only active during lights-on (injection disabled at night)
- CO2 advisories suppressed during lights-off to avoid false alarms