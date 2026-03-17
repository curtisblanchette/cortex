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

**Known trade-off:** Clamp can cause saw-tooth oscillation when QP plans to use both actuators (e.g., dehum for waste heat).

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