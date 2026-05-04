# Task 4 — EKF observability monitor + sensor dropout advisory

**Status:** pending

## The punch

`estimation/ekf.py` handles total sensor dropout (NaN guard, prediction-only fallback, pseudo-inverse on singular innovation covariance). It does **not** monitor covariance growth, per-state observability collapse, or per-sensor staleness. The downstream solver consumes the state estimate unconditionally — even when its uncertainty has ballooned past usefulness.

UX consequence: the only signal a grower gets when a single sensor fails is rising EKF innovation, which the advisory engine reports as "model drift". The actionable message — "your CO2 sensor has been NaN for 4 minutes" — is never surfaced.

## Goal

Two layered checks:

1. **Per-state observability monitor.** Track P eigenvalues per state. When any state's variance exceeds a threshold (configurable per state), fire a `state_unobservable` advisory and downgrade trust on that state in the solver (e.g., widen tolerance band, or temporarily freeze its setpoint tracking).
2. **Per-sensor health monitor.** For each sensor input, track time-since-valid-reading and rolling NaN rate. Fire a `sensor_offline` advisory naming the specific sensor when staleness or NaN rate crosses a threshold.

## Acceptance criteria

- Code: per-state P diagonal logging in `estimation/ekf.py`, per-sensor staleness tracking at the sensor read site (Redis adapter or solver_process).
- New advisory types in `advisory/engine.py` and templates in `advisory/templates.py` with grower-friendly text and recommended actions.
- Tests: synthetic NaN injection on each sensor channel, verify the right advisory fires within an agreed window. Synthetic disturbance that drives one state's variance up, verify state-unobservable advisory.
- Documentation pointer in `docs/architecture.md` explaining the new failure-visibility layer.

## Research questions for the team

**Thermodynamics specialist**
- Which states are *naturally* poorly observable from the current sensor set, and which become unobservable only on sensor loss? E.g., theta has only one measurement source; T_wall is observed indirectly. The thresholds should reflect physics, not be uniform.
- Is there a graceful-degradation path where a missing sensor is replaced by a model-based pseudo-measurement (e.g., compute T_leaf from energy balance if MLX90614 fails)?

**MPC specialist**
- When one state goes unobservable, what should the MPC do? Options: widen the tolerance band on that state, drop its tracking term entirely, or shift to a default trajectory. Each has trade-offs — pick by phase.
- Is there a known relationship between EKF P trace and downstream control performance that would motivate a specific threshold?

**SWE architecture**
- Where does sensor freshness live today? `control/io.py`, `control/gateway.py`, or buried in Redis adapters? The tracking belongs at the lowest read site so it is not bypassed.
- Advisory plumbing: cooldowns, severities, resolution conditions. Match the existing 7-type pattern (`advisory/engine.py`, see `_check_leaf_temp_deficit` for the recent template).
