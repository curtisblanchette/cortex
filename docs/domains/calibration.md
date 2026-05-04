<!-- SUMMARY -->
> **Scope:** SLSQP background calibrator, SharedParams atomic swap, calibrated parameter list, trigger conditions, what gets tuned vs. what doesn't.
> **Key constraint:** SLSQP never runs in the control loop. It runs in a background thread and writes via atomic reference swap. Control loop reads lock-free.

---

# Calibration

`calibration/calibrator.py:ModelCalibrator`, `calibration/params.py:SharedParams`

SLSQP background optimizer that minimizes the gap between model predictions and real sensor data by tuning physical parameters. Runs alongside the controller; never blocks it.

## Calibrated Parameters (7)

`calibration/params.py:CalibrationParams`

| Param | Unit | Description |
|-------|------|-------------|
| `UA_wall` | W/K | Wall thermal conductance |
| `UA_env` | W/K | Envelope conductance to outdoors |
| `tau_hvac` | s | HVAC thermal time constant |
| `g_s_max` | mol/m²/s | Max stomatal conductance |
| `K_drain` | 1/hr | Substrate drainage coefficient |
| `C_wall` | J/K | Wall/thermal mass capacitance |
| `eta_hvac` | — | HVAC efficiency factor |

Bounds in `calibration/bounds.py:PARAM_BOUNDS`. SLSQP clips to physical limits.

## What Is *Not* Calibrated

- **Room geometry** (V, A_floor, A_canopy) — measured, not tuned.
- **Equipment specs** (capacity, airflow) — fixed in `equipment.yaml`.
- **LAI** — phase-derived, lives on `GrowRoomDynamics`, not in `CalibrationParams`.
- **Sensor noise** — set in EKF `R` matrix, not via SLSQP.

## Trigger Conditions

The calibrator runs on whichever fires first:
- **Periodic** — default `interval_s=600` (10 min), configurable
- **Force trigger** — `trigger_fast_calibration()` is called by other components on:
  - EKF innovation threshold exceeded
  - Phase change or lights transition
  - Irrigation event

## SharedParams (atomic swap)

`calibration/params.py:SharedParams`

- **Reads** are lock-free: control loop grabs the current reference at cycle start and uses it for the whole cycle
- **Writes** are atomic reference swap: SLSQP builds new `CalibrationParams`, swaps the reference in one operation
- Even if SLSQP writes mid-cycle, the control loop sees a consistent set for that cycle (it captured the reference at the start)
- `_version` increments on each write — readers can detect updates if they care

## Convergence and History

- `_calibrate()` returns `(params, cost, info)` from `scipy.optimize.minimize(method='SLSQP')`
- History list grows; truncate externally if memory matters
- `min_samples=300` — won't run with fewer telemetry samples

## Integration Points

- **Constructed by:** `cli/bootstrap.py` (production) and `api/simulation_manager.py` (sim)
- **Driven by:** its own background thread (`start()` / `stop()`)
- **Reads:** telemetry from `telemetry_source.get_recent_data(window_s)` (Redis or in-memory)
- **Writes:** `SharedParams` reference; `dynamics.params` reads from the same store