<!-- SUMMARY -->
> **Scope:** Extended Kalman Filter for state estimation; predict/update split; sensor fusion; observation matrix; innovation diagnostics.
> **Key constraint:** EKF predict step uses the SAME nonlinear model as simulation. The EKF is what makes the linear OSQP model work despite nonlinearities — it keeps the operating point accurate.

---

# State Estimation (EKF)

`estimation/ekf.py:StateEstimator`

The EKF fuses noisy sensor data with model predictions to produce the best estimate of the room's true state. It runs every control cycle (typically 1 Hz).

## Predict / Update Split

| Step | What it does | Model used |
|------|--------------|-----------|
| **Predict** | Propagate state estimate forward via `dynamics.nonlinear_step()` | Full nonlinear ODE (RK4) |
| **Update** | Correct prediction using sensor measurements | Linearized observation `z = H @ x` |

The predict step uses the *same* nonlinear code path as the simulation environment — there is never a second copy of the ODEs.

## State and Measurement Vectors

- State `x_hat`: 7 elements (matches `dynamics.N_STATES`)
- Measurement `z`: 6 elements (`simulation.noise.N_MEASUREMENTS`)
- Observation matrix `H` is sparse selector — `STATE_TO_MEAS` maps each measured state to its row in `H`

## Covariance Matrices

| Matrix | Role | Default magnitude |
|--------|------|-------------------|
| `Q` (7×7) | Process noise — model uncertainty per step | T_a/T_l: 0.01, w_a: 0.004, CO2: 1.0, theta: 0.0001 |
| `R` (6×6) | Measurement noise — sensor σ² | T_air: 0.09 (σ=0.3°C), CO2: 900 (σ=30 ppm), theta: 0.0001 (σ=0.01) |
| `P` (7×7) | State covariance, evolves online | Initialized via `P0` |

Tune `Q` higher to trust the model less; `R` higher to trust sensors less.

## Innovation Tracking

The EKF records innovation magnitude per step. This drives:
- **Model drift advisory** (`advisory/engine.py`) — sustained innovation > 2.0 for 60s flags `TYPE_MODEL_DRIFT`
- **Calibrator trigger** — high innovation can fire `ModelCalibrator.trigger_fast_calibration()`

## Integration Points

- **Constructed by:** `cli/bootstrap.py` and `api/simulation_manager.py`
- **Driven by:** `control/solver_process.py:SolverProcess` — calls `predict()` then `update(z)` each tick
- **Output consumed by:** MPC solver (as `x_hat`), advisory engine (for current-state checks)

## Invariants

- The EKF is the **only** source of `x_hat` for downstream consumers. Don't bypass it.
- The predict step must use the latest `SharedParams` so it stays consistent with the calibrator.
- After EKF init, `x_hat` is `GrowRoomDynamics.default_initial_state()` — for sims this is overridden to use the night-target theta.