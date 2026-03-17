<!-- SUMMARY -->
> **Scope:** System design, three-process architecture, component boundaries, IO adapter pattern.
> **Key constraint:** ActuatorRegistry is the single source of truth for n_u, bounds, weights — dynamics raises ValueError without it.

---

# Architecture

## Three-Process Design

Cortex is three systems sharing a 7-state ODE physics model:

1. **Production Control System** — Pi 5, real sensors/actuators, 1 Hz closed loop
2. **Physics Simulation** — Dev laptop / CI, no hardware, fast-forward capable (~87x real-time)
3. **Background Calibrator** — SLSQP optimizer running alongside production, tunes 8 model parameters

Production uses Redis IPC between Solver, Gateway, and Calibrator processes.

## IO Adapter Pattern

`SimulatedIO` and `RealIO` implement `IOInterface` ABC — identical control loop code for both.

- `SimulatedIO.write_actuators()` advances the sim room — **must always be called**, even on failsafe
- `RealIO` reads/writes via Redis → Gateway dispatches to hardware (MQTT/Modbus)

## Control Loop (solver_process.py)

EKF+MPC orchestrator with integral action:
- EKF fuses sensor data into state estimate
- MPC solves QP at configurable interval (default 30s)
- Integral action biases T reference to compensate for linearization mismatch (gain=0.003, max=4.0°C)
- Integral resets on light transitions

## Key Component Boundaries

| Component | Responsibility |
|-----------|---------------|
| `core/dynamics.py` | Physics model, delegates actuator effects to plugin registry |
| `mpc/solver.py` | OSQP QP formulation (Np=60, Nc=15, dynamic n_u) |
| `mpc/constraints.py` | Bounds from registry + exclusive pairs + post-solve clamping |
| `estimation/ekf.py` | Extended Kalman Filter (state-only, unaffected by actuator count) |
| `control/gateway.py` | Redis → actuator dispatch via registry dispatch specs |
| `advisory/engine.py` | Predictive warnings, 7 advisory types, severity tiers |
| `calibration/` | SLSQP background optimizer + SharedParams atomic update |

## WebSocket API

FastAPI server broadcasts real-time telemetry via WebSocket:
- State snapshots (T_air, T_leaf, RH, VPD, CO2, theta)
- MPC trajectory forecasts
- Equipment manifest with plugin metadata
- Advisory alerts
- Simulation progress events