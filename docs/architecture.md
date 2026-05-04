<!-- SUMMARY -->
> **Scope:** Three-system design, process boundaries, component dependency graph, IO adapter pattern, key invariants.
> **Key constraint:** `GrowRoomDynamics` is the single source of truth for physics. `ActuatorRegistry` is the single source of truth for n_u, bounds, weights. Dynamics raises `ValueError` without a registry — no auto-build fallback.

---

# Architecture

Cortex is three systems sharing one physics model:

1. **Production Control** — Pi 5, real sensors/actuators, ~1 Hz closed loop
2. **Physics Simulation** — dev laptop / CI, no hardware, fast-forward (~87× real-time)
3. **Background Calibrator** — SLSQP optimizer alongside production, tunes 7 model parameters

All three use the same `core/dynamics.py:GrowRoomDynamics` code. The difference is what drives inputs and consumes outputs.

## Process Boundaries (production)

```
┌──────────────┐  publish   ┌──────────────────┐  MQTT/Modbus   ┌─────────┐
│ SolverProcess├──Redis────▶│ ActuatorGateway  ├───────────────▶│ Hardware│
│ (EKF + MPC)  │            │ (dispatch rules) │                └─────────┘
└──┬───────────┘            └──────────────────┘
   │ Redis (telemetry)
   ▼
┌──────────────┐
│ Calibrator   │  background SLSQP, atomic SharedParams swap
└──────────────┘
                 ┌──────────────┐
                 │ FastAPI/WS   │  dashboard, control overrides
                 └──────────────┘
```

| Process | Owns | Cadence |
|---------|------|---------|
| **Solver** (`control/solver_process.py`) | EKF, MPC, advisory engine, schedulers | 1 Hz tick; MPC solve every 30 s by default |
| **Gateway** (`control/gateway.py`) | Per-actuator dispatch rules, hardware writes | 1 Hz |
| **Calibrator** (`calibration/calibrator.py`) | SLSQP loop, parameter updates | 5–30 min, force-trigger on innovation/phase/light/irrigation |
| **API** (`api/server.py`) | WebSocket telemetry, REST control, sim manager | event-driven |

The Gateway is the **only** process that writes to hardware. The Solver publishes solutions; it never touches MQTT/Modbus directly.

## Component Dependency Graph

```
                  config/*.yaml
                       │
                       ▼
                ┌──────────────┐
                │ ActuatorRegistry │◀──────── equipment profile
                └──────┬───────┘                (hvacd_full / ac_dehum / ...)
                       │
                       ▼
                ┌──────────────────┐
                │ GrowRoomDynamics │  the 7-state ODE
                └────┬─────┬───────┘
                     │     │
        ┌────────────┘     └───────────────┐
        ▼                                  ▼
┌──────────────┐                  ┌──────────────┐
│ StateEstimator│                  │ ModelCalibrator│
│ (EKF)         │                  │ (SLSQP, bg)    │
└──────┬───────┘                  └──────┬───────┘
       │ x_hat                           │ writes
       ▼                                  ▼
┌──────────────┐    bounds       ┌──────────────┐
│ MPCSolver    │◀──n_u, u_min/max│ SharedParams │
│ (OSQP)       │                  └──────────────┘
└──────┬───────┘
       │ u_opt, trajectory
       ▼
┌──────────────┐    advisories    ┌──────────────┐
│ SolverProcess├─────────────────▶│ AdvisoryEngine│
│              │                  └──────────────┘
└──────┬───────┘
       │ publishes solution
       ▼
┌──────────────┐
│ ActuatorGateway│
└──────────────┘
```

See [`physics-model.md`](domains/physics-model.md), [`mpc-solver.md`](domains/mpc-solver.md), [`ekf.md`](domains/ekf.md), [`calibration.md`](domains/calibration.md), [`actuator-plugins.md`](domains/actuator-plugins.md), [`advisory.md`](domains/advisory.md), [`control-loop.md`](domains/control-loop.md).

## IO Adapter Pattern

`control/io.py:IOInterface` (ABC), with two backends:

- `RealIO` — reads sensors from Redis, writes commands via Gateway → MQTT/Modbus
- `SimulatedIO` — wraps `SimulatedRoom`, advances physics on each `write_actuators()` call

The control loop code is **identical** in both. Only the IO backend changes.

**Invariant:** `SimulatedIO.write_actuators()` advances the sim room. It must be called every cycle, even on failsafe — otherwise sim time stops.

## Key Invariants

1. `GrowRoomDynamics` is the single source of truth for physics. EKF, OSQP, SLSQP, and SimulatedRoom share it; never a second copy.
2. OSQP never sees the nonlinear model — only linearized `A_d`, `B_d`. Linearization happens every MPC solve.
3. SLSQP never runs in the control loop. It runs in a background thread; writes via atomic reference swap. Control loop reads lock-free.
4. The EKF predict step uses the same nonlinear model as the simulation. The update step uses real sensors. The EKF makes the linear OSQP work despite nonlinearities.
5. `SharedParams` is the only bridge between calibration and control.
6. The IO adapter pattern means control loop code is identical for sim and production.
7. Fail-safe degradation is layered (P1–P6) and independent. Each layer works without the layers above. See [`control-loop.md`](domains/control-loop.md).
8. All sensor data enters through the IO interface; all commands exit through the Gateway. The Solver never directly touches MQTT or Modbus.
9. The Gateway never modifies MPC setpoints — only suppresses or delays via dispatch rules.

## Decoupled Solver/Gateway Cadence

The solver decides **what** to command (setpoints over a horizon). The gateway decides **whether and when** to dispatch (dead-band, hold time, rate limit, anti-short-cycle). Decoupling these means:
- MPC can solve infrequently (~30 s) without sacrificing actuator hygiene
- Dispatch rules can change without rebuilding the QP
- A stale solution still produces safe behavior (gateway holds last command, then falls to firmware PID per failsafe ladder)