# Cortex MPC

Grow room Model Predictive Control system. 7-state ODE physics model with OSQP-based MPC, EKF state estimation, and a modular actuator plugin architecture. Runs in production on a Pi 5; runs in simulation on any laptop.

> **For agents and contributors:** the source of truth for current architecture, conventions, and decisions lives in [`/docs`](docs/) and [`AGENTS.md`](AGENTS.md). This README is the entry point only — short, durable, and link-heavy.

---

## Quick Start

```bash
pip install -r requirements.txt

# 12-hour fast simulation with chart
python main.py simulate --fast --duration 43200 --chart --profile hvacd_full

# Specific phase + minimal profile
python main.py simulate --phase clone --profile ac_minimal

# Production: solver + gateway + API + calibrator
python main.py all
```

CLI individual processes: `python main.py {solver,gateway,api,calibrator,simulate}`.

## What It Is

Three systems sharing one physics model (`core/dynamics.py:GrowRoomDynamics`):

| System | Purpose | Where |
|--------|---------|-------|
| **Production Control** | Real sensors and actuators, ~1 Hz closed loop | Pi 5 in the grow room |
| **Physics Simulation** | Same control code, no hardware, ~87× real-time | Dev laptop / CI |
| **Background Calibrator** | SLSQP tunes 7 model parameters from telemetry | Alongside production |

Production runs as decoupled processes communicating via Redis: **Solver** (EKF + MPC) → **Gateway** (dispatch rules → MQTT/Modbus) → hardware. The Gateway is the only process that writes to hardware. See [`docs/architecture.md`](docs/architecture.md) and [`docs/deployment.md`](docs/deployment.md).

## Where Everything Lives

```
core/dynamics.py         7-state ODE — single source of truth for physics
core/actuators/          Plugin system: registry, factory, 8 plugins
core/{hvacd,transpiration,substrate,thermodynamics}.py   Sub-models
mpc/                     OSQP QP, constraints, reference, forecast
estimation/ekf.py        Extended Kalman Filter
calibration/             SLSQP background optimizer + SharedParams
control/                 SolverProcess, Gateway, schedulers, dispatch, failsafe, IO
advisory/                Predictive advisory engine (7 types)
simulation/              SimulatedRoom, disturbance scheduler, scenarios, faults
api/                     FastAPI server, WebSocket broadcast, simulation manager
cli/                     Entry points, bootstrap, visualization
config/                  YAML configs (room, equipment, phases, substrate, profiles)
tests/                   Sparse — pins specific invariants only
scripts/                 Measurement / regression scripts
docs/                    Domain knowledge — start here
```

## Domain Knowledge

For depth on any subsystem, follow the [`AGENTS.md`](AGENTS.md) Domain Knowledge Index. Quick links:

- [`docs/architecture.md`](docs/architecture.md) — process boundaries, dependency graph, IO adapter, key invariants
- [`docs/domains/physics-model.md`](docs/domains/physics-model.md) — state/control/disturbance vectors, calibrated params, dynamics interfaces
- [`docs/domains/mpc-solver.md`](docs/domains/mpc-solver.md) — OSQP QP, exclusive pair clamp, forecast, CO2 hysteresis
- [`docs/domains/actuator-plugins.md`](docs/domains/actuator-plugins.md) — registry, equipment profiles, channel mappings
- [`docs/domains/ekf.md`](docs/domains/ekf.md) — predict/update, sensor fusion, innovation diagnostics
- [`docs/domains/calibration.md`](docs/domains/calibration.md) — SLSQP loop, SharedParams atomic swap, what's tuned vs. not
- [`docs/domains/advisory.md`](docs/domains/advisory.md) — 7 advisory types, lifecycle, suppression rules
- [`docs/domains/control-loop.md`](docs/domains/control-loop.md) — SolverProcess, Gateway, schedulers, failsafe ladder
- [`docs/domains/simulation.md`](docs/domains/simulation.md) — sim env, weather presets, scenarios, faults
- [`docs/testing.md`](docs/testing.md) — two-layer validation strategy
- [`docs/decisions-recent.md`](docs/decisions-recent.md) — last-90-days architecture decisions

## Conventions

- **No legacy / dual code paths** — clean single implementation only
- **No auto-building fallback objects** — raise errors for missing dependencies (e.g., dynamics requires a registry)
- **No unnecessary abstractions** — don't add helpers or wrappers for one-time operations
- YAML configs use top-level keys; code must handle both nested and flat dicts
- R weight in MPC penalizes `||u − u_prev||²` (deviation from previous), NOT `||u||²`
- OSQP needs upper-triangular P, regularization for positive definiteness, high `max_iter` (~4000)

## Dependencies

`numpy`, `scipy`, `osqp`, `numba`, `pyyaml` (core); `fastapi`, `uvicorn`, `redis`, `paho-mqtt` (production). Managed with `pip` and `requirements.txt`.

## Testing

```bash
python -m pytest tests/
```

Pytest is intentionally sparse — primary validation is **simulation compliance** (>97% target on 12 h sims). See [`docs/testing.md`](docs/testing.md).