# Cortex MPC

Grow room Model Predictive Control system. 7-state ODE physics model with OSQP-based MPC, EKF state estimation, and a modular actuator plugin architecture.

## Non-negotiable
 - After completing a plan's implementation or any implementation work, run `/docs` to update AGENTS.md, README.md, and the `/docs` domain library.

## Domain Knowledge Index

Domain docs live in `/docs`. Each starts with a `<!-- SUMMARY -->` block (Scope + Key constraint) — **scan that first**, only read the full file if the task actually requires the detail. Load reactively, not preemptively.

**Top-level**

> Architecture (3-process design, dependency graph, IO adapter, invariants) → [`docs/architecture.md`](docs/architecture.md)
> Deployment (Pi 5, Redis IPC, MQTT/Modbus, data stores) → [`docs/deployment.md`](docs/deployment.md)
> Testing (two-layer strategy, sim compliance, regression baselines) → [`docs/testing.md`](docs/testing.md)

**Domain (`docs/domains/`)**

> Physics Model (7-state ODE, vectors, calibrated params, sub-models) → [`docs/domains/physics-model.md`](docs/domains/physics-model.md)
> MPC Solver (OSQP, exclusive pair clamp, forecast, CO2 scheduler) → [`docs/domains/mpc-solver.md`](docs/domains/mpc-solver.md)
> EKF (predict/update, sensor fusion, innovation diagnostics) → [`docs/domains/ekf.md`](docs/domains/ekf.md)
> Calibration (SLSQP loop, SharedParams atomic swap, what's tuned) → [`docs/domains/calibration.md`](docs/domains/calibration.md)
> Actuator Plugins (registry, profiles, channels, dispatch specs) → [`docs/domains/actuator-plugins.md`](docs/domains/actuator-plugins.md)
> Advisory (7 types, lifecycle, suppression rules) → [`docs/domains/advisory.md`](docs/domains/advisory.md)
> Control Loop (SolverProcess, Gateway, schedulers, failsafe ladder) → [`docs/domains/control-loop.md`](docs/domains/control-loop.md)
> Simulation (SimulatedRoom, weather presets, scenarios, faults) → [`docs/domains/simulation.md`](docs/domains/simulation.md)

**Decisions and tasks**

> Decisions — recent (last 90 days) → [`docs/decisions-recent.md`](docs/decisions-recent.md)
> Decisions — archive → [`docs/decisions-archive.md`](docs/decisions-archive.md)
> Hardening tasks workstream → [`docs/tasks/`](docs/tasks)

**When to load a doc:**
- You encounter a constraint or convention you don't recognize
- You're about to make a structural decision (new actuator, new advisory, schema change)
- Something in the code looks like it conflicts with a doc — flag it; one of them is out of date

**When NOT to load:** small bug fixes, config tweaks, or tasks already self-contained from this conversation.

## Post-Implementation Workflow

After completing a plan or significant implementation work, run the docs sync workflow (`/docs`) to update AGENTS.md, README.md, and `/docs/`. Decisions made under uncertainty (rejected alternatives, accepted trade-offs, quick fixes over proper fixes) belong in `docs/decisions-recent.md` so future sessions don't re-litigate them.

## Quick Start

```bash
pip install -r requirements.txt
python main.py simulate --fast --duration 86400 --chart    # 12hr fast sim with chart
python main.py simulate --phase clone --profile ac_minimal
python main.py all                                        # production: solver + gateway + API + calibrator
```

## Project Structure

```
main.py                  Entry point — delegates to cli/main.py
cli/                     CLI commands (simulate, solver, gateway, api, calibrator)
core/dynamics.py         Central physics model (7-state ODE), delegates to plugin registry
core/actuators/          Actuator plugin system: base ABC, registry, factory, 8 plugins
mpc/solver.py            OSQP QP formulation (Np=60, Nc=15, dynamic n_u)
mpc/constraints.py       Bounds, exclusive pair constraints, post-solve clamping
estimation/ekf.py        Extended Kalman Filter (state-only)
calibration/             SLSQP background optimizer + SharedParams
simulation/              SimulatedRoom, noise, disturbance scheduler
control/io.py            IOInterface — SimulatedIO and RealIO backends
control/gateway.py       Redis → actuator dispatch via registry
control/solver_process.py  EKF+MPC orchestrator with integral action
config/                  YAML configs: room, equipment profiles, growth phases, substrate
api/                     FastAPI dashboard/endpoints
```

## Key Architecture

### Actuator Plugin System
- `ActuatorRegistry` is the **single source of truth** for actuator count (n_u), bounds, weights, and field mappings
- Equipment profiles defined in `config/equipment.yaml` under `profile:` key
- Built-in profiles: `hvacd_full` (default), `ac_dehum`, `ac_minimal`, `hvacd_no_irr`
- CLI `--profile` flag overrides the config profile
- The registry is **required** by dynamics — it raises `ValueError` if None (no auto-build fallback)

### IO Adapter Pattern
- `SimulatedIO` and `RealIO` share `IOInterface` ABC — same control loop for sim and production
- `SimulatedIO.write_actuators()` advances the sim room — **must always be called**, even on failsafe

### Exclusive Pair Constraints
- Linear QP constraint `u_a + u_b <= max_sum` + post-solve `clamp_exclusive()` zeros the smaller actuator
- Active pairs: `(u_hum, u_dehum, 100)`, `(u_heat, u_cool, 100)`
- Cross-penalty (bilinear in P) does NOT work — makes P indefinite, violates OSQP PSD requirement

## Conventions

- **No legacy/dual code paths** — clean single implementation only
- **No auto-building fallback objects** — raise errors for missing dependencies
- **No unnecessary abstractions** — don't add helpers or wrappers for one-time operations
- YAML configs have top-level keys (e.g., `room:`, `hvacd:`) — code must handle both nested and flat dicts
- R weight penalizes `||u - u_prev||²` (deviation from previous), NOT `||u||²` — this means the QP has no incentive to drive u toward a specific value at steady state, so persistent bias must be absorbed elsewhere (the reference-bias integral; see `docs/decisions-recent.md` "Integral action diagnosis")
- Reference-bias integral in `control/solver_process.py` is **not** a linearization patch — it absorbs slow unmodeled dynamics (wall thermal mass with τ ≫ horizon, outdoor weather persistence-forecast bias, HVAC supply lag). Tunable per room via `room.integral_action` in `config/room.yaml`
- OSQP needs: upper-triangular P, regularization for positive definiteness, high max_iter (~4000)

## Config

- `config/room.yaml` — Room dimensions, thermal properties
- `config/equipment.yaml` — Equipment specs and named profiles
- `config/substrate.yaml` — Growing medium properties
- `config/phases/*.yaml` — Growth phase setpoints (clone, veg, dry, early/mid/late flower)

## Dependencies

numpy, scipy, osqp, numba, pyyaml (core) — fastapi, uvicorn, redis, paho-mqtt (production). Managed with pip + `requirements.txt`.

## Testing

```bash
python -m pytest tests/
```

## Domain Knowledge Index

Domain docs live in `/docs`. Load them **reactively, not preemptively** — only read a file when the task actually touches that domain. Scan the summary header first; only read the full file if you need the detail.

> Architecture → [`docs/architecture.md`](docs/architecture.md)
> Physics Model → [`docs/physics-model.md`](docs/physics-model.md)
> MPC Solver → [`docs/mpc-solver.md`](docs/mpc-solver.md)
> Actuator Plugins → [`docs/actuator-plugins.md`](docs/actuator-plugins.md)
> Simulation → [`docs/simulation.md`](docs/simulation.md)
> Deployment & Infra → [`docs/deployment.md`](docs/deployment.md)
> Testing Strategy → [`docs/testing.md`](docs/testing.md)
> Recent Decisions → [`docs/decisions-recent.md`](docs/decisions-recent.md)
> Decision Archive → [`docs/decisions-archive.md`](docs/decisions-archive.md)

**When to load a doc:**
- You encounter a pattern, constraint, or naming convention you don't recognize
- You're about to make a structural decision (new plugin, constraint change, physics modification)
- Something in the code looks like it conflicts with a doc

**When NOT to load a doc:**
- The task is self-contained and doesn't touch domain boundaries
- You already have sufficient context from the current session
- The task is a small bug fix or config change

If something in the code contradicts a doc, flag it — one of them is out of date.

## Post-Implementation Workflow

After completing a plan or significant implementation work, run `/docs` to sync
README.md, CLAUDE.md, and the /docs domain knowledge library.

## Preferences
- Don't work in ~/code/esp/my-esp32-project/apps/cortex - this is the legacy version.
- The web app is in ~/code/esp/my-esp32-project/apps/web
- simulations are run using this format
```
python main.py simulate \
    --fast \
    --duration 43200 \
    --chart --chart-output output/$(date +%s).png \
    --profile hvacd_full
```