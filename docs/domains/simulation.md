<!-- SUMMARY -->
> **Scope:** Simulation environment, SimulatedRoom, IO adapter pattern, disturbance scheduling, weather presets, growth phases, scenarios, CLI usage.
> **Key constraint:** Simulation starts at midnight — initial theta uses night target, not day target. `SimulatedIO.write_actuators()` advances the sim room and **must always be called**, even on failsafe paths.

---

# Simulation

## What It Is

`simulation/room.py:SimulatedRoom` is a software grow room. The same `GrowRoomDynamics` code that runs in production drives the simulation — but parameters can be intentionally perturbed (`true_params ≠ controller_params`) to test robustness to model error.

## Running Simulations

```bash
python main.py simulate \
    --fast \
    --duration 43200 \
    --chart --chart-output output/$(date +%s).png \
    --profile hvacd_full
```

Common flags:

| Flag | Meaning |
|------|---------|
| `--fast` | Fast-forward mode (~87× real-time on Mac) |
| `--duration <s>` | Simulated duration in seconds |
| `--phase <name>` | Growth phase (`clone`, `veg`, `early_flower`, `mid_flower`, `late_flower`, `dry`) |
| `--profile <name>` | Equipment profile (`hvacd_full`, `ac_dehum`, `ac_minimal`, `hvacd_no_irr`) |
| `--substrate <name>` | Substrate type |
| `--weather <preset>` | Outdoor weather preset |
| `--chart` | Render compliance chart at end |

## IO Adapter

`control/io.py` defines `IOInterface` (ABC) with two backends:

- `RealIO` — reads from Redis, writes to Gateway → hardware
- `SimulatedIO` — wraps `SimulatedRoom`, advances physics each `write_actuators()` call

The control loop code is identical — only the backend changes.

**Invariant:** `SimulatedIO.write_actuators()` is what advances simulated time. If a failsafe path skips it, the simulation freezes. The failsafe always calls write, even when applying a hold.

## Disturbance Scheduling

`simulation/disturbance.py:DisturbanceScheduler` produces the `d` vector over time:

- **Light:** scheduled on/off transitions (configurable times); also drives the MPC's predictive forecast
- **Outdoor weather:** sinusoidal `T_out` with configurable mean/amplitude; `w_out` from preset
- **Plant count, phase:** static per-run
- **Irrigation rate:** set by `IrrigationScheduler` state machine (not a control variable; see [`physics-model.md`](physics-model.md#irrigation-as-feedforward-disturbance))

## Weather Presets

Defined in `cli/commands/simulate.py:WEATHER_PRESETS` and accessible from:
- CLI: `--weather <preset>`
- API: `SimulateRequest.weather` field

Presets capture realistic combinations like `hot_humid`, `cold_dry`, etc. for robustness testing.

## Growth Phases

`config/phases/*.yaml` — six phases: clone, veg, early_flower, mid_flower, late_flower, dry. Each defines:
- Day/night setpoints for T, RH, VPD, CO2, theta
- Compliance tolerances (used by [`advisory engine`](advisory.md))
- Light schedule (on/off times)

RH reference is **derived from VPD target and leaf-air offset** — not hardcoded. See `decisions-recent.md` "RH reference derived from VPD target".

## Scenarios

`simulation/scenarios.py` — pre-built scenario definitions for testing:
- Long-form runs (e.g., 24 h mid-flower)
- Specific events (light transitions, HVAC failure, phase change)
- Used by `cli/commands/simulate.py` and `api/simulation_manager.py`

## API-Driven Simulation

`api/simulation_manager.py:SimulationManager` runs sims triggered from the web dashboard:
- Builds all components from config (registry, dynamics, EKF, MPC, calibrator, advisory)
- Broadcasts progress and live telemetry via WebSocket (`api/broadcast.py`)
- Same parameter surface as CLI

## Faults

`simulation/faults.py:FaultInjector` — programmatic injection of:
- Sensor drift / dropout
- Actuator failures (stuck open/closed, partial response)
- Communication delays

Used to validate failsafe ladder behavior without real hardware failures.

## Performance

| Metric | Value |
|--------|-------|
| Throughput on Mac | ~87 ticks/s (~87× real-time) |
| MPC solve steady state | ~9 ms |
| MPC solve cold start | ~28–113 ms |
| Compliance target | >97% across all profiles on 12 h sims |