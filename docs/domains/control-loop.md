<!-- SUMMARY -->
> **Scope:** SolverProcess (EKF+MPC), ActuatorGateway (dispatch), schedulers (irrigation, CO2), failsafe degradation, IO adapter pattern.
> **Key constraint:** The Gateway is the ONLY process that writes to hardware. SolverProcess publishes solutions to a store; it never touches MQTT/Modbus directly.

---

# Control Loop

The closed-loop control system is split across two cooperating processes plus two schedulers and a failsafe layer.

## Process Boundaries

```
┌─────────────────┐    publish     ┌─────────────────┐    MQTT/Modbus    ┌──────────┐
│  SolverProcess  │───solution───▶│ ActuatorGateway │──────commands────▶│ Hardware │
│  (EKF + MPC)    │   (Redis      │  (dispatch)     │                    │          │
└─────────────────┘    or         └─────────────────┘                    └──────────┘
                       in-memory)
```

## SolverProcess

`control/solver_process.py:SolverProcess`

Owns the brain of the system:
- Reads sensors via `IOInterface` (Redis in production, SimulatedIO in sim)
- Runs EKF predict + update each tick
- Decides whether to solve (via `SolveTrigger`) — every solve is expensive (~9ms steady-state, ~28-113ms cold start)
- Calls `MPCSolver.solve()` and applies post-solve clamps + integral action
- Publishes the solution to a `SolutionStore` (in-memory) or Redis (production)
- Drives the advisory engine after each solve

**Does NOT:** write actuator commands to hardware. That's the Gateway.

In simulation mode, `SolverProcess` *also* calls `io.write_actuators()` — but that just advances the sim room. The Gateway in sim mode validates dispatch logic against the same solution but doesn't actuate.

### Integral Action

T-reference biased by integrated tracking error to compensate for linearization mismatch:
- Gain 0.003, max 4.0°C
- `dt_solve = float(self._mpc_interval)` — must be the actual seconds between solves (not `step_s * mpc_interval`)
- Resets on light transitions to prevent stale bias carrying over

## ActuatorGateway

`control/gateway.py:ActuatorGateway`

The only process that writes to hardware:
- Reads latest solution from `SolutionStore` / Redis
- Evaluates per-actuator `DispatchRule` (dead-band, hold time, rate limit, anti-short-cycle, lights-only, daily caps)
- Dispatches via MQTT (ESP32 relays, 0-10V DAC) or Modbus (HVACD)
- Never modifies setpoints — only suppresses or delays

`control/dispatch.py:DispatchAction` enumerates suppression reasons (`HOLD_DEADBAND`, `HOLD_MIN_TIME`, `HOLD_RATE_LIMIT`, etc.) for observability.

## Schedulers (separate from MPC)

| Scheduler | Lives in | Drives | Why not MPC |
|-----------|---------|--------|-------------|
| `IrrigationScheduler` | `control/irrigation_scheduler.py` | `disturbance_schedule.irr_rate` (d[5]) | Agronomic decision (theta trigger + fixed shot volume); not a continuous control variable |
| `CO2Scheduler` | `control/co2_scheduler.py` | `u_co2` channel | Binary actuator (valve open/closed); continuous MPC causes rapid cycling |

Both are state machines, both run in the SolverProcess tick.

## IO Adapter

`control/io.py:IOInterface` (ABC), `SimulatedIO`, `RealIO`

Same control loop code for sim and production — only the IO backend changes:
- `RealIO` — reads sensors from Redis, writes commands via Gateway
- `SimulatedIO` — wraps a `SimulatedRoom`, advances physics on each `write_actuators()` call

**Invariant:** `SimulatedIO.write_actuators()` advances the sim room. It MUST be called every cycle, even on failsafe. If the failsafe path skips it, simulated time stops.

## Fail-Safe Degradation

`control/failsafe.py:FailSafeManager`

Six priority levels:

| Level | Trigger | Action |
|-------|---------|--------|
| P1_NORMAL | Solver healthy | Apply MPC solution |
| P2 (counter) | OSQP solve fails | Hold last valid u, confidence=0.5 |
| P3 (time) | Solution stale > `stale_threshold_s` | Gateway holds last commands |
| P4 | Solution stale > `esp32_fallback_s` | ESP32 firmware PID takes over |
| P5 | ESP32 offline | TrolMaster static setpoints |
| P6 | Total loss | HVAC built-in safety limits |

P1–P3 live in `FailSafeManager`. P4–P6 are firmware/hardware fallbacks.

## SolveTrigger

`control/triggers.py:SolveTrigger`

Decides when to solve. The MPC doesn't run every tick — it runs:
- On a configurable cadence (default 30s)
- On large state deltas (e.g., light transition)
- On phase change

Between solves, the gateway dispatches the existing solution.