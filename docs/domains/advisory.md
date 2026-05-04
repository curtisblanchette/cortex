<!-- SUMMARY -->
> **Scope:** Predictive advisory engine; 7 advisory types; severity tiers; deduplication and cooldown lifecycle; suppression rules.
> **Key constraint:** CO2 advisories are suppressed during lights-off (injection is disabled, drift to ambient is expected). Do not surface false breach warnings when CO2 control is intentionally inactive.

---

# Advisory Engine

`advisory/engine.py:AdvisoryEngine`

Scans MPC trajectories and current state each cycle; produces `Advisory` objects for grower-facing early warnings. Handles deduplication, cooldowns, severity upgrades, and auto-expiry internally.

## The 7 Advisory Types

`advisory/models.py`

| Type | When it fires | Severity tiers |
|------|--------------|----------------|
| `env_breach` | MPC trajectory predicts setpoint breach within horizon | info / warning / critical (by ETA) |
| `env_trend` | Sustained drift toward limits across recent cycles | info / warning |
| `light_transition` | Photoperiod boundary approaching | info |
| `model_drift` | EKF innovation > 2.0 sustained for 60s | warning |
| `solver_degraded` | ≥3 consecutive solver failures | warning / critical |
| `compliance_declining` | Rolling compliance below 90% floor | warning |
| `co2_flow` | CO2 injection requested but flow not observed | warning |
| `leaf_temp_deficit` | T_air − T_leaf outside [0, 3]°C band | info (>3) / warning (>5 or reversed) / critical (>7) |

## Leaf Temperature Deficit (special case)

- Uses **current state** (x_hat) for severity, **trajectory mean** for persistence signal
- Reversed deficit (T_leaf > T_air) signals stomatal closure / radiant heat — not surfaced anywhere else
- Auto-resolves when deficit returns to 0–3°C
- Independent of VPD/RH channels — those don't catch this failure mode

## Lifecycle

- **Cooldown:** default 300s per advisory key; prevents flapping
- **Severity upgrade:** allowed within active advisory (info → warning → critical)
- **Auto-resolve:** when condition clears (e.g., trajectory no longer breaches)
- **Deduplication:** keyed by `(type, channel)` — only one active per key

## Suppression Rules

- **CO2 breach/trend** suppressed during lights-off:
  - Injection is disabled at night → CO2 drifts to ambient ~420 ppm by design
  - Active CO2 advisories from lights-on are auto-resolved at lights-off transition

## Integration Points

- **Constructed by:** `cli/bootstrap.py`, `api/simulation_manager.py`
- **Driven by:** `control/solver_process.py:SolverProcess` — calls `evaluate()` after each MPC solve
- **Reads:** MPC trajectory, x_hat, solve_info, reference, phase config (for tolerances)
- **Output consumed by:** `api/broadcast.py:WebSocketBroadcaster` — pushes advisories to dashboard

## Tuning

Compliance tolerances come from `phase_config['compliance']`:
- `VPD_tolerance` (default 0.15)
- `T_tolerance` (default 1.0°C)
- `CO2_tolerance` (default 100 ppm)
- `theta_tolerance` (default 0.05)

Confidence decays with prediction step (closer-in-horizon predictions are higher confidence).