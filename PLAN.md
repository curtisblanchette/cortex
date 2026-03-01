# Advisory System — Phase 1 (Passive Monitoring)

## Concept

In Phase 1, Cortex does NOT fire actuators. It passively reads sensors, runs the EKF + MPC model forward, and generates early-warning advisories to growers via API + WebSocket. The MPC already produces 60-minute predicted trajectories — the advisory engine scans these for compliance breaches and translates them into actionable messages.

## Data Model

```python
@dataclass
class Advisory:
    id: str                    # uuid4
    type: str                  # advisory type key (see below)
    severity: str              # "info" | "warning" | "critical"
    channel: str | None        # which variable: "VPD", "T", "RH", "CO2", "theta", or None
    title: str                 # short headline
    body: str                  # detail with current value, predicted value, trend
    recommendation: str        # what the grower should do
    eta_s: float | None        # seconds until predicted violation (None if current)
    current_value: float | None
    predicted_value: float | None
    target_value: float | None
    tolerance: float | None
    confidence: float          # 0-1, degrades with horizon distance
    created_at: float          # time.time()
    expires_at: float          # auto-clear stale advisories
    room_id: str
    phase: str
```

## Severity Tiers (based on time-to-violation)

| Severity | ETA | Confidence | Use |
|----------|-----|------------|-----|
| `info` | 30-60 min | Low (0.5-0.7) | Early heads-up, may self-resolve |
| `warning` | 10-30 min | Medium (0.7-0.9) | Actionable — grower should prepare |
| `critical` | <10 min or currently violating | High (0.9-1.0) | Immediate action needed |

Confidence formula: `1.0 - 0.5 * (step_index / Np)` — full confidence at step 0, 0.5 at horizon end.

## Advisory Types

### 1. `env_breach` — Environment Compliance Breach Prediction

**Source:** Scan the 60-step predicted trajectory against compliance tolerances from phase config.

**Detection:** For each output channel (VPD, T, RH, CO2, theta), find the earliest trajectory step where `|predicted - target| > tolerance`. Convert step index to ETA seconds.

**Channels & templates:**

#### VPD
```
title:    "VPD {rising above | dropping below} target"
body:     "VPD predicted to reach {predicted} kPa in {eta} minutes.
           Current: {current} kPa → Target: {target} ± {tol} kPa
           Trend: {trend:+.3f} kPa/min"
recommend (high):  "Increase dehumidification or lower room temperature 1-2°C to reduce VPD."
recommend (low):   "Increase humidification or raise room temperature to increase VPD."
```

#### Temperature
```
title:    "Room temperature {rising | dropping}"
body:     "Temperature predicted to reach {predicted}°C in {eta} minutes.
           Current: {current}°C → Target: {target} ± {tol}°C
           Trend: {trend:+.1f}°C/min"
recommend (high):  "Increase cooling capacity. Verify AC is operating normally."
recommend (low):   "Increase heating. Check for unexpected ventilation or open doors."
```

#### RH
```
title:    "Humidity {rising above | dropping below} target"
body:     "RH predicted to reach {predicted}% in {eta} minutes.
           Current: {current}% → Target: {target} ± {tol}%
           Trend: {trend:+.1f}%/min"
recommend (high):  "Increase dehumidification. High RH increases mold and pathogen risk."
recommend (low):   "Reduce dehumidification or add humidification."
```

#### CO2
```
title:    "CO2 {rising above | dropping below} target"
body:     "CO2 predicted to reach {predicted} ppm in {eta} minutes.
           Current: {current} ppm → Target: {target} ± {tol} ppm
           Trend: {trend:+.0f} ppm/min"
recommend (high):  "Reduce CO2 injection rate."
recommend (low):   "Increase CO2 injection. Verify tank level and regulator."
```

#### Substrate Moisture (theta)
```
title:    "Substrate moisture {drying | saturating}"
body:     "Substrate moisture predicted to reach {predicted} m³/m³ in {eta} minutes.
           Current: {current} → Target: {target} ± {tol}
           Trend: {trend:+.4f}/min"
recommend (dry):   "Schedule irrigation. Substrate approaching dry-back threshold."
recommend (wet):   "Reduce irrigation frequency. Substrate is over-saturated."
```

### 2. `light_transition` — Upcoming Light State Change

**Source:** Light schedule — exact times known. Computed from `LightSchedule.on_hour` / `off_hour` relative to current sim/wall time.

**Detection:** Compute time until next transition. Fire advisory at 60, 30, and 10 minutes before.

```
title:    "Lights {on | off} in {eta} minutes"
body:     "Light transition at {transition_time}.
           Day targets: T={day_T}°C, VPD={day_VPD}, RH={day_RH}%, CO2={day_CO2} ppm
           Night targets: T={night_T}°C, VPD={night_VPD}, RH={night_RH}%, CO2={night_CO2} ppm
           Ramp duration: {ramp_min} minutes"
recommend (off):  "Prepare for temperature drop and VPD shift. Verify heating and dehumidification are ready."
recommend (on):   "Prepare for heat load increase. Verify cooling capacity is available."
```

### 3. `model_drift` — Sensor/Model Disagreement

**Source:** EKF innovation magnitude (RMS of normalized innovations). Available every tick.

**Detection:** Innovation RMS sustained above threshold (e.g., > 2.0) for > 60 seconds.

```
title:    "Sensor readings diverging from model"
body:     "EKF innovation RMS at {innovation_rms:.2f} (normal < 1.0) for {duration} seconds.
           The model's predictions are not matching sensor observations."
recommend: "Check sensors for drift or obstruction. If readings look correct, the room
            conditions may have changed (door open, equipment failure, unexpected load)."
```

### 4. `solver_degraded` — MPC Solver Health

**Source:** OSQP solve status from `solve_info['status']`.

**Detection:** Status is `solved_inaccurate` or `failed` for > 3 consecutive solves.

```
title:    "Control model accuracy degraded"
body:     "MPC solver returning {status} for {count} consecutive solves.
           Predictions may be unreliable. Confidence: {confidence:.0%}"
recommend: "Advisory predictions may be less accurate. Monitor conditions more closely.
            If persistent, the system may need recalibration."
```

### 5. `compliance_declining` — Rolling Compliance Dropping

**Source:** ComplianceTracker rolling 1-hour report.

**Detection:** Any channel drops below 90% compliance in the rolling window.

```
title:    "{channel} compliance declining"
body:     "{channel} compliance at {pct}% over the last hour (target: >95%).
           Mean: {mean}, Std: {std}, Range: [{min}, {max}]
           Tolerance: ±{tolerance}"
recommend: "Review {channel} setpoints and equipment capacity. Sustained low compliance
            indicates the system cannot maintain target conditions."
```

## Deduplication & Lifecycle

- **Cooldown:** After an advisory is created for a given (type, channel) pair, suppress duplicates for `cooldown_s` seconds (default: 300s / 5 min).
- **Severity upgrade:** If a new detection has higher severity than an active advisory for the same (type, channel), replace it immediately (no cooldown).
- **Auto-expire:** Advisories expire at `created_at + ttl_s`. Default TTLs:
  - `env_breach`: 600s (10 min) — trajectory changes each solve
  - `light_transition`: expires at transition time
  - `model_drift`: 300s (5 min)
  - `solver_degraded`: 300s
  - `compliance_declining`: 900s (15 min)
- **Clear on resolution:** If the trajectory no longer shows a breach for an active `env_breach` advisory, mark it `resolved` immediately.

## Implementation

### New files
- `advisory/engine.py` — `AdvisoryEngine` class: scans trajectories + state, produces `Advisory` objects
- `advisory/templates.py` — Template strings and rendering logic
- `advisory/models.py` — `Advisory` dataclass and severity constants

### Modified files
- `control/solver_process.py` — After each MPC solve, pass trajectory + state + info to `AdvisoryEngine`
- `api/server.py` — Add `GET /api/advisories` endpoint + push advisories over existing WebSocket

### Integration point in solver_process.py

After the MPC solve in `solve_once()`, add:

```python
if self.advisory_engine:
    self.advisory_engine.evaluate(
        trajectory=trajectory,
        x_hat=x_hat,
        ref_targets=self._cached_ref_targets,
        solve_info=solve_info,
        current_time=current_time,
        light_schedule=self.light_schedule,
        compliance_report=self.compliance.report() if self.compliance else None,
        innovation_rms=self.ekf.innovation_magnitude(),
    )
```

### API shape

```
GET /api/advisories
→ { "advisories": [Advisory, ...], "count": N }

GET /api/advisories/active
→ { "advisories": [Advisory with status != "resolved"], "count": N }

WS /ws/dashboard
→ pushes {"type": "advisory", "data": Advisory} on new/updated advisories
```
