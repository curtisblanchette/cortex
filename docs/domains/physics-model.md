<!-- SUMMARY -->
> **Scope:** 7-state ODE, state/control/disturbance vectors, calibrated parameters, transpiration, substrate dynamics, the three dynamics interfaces.
> **Key constraint:** MPC linearization (B matrix) is accurate within 0.01% of nonlinear ground truth — do not introduce approximations. `GrowRoomDynamics` is the single source of truth for room physics; EKF, OSQP, SLSQP, and SimulatedRoom all use the same code.

---

# Physics Model

`core/dynamics.py:GrowRoomDynamics`

Seven coupled ODEs describing the thermodynamics of a grow room. Every other component either feeds inputs into this model or consumes its outputs.

## State Vector `x` (7)

| Idx | Symbol | State | Unit |
|----|--------|-------|------|
| 0 | T_a | Air temperature | °C |
| 1 | w_a | Absolute humidity (mixing ratio) | g/kg |
| 2 | C | CO2 concentration | ppm |
| 3 | T_l | Leaf temperature | °C |
| 4 | θ | Substrate volumetric water content | m³/m³ |
| 5 | T_s | HVAC supply air temperature | °C |
| 6 | T_w | Wall / thermal mass temperature | °C |

`STATE_NAMES = ['T_a', 'w_a', 'CO2', 'T_l', 'theta', 'T_s', 'T_w']` (see `mpc/constraints.py`).

## Control Vector `u` (registry-driven)

`n_u`, channel names, bounds, and weights are all **dynamic** — set by the active equipment profile via `ActuatorRegistry`. There is no fixed control vector layout.

Common channels (which actually exist depends on the profile):

| Channel | Plugin | Type | Range |
|---------|--------|------|-------|
| `cool_sp` | HVACDPlugin | setpoint °C | room-specific |
| `heat_sp` | HVACDPlugin | setpoint °C | room-specific |
| `dehum_sp` | HVACDPlugin | setpoint %RH | 30–80 |
| `fan_pct` | HVACDPlugin | fraction | 0–1 |
| `u_cool` | StandaloneACPlugin | % | 0–100 |
| `u_heat` | HeaterPlugin | % | 0–100 |
| `u_hum` | HumidifierPlugin | % | 0–100 |
| `u_dehum` | StandaloneDehumPlugin | % | 0–100 |
| `u_co2` | CO2InjectorPlugin | % | 0–100 |
| `u_fan` | FanPlugin | % | 0–100 |
| `u_irr` | IrrigationPlugin | % | 0–100 |

See [`actuator-plugins.md`](actuator-plugins.md) for plugin → channel mapping and active profiles.

## Disturbance Vector `d` (6 — known/forecast)

| Idx | Symbol | Disturbance | Unit |
|----|--------|-------------|------|
| 0 | Q_light | Lighting heat load | W |
| 1 | T_out | Outdoor temperature | °C |
| 2 | w_out | Outdoor humidity | g/kg |
| 3 | N_plants | Plant count / canopy density | — |
| 4 | phase | Growth phase | enum |
| 5 | irr_rate | Irrigation rate | m³/m³/s |

**`d[5]` is irrigation rate**, set by the `IrrigationScheduler` state machine (not predicted in MPC forecast — set to 0 there). Surface evaporation (5% of irrigation) also enters humidity ODE via `d[5]`. The `Bd` Jacobian column for `d[5]` captures theta/humidity sensitivity correctly.

## Calibrated Parameters `p` (7)

`calibration/params.py:CalibrationParams` — tuned by SLSQP, not measured. See [`calibration.md`](calibration.md):

`UA_wall`, `UA_env`, `tau_hvac`, `g_s_max`, `K_drain`, `C_wall`, `eta_hvac`.

`LAI` is **phase-derived** (lives on `GrowRoomDynamics`), not in `CalibrationParams`.

## Three Interfaces

| Method | Use case | Cost |
|--------|---------|------|
| `nonlinear_step(x, u, d, p, dt) → x_next` | Simulation, EKF predict, SLSQP replay | ~0.1 ms (RK4) |
| `linearize(x0, u0, d0, p) → A_d, B_d, B_dd` | OSQP build, EKF covariance propagation | ~0.5 ms |
| `derived_outputs(x) → {VPD, RH, T_leaf, theta}` | Reference generation, advisory checks | <0.05 ms |

The MPC sees only the linearized matrices; the EKF predict step uses the full nonlinear ODE. They share the same `GrowRoomDynamics` instance.

## Sub-Models (delegated)

| Sub-model | File | What it computes |
|-----------|------|------------------|
| HVACD | `core/hvacd.py:HVACDModel` | Q_sensible, Q_latent, moisture removal, T_supply (multi-mode: cool/heat/dehum) |
| Transpiration | `core/transpiration.py` | Penman-Monteith with stomatal + cuticular conductance → g/m²/hr |
| Substrate | `core/substrate.py` | Drainage, water uptake, dryback dynamics |
| Thermodynamics | `core/thermodynamics.py` | SVP, w_sat, dew point — shared utilities |

Actuator effects on the ODEs are delegated to plugins via `ActuatorRegistry`. See [`actuator-plugins.md`](actuator-plugins.md).

## Derived Outputs

Computed from `x` (not state variables themselves):
- `VPD = SVP(T_leaf) − e_air` (kPa)
- `RH = e_air / SVP(T_air) × 100` (%)
- `dryback_rate = dθ/dt` (m³/m³/hr)
- `transpiration = f(VPD, PAR, θ, LAI)` (g/m²/hr)

## Irrigation as Feedforward Disturbance

Irrigation is **not** an MPC control variable:
- `IrrigationScheduler` (`control/irrigation_scheduler.py`) fires discrete shots based on theta trigger
- Sets `disturbance_schedule.irr_rate` → enters physics via `d[5]`
- MPC forecast: `d[5] = 0` (scheduler-driven, not predicted)
- EKF observes theta in real-time and corrects estimate

See `decisions-recent.md` "Irrigation as feedforward disturbance" for rationale.

## Room & Equipment Sizing

- Room: 280 m³ (10×7×4 m), 70 m² floor, 56 m² canopy
- Lights: 18.7 kW (900 PPFD × 56 m² ÷ 2.7 PPE) — sensible heat ≈ 11.2 kW
- AC: 35 kW cooling
- Dehumidifier: 15 L/hr (Quest 506, sized for mid-flower transpiration ~10.5 L/hr)
- Heater: 5 kW resistive
- Humidifier: 2000 g/hr (2 L/hr)
- HVACD blower is **internal** — real Desert Aire CM3550 manages blower speed via onboard controller, not BMS/MPC

## Key Invariants

- `GrowRoomDynamics` is the **single source of truth** for room physics — never a second copy
- The EKF predict step uses the **same** nonlinear code as the simulation — no divergent implementations
- Linearization happens **every** MPC cycle; OSQP never sees the nonlinear model
- Dehumidifier capacity asymmetry: 5× more powerful than humidifier per % output — correctly captured in B matrix