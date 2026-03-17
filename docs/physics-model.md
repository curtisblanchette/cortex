<!-- SUMMARY -->
> **Scope:** 7-state ODE, state vector, disturbance vector, transpiration, substrate dynamics.
> **Key constraint:** MPC linearization (B matrix) is accurate within 0.01% of nonlinear ground truth — do not introduce approximations.

---

# Physics Model

## State Vector (7 states)

| Index | State | Unit | Description |
|-------|-------|------|-------------|
| 0 | T_air | °C | Air temperature |
| 1 | w_a | g/kg | Absolute humidity (mixing ratio) |
| 2 | CO2 | ppm | CO2 concentration |
| 3 | T_leaf | °C | Leaf temperature |
| 4 | theta | m³/m³ | Volumetric water content |
| 5 | T_supply | °C | HVAC supply air temperature |
| 6 | T_wall | °C | Wall/structure temperature |

## Disturbance Vector (6 elements)

| Index | Disturbance | Description |
|-------|------------|-------------|
| 0 | Q_light | Light heat input (W) |
| 1 | T_out | Outdoor temperature (°C) |
| 2 | w_out | Outdoor humidity (g/kg) |
| 3 | N_plants | Plant count |
| 4 | phase | Growth phase index |
| 5 | irr_rate | Irrigation rate (m³/m³/s) |

## Key Physics

- **Transpiration:** Penman-Monteith with cuticular conductance for overnight dryback
- **Fresh air exchange:** Models carbon filter exhaust and infiltration
- **Substrate:** Theta dynamics driven by plant uptake + irrigation disturbance; surface evaporation (5% of irrigation) enters humidity ODE
- **HVACD:** Supply air temperature tracks setpoint with thermal lag; blower is internal (fan_pct=1.0 hardcoded)

## Irrigation as Feedforward Disturbance

Irrigation is NOT an MPC control variable — it's a separate agronomic domain:
- `IrrigationScheduler` fires discrete shots based on theta trigger (state machine)
- Sets `disturbance_schedule.irr_rate` → enters physics via d[5]
- MPC forecast: d[5]=0 (not predicted; EKF observes theta in real-time)

## Room & Equipment Sizing

- Room: 280 m³ (10×7×4m), 70 m² floor, 56 m² canopy
- Lights: 18.7 kW (900 PPFD × 56 m² ÷ 2.7 PPE) — sensible heat = 11.2 kW
- AC: 35 kW cooling
- Dehumidifier: 15 L/hr (Quest 506, sized for mid-flower ~10.5 L/hr)
- Heater: 5 kW resistive
- Humidifier: 2000 g/hr (2 L/hr)