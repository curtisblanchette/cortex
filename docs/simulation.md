<!-- SUMMARY -->
> **Scope:** Simulation environment, disturbance scheduling, weather presets, growth phases, CLI usage.
> **Key constraint:** Simulation starts at midnight — initial theta uses night target, not day target.

---

# Simulation

## Running Simulations

```bash
python main.py simulate \
    --fast \
    --duration 43200 \
    --chart --chart-output output/$(date +%s).png \
    --profile hvacd_full
```

Key flags:
- `--fast` — Fast-forward mode (~87x real-time on Mac)
- `--phase <name>` — Growth phase (clone, veg, early_flower, mid_flower, late_flower, dry)
- `--profile <name>` — Equipment profile
- `--substrate <name>` — Substrate type
- `--weather <preset>` — Outdoor weather preset

## Weather Presets

Outdoor conditions configurable via presets (T_out mean/amplitude, w_out):
- Defined in `cli/commands/simulate.py` as `WEATHER_PRESETS`
- Accessible via CLI `--weather` flag and API `SimulateRequest.weather` field

## Growth Phases

Config files in `config/phases/*.yaml`:
- clone, veg, early_flower, mid_flower, late_flower, dry
- Each defines day/night setpoints for T, RH, VPD, CO2, theta
- RH reference derived from VPD target and leaf-air offset (not hardcoded)

## Disturbance Scheduling

`simulation/disturbance.py` manages time-varying disturbances:
- Light schedule (on/off with configurable times)
- Outdoor weather (sinusoidal T_out with configurable amplitude)
- Irrigation rate (set by IrrigationScheduler state machine)

## Simulation Manager (API-driven)

`api/simulation_manager.py` runs simulations triggered from the web dashboard:
- Builds all components from config
- Broadcasts progress/telemetry via WebSocket
- Supports same parameters as CLI (phase, profile, substrate, weather, duration)

## Performance

- ~87 ticks/s on Mac (87x real-time)
- MPC solve time: ~9ms steady-state, ~28-113ms on cold start
- All profiles achieving >97% compliance on 12hr sims