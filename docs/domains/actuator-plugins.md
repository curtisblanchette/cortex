<!-- SUMMARY -->
> **Scope:** Actuator plugin system, registry, equipment profiles, channel mappings.
> **Key constraint:** Registry is required by dynamics (raises ValueError if None) — no auto-build fallback.

---

# Actuator Plugin System

## Registry Architecture

`ActuatorRegistry` is the **single source of truth** for:
- Actuator count (n_u)
- Channel bounds, weights, defaults
- Field mappings (channel name → solution field)
- Dispatch specs (for gateway → hardware)
- Plugin → channel associations

## Plugins (8 built-in)

| Plugin | File | Channels |
|--------|------|----------|
| HVACDPlugin | `hvacd_plugin.py` | cool_sp, heat_sp, dehum_sp, fan_pct |
| StandaloneACPlugin | `ac_plugin.py` | u_cool |
| StandaloneDehumPlugin | `dehumidifier_plugin.py` | u_dehum |
| HumidifierPlugin | `humidifier_plugin.py` | u_hum |
| HeaterPlugin | `heater_plugin.py` | u_heat |
| CO2InjectorPlugin | `co2_plugin.py` | u_co2 |
| FanPlugin | `fan_plugin.py` | u_fan |
| IrrigationPlugin | `irrigation_plugin.py` | u_irr |

## Equipment Profiles

Defined in `config/equipment.yaml` under `profile:` key:

| Profile | Description |
|---------|-------------|
| `hvacd_full` | HVACD unit + humidifier + CO2 + fan + irrigation (default) |
| `ac_dehum` | Standalone AC + standalone dehumidifier |
| `ac_minimal` | Standalone AC only |
| `hvacd_no_irr` | HVACD without irrigation |

CLI `--profile` flag overrides config profile.

## Fan Plugin Ordering

Fan plugin (canopy fans) computes `fan_pct` first, passed to other plugins via `room_props`. HVACD blower is internal — real units (Desert Aire CM3550) manage blower speed via onboard controller, not BMS/MPC.

## Plugin ABC

Each plugin implements:
- `compute_physics(u_slice, room_props, dt)` → dict of ODE contributions
- `channels()` → list of channel definitions (name, bounds, weight, default)
- `dispatch_spec()` → how gateway sends commands to hardware