<!-- SUMMARY -->
> **Scope:** Production deployment, Pi 5 process layout, Redis IPC, MQTT/Modbus adapters, data stores, dependencies, configuration.
> **Key constraint:** Production runs three separate processes (Solver, Gateway, Calibrator) plus the API, communicating via Redis. The Gateway is the **only** process that writes to hardware.

---

# Deployment

## Production Layout (Raspberry Pi 5)

```
┌─────────────────────────────────────────────────────────┐
│ Pi 5                                                    │
│                                                         │
│  ┌──────────────┐   Redis   ┌──────────────────┐       │
│  │ SolverProcess├──────────▶│ ActuatorGateway  │       │
│  │ (EKF + MPC)  │           │ (dispatch)       │       │
│  └──┬───────────┘           └────────┬─────────┘       │
│     │ telemetry                       │ MQTT/Modbus     │
│     ▼                                 ▼                 │
│  ┌──────────────┐                   (out to hardware)   │
│  │ Calibrator   │                                       │
│  │ (background) │                                       │
│  └──────────────┘                                       │
│                                                         │
│  ┌──────────────┐         ┌──────────────┐             │
│  │ API (FastAPI)│         │ Redis (hot)  │             │
│  │ + WebSocket  │         │ SQLite (cold)│             │
│  └──────────────┘         └──────────────┘             │
└─────────────────────────────────────────────────────────┘
```

## Starting Production

```bash
python main.py all          # solver + gateway + API + calibrator
```

Individual processes:

```bash
python main.py solver
python main.py gateway
python main.py api
python main.py calibrator
```

CLI flags propagate (e.g., `--profile`, `--phase`).

## Data Stores

| Store | Role | Retention |
|-------|------|-----------|
| **Redis** | Hot store: latest sensor data, MPC solution, telemetry ring buffer | ~last 4 hours |
| **SQLite** | Cold store: full telemetry history, calibration history | Full history, on-disk |

The control loop never blocks on SQLite — telemetry is queued asynchronously.

## Hardware Interfaces

- **MQTT** — ESP32 sensors (SHT31 T/RH, SCD30 CO2), ESP32 relays (humidifier, heater, CO2 valve), MLX90614 leaf-IR sensor
- **Modbus** — Desert Aire CM3550 HVACD (setpoint mode for cool/heat/dehum)
- **0–10 V** — analog actuators via DAC where applicable

All hardware I/O is owned by the Gateway. The Solver is hardware-agnostic — it reads sensors via Redis (production) or SimulatedIO (sim).

## Configuration

YAML configs in `config/`:

| File | What it defines |
|------|-----------------|
| `room.yaml` | Volume, floor/canopy area, thermal properties, outdoor defaults |
| `equipment.yaml` | Equipment specs and named profiles (`hvacd_full`, `ac_dehum`, `ac_minimal`, `hvacd_no_irr`) |
| `substrate.yaml` | Growing medium properties |
| `phases/*.yaml` | Per-phase setpoints (clone, veg, early/mid/late_flower, dry) |
| `cultivar/*.yaml` | Per-strain VPD/temperature preferences |
| `profiles/*.yaml` | Equipment profile definitions referenced by `equipment.yaml` |

YAML configs use top-level keys (e.g., `room:`, `hvacd:`). Code must handle both nested and flat dicts.

## Dependencies

| Tier | Packages |
|------|----------|
| Core | `numpy`, `scipy`, `osqp`, `numba`, `pyyaml` |
| Production | `fastapi`, `uvicorn`, `redis`, `paho-mqtt` |

Managed with `pip` and `requirements.txt`. No poetry/uv.

## Web Dashboard

- FastAPI server at `api/server.py`
- WebSocket broadcast at `api/broadcast.py:WebSocketBroadcaster` — pushes state, MPC trajectory, advisories, equipment manifest
- REST endpoints: simulation control (`api/simulation_manager.py`), config options
- Frontend repo: `~/code/esp/my-esp32-project/apps/web` (separate)

## Performance (current measurements)

| Metric | Value |
|--------|-------|
| MPC solve (steady state) | ~9 ms |
| MPC solve (cold start) | ~28–113 ms |
| Sim throughput on Mac | ~87 ticks/s (~87× real-time) |
| Compliance target | >97% (achieved on all profiles in 12 h sims) |

The 100 ms solve budget is for the entire MPC step (linearize + build QP + solve + extract), not just the OSQP solve call.