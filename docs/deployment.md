<!-- SUMMARY -->
> **Scope:** Production deployment, Pi 5 setup, Redis IPC, MQTT/Modbus, dependencies.
> **Key constraint:** Production runs three separate processes (Solver, Gateway, Calibrator) communicating via Redis.

---

# Deployment

## Production Architecture

Runs on Raspberry Pi 5 in the grow room:

```
Solver Process ──Redis──▶ Gateway Process ──MQTT/Modbus──▶ Hardware
    │                         │
    └───Redis───▶ Calibrator Process
```

- **Solver:** EKF + MPC at 1 Hz, publishes solutions to Redis
- **Gateway:** Reads Redis solutions, dispatches to actuators with dead-band, hold time, rate limiting
- **Calibrator:** Background SLSQP optimizer, compares predictions to real sensor data, tunes SharedParams

## Starting Production

```bash
python main.py all    # Starts solver + gateway + API + calibrator
```

Individual processes:
```bash
python main.py solver
python main.py gateway
python main.py api
python main.py calibrator
```

## Dependencies

**Core:** numpy, scipy, osqp, numba, pyyaml
**Production:** fastapi, uvicorn, redis, paho-mqtt
**Management:** pip + `requirements.txt`

## Configuration

All YAML configs in `config/`:
- `room.yaml` — Room dimensions, thermal properties, outdoor defaults
- `equipment.yaml` — Equipment specs and named profiles
- `substrate.yaml` — Growing medium properties
- `phases/*.yaml` — Growth phase setpoints (6 phases)

YAML configs have top-level keys (e.g., `room:`, `hvacd:`) — code must handle both nested and flat dicts.

## Web Dashboard

- FastAPI server with WebSocket for real-time telemetry
- Web app lives at `~/code/esp/my-esp32-project/apps/web` (separate repo)
- REST endpoints for simulation control, equipment manifest, config options