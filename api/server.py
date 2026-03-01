"""FastAPI application for dashboard, config, and control API.

Supports two modes:
  - In-process: receives live references (control_loop, compliance, etc.)
  - Redis-backed: reads all data from Redis (for standalone `cortex api`)
"""

import json

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import numpy as np


def create_api(control_loop=None, compliance=None, shared_params=None,
               calibrator=None, advisory_engine=None,
               redis_client=None, room_id='room1'):
    """Create FastAPI application.

    In-process mode (simulation / legacy):
        Pass control_loop, compliance, shared_params, calibrator.

    Redis-backed mode (production standalone):
        Pass redis_client and room_id. Endpoints read from Redis keys.

    Args:
        control_loop: SolverProcess instance (in-process).
        compliance: ComplianceTracker instance (in-process).
        shared_params: SharedParams instance (in-process).
        calibrator: ModelCalibrator instance (in-process).
        redis_client: Redis client (production standalone).
        room_id: Room identifier for Redis key prefixes.

    Returns:
        FastAPI app.
    """
    app = FastAPI(title="Cortex MPC", version="0.1.0")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Store references
    app.state.loop = control_loop
    app.state.compliance = compliance
    app.state.params = shared_params
    app.state.calibrator = calibrator
    app.state.advisory_engine = advisory_engine
    app.state.redis = redis_client
    app.state.room_id = room_id

    # WebSocket connections
    ws_clients = set()

    @app.get("/api/status")
    async def get_status():
        """Current system status."""
        # Redis-backed mode
        if app.state.redis and app.state.loop is None:
            return _redis_status(app.state.redis, app.state.room_id)

        # In-process mode
        loop = app.state.loop
        if loop is None:
            return {"status": "not_initialized"}

        return {
            "status": "running" if loop.running else "stopped",
            "tick": getattr(loop, 'tick_count', getattr(loop, 'solve_count', 0)),
            "failsafe_level": (loop.failsafe.current_level
                               if loop.failsafe else "unknown"),
            "params_version": (app.state.params.version
                               if app.state.params else 0),
        }

    @app.get("/api/state")
    async def get_state():
        """Current state estimate and derived outputs."""
        # Redis-backed mode
        if app.state.redis and app.state.loop is None:
            return _redis_state(app.state.redis, app.state.room_id)

        # In-process mode
        loop = app.state.loop
        if loop is None or loop.ekf is None:
            return {"error": "not_initialized"}

        x_hat = loop.ekf.x_hat.tolist()
        derived = loop.dynamics.derived_outputs(loop.ekf.x_hat)

        return {
            "state": {
                "T_air": x_hat[0],
                "w_a": x_hat[1],
                "CO2": x_hat[2],
                "T_leaf": x_hat[3],
                "theta": x_hat[4],
                "T_supply": x_hat[5],
                "T_wall": x_hat[6],
            },
            "derived": {k: float(v) for k, v in derived.items()},
            "control": loop.last_u.tolist(),
        }

    @app.get("/api/compliance")
    async def get_compliance():
        """Compliance report."""
        # Redis-backed mode
        if app.state.redis and app.state.compliance is None:
            return _redis_compliance(app.state.redis, app.state.room_id)

        if app.state.compliance is None:
            return {"error": "not_initialized"}
        return app.state.compliance.report()

    @app.get("/api/params")
    async def get_params():
        """Current model parameters."""
        # Redis-backed mode
        if app.state.redis and app.state.params is None:
            return _redis_params(app.state.redis, app.state.room_id)

        if app.state.params is None:
            return {"error": "not_initialized"}
        p = app.state.params.current()
        return {
            "version": app.state.params.version,
            "params": p.to_dict(),
        }

    @app.get("/api/calibration/history")
    async def get_calibration_history():
        """Calibration history."""
        if app.state.calibrator is None:
            return {"error": "not_initialized"}
        return {"history": app.state.calibrator.history[-20:]}

    @app.post("/api/calibration/trigger")
    async def trigger_calibration():
        """Trigger immediate calibration."""
        if app.state.calibrator is None:
            return {"error": "not_initialized"}
        app.state.calibrator.trigger_fast_calibration()
        return {"status": "triggered"}

    @app.get("/api/solution")
    async def get_solution():
        """Latest MPC solution (Redis-backed only)."""
        if app.state.redis is None:
            return {"error": "redis_not_configured"}
        key = f'mpc:{app.state.room_id}:solution'
        data = app.state.redis.hgetall(key)
        if not data:
            return {"error": "no_solution"}
        return data

    @app.get("/api/gateway/status")
    async def get_gateway_status():
        """Gateway status (Redis-backed only)."""
        if app.state.redis is None:
            return {"error": "redis_not_configured"}
        key = f'gateway:{app.state.room_id}:status'
        data = app.state.redis.hgetall(key)
        if not data:
            return {"error": "no_gateway_status"}
        return data

    @app.get("/api/advisories")
    async def get_advisories():
        """All advisories (active + resolved)."""
        engine = app.state.advisory_engine
        if engine is None:
            return {"advisories": [], "count": 0}
        advisories = [a.to_dict() for a in engine.all_advisories()]
        return {"advisories": advisories, "count": len(advisories)}

    @app.get("/api/advisories/active")
    async def get_active_advisories():
        """Active (non-expired, non-resolved) advisories only."""
        engine = app.state.advisory_engine
        if engine is None:
            return {"advisories": [], "count": 0}
        advisories = [a.to_dict() for a in engine.active_advisories()]
        return {"advisories": advisories, "count": len(advisories)}

    @app.websocket("/ws/dashboard")
    async def dashboard_ws(websocket: WebSocket):
        """WebSocket for live dashboard updates."""
        await websocket.accept()
        ws_clients.add(websocket)
        try:
            while True:
                await websocket.receive_text()
        except WebSocketDisconnect:
            ws_clients.discard(websocket)

    return app


# -- Redis-backed helpers --------------------------------------------------

def _redis_status(r, room_id):
    """Build /api/status response from Redis."""
    sol_key = f'mpc:{room_id}:solution'
    gw_key = f'gateway:{room_id}:status'
    solution = r.hgetall(sol_key)
    gw_status = r.hgetall(gw_key)
    return {
        "status": "running" if solution else "no_solution",
        "solve_id": solution.get('solve_id', '0'),
        "solve_ms": solution.get('solve_ms', '0'),
        "confidence": solution.get('confidence', '0'),
        "gateway_stale": gw_status.get('stale', 'unknown'),
    }


def _redis_state(r, room_id):
    """Build /api/state response from Redis solution hash."""
    key = f'mpc:{room_id}:solution'
    solution = r.hgetall(key)
    if not solution:
        return {"error": "no_solution"}

    x_hat = json.loads(solution.get('x_hat', '[]'))
    labels = ['T_air', 'w_a', 'CO2', 'T_leaf', 'theta', 'T_supply', 'T_wall']
    state = {}
    for i, label in enumerate(labels):
        state[label] = x_hat[i] if i < len(x_hat) else None

    return {
        "state": state,
        "control": {
            "T_sp": solution.get('u_T_sp'),
            "RH_sp": solution.get('u_RH_sp'),
            "u_co2": solution.get('u_co2'),
            "u_hum": solution.get('u_hum'),
            "u_fan": solution.get('u_fan'),
            "u_irr": solution.get('u_irr'),
        },
    }


def _redis_compliance(r, room_id):
    """Read compliance report from Redis."""
    key = f'compliance:{room_id}:report'
    data = r.hgetall(key)
    if not data:
        return {"error": "no_compliance_data"}
    # Stored as JSON string
    if 'report' in data:
        return json.loads(data['report'])
    return data


def _redis_params(r, room_id):
    """Read calibration params from Redis."""
    key = f'calibration:{room_id}:params'
    data = r.hgetall(key)
    if not data:
        return {"error": "no_params"}
    return data
