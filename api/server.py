"""FastAPI application for dashboard, config, and control API."""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import json
import numpy as np


def create_api(control_loop=None, compliance=None, shared_params=None, calibrator=None):
    """Create FastAPI application.

    Args:
        control_loop: ControlLoop instance.
        compliance: ComplianceTracker instance.
        shared_params: SharedParams instance.
        calibrator: ModelCalibrator instance.

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

    # WebSocket connections
    ws_clients = set()

    @app.get("/api/status")
    async def get_status():
        """Current system status."""
        loop = app.state.loop
        if loop is None:
            return {"status": "not_initialized"}

        return {
            "status": "running" if loop.running else "stopped",
            "tick": loop.tick_count,
            "failsafe_level": loop.failsafe.current_level if loop.failsafe else "unknown",
            "params_version": app.state.params.version if app.state.params else 0,
        }

    @app.get("/api/state")
    async def get_state():
        """Current state estimate and derived outputs."""
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
        if app.state.compliance is None:
            return {"error": "not_initialized"}
        return app.state.compliance.report()

    @app.get("/api/params")
    async def get_params():
        """Current model parameters."""
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
