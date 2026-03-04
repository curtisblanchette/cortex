"""SimulationManager — owns a background thread running one simulation at a time.

Mirrors the _run_decoupled loop from cli/commands/simulate.py but driven by
REST API params instead of argparse.  The broadcaster uses the same WebSocket
infrastructure that already powers the live dashboard.
"""

import asyncio
import logging
import threading
import time
import traceback

from cli.bootstrap import (
    load_configs, build_substrate, build_registry, build_shared_params,
    build_dynamics, build_ekf, build_mpc, build_reference,
    build_light_schedule, build_forecast, build_compliance, build_failsafe,
    build_simulation_io, build_irrigation_scheduler, build_co2_scheduler,
    build_advisory_engine,
)

logger = logging.getLogger(__name__)


class SimulationManager:
    """Manages a single simulation lifecycle on a background thread."""

    # Speed presets: ticks per second (0 = paused)
    SPEED_MAX = float('inf')

    def __init__(self):
        self._thread: threading.Thread | None = None
        self._stop_flag = threading.Event()
        self._paused = threading.Event()
        self._status = 'idle'      # idle | running | completed | error
        self._error: str | None = None
        self._speed: float = self.SPEED_MAX  # ticks/s, inf = no sleep
        self._progress = {
            'elapsed_s': 0,
            'duration_s': 0,
            'pct': 0.0,
            'phase': None,
            'profile': None,
        }
        self._lock = threading.Lock()

    @property
    def status(self) -> str:
        return self._status

    @property
    def progress(self) -> dict:
        with self._lock:
            return dict(self._progress)

    @property
    def error(self) -> str | None:
        return self._error

    @property
    def speed(self) -> float:
        """Current speed in ticks/s. 0 = paused, inf = max."""
        if self._paused.is_set():
            return 0.0
        return self._speed

    def set_speed(self, ticks_per_s: float) -> None:
        """Set simulation speed. 0 = pause, inf = max."""
        if ticks_per_s <= 0:
            self._paused.set()
        else:
            self._paused.clear()
            self._speed = ticks_per_s

    def start(self, params: dict, app) -> None:
        """Stop any running sim, then spawn a new one."""
        self.stop()
        self._stop_flag.clear()
        self._paused.clear()
        self._speed = self.SPEED_MAX
        self._error = None
        self._status = 'running'
        with self._lock:
            self._progress = {
                'elapsed_s': 0,
                'duration_s': params.get('duration', 43200),
                'pct': 0.0,
                'phase': params.get('phase'),
                'profile': params.get('profile'),
            }
        self._thread = threading.Thread(
            target=self._run_simulation,
            args=(params, app),
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Signal the running sim to stop and wait for the thread."""
        if self._thread is not None and self._thread.is_alive():
            self._stop_flag.set()
            self._thread.join(timeout=5.0)
        self._thread = None
        if self._status == 'running':
            self._status = 'idle'

    # ------------------------------------------------------------------
    # Internal: runs on background thread
    # ------------------------------------------------------------------

    def _run_simulation(self, params: dict, app) -> None:
        """Full simulation loop — mirrors _run_decoupled."""
        from control.solver_process import SolverProcess, SolutionStore
        from control.gateway import ActuatorGateway
        from api.broadcast import WebSocketBroadcaster

        phase = params.get('phase')
        profile = params.get('profile')
        substrate_name = params.get('substrate')
        duration = params.get('duration', 43200)
        config_dir = params.get('config_dir', 'config')

        logger.info(
            "Simulation starting: duration=%ds phase=%s profile=%s substrate=%s",
            duration, phase, profile, substrate_name,
        )

        try:
            # ---- Build all components ----
            configs = load_configs(config_dir, phase_override=phase)
            substrate = build_substrate(configs, substrate_override=substrate_name)
            registry = build_registry(configs, profile_override=profile, substrate=substrate)
            shared_params = build_shared_params(substrate=substrate)
            dynamics = build_dynamics(configs, shared_params, registry, substrate=substrate)
            io, sim_room = build_simulation_io(configs, registry, substrate=substrate)
            ekf = build_ekf(dynamics)
            mpc = build_mpc(dynamics, shared_params, registry, configs)
            reference = build_reference(configs, registry=registry, substrate=substrate)
            light_schedule = build_light_schedule(configs)
            forecast = build_forecast(configs)
            compliance = build_compliance(configs)
            failsafe = build_failsafe()

            irr_scheduler = build_irrigation_scheduler(
                configs, registry, substrate, light_schedule)
            if irr_scheduler:
                mpc.constraints.lock_channel('u_irr', 0.0)

            co2_scheduler = build_co2_scheduler(
                configs, registry, light_schedule, reference)
            if co2_scheduler:
                mpc.constraints.lock_channel('u_co2', 0.0)

            advisory_engine = build_advisory_engine(configs, reference, dynamics)
            if co2_scheduler:
                advisory_engine.co2_scheduler = co2_scheduler

            solution_store = SolutionStore()

            solver = SolverProcess(
                io=io,
                dynamics=dynamics,
                ekf=ekf,
                mpc=mpc,
                reference=reference,
                forecast=forecast,
                failsafe=failsafe,
                shared_params=shared_params,
                light_schedule=light_schedule,
                registry=registry,
                solution_store=solution_store,
                compliance=compliance,
                advisory_engine=advisory_engine,
            )

            dispatch_config = configs['equipment'].get('dispatch', {})
            gateway = ActuatorGateway(
                solution_store=solution_store,
                dispatch_config=dispatch_config,
                registry=registry,
                light_schedule=light_schedule,
            )

            # ---- Hot-swap app.state so REST endpoints reflect new sim ----
            app.state.loop = solver
            app.state.compliance = compliance
            app.state.advisory_engine = advisory_engine

            # ---- Broadcaster ----
            event_loop = getattr(app.state, 'event_loop', None)

            broadcaster = WebSocketBroadcaster(
                app=app,
                dynamics=dynamics,
                reference=reference,
                compliance=compliance,
                advisory_engine=advisory_engine,
                registry=registry,
                mpc_step_s=mpc.step_s,
                mpc_interval=int(mpc.step_s),
                light_schedule=light_schedule,
            )
            if event_loop:
                broadcaster.set_event_loop(event_loop)

            # Send equipment manifest so the UI can render channel cards
            broadcaster.notify_equipment_manifest()

            # ---- Telemetry callback ----
            def telemetry_cb(x_hat, u, derived, info):
                broadcaster.on_telemetry(x_hat, u, derived, info)

                # Update progress
                elapsed = info.get('time_s', 0)
                pct = min(100.0, (elapsed / max(1, duration)) * 100.0)
                with self._lock:
                    self._progress['elapsed_s'] = elapsed
                    self._progress['pct'] = round(pct, 1)

                # Broadcast progress every 5 ticks (matches state decimation)
                if info.get('tick', 0) % 5 == 0:
                    broadcaster.notify_sim_progress(
                        elapsed, duration, pct,
                        phase=phase, profile=profile,
                    )

            solver.set_telemetry_callback(telemetry_cb)
            solver.set_mpc_interval(int(mpc.step_s))

            # ---- Solve loop ----
            solver.running = True
            n_steps = int(duration)

            for step in range(n_steps):
                if self._stop_flag.is_set():
                    logger.info("Simulation stopped by user at step %d", step)
                    break

                # Pause: block until unpaused or stopped
                while self._paused.is_set() and not self._stop_flag.is_set():
                    time.sleep(0.05)
                if self._stop_flag.is_set():
                    break

                try:
                    solver.solve_once()

                    if solver._cached_trajectory is not None:
                        ref_targets = solver._cached_ref_targets or {}
                        tolerances = {}
                        if compliance:
                            tolerances = compliance.tolerances
                        broadcaster.notify_solve(
                            solver._cached_trajectory,
                            solver.io.get_time(),
                            ref_targets,
                            tolerances,
                        )

                    if irr_scheduler:
                        x_hat = solver.ekf.x_hat
                        irr_scheduler.tick(x_hat, solver.io.get_time(), solver.last_u)

                    if co2_scheduler:
                        co2_scheduler.tick(solver.ekf.x_hat, solver.io.get_time(), solver.last_u)

                    gateway.evaluate_sync()
                    solver.io.write_actuators(solver.last_u)

                except Exception as e:
                    logger.warning("Sim tick %d error: %s", step, e)
                    solver.io.write_actuators(solver.last_u)

                # Speed throttle
                spd = self._speed
                if spd != float('inf') and spd > 0:
                    time.sleep(1.0 / spd)

            # ---- Completion ----
            report = compliance.report()
            broadcaster.notify_sim_complete(report, duration)

            with self._lock:
                self._progress['pct'] = 100.0
                self._progress['elapsed_s'] = duration

            if self._stop_flag.is_set():
                self._status = 'idle'
            else:
                self._status = 'completed'

            logger.info("Simulation finished: status=%s", self._status)

        except Exception:
            self._error = traceback.format_exc()
            self._status = 'error'
            logger.error("Simulation error:\n%s", self._error)
