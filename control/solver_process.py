"""Solver Process — EKF + OSQP, publishes solutions to Redis.

This is Process 1 in the decoupled architecture. It owns:
  - Reading sensor data (from Redis or SimulatedIO)
  - EKF predict + update (state estimation)
  - MPC solve (OSQP QP)
  - Publishing the solution to Redis (mpc:room1:solution)

It does NOT touch hardware. The ActuatorGateway owns all dispatch.

Supports two modes:
  - Production: reads sensors from Redis, publishes solution to Redis.
  - Simulation: reads from SimulatedIO, publishes to an in-memory dict
    (SolutionStore) that the gateway reads from directly.
"""

import asyncio
import json
import time
import logging
from datetime import datetime, timezone

import numpy as np

from estimation.ekf import StateEstimator
from mpc.solver import MPCSolver
from mpc.reference import ReferenceGenerator, LightSchedule
from mpc.forecast import DisturbanceForecast
from control.triggers import SolveTrigger
from control.failsafe import FailSafeManager
from core.thermodynamics import saturation_vapor_pressure

logger = logging.getLogger(__name__)


class SolutionStore:
    """In-memory solution store for simulation mode (replaces Redis).

    Thread-safe via a simple lock. In production, this is Redis.
    """

    def __init__(self):
        self._solution = None
        try:
            loop = asyncio.get_running_loop()
            self._lock = asyncio.Lock()
        except RuntimeError:
            self._lock = None

    def publish(self, solution):
        """Write a new solution."""
        self._solution = solution.copy()

    def read(self):
        """Read the latest solution. Returns None if no solution yet."""
        if self._solution is None:
            return None
        return self._solution.copy()


class SolverProcess:
    """Owns EKF + OSQP. Publishes solutions to a store (Redis or in-memory).

    In simulation mode, still uses the IO interface for sensor data
    and drives the simulation forward by calling write_actuators().
    The solve cadence is decoupled from the actuation cadence.
    """

    def __init__(self, io, dynamics, ekf, mpc, reference, forecast,
                 failsafe, shared_params, light_schedule, registry,
                 solution_store=None, redis_client=None,
                 room_id='room1', trigger_config=None,
                 compliance=None, advisory_engine=None,
                 integral_config=None):
        self.io = io
        self.dynamics = dynamics
        self.ekf = ekf
        self.mpc = mpc
        self.reference = reference
        self.forecast = forecast
        self.failsafe = failsafe
        self.shared_params = shared_params
        self.light_schedule = light_schedule or LightSchedule(6.0, 18.0)
        self.registry = registry
        self.compliance = compliance
        self.advisory_engine = advisory_engine

        # Solution output
        self.solution_store = solution_store or SolutionStore()
        self.redis = redis_client
        self.room_id = room_id

        # Solve triggers
        self.trigger = SolveTrigger(trigger_config)

        # State
        self.last_u = registry.default_control()
        self.solve_count = 0
        self.running = False

        # Light state tracking for OSQP cold-start on transitions
        self._last_lights_on = None

        # MPC cadence control: solve every _mpc_interval ticks, not every tick.
        # EKF runs every tick for up-to-date state estimation.
        self._tick_count = 0
        self._mpc_interval = 1  # default: solve every tick (production)
        self._cached_trajectory = None
        self._cached_solve_info = {'status': 'not_solved', 'iterations': 0}
        self._cached_fs_level = 'P1_NORMAL'
        self._cached_ref_targets = None

        # Smoothed leaf-air offset for derived RH target (avoids tick-to-tick jitter)
        self._ema_delta_T_leaf = None

        # Integral action — biases references to compensate for slow unmodeled
        # dynamics (wall thermal mass, outdoor weather forecast bias, HVAC supply
        # lag) that the MPC's in-horizon plan cannot resolve. NOT a linearization
        # patch — see docs/decisions-recent.md "Integral action diagnosis".
        cfg = integral_config or {}
        self._integral_gain = float(cfg.get('gain', 0.003))
        self._t_clamp = float(cfg.get('t_clamp_C', 4.0))
        self._vpd_clamp = float(cfg.get('vpd_clamp_kPa', 0.3))
        self._integral_saturation_warn_s = float(cfg.get('saturation_warn_s', 300.0))

        self._vpd_integral = 0.0
        self._t_integral = 0.0

        # Saturation tracking — None when inside clamp, sim-time-of-onset when pegged.
        self._t_saturated_since = None
        self._vpd_saturated_since = None

        # Telemetry callback (optional)
        self._telemetry_callback = None

    def set_mpc_interval(self, interval):
        """Set MPC solve interval in simulation ticks.

        For simulation, set to mpc.step_s so the MPC solves at the same
        cadence as its prediction step. The EKF still runs every tick.
        """
        self._mpc_interval = max(1, int(interval))

    def solve_once(self):
        """Run one EKF cycle, and MPC solve on cadence.

        The EKF runs every call for up-to-date state estimation.
        The MPC only solves every _mpc_interval calls to match the
        prediction step size and prevent du constraint mismatch.

        Returns:
            (x_hat, u_opt, trajectory, info)
        """
        t_start = time.perf_counter()

        # Read sensors
        z_meas = self.io.read_sensors()
        d_current = self.io.get_disturbance()
        current_time = self.io.get_time()

        # Detect light state changes — force OSQP cold start + failsafe reset
        current_lights = self.light_schedule.is_on(current_time)
        force_solve = False
        if self._last_lights_on is not None and current_lights != self._last_lights_on:
            self.mpc.force_cold_start()
            self.failsafe.reset()
            self._ema_delta_T_leaf = None  # re-seed on light transition
            self._vpd_integral = 0.0       # prevent stale bias carrying over
            self._t_integral = 0.0
            # Clear saturation flags too — integral is now 0, no longer pegged.
            self._t_saturated_since = None
            self._vpd_saturated_since = None
            force_solve = True
            logger.info(f"[SolverProcess] Light transition at t={current_time:.0f}s — "
                        f"forcing OSQP cold start + failsafe reset")
        self._last_lights_on = current_lights

        # EKF update (every tick)
        params = self._get_params()
        x_hat, P, innovation = self.ekf.update(
            z_meas, self.last_u, d_current, params, dt=1.0
        )

        self._tick_count += 1
        should_solve = (
            force_solve
            or self._cached_trajectory is None
            or self._tick_count % self._mpc_interval == 0
        )

        if should_solve:
            # MPC solve
            ref_trajectory = self.reference.generate(
                current_time, self.light_schedule,
                horizon_steps=self.mpc.Np, step_s=self.mpc.step_s,
            )
            d_forecast = self.forecast.generate(
                current_time, d_current,
                horizon_steps=self.mpc.Np, step_s=self.mpc.step_s,
            )

            # Derive VPD-consistent RH targets from smoothed leaf-air offset
            raw_delta = x_hat[3] - x_hat[0]  # T_leaf - T_air
            if self._ema_delta_T_leaf is None:
                self._ema_delta_T_leaf = raw_delta
            else:
                self._ema_delta_T_leaf += 0.02 * (raw_delta - self._ema_delta_T_leaf)
            delta_T_leaf = self._ema_delta_T_leaf
            for k in range(ref_trajectory.shape[0]):
                T_leaf_k = ref_trajectory[k, 1] + delta_T_leaf
                svp_leaf = saturation_vapor_pressure(T_leaf_k)
                e_air_needed = svp_leaf - ref_trajectory[k, 0]  # SVP(T_leaf) - VPD
                svp_air = saturation_vapor_pressure(ref_trajectory[k, 1])
                if svp_air > 0:
                    ref_trajectory[k, 5] = np.clip(
                        100.0 * e_air_needed / svp_air, 30.0, 85.0
                    )

            # Cache UNBIASED reference targets for compliance tracking
            # (must happen before integral bias is applied)
            self._cached_ref_targets = {
                'VPD': ref_trajectory[0, 0],
                'T': ref_trajectory[0, 1],
                'CO2': ref_trajectory[0, 2],
                'theta': ref_trajectory[0, 4],
                'RH': ref_trajectory[0, 5],
            }

            # Integral action — biases references to absorb persistent bias
            # from slow unmodeled dynamics (wall thermal mass beyond horizon,
            # outdoor weather persistence-forecast gap, HVAC supply lag).
            # Sign convention: error = (target - actual). Positive error means
            # actual is below target → integral accumulates positive → raises
            # reference → MPC pushes harder upward.
            dt_solve = float(self._mpc_interval)
            derived_now = self.dynamics.derived_outputs(x_hat)

            vpd_error = ref_trajectory[0, 0] - derived_now['VPD']
            self._vpd_integral = float(np.clip(
                self._vpd_integral + self._integral_gain * vpd_error * dt_solve,
                -self._vpd_clamp, self._vpd_clamp,
            ))
            ref_trajectory[:, 0] += self._vpd_integral

            t_error = ref_trajectory[0, 1] - x_hat[0]
            self._t_integral = float(np.clip(
                self._t_integral + self._integral_gain * t_error * dt_solve,
                -self._t_clamp, self._t_clamp,
            ))
            ref_trajectory[:, 1] += self._t_integral

            # Track saturation onset (sim-time) for advisory and telemetry.
            t_at_clamp = abs(abs(self._t_integral) - self._t_clamp) < 1e-9
            vpd_at_clamp = abs(abs(self._vpd_integral) - self._vpd_clamp) < 1e-9
            if t_at_clamp and self._t_saturated_since is None:
                self._t_saturated_since = current_time
            elif not t_at_clamp:
                self._t_saturated_since = None
            if vpd_at_clamp and self._vpd_saturated_since is None:
                self._vpd_saturated_since = current_time
            elif not vpd_at_clamp:
                self._vpd_saturated_since = None

            u_opt, trajectory, solve_info = self.mpc.solve(
                x_hat, d_forecast, ref_trajectory, u_prev=self.last_u,
                Q_diag=self.reference.Q_diag,
                R_diag=self.reference.R_diag,
                S_diag=self.reference.S_diag,
            )

            # Fail-safe check
            u_apply, fs_level = self.failsafe.check_solve(
                solve_info['status'], u_opt, solve_info
            )

            if u_apply is not None:
                self.last_u = u_apply.copy()
            elif fs_level == 'P3_SOLVER_OFFLINE':
                self.last_u = self.registry.default_control()

            self.solve_count += 1

            # Build solution dict
            confidence = 1.0
            if 'inaccurate' in solve_info.get('status', ''):
                confidence = 0.7
            if fs_level != 'P1_NORMAL':
                confidence = 0.5

            solution = self._build_solution(
                u_apply if u_apply is not None else self.last_u,
                x_hat, trajectory, solve_info,
                (time.perf_counter() - t_start) * 1000.0,
                confidence, current_time,
            )

            # Publish solution
            self.solution_store.publish(solution)
            if self.redis:
                self._publish_to_redis(solution)

            # Record solve in trigger
            self.trigger.record_solve()

            # Cache for between-solve ticks
            self._cached_trajectory = trajectory
            self._cached_solve_info = solve_info
            self._cached_fs_level = fs_level
        else:
            trajectory = self._cached_trajectory
            solve_info = self._cached_solve_info
            fs_level = self._cached_fs_level

        t_elapsed = (time.perf_counter() - t_start) * 1000.0

        # Compliance tracking (every tick)
        if self.compliance and self._cached_ref_targets is not None:
            derived = self.dynamics.derived_outputs(x_hat)
            self.compliance.update(
                x_hat, derived, self._cached_ref_targets, current_time,
                lights_on=current_lights,
            )

        # Advisory engine (runs on MPC solve cadence, not every tick)
        if self.advisory_engine and should_solve and trajectory is not None:
            compliance_report = self.compliance.report() if self.compliance else None
            integral_state = {
                't_value': self._t_integral,
                't_clamp': self._t_clamp,
                't_saturated_since': self._t_saturated_since,
                'vpd_value': self._vpd_integral,
                'vpd_clamp': self._vpd_clamp,
                'vpd_saturated_since': self._vpd_saturated_since,
                'saturation_warn_s': self._integral_saturation_warn_s,
            }
            self.advisory_engine.evaluate(
                trajectory=trajectory,
                x_hat=x_hat,
                ref_targets=self._cached_ref_targets,
                solve_info=solve_info,
                current_time=current_time,
                light_schedule=self.light_schedule,
                step_s=self.mpc.step_s,
                compliance_report=compliance_report,
                innovation_rms=self.ekf.innovation_magnitude(),
                innovation_vector=self.ekf.innovation.copy(),
                integral_state=integral_state,
            )

        info = {
            'tick': self._tick_count - 1,
            'time_s': current_time,
            'solve_status': solve_info['status'],
            'solve_ms': t_elapsed,
            'total_ms': t_elapsed,
            'failsafe_level': fs_level,
            'innovation_rms': self.ekf.innovation_magnitude(),
            't_integral': float(self._t_integral),
            'vpd_integral': float(self._vpd_integral),
            't_integral_saturated': self._t_saturated_since is not None,
            'vpd_integral_saturated': self._vpd_saturated_since is not None,
        }

        # Telemetry
        if self._telemetry_callback:
            derived = self.dynamics.derived_outputs(x_hat)
            derived['effort'] = self.registry.get_all_effort_metrics()
            self._telemetry_callback(x_hat, self.last_u, derived, info)

        return x_hat, self.last_u, trajectory, info

    def _build_solution(self, u, x_hat, trajectory, solve_info,
                        solve_ms, confidence, sim_time):
        """Build the solution dict matching the Redis schema."""
        now_iso = datetime.now(timezone.utc).isoformat()

        # Extract predicted trajectories for dashboard
        traj_T = trajectory[:, 0].tolist()
        traj_CO2 = trajectory[:, 2].tolist()

        # Compute VPD and RH trajectories
        traj_VPD = []
        traj_RH = []
        for i in range(len(trajectory)):
            derived = self.dynamics.derived_outputs(trajectory[i])
            traj_VPD.append(round(derived['VPD'], 3))
            traj_RH.append(round(derived['RH'], 1))

        status = solve_info.get('status', 'solved')
        if status == 'solved inaccurate':
            status = 'solved_inaccurate'
        elif status not in ('solved', 'solved_inaccurate', 'failed'):
            if 'error' in status:
                status = 'failed'

        # Build setpoint fields dynamically from registry
        setpoints = {}
        idx_to_field = self.registry.index_to_solution_field()
        for idx, field_name in idx_to_field.items():
            setpoints[field_name] = round(float(u[idx]), 2)

        solution = {
            'timestamp': now_iso,
            'solve_id': self.solve_count,
            'status': status,
            **setpoints,

            # State estimate
            'x_hat': json.dumps([round(float(v), 3) for v in x_hat]),

            # Predicted trajectories
            'trajectory_T': json.dumps([round(v, 2) for v in traj_T]),
            'trajectory_VPD': json.dumps(traj_VPD),
            'trajectory_RH': json.dumps(traj_RH),
            'trajectory_CO2': json.dumps([round(v, 1) for v in traj_CO2]),

            # Solve metadata
            'solve_ms': round(solve_ms, 1),
            'osqp_iterations': solve_info.get('iterations', 0),
            'osqp_status': solve_info.get('status', 'unknown'),
            'solve_trigger': 'init' if self.solve_count <= 1 else 'timer',
            'confidence': round(confidence, 2),

            # Next solve hint
            'next_solve_s': self.trigger.next_solve_hint_s,

            # Simulation time (for sim mode coordination)
            'sim_time': round(sim_time, 1),
        }

        return solution

    def _publish_to_redis(self, solution):
        """Publish solution to Redis hash with TTL.

        Also appends a telemetry record (x_hat, last_u, disturbance) to
        a Redis list for the calibrator to consume.
        """
        key = f'mpc:{self.room_id}:solution'
        str_solution = {k: str(v) for k, v in solution.items()}
        self.redis.hset(key, mapping=str_solution)
        self.redis.expire(key, 120)

        # Append telemetry for calibrator
        try:
            x_hat = json.loads(solution.get('x_hat', '[]'))
            d_current = self.io.get_disturbance()
            record = json.dumps({
                'x': x_hat,
                'u': [round(float(v), 4) for v in self.last_u],
                'd': [round(float(v), 4) for v in d_current],
                't': solution.get('sim_time', 0),
            })
            history_key = f'telemetry:{self.room_id}:history'
            self.redis.rpush(history_key, record)
            self.redis.ltrim(history_key, -7200, -1)
        except Exception:
            pass  # telemetry is best-effort

    async def run(self, duration_s=None):
        """Run the solver process with adaptive cadence.

        For production use — runs as an independent async process.
        """
        self.running = True

        while self.running:
            if duration_s is not None and self.io.get_time() >= duration_s:
                break

            # Check triggers
            innovation_rms = self.ekf.innovation_magnitude()
            should_solve, trigger_name, wait_s = self.trigger.check(
                innovation_rms=innovation_rms,
                light_schedule=self.light_schedule,
                current_time_s=self.io.get_time(),
            )

            if should_solve:
                try:
                    _, _, _, info = self.solve_once()
                    # Update trigger with actual trigger name
                    if self.solution_store._solution:
                        self.solution_store._solution['solve_trigger'] = trigger_name
                except Exception as e:
                    logger.error(f"[SolverProcess] solve error: {e}")

            await asyncio.sleep(max(0.1, wait_s))

    def run_sync(self, duration_s, realtime=False):
        """Synchronous run for fast-forward simulation.

        In simulation mode, we still need to advance the sim at ~1 Hz
        (since SimulatedIO.write_actuators advances physics). The solver
        runs at its own cadence WITHIN this loop.

        Args:
            duration_s: Total simulation duration.
            realtime: If True, sleep between ticks.
        """
        self.running = True
        n_steps = int(duration_s)
        tick_count = 0

        for step in range(n_steps):
            if not self.running:
                break

            try:
                # Always solve on every tick in sim mode to advance the
                # simulation physics. The gateway still reads from the
                # solution store, but in sim mode they share the same
                # process for simplicity.
                self.solve_once()

                # Write actuators to advance simulation physics
                self.io.write_actuators(self.last_u)

                tick_count += 1
            except Exception as e:
                logger.error(f"[SolverProcess] tick {step} error: {e}")
                # Still advance simulation to avoid getting stuck
                self.io.write_actuators(self.last_u)

            if realtime:
                time.sleep(max(0, 1.0 - 0.025))

    def stop(self):
        self.running = False

    def set_telemetry_callback(self, callback):
        """Set callback: callback(x_hat, u, derived, info)."""
        self._telemetry_callback = callback

    def _get_params(self):
        if hasattr(self.shared_params, 'current'):
            return self.shared_params.current()
        return self.shared_params
