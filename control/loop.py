"""Control loop — the 1 Hz orchestrator.

Same class for production and simulation. The only difference
is which IOInterface it's given.

Each tick:
  1. Read sensors
  2. EKF update (state estimation)
  3. MPC solve (optimal control)
  4. Apply control
  5. Log + compliance
"""

import asyncio
import time
import numpy as np

from core.dynamics import GrowRoomDynamics
from estimation.ekf import StateEstimator
from mpc.solver import MPCSolver
from mpc.reference import ReferenceGenerator, LightSchedule
from mpc.forecast import DisturbanceForecast
from .io import IOInterface
from .failsafe import FailSafeManager
from .compliance import ComplianceTracker


class ControlLoop:
    """Main 1 Hz control loop orchestrator.

    Ties together EKF, MPC, IO, compliance, and failsafe.
    """

    def __init__(self, io, dynamics, ekf, mpc, reference, forecast,
                 compliance, failsafe, shared_params, light_schedule=None):
        """Initialize control loop.

        Args:
            io: IOInterface (SimulatedIO or RealIO).
            dynamics: GrowRoomDynamics instance.
            ekf: StateEstimator instance.
            mpc: MPCSolver instance.
            reference: ReferenceGenerator instance.
            forecast: DisturbanceForecast instance.
            compliance: ComplianceTracker instance.
            failsafe: FailSafeManager instance.
            shared_params: SharedParams instance.
            light_schedule: LightSchedule instance.
        """
        self.io = io
        self.dynamics = dynamics
        self.ekf = ekf
        self.mpc = mpc
        self.reference = reference
        self.forecast = forecast
        self.compliance = compliance
        self.failsafe = failsafe
        self.shared_params = shared_params
        self.light_schedule = light_schedule or LightSchedule(6.0, 18.0)

        # Control state
        self.last_u = GrowRoomDynamics.default_control()
        self.tick_count = 0
        self.running = False

        # Telemetry callback (optional)
        self._telemetry_callback = None

    def tick(self):
        """One complete control cycle. Called at 1 Hz.

        Returns:
            x_hat: State estimate (7,).
            u_opt: Applied control (6,).
            trajectory: Predicted trajectory (Np, 7).
            info: Dict with diagnostics.
        """
        t_start = time.perf_counter()

        # Step 1: Read sensors
        z_meas = self.io.read_sensors()
        d_current = self.io.get_disturbance()
        current_time = self.io.get_time()

        # Step 2: State estimation (EKF)
        params = self._get_params()
        x_hat, P, innovation = self.ekf.update(
            z_meas, self.last_u, d_current, params, dt=1.0
        )

        # Step 3: MPC solve
        ref_trajectory = self.reference.generate(
            current_time, self.light_schedule,
            horizon_steps=self.mpc.Np, step_s=self.mpc.step_s,
        )
        d_forecast = self.forecast.generate(
            current_time, d_current,
            horizon_steps=self.mpc.Np, step_s=self.mpc.step_s,
        )

        u_opt, trajectory, solve_info = self.mpc.solve(
            x_hat, d_forecast, ref_trajectory, u_prev=self.last_u,
            Q_diag=self.reference.Q_diag,
            R_diag=self.reference.R_diag,
            S_diag=self.reference.S_diag,
        )

        # Step 4: Fail-safe check and apply
        u_apply, fs_level = self.failsafe.check_solve(
            solve_info['status'], u_opt, solve_info
        )

        if u_apply is not None:
            self.last_u = u_apply.copy()
        # Always write actuators to advance simulation / real hardware
        self.io.write_actuators(self.last_u)

        # Step 5: Compliance + logging
        derived = self.dynamics.derived_outputs(x_hat)
        ref_targets = {
            'VPD': ref_trajectory[0, 0],
            'T': ref_trajectory[0, 1],
            'CO2': ref_trajectory[0, 2],
            'theta': ref_trajectory[0, 4],
        }
        self.compliance.update(x_hat, derived, ref_targets, current_time)

        t_elapsed = (time.perf_counter() - t_start) * 1000.0

        info = {
            'tick': self.tick_count,
            'time_s': current_time,
            'solve_status': solve_info['status'],
            'solve_ms': solve_info.get('solve_time_ms', 0),
            'total_ms': t_elapsed,
            'failsafe_level': fs_level,
            'innovation_rms': self.ekf.innovation_magnitude(),
        }

        self.tick_count += 1

        # Telemetry callback
        if self._telemetry_callback:
            self._telemetry_callback(x_hat, u_apply, derived, info)

        return x_hat, u_apply if u_apply is not None else self.last_u, trajectory, info

    async def run(self, duration_s=None, realtime=True):
        """Run the control loop.

        Args:
            duration_s: Run for this many seconds (None = forever).
            realtime: If True, sleep to maintain 1 Hz. If False, run as fast as possible.
        """
        self.running = True
        start = time.time()

        while self.running:
            t0 = time.perf_counter()

            try:
                self.tick()
            except Exception as e:
                print(f"[ControlLoop] tick {self.tick_count} error: {e}")

            if duration_s is not None and self.io.get_time() >= duration_s:
                break

            if realtime:
                elapsed = time.perf_counter() - t0
                sleep_time = max(0, 1.0 - elapsed)
                await asyncio.sleep(sleep_time)

    def run_sync(self, duration_s, realtime=False):
        """Synchronous run for fast-forward simulation.

        Args:
            duration_s: Run for this many seconds.
            realtime: If True, sleep between ticks.
        """
        self.running = True
        n_steps = int(duration_s)

        for step in range(n_steps):
            if not self.running:
                break
            try:
                self.tick()
            except Exception as e:
                print(f"[ControlLoop] tick {step} error: {e}")

            if realtime:
                time.sleep(max(0, 1.0 - 0.025))  # approximate

    def stop(self):
        self.running = False

    def set_telemetry_callback(self, callback):
        """Set callback for telemetry: callback(x_hat, u, derived, info)."""
        self._telemetry_callback = callback

    def _get_params(self):
        if hasattr(self.shared_params, 'current'):
            return self.shared_params.current()
        return self.shared_params
