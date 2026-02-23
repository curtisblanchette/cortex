"""SLSQP background calibrator.

Runs in a background thread, periodically replaying recent telemetry
through the nonlinear model with candidate parameters and minimizing
the prediction error via scipy.optimize.minimize(method='SLSQP').

Trigger conditions:
  - Periodic (every 5-30 min)
  - EKF innovation threshold exceeded
  - Phase change or lights transition
  - Irrigation event
"""

import threading
import time
import numpy as np
from scipy.optimize import minimize

from core.dynamics import GrowRoomDynamics
from .params import CalibrationParams, SharedParams, N_PARAMS
from .bounds import bounds_as_tuples, clip_params


class ModelCalibrator:
    """Background SLSQP optimizer for model parameter tuning.

    Reads telemetry history, replays through the nonlinear model,
    and finds parameters that minimize the prediction error.
    """

    def __init__(self, dynamics, telemetry_source, shared_params,
                 interval_s=600, min_samples=300):
        """Initialize calibrator.

        Args:
            dynamics: GrowRoomDynamics instance.
            telemetry_source: Object with .get_recent_data(window_s) method,
                            or a dict/list of telemetry records.
            shared_params: SharedParams instance for atomic param updates.
            interval_s: Default calibration interval in seconds.
            min_samples: Minimum number of samples required to calibrate.
        """
        self.dynamics = dynamics
        self.telemetry_source = telemetry_source
        self.shared_params = shared_params
        self.interval_s = interval_s
        self.min_samples = min_samples

        self._thread = None
        self._stop_event = threading.Event()
        self._force_event = threading.Event()

        # Calibration history
        self.history = []

    def start(self):
        """Start background calibration thread."""
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop background calibration."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=10.0)

    def trigger_fast_calibration(self):
        """Trigger an immediate calibration run."""
        self._force_event.set()

    def _run_loop(self):
        """Background calibration loop."""
        while not self._stop_event.is_set():
            # Wait for interval or force trigger
            triggered = self._force_event.wait(timeout=self.interval_s)
            if self._stop_event.is_set():
                break

            self._force_event.clear()

            try:
                result = self.calibrate()
                if result:
                    self.history.append(result)
            except Exception as e:
                print(f"[Calibrator] error: {e}")

    def calibrate(self, telemetry_data=None):
        """Run one calibration cycle.

        Args:
            telemetry_data: Optional explicit data. If None, reads from source.

        Returns:
            Dict with calibration results, or None if insufficient data.
        """
        # Get telemetry data
        if telemetry_data is None:
            telemetry_data = self._get_telemetry()

        if telemetry_data is None or len(telemetry_data['x']) < self.min_samples:
            return None

        x_actual = telemetry_data['x']       # (N, 7) state estimates
        u_applied = telemetry_data['u']      # (N, 6) applied controls
        d_observed = telemetry_data['d']     # (N, 5) observed disturbances
        N = len(x_actual) - 1

        # Current parameters as starting point
        p0 = self.shared_params.current_array()
        bounds = bounds_as_tuples()

        # Cost before optimization
        cost_before = self._objective(p0, x_actual, u_applied, d_observed, N)

        t_start = time.perf_counter()

        # SLSQP optimization
        result = minimize(
            self._objective,
            p0,
            args=(x_actual, u_applied, d_observed, N),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 100, 'ftol': 1e-8, 'disp': False},
        )

        t_elapsed = time.perf_counter() - t_start

        p_new = clip_params(result.x)
        cost_after = result.fun

        # Only apply if cost actually improved
        if cost_after < cost_before * 1.01:
            new_params = CalibrationParams.from_array(p_new)
            version = self.shared_params.update(new_params)
            applied = True
        else:
            version = self.shared_params.version
            applied = False

        return {
            'timestamp': time.time(),
            'cost_before': cost_before,
            'cost_after': cost_after,
            'improvement_pct': (1.0 - cost_after / max(cost_before, 1e-10)) * 100,
            'params': p_new.tolist(),
            'iterations': result.nit,
            'solve_time_s': t_elapsed,
            'applied': applied,
            'version': version,
            'success': result.success,
            'message': result.message,
        }

    def _objective(self, params_array, x_actual, u_applied, d_observed, N):
        """SLSQP objective: sum of squared prediction errors.

        Replays actual control actions and disturbances through the
        nonlinear model with candidate parameters, compares to what
        actually happened.
        """
        params = CalibrationParams.from_array(params_array)

        # Weighted state importance
        weights = np.array([1.0, 0.5, 0.01, 1.0, 10.0, 0.5, 0.2])

        x_sim = x_actual[0].copy()
        total_error = 0.0

        for k in range(N):
            x_sim = self.dynamics.nonlinear_step(
                x_sim, u_applied[k], d_observed[k], params, dt=1.0
            )
            error = (x_sim - x_actual[k + 1]) * weights
            total_error += np.sum(error ** 2)

        return total_error / N

    def _get_telemetry(self):
        """Get recent telemetry data from the source."""
        if hasattr(self.telemetry_source, 'get_recent_data'):
            return self.telemetry_source.get_recent_data(self.interval_s * 2)

        # If source is a dict-like with history arrays
        if isinstance(self.telemetry_source, dict):
            return self.telemetry_source

        return None
