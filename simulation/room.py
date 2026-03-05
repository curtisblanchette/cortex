"""SimulatedRoom — software replacement for the physical grow room.

Uses the SAME GrowRoomDynamics as production, but:
- Maintains "true" state that the controller can't see directly
- Adds realistic sensor noise to measurements
- Accepts control commands and propagates physics forward
- Generates disturbances on a schedule
"""

import numpy as np

from core.dynamics import GrowRoomDynamics, N_STATES, N_DISTURBANCES
from calibration.params import CalibrationParams
from .noise import SensorNoiseModel, N_MEASUREMENTS
from .disturbance import DisturbanceScheduler


class SimulatedRoom:
    """Software model of the grow room for testing and development.

    The controller doesn't know it's talking to a simulation —
    same interface as real hardware via the IO adapter pattern.
    """

    def __init__(self, room_config, equipment_config, registry,
                 initial_state=None,
                 sim_params=None, noise_seed=None, max_steps=86400,
                 start_time_s=None, substrate=None, LAI_ref=3.0):
        """Initialize simulated room.

        Args:
            room_config: Room configuration dict (from room.yaml).
            equipment_config: Equipment configuration dict (from equipment.yaml).
            registry: ActuatorRegistry instance.
            initial_state: Initial state vector (7,). If None, uses defaults.
            sim_params: CalibrationParams for the "true" room physics.
                       Can differ from controller's params to test robustness.
            noise_seed: Random seed for reproducible noise.
            max_steps: Pre-allocated history capacity (grows if exceeded).
            start_time_s: Simulation start time in seconds (default: light on_hour).
            substrate: SubstrateProfile instance.
        """
        self.params = sim_params or CalibrationParams()
        self.registry = registry
        self.dynamics = GrowRoomDynamics(room_config, equipment_config,
                                         self.params, registry=registry,
                                         substrate=substrate, LAI_ref=LAI_ref)

        # The "true" state of the room (controller never sees this directly)
        if initial_state is not None:
            self.x_true = np.array(initial_state, dtype=np.float64)
        else:
            self.x_true = GrowRoomDynamics.default_initial_state()

        # Disturbance schedule
        self.disturbance_schedule = DisturbanceScheduler(room_config)

        # Sensor noise model
        self.noise = SensorNoiseModel(seed=noise_seed)

        # Simulation clock
        if start_time_s is not None:
            self._start_time = float(start_time_s)
        else:
            self._start_time = 0.0
        self.time = self._start_time

        # Pre-allocated history arrays (avoids 86k dict allocations)
        self._max_steps = max_steps
        self._hist_idx = 0
        self._hist_time = np.empty(max_steps, dtype=np.float64)
        self._hist_x = np.empty((max_steps, N_STATES), dtype=np.float64)
        self._hist_z = np.empty((max_steps, N_MEASUREMENTS), dtype=np.float64)
        self._hist_u = np.empty((max_steps, registry.n_u), dtype=np.float64)
        self._hist_d = np.empty((max_steps, N_DISTURBANCES), dtype=np.float64)

    @property
    def history(self):
        """Legacy access — returns number of recorded steps (for len() checks)."""
        return range(self._hist_idx)

    def step(self, u, dt=1.0):
        """Advance the simulated room by dt seconds.

        Args:
            u: Control vector from the controller (6,).
            dt: Time step in seconds (default 1s to match control loop).

        Returns:
            z_meas: Noisy sensor measurements (6,) — what the controller sees.
            d: Current disturbances (5,) — what the controller knows about.
        """
        # Get current disturbances
        d = self.disturbance_schedule.get(self.time)

        # Propagate TRUE physics (full nonlinear, no approximations)
        self.x_true = self.dynamics.nonlinear_step(
            self.x_true, u, d, self.params, dt
        )

        # Generate noisy measurements (what sensors would report)
        z_meas = self.noise.corrupt(self.x_true, self.time)

        # Record history into pre-allocated arrays
        idx = self._hist_idx
        if idx >= self._max_steps:
            self._grow_history()
        self._hist_time[idx] = self.time
        self._hist_x[idx] = self.x_true
        self._hist_z[idx] = z_meas
        self._hist_u[idx] = u
        self._hist_d[idx] = d
        self._hist_idx = idx + 1

        self.time += dt
        return z_meas, d

    def _grow_history(self):
        """Double history capacity when full."""
        new_cap = self._max_steps * 2
        for attr, arr in [('_hist_time', self._hist_time),
                          ('_hist_x', self._hist_x),
                          ('_hist_z', self._hist_z),
                          ('_hist_u', self._hist_u),
                          ('_hist_d', self._hist_d)]:
            new = np.empty((new_cap,) + arr.shape[1:], dtype=arr.dtype)
            new[:self._max_steps] = arr
            setattr(self, attr, new)
        self._max_steps = new_cap

    def get_true_state(self):
        """For post-hoc analysis only. Controller never calls this."""
        return self.x_true.copy()

    def get_history_arrays(self):
        """Return history as numpy arrays for plotting/analysis.

        Returns:
            Dict with keys: time, x_true, z_meas, u, d, VPD, RH — each a numpy array.
        """
        n = self._hist_idx
        if n == 0:
            return {}

        x = self._hist_x[:n]
        result = {
            'time': self._hist_time[:n].copy(),
            'x_true': x.copy(),
            'z_meas': self._hist_z[:n].copy(),
            'u': self._hist_u[:n].copy(),
            'd': self._hist_d[:n].copy(),
        }

        # Compute derived outputs from stored states (vectorized)
        for i in range(n):
            derived = self.dynamics.derived_outputs(x[i])
            if i == 0:
                result['VPD'] = np.empty(n, dtype=np.float64)
                result['RH'] = np.empty(n, dtype=np.float64)
            result['VPD'][i] = derived['VPD']
            result['RH'][i] = derived['RH']

        return result

    def reset(self, initial_state=None, noise_seed=None, start_time_s=None):
        """Reset simulation to initial conditions."""
        if initial_state is not None:
            self.x_true = np.array(initial_state, dtype=np.float64)
        else:
            self.x_true = GrowRoomDynamics.default_initial_state()

        self.time = float(start_time_s) if start_time_s is not None else self._start_time
        self._hist_idx = 0
        self.noise.reset(seed=noise_seed)
