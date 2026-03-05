"""Thread-safe shared parameter store for model calibration.

SharedParams provides lock-free reads and atomic writes so the real-time
control loop can read parameters without blocking while the background
SLSQP calibrator writes updated values.
"""

import threading
from dataclasses import dataclass, field, asdict
from typing import Optional

import numpy as np


@dataclass
class CalibrationParams:
    """The 7 tunable model parameters.

    These describe the physics of the room and are tuned by SLSQP
    to minimize the gap between model predictions and sensor data.
    LAI is phase-derived (not tunable) — stored on GrowRoomDynamics.
    """
    UA_wall: float = 50.0       # W/K - wall thermal conductance
    UA_env: float = 25.0        # W/K - envelope conductance to outdoors
    tau_hvac: float = 120.0     # s - HVAC thermal time constant
    g_s_max: float = 0.4        # mol/m²/s - max stomatal conductance
    K_drain: float = 0.05       # 1/hr - substrate drainage coefficient
    C_wall: float = 500000.0    # J/K - wall/thermal mass capacitance
    eta_hvac: float = 0.85      # - HVAC efficiency factor

    def to_array(self) -> np.ndarray:
        """Convert to numpy array in canonical order."""
        return np.array([
            self.UA_wall, self.UA_env, self.tau_hvac, self.g_s_max,
            self.K_drain, self.C_wall, self.eta_hvac,
        ])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'CalibrationParams':
        """Create from numpy array in canonical order."""
        return cls(
            UA_wall=float(arr[0]),
            UA_env=float(arr[1]),
            tau_hvac=float(arr[2]),
            g_s_max=float(arr[3]),
            K_drain=float(arr[4]),
            C_wall=float(arr[5]),
            eta_hvac=float(arr[6]),
        )

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> 'CalibrationParams':
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


PARAM_NAMES = [
    'UA_wall', 'UA_env', 'tau_hvac', 'g_s_max',
    'K_drain', 'C_wall', 'eta_hvac',
]

N_PARAMS = len(PARAM_NAMES)


class SharedParams:
    """Thread-safe parameter store.

    The control loop reads parameters every cycle (1 Hz) via .current().
    The SLSQP calibrator writes new parameters every 5-30 min via .update().

    Implementation: simple reference swap. The control loop grabs the
    reference at the start of each cycle. Even if SLSQP updates mid-cycle,
    the control loop uses a consistent set of parameters for that cycle.
    """

    def __init__(self, initial: Optional[CalibrationParams] = None):
        self._params = initial or CalibrationParams()
        self._version = 0
        self._lock = threading.Lock()

    def current(self) -> CalibrationParams:
        """Lock-free read of current parameters.

        Returns a reference to the current CalibrationParams.
        Safe because Python's GIL makes reference reads atomic.
        """
        return self._params

    def current_array(self) -> np.ndarray:
        """Current parameters as numpy array."""
        return self._params.to_array()

    def update(self, new_params: CalibrationParams) -> int:
        """Atomic swap of parameters. Returns new version number."""
        with self._lock:
            self._params = new_params
            self._version += 1
            return self._version

    @property
    def version(self) -> int:
        return self._version
