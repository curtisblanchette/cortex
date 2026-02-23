"""Sensor noise model for simulation.

Generates realistic measurement noise matching real hardware specs.
Each sensor has:
  - Gaussian white noise (std)
  - Slowly drifting bias (random walk)
  - Quantization (ADC resolution)
"""

import numpy as np


# Measurement indices (which state maps to which sensor)
# z[0] = T_air (from SHT31)
# z[1] = w_a (computed from SHT31 T + RH)
# z[2] = CO2 (from SCD30)
# z[3] = T_leaf (from MLX90614)
# z[4] = theta (from TEROS ONE)
# z[5] = T_supply (from thermocouple)

N_MEASUREMENTS = 6

# State-to-measurement mapping: z[i] = x[STATE_TO_MEAS[i]]
STATE_TO_MEAS = [0, 1, 2, 3, 4, 5]


DEFAULT_NOISE_CONFIG = {
    'T_air':    {'std': 0.3, 'bias_drift': 0.01, 'quantization': 0.01},
    'w_a':      {'std': 0.2, 'bias_drift': 0.005, 'quantization': 0.01},
    'CO2':      {'std': 30.0, 'bias_drift': 1.0, 'quantization': 1.0},
    'T_leaf':   {'std': 0.5, 'bias_drift': 0.02, 'quantization': 0.01},
    'theta':    {'std': 0.01, 'bias_drift': 0.001, 'quantization': 0.001},
    'T_supply': {'std': 0.3, 'bias_drift': 0.01, 'quantization': 0.01},
}

SENSOR_NAMES = ['T_air', 'w_a', 'CO2', 'T_leaf', 'theta', 'T_supply']


class SensorNoiseModel:
    """Generates realistic sensor noise for simulation.

    Matches real hardware specs: SHT31, SCD30, MLX90614, TEROS ONE.
    """

    def __init__(self, config=None, seed=None):
        self.config = config or DEFAULT_NOISE_CONFIG
        self.rng = np.random.default_rng(seed)

        # Initialize bias for each sensor (starts at 0, random walks)
        self.bias = np.zeros(N_MEASUREMENTS)

        # Parse config into arrays for vectorized ops
        self.std = np.array([self.config[name]['std'] for name in SENSOR_NAMES])
        self.bias_drift = np.array([self.config[name]['bias_drift'] for name in SENSOR_NAMES])
        self.quant = np.array([self.config[name].get('quantization', 0.0) for name in SENSOR_NAMES])

    def corrupt(self, x_true, time_s):
        """Add realistic noise to true state to produce sensor readings.

        Args:
            x_true: True state vector (7,). Only first 6 are measured.
            time_s: Current simulation time in seconds.

        Returns:
            z_meas: Noisy measurement vector (6,).
        """
        # Extract measurable states
        z_clean = x_true[STATE_TO_MEAS].copy()

        # Update bias random walk
        self.bias += self.rng.normal(0, self.bias_drift)

        # White noise
        white = self.rng.normal(0, self.std)

        # Combine
        z_noisy = z_clean + self.bias + white

        # Quantization
        for i in range(N_MEASUREMENTS):
            if self.quant[i] > 0:
                z_noisy[i] = np.round(z_noisy[i] / self.quant[i]) * self.quant[i]

        return z_noisy

    def reset(self, seed=None):
        """Reset bias and RNG."""
        self.bias = np.zeros(N_MEASUREMENTS)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
