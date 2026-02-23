"""Constraint builder for MPC QP formulation.

Constructs bounds and inequality constraints for the OSQP solver:
  - State bounds (safety limits on temperature, humidity, CO2, etc.)
  - Control bounds (actuator min/max)
  - Control rate-of-change bounds (Δu limits for smooth actuation)
"""

import numpy as np

from core.dynamics import N_STATES, N_CONTROLS


# Default state bounds [lower, upper]
DEFAULT_STATE_BOUNDS = {
    'T_a':    (15.0, 35.0),     # °C
    'w_a':    (3.0, 20.0),      # g/kg
    'CO2':    (400.0, 2000.0),  # ppm
    'T_l':    (15.0, 35.0),     # °C
    'theta':  (0.05, 0.55),     # m³/m³
    'T_s':    (5.0, 45.0),      # °C
    'T_w':    (10.0, 40.0),     # °C
}

# Default control bounds [lower, upper]
DEFAULT_CONTROL_BOUNDS = {
    'T_sp':   (15.0, 32.0),    # °C (or u_cool: 0-100%)
    'RH_sp':  (40.0, 80.0),    # % (or u_heat: 0-100%)
    'u_hum':  (0.0, 100.0),    # %
    'u_co2':  (0.0, 100.0),    # %
    'u_fan':  (20.0, 100.0),   # %
    'u_irr':  (0.0, 100.0),    # %
}

# Default rate-of-change bounds [lower, upper] per MPC step
DEFAULT_DU_BOUNDS = {
    'T_sp':   (-2.0, 2.0),     # °C per step
    'RH_sp':  (-5.0, 5.0),     # % per step
    'u_hum':  (-20.0, 20.0),   # % per step
    'u_co2':  (-30.0, 30.0),   # % per step
    'u_fan':  (-10.0, 10.0),   # % per step
    'u_irr':  (-50.0, 50.0),   # % per step
}

STATE_NAMES = ['T_a', 'w_a', 'CO2', 'T_l', 'theta', 'T_s', 'T_w']
CONTROL_NAMES = ['T_sp', 'RH_sp', 'u_hum', 'u_co2', 'u_fan', 'u_irr']


class ConstraintBuilder:
    """Builds constraint vectors for OSQP QP formulation."""

    def __init__(self, equipment_config=None):
        self.n_x = N_STATES
        self.n_u = N_CONTROLS

        # State bounds
        self.x_min = np.array([DEFAULT_STATE_BOUNDS[n][0] for n in STATE_NAMES])
        self.x_max = np.array([DEFAULT_STATE_BOUNDS[n][1] for n in STATE_NAMES])

        # Control bounds
        self.u_min = np.array([DEFAULT_CONTROL_BOUNDS[n][0] for n in CONTROL_NAMES])
        self.u_max = np.array([DEFAULT_CONTROL_BOUNDS[n][1] for n in CONTROL_NAMES])

        # Rate-of-change bounds
        self.du_min = np.array([DEFAULT_DU_BOUNDS[n][0] for n in CONTROL_NAMES])
        self.du_max = np.array([DEFAULT_DU_BOUNDS[n][1] for n in CONTROL_NAMES])

        # Override from equipment config if provided
        if equipment_config:
            self._apply_equipment_limits(equipment_config)

    def _apply_equipment_limits(self, config):
        """Apply equipment-specific limits from config."""
        hvacd = config.get('hvacd', {})
        limits = hvacd.get('limits', {})

        if 'T_sp_min' in limits:
            self.u_min[0] = limits['T_sp_min']
        if 'T_sp_max' in limits:
            self.u_max[0] = limits['T_sp_max']
        if 'RH_sp_min' in limits:
            self.u_min[1] = limits['RH_sp_min']
        if 'RH_sp_max' in limits:
            self.u_max[1] = limits['RH_sp_max']
        if 'fan_min_pct' in limits:
            self.u_min[4] = limits['fan_min_pct']
        if 'fan_max_pct' in limits:
            self.u_max[4] = limits['fan_max_pct']

    def get_state_bounds(self, Np):
        """Get state bounds stacked over prediction horizon.

        Args:
            Np: Prediction horizon length.

        Returns:
            x_lb: Lower bounds (Np * n_x,).
            x_ub: Upper bounds (Np * n_x,).
        """
        x_lb = np.tile(self.x_min, Np)
        x_ub = np.tile(self.x_max, Np)
        return x_lb, x_ub

    def get_control_bounds(self, Nc):
        """Get control bounds stacked over control horizon.

        Args:
            Nc: Control horizon length.

        Returns:
            u_lb: Lower bounds (Nc * n_u,).
            u_ub: Upper bounds (Nc * n_u,).
        """
        u_lb = np.tile(self.u_min, Nc)
        u_ub = np.tile(self.u_max, Nc)
        return u_lb, u_ub

    def get_du_bounds(self, Nc):
        """Get rate-of-change bounds stacked over control horizon.

        Args:
            Nc: Control horizon length.

        Returns:
            du_lb: Lower bounds (Nc * n_u,).
            du_ub: Upper bounds (Nc * n_u,).
        """
        du_lb = np.tile(self.du_min, Nc)
        du_ub = np.tile(self.du_max, Nc)
        return du_lb, du_ub
