"""Constraint builder for MPC QP formulation.

Constructs bounds and inequality constraints for the OSQP solver:
  - State bounds (safety limits on temperature, humidity, CO2, etc.)
  - Control bounds (actuator min/max) — from ActuatorRegistry
  - Control rate-of-change bounds (du limits) — from ActuatorRegistry
"""

import numpy as np

from core.dynamics import N_STATES


# Default state bounds [lower, upper]
# Physical feasibility bounds (not comfort targets).
# Wide enough for QP feasibility during transients.
DEFAULT_STATE_BOUNDS = {
    'T_a':    (-200.0, 200.0),  # C — wide bounds prevent infeasibility from linearization error
    'w_a':    (-50.0, 100.0),   # g/kg — wide for linearization feasibility
    'CO2':    (0.0, 10000.0),   # ppm
    'T_l':    (-200.0, 200.0),  # C — coupled to T_a, must be equally wide
    'theta':  (0.0, 1.0),       # m3/m3
    'T_s':    (-200.0, 200.0),  # C — supply temp can go low at high cooling
    'T_w':    (-200.0, 200.0),  # C — wall temp coupled to air
}

STATE_NAMES = ['T_a', 'w_a', 'CO2', 'T_l', 'theta', 'T_s', 'T_w']


class ConstraintBuilder:
    """Builds constraint vectors for OSQP QP formulation.

    Control bounds and rate-of-change bounds come from the ActuatorRegistry.
    State bounds use sensible defaults.
    """

    def __init__(self, registry, equipment_config=None):
        """
        Args:
            registry: ActuatorRegistry instance.
            equipment_config: Optional equipment config for HVACD limit overrides.
        """
        self.n_x = N_STATES
        self.n_u = registry.n_u

        # State bounds
        self.x_min = np.array([DEFAULT_STATE_BOUNDS[n][0] for n in STATE_NAMES])
        self.x_max = np.array([DEFAULT_STATE_BOUNDS[n][1] for n in STATE_NAMES])

        # Control bounds from registry
        self.u_min = registry.u_min.copy()
        self.u_max = registry.u_max.copy()

        # Rate-of-change bounds from registry
        self.du_min = registry.du_min.copy()
        self.du_max = registry.du_max.copy()

    def get_state_bounds(self, Np):
        """Get state bounds stacked over prediction horizon."""
        x_lb = np.tile(self.x_min, Np)
        x_ub = np.tile(self.x_max, Np)
        return x_lb, x_ub

    def get_control_bounds(self, Nc):
        """Get control bounds stacked over control horizon."""
        u_lb = np.tile(self.u_min, Nc)
        u_ub = np.tile(self.u_max, Nc)
        return u_lb, u_ub

    def get_du_bounds(self, Nc):
        """Get rate-of-change bounds stacked over control horizon."""
        du_lb = np.tile(self.du_min, Nc)
        du_ub = np.tile(self.du_max, Nc)
        return du_lb, du_ub
