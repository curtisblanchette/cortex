"""Constraint builder for MPC QP formulation.

Constructs bounds and inequality constraints for the OSQP solver:
  - State bounds (safety limits on temperature, humidity, CO2, etc.)
  - Control bounds (actuator min/max) — from ActuatorRegistry
  - Control rate-of-change bounds (du limits) — from ActuatorRegistry
  - Exclusive pair constraints (e.g., humidifier + dehumidifier)
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

    # Actuator pairs that should never run simultaneously.
    # Enforced via linear constraint u_a + u_b <= max_sum in the QP,
    # plus post-solve clamping to enforce true mutual exclusivity.
    # max_sum=100 means the pair shares a 0-100% budget: if one runs
    # at 80%, the other is capped at 20%. The post-solve clamp then
    # zeros the smaller one for true mutual exclusivity.
    # Each tuple is (channel_a, channel_b, max_sum).
    EXCLUSIVE_PAIRS = [
        ('u_hum', 'u_dehum', 100.0),
        ('u_heat', 'u_cool', 100.0),
    ]

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

        # Detect exclusive pairs present in this profile
        names = registry.channel_names
        self.exclusive_pairs = []  # list of (idx_a, idx_b, max_sum)
        for ch_a, ch_b, max_sum in self.EXCLUSIVE_PAIRS:
            if ch_a in names and ch_b in names:
                self.exclusive_pairs.append((names.index(ch_a), names.index(ch_b), max_sum))

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

    @property
    def n_exclusive(self):
        """Number of active exclusive pairs."""
        return len(self.exclusive_pairs)

    def get_exclusive_constraints(self, Nc, n_dec):
        """Build exclusive pair inequality rows: u_a[k] + u_b[k] <= max_sum.

        Returns:
            A_excl: (n_exclusive * Nc, n_dec) constraint matrix.
            l_excl: lower bounds (-inf).
            u_excl: upper bounds (max_sum per row).
        """
        n_rows = self.n_exclusive * Nc
        if n_rows == 0:
            return np.zeros((0, n_dec)), np.zeros(0), np.zeros(0)

        A_excl = np.zeros((n_rows, n_dec))
        l_excl = np.full(n_rows, -np.inf)
        u_excl = np.zeros(n_rows)

        row = 0
        for idx_a, idx_b, max_sum in self.exclusive_pairs:
            for k in range(Nc):
                col_a = k * self.n_u + idx_a
                col_b = k * self.n_u + idx_b
                A_excl[row, col_a] = 1.0
                A_excl[row, col_b] = 1.0
                u_excl[row] = max_sum
                row += 1

        return A_excl, l_excl, u_excl

    def clamp_exclusive(self, u):
        """Post-solve clamp: zero out the smaller actuator in each exclusive pair.

        The QP constraint u_a + u_b <= 100 limits overlap but can't enforce
        true mutual exclusivity (that's non-convex). This clamp enforces it
        after the solve. The MPC re-solves every tick, so it adapts immediately.
        """
        for idx_a, idx_b, _ in self.exclusive_pairs:
            if u[idx_a] > 0 and u[idx_b] > 0:
                if u[idx_a] >= u[idx_b]:
                    u[idx_b] = 0.0
                else:
                    u[idx_a] = 0.0
        return u
