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
        self._channel_names = list(registry.channel_names)

        # Per-pair previous winner ('a' / 'b' / None) and observability counters.
        # The MPC's QP cost structure already produces stable winner selection
        # (R penalises ‖Δu‖², B is re-linearized every solve, clamp re-runs at
        # 1 Hz). Measurement on a 12hr ac_dehum sim showed no sawtooth: median
        # pre-clamp gap is 0.93 (95/5 splits, not tie-breaking), and the flips
        # that do occur are regime shifts (median gap 0.74), not numerical
        # noise. These counters surface the behaviour for production
        # observability so a real deviation would be visible.
        # Keyed by (idx_a, idx_b) for stability across reconfiguration.
        self._prev_winner = {}
        self._fire_count = {}    # solves where both pre-clamp values > 0
        self._flip_count = {}    # solves where the winner changed

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

        # Implicit exclusion: HVACD dehumidification (via RH_sp) vs humidifier.
        # When the HVACD would dehumidify (room RH > solved RH_sp), running
        # the humidifier is wasteful — they fight each other.
        self._hvacd_hum_pair = None
        if 'RH_sp' in names and 'u_hum' in names:
            self._hvacd_hum_pair = (names.index('RH_sp'), names.index('u_hum'))

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

        Tracks per-pair fire and flip counts for telemetry; read via
        `clamp_metrics()` and reset via `reset_clamp_metrics()`. See
        `docs/decisions-recent.md` "Exclusive-pair clamp measurement" for
        the data showing no sawtooth in current sims.
        """
        for idx_a, idx_b, _ in self.exclusive_pairs:
            key = (idx_a, idx_b)
            a, b = u[idx_a], u[idx_b]
            if a <= 0 or b <= 0:
                continue

            self._fire_count[key] = self._fire_count.get(key, 0) + 1

            winner = 'a' if a >= b else 'b'
            if winner == 'a':
                u[idx_b] = 0.0
            else:
                u[idx_a] = 0.0

            prev = self._prev_winner.get(key)
            if prev is not None and prev != winner:
                self._flip_count[key] = self._flip_count.get(key, 0) + 1
            self._prev_winner[key] = winner
        return u

    def clamp_metrics(self):
        """Return per-pair clamp activity counters.

        Returns a dict keyed by `'<channel_a>__<channel_b>'` with `fires`
        (solves where both pre-clamp > 0) and `flips` (winner changes).
        Used by SolverProcess to surface the counters in the per-tick info
        dict and by the regression test in `tests/test_clamp_oscillation.py`.
        """
        out = {}
        for idx_a, idx_b, _ in self.exclusive_pairs:
            key = (idx_a, idx_b)
            name = f'{self._channel_names[idx_a]}__{self._channel_names[idx_b]}'
            out[name] = {
                'fires': self._fire_count.get(key, 0),
                'flips': self._flip_count.get(key, 0),
                'winner': self._prev_winner.get(key),
            }
        return out

    def reset_clamp_metrics(self):
        """Zero the fire/flip counters. Used between sim profiles in
        scripts/measure_clamp_oscillation.py and at the start of tests."""
        self._fire_count.clear()
        self._flip_count.clear()
        self._prev_winner.clear()

    def apply_mode_constraints(self, x_hat, u_prev):
        """Pre-solve bound tightening based on current operating mode.

        When the HVACD is dehumidifying (room RH > previous RH_sp),
        lock the humidifier to 0 so the QP doesn't fight itself.
        The MPC optimizes knowing the humidifier is unavailable.

        Call before solve; call release_mode_constraints() after.
        """
        self._mode_overrides = {}

        if self._hvacd_hum_pair is None:
            return

        rh_sp_idx, hum_idx = self._hvacd_hum_pair
        from core.thermodynamics import saturation_vapor_pressure, P_ATM
        T_a, w_a = x_hat[0], x_hat[1]
        SVP = saturation_vapor_pressure(T_a)
        e_air = w_a * P_ATM / (622.0 + w_a)
        current_RH = e_air / SVP * 100.0 if SVP > 0 else 50.0

        if current_RH > u_prev[rh_sp_idx]:
            # HVACD is dehumidifying — lock humidifier off for this solve
            self._mode_overrides[hum_idx] = (self.u_min[hum_idx], self.u_max[hum_idx])
            self.u_min[hum_idx] = 0.0
            self.u_max[hum_idx] = 0.0

    def release_mode_constraints(self):
        """Restore bounds after solve."""
        for idx, (lo, hi) in self._mode_overrides.items():
            self.u_min[idx] = lo
            self.u_max[idx] = hi
        self._mode_overrides = {}

    def lock_channel(self, channel_name, value=0.0):
        """Lock a control channel to a fixed value.

        Sets u_min, u_max to `value` and du_min, du_max to 0 so the QP
        cannot move the channel. Used by the irrigation scheduler to prevent
        MPC from optimizing u_irr while retaining the B-matrix column.

        Args:
            channel_name: Name of the channel (e.g. 'u_irr').
            value: Fixed value to lock at (default 0.0).
        """
        if channel_name not in self._channel_names:
            return
        idx = self._channel_names.index(channel_name)
        self.u_min[idx] = value
        self.u_max[idx] = value
        self.du_min[idx] = 0.0
        self.du_max[idx] = 0.0
