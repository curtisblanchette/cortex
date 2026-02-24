"""Fail-safe manager for graceful degradation.

Six priority levels:
  P1: MPC solver + gateway running normally — full optimization
  P2: MPC solve fails (status != 'solved') — hold last valid u, confidence=0.5
  P3: MPC solver offline (stale solution) — gateway holds last commands
  P4: Gateway falls to ESP32 PID (stale > 5 min)
  P5: ESP32 offline → TrolMaster static setpoints
  P6: Total loss → HVAC built-in safety limits

This class manages P1 → P3 transitions.
P4-P6 are handled by firmware and external equipment.

Staleness detection (P3) uses time-based approach: if the solution
age exceeds 2x the solver's declared next_solve_s, it's stale.
"""

import time


class FailSafeManager:
    """Monitors system health and manages degradation."""

    def __init__(self, max_hold_seconds=30, stale_threshold_s=120,
                 esp32_fallback_s=300):
        # Counter-based (solver process uses this)
        self.consecutive_failures = 0
        self.max_hold_seconds = max_hold_seconds

        # Time-based staleness (gateway uses this)
        self.stale_threshold_s = stale_threshold_s
        self.esp32_fallback_s = esp32_fallback_s

        # Shared state
        self.last_valid_u = None
        self.last_valid_time = None
        self.current_level = 'P1_NORMAL'

    def check_solve(self, result_status, u_opt, solve_info):
        """Check MPC solve result and return appropriate control action.

        Used by the solver process after each OSQP solve.

        Args:
            result_status: OSQP status string.
            u_opt: Optimal control from solver (may be None on failure).
            solve_info: Dict with solve diagnostics.

        Returns:
            u: Control action to apply.
            level: Current fail-safe level string.
        """
        if result_status in ('solved', 'solved_inaccurate',
                             'solved inaccurate'):
            self.consecutive_failures = 0
            self.last_valid_u = u_opt.copy()
            self.last_valid_time = time.time()
            self.current_level = 'P1_NORMAL'
            return u_opt, 'P1_NORMAL'

        self.consecutive_failures += 1

        if self.consecutive_failures <= self.max_hold_seconds and self.last_valid_u is not None:
            self.current_level = 'P2_HOLD_LAST'
            return self.last_valid_u.copy(), 'P2_HOLD_LAST'

        # P2 exhausted → fall to P3
        self.current_level = 'P3_SOLVER_OFFLINE'
        return None, 'P3_SOLVER_OFFLINE'

    def check_staleness(self, solution_age_s, expected_interval_s=None):
        """Check solution staleness (used by the gateway).

        Args:
            solution_age_s: Seconds since last valid solution was received.
            expected_interval_s: The solver's declared next_solve_s.
                If None, uses self.stale_threshold_s.

        Returns:
            level: Current fail-safe level string.
            is_stale: Whether the solution is considered stale.
        """
        if expected_interval_s is not None:
            threshold = expected_interval_s * 2.0
        else:
            threshold = self.stale_threshold_s

        if solution_age_s <= threshold:
            return 'P1_NORMAL', False

        if solution_age_s <= self.esp32_fallback_s:
            return 'P3_SOLVER_OFFLINE', True

        # Stale for too long — fall to ESP32 PID
        return 'P4_ESP32_PID', True

    def reset(self):
        """Reset after recovery."""
        self.consecutive_failures = 0
        self.current_level = 'P1_NORMAL'
