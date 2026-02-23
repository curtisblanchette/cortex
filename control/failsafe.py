"""Fail-safe manager for graceful degradation.

Five priority levels:
  P1: MPC normal → full optimization
  P2: MPC fails  → hold last valid u for 30s
  P3: MPC offline → ESP32 local PID on cached setpoints
  P4: ESP32 offline → TrolMaster static setpoints
  P5: Total loss → HVAC built-in safety limits

This class manages P1 → P2 transitions.
P3-P5 are handled by firmware and external equipment.
"""

import time


class FailSafeManager:
    """Monitors system health and manages degradation."""

    def __init__(self, max_hold_seconds=30):
        self.consecutive_failures = 0
        self.last_valid_u = None
        self.last_valid_time = None
        self.max_hold_seconds = max_hold_seconds
        self.current_level = 'P1_NORMAL'

    def check_solve(self, result_status, u_opt, solve_info):
        """Check MPC solve result and return appropriate control action.

        Args:
            result_status: OSQP status string.
            u_opt: Optimal control from solver (may be None on failure).
            solve_info: Dict with solve diagnostics.

        Returns:
            u: Control action to apply.
            level: Current fail-safe level string.
        """
        if result_status in ('solved', 'solved_inaccurate'):
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
        self.current_level = 'P3_ESP32_PID'
        return None, 'P3_ESP32_PID'

    def reset(self):
        """Reset after recovery."""
        self.consecutive_failures = 0
        self.current_level = 'P1_NORMAL'
