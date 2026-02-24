"""Adaptive solve trigger logic.

Determines when the MPC solver should re-solve instead of using a
fixed 1 Hz cadence. Triggers include:

- Timer expired (default 30s steady-state, configurable)
- Large EKF innovation (sensor reading far from prediction)
- Light schedule transition within lookahead window
- Irrigation event detected
- Phase/reference change
- Manual re-solve request via API
"""

import time


class SolveTrigger:
    """Evaluates whether the MPC solver should re-solve now.

    The solver calls check() in its main loop. If any trigger fires,
    the solver re-runs OSQP. Otherwise it sleeps until the next check.
    """

    def __init__(self, config=None):
        cfg = config or {}

        # Timer settings
        self.steady_state_interval = cfg.get('steady_state_interval_s', 30.0)
        self.deep_steady_interval = cfg.get('deep_steady_interval_s', 60.0)
        self.min_interval = cfg.get('min_interval_s', 5.0)

        # EKF innovation threshold (normalized RMS)
        self.innovation_threshold = cfg.get('innovation_threshold', 2.0)

        # Light transition lookahead (seconds)
        self.light_lookahead_s = cfg.get('light_lookahead_s', 300.0)  # 5 min

        # State
        self.last_solve_time = 0.0
        self._manual_trigger = False
        self._phase_changed = False
        self._last_innovation_rms = 0.0
        self._consecutive_calm = 0  # Ticks with low innovation
        self._calm_threshold = 10   # After N calm checks, use deep_steady_interval

    def check(self, now=None, innovation_rms=0.0,
              light_schedule=None, current_time_s=0.0):
        """Check all triggers and return whether to solve.

        Args:
            now: Wall-clock time (time.time()). Auto if None.
            innovation_rms: Normalized EKF innovation magnitude.
            light_schedule: LightSchedule instance (for transition check).
            current_time_s: Simulation/process time for light schedule eval.

        Returns:
            (should_solve, trigger_name, next_interval)
        """
        if now is None:
            now = time.time()

        elapsed = now - self.last_solve_time

        # Enforce minimum interval to prevent solver spam
        if elapsed < self.min_interval:
            return False, None, self.min_interval - elapsed

        # Trigger 1: Manual request (API, operator)
        if self._manual_trigger:
            self._manual_trigger = False
            return True, 'manual', self.steady_state_interval

        # Trigger 2: Phase/reference change
        if self._phase_changed:
            self._phase_changed = False
            return True, 'phase_change', self.steady_state_interval

        # Trigger 3: Large EKF innovation (disturbance detected)
        self._last_innovation_rms = innovation_rms
        if innovation_rms > self.innovation_threshold:
            self._consecutive_calm = 0
            return True, 'innovation', self.steady_state_interval

        # Trigger 4: Light transition approaching
        if light_schedule is not None:
            if self._light_transition_near(light_schedule, current_time_s):
                return True, 'light_transition', self.steady_state_interval

        # Trigger 5: Timer expired
        interval = self._current_interval()
        if elapsed >= interval:
            return True, 'timer', interval

        # No trigger — report time until next timer
        remaining = interval - elapsed
        return False, None, remaining

    def record_solve(self, now=None):
        """Record that a solve just completed."""
        self.last_solve_time = now if now is not None else time.time()

        # Track calm periods
        if self._last_innovation_rms < self.innovation_threshold * 0.5:
            self._consecutive_calm += 1
        else:
            self._consecutive_calm = 0

    def request_solve(self):
        """External trigger: request an immediate re-solve."""
        self._manual_trigger = True

    def notify_phase_change(self):
        """Notify that the growth phase or reference targets changed."""
        self._phase_changed = True

    def _current_interval(self):
        """Get current solve interval based on recent activity."""
        if self._consecutive_calm >= self._calm_threshold:
            return self.deep_steady_interval
        return self.steady_state_interval

    def _light_transition_near(self, light_schedule, current_time_s):
        """Check if a light on/off transition is within the lookahead window."""
        now_on = light_schedule.is_on(current_time_s)
        future_on = light_schedule.is_on(current_time_s + self.light_lookahead_s)
        return now_on != future_on

    @property
    def next_solve_hint_s(self):
        """Estimated seconds until next solve (for Redis publication)."""
        return self._current_interval()
