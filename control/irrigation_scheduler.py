"""Irrigation Scheduler — shot-based binary irrigation.

Fires fixed-duration shots based on theta depletion. Irrigation is a
feedforward disturbance (not an MPC control variable) — the scheduler
sets self.current_rate which the control loop injects into the
disturbance vector d[5] before physics integration.

State machine:
    IDLE ──[in window]──> WAITING ──[theta ≤ trigger]──> FIRING ──[duration elapsed]──> COOLDOWN
      ^                     |                                                              |
      |                     |<─────────────────────[settle elapsed]─────────────────────────┘
      |<───[window closes]──┘
      |<───[lights off from any state]
"""

from dataclasses import dataclass
from enum import Enum, auto

import numpy as np


class IrrState(Enum):
    IDLE = auto()
    WAITING = auto()
    FIRING = auto()
    COOLDOWN = auto()


@dataclass
class IrrigationConfig:
    """Phase-specific irrigation schedule parameters."""
    enabled: bool
    first_shot_delay_s: float
    last_shot_cutoff_s: float
    min_interval_s: float
    theta_trigger_fraction: float   # fire when theta < target - fraction * AWC
    shot_duration_s: float
    max_daily_shots: int
    drain_settle_s: float


class IrrigationScheduler:
    """Shot-based irrigation state machine.

    Ticks each sim step, reads theta from EKF state, and sets
    self.current_rate (m³/m³/s theta injection rate) for the
    disturbance vector.
    """

    def __init__(self, config, substrate, light_schedule,
                 flow_rate_ml_s, N_plants, container_volume_L,
                 canopy_area_m2, theta_target=None):
        """
        Args:
            config: IrrigationConfig with phase-specific timing.
            substrate: SubstrateProfile (for theta_fc, theta_wp).
            light_schedule: LightSchedule instance.
            flow_rate_ml_s: Manifold flow rate (ml/s at 100%).
            N_plants: Plant count.
            container_volume_L: Per-plant container volume in liters.
            canopy_area_m2: Canopy area (used for logging only).
            theta_target: Resolved day theta target. Trigger is computed
                          relative to this so irrigation oscillates around
                          the target, not around theta_fc.
        """
        self.config = config
        self.substrate = substrate
        self.light_schedule = light_schedule
        self.flow_rate_ml_s = flow_rate_ml_s
        self.canopy_area_m2 = canopy_area_m2

        # Compute the theta injection rate when firing (m³/m³/s)
        V_sub = container_volume_L * N_plants / 1000.0  # m³ total substrate
        self._irr_rate = (flow_rate_ml_s / 1e6) / V_sub  # m³/m³/s

        # Theta trigger: dryback below target by fraction * AWC
        awc = substrate.theta_fc - substrate.theta_wp
        if theta_target is not None:
            self.theta_trigger = theta_target - config.theta_trigger_fraction * awc
        else:
            self.theta_trigger = substrate.theta_fc * (1.0 - config.theta_trigger_fraction)
        self.theta_trigger = max(self.theta_trigger, substrate.theta_wp)

        # State machine
        self._state = IrrState.IDLE
        self._shot_start_time = None
        self._cooldown_start_time = None
        self._last_shot_start_time = None   # for min_interval enforcement
        self._daily_shots = 0
        self._last_lights_on = None         # for daily reset detection

        # Current irrigation rate — read by control loop for d[5]
        self.current_rate = 0.0

        # Telemetry
        self.shot_log = []  # list of (start_time_s, end_time_s)

    @property
    def state(self):
        return self._state

    def tick(self, x_hat, current_time_s):
        """Main loop entry — call each sim tick.

        Reads theta from x_hat[4], sets self.current_rate.
        """
        self._maybe_reset_daily(current_time_s)

        lights_on = self.light_schedule.is_on(current_time_s)

        # If lights are off, force back to IDLE (except FIRING completes)
        if not lights_on and self._state != IrrState.FIRING:
            self._state = IrrState.IDLE
            self.current_rate = 0.0
            return

        theta = x_hat[4]

        if self._state == IrrState.IDLE:
            self._handle_idle(current_time_s, lights_on)
        elif self._state == IrrState.WAITING:
            self._handle_waiting(theta, current_time_s, lights_on)
        elif self._state == IrrState.FIRING:
            self._handle_firing(current_time_s)
        elif self._state == IrrState.COOLDOWN:
            self._handle_cooldown(current_time_s)

    def _handle_idle(self, current_time_s, lights_on):
        self.current_rate = 0.0
        if lights_on and self._in_irrigation_window(current_time_s, lights_on):
            self._state = IrrState.WAITING

    def _handle_waiting(self, theta, current_time_s, lights_on):
        self.current_rate = 0.0

        # Window closed?
        if not self._in_irrigation_window(current_time_s, lights_on):
            self._state = IrrState.IDLE
            return

        # Daily cap reached?
        if self._daily_shots >= self.config.max_daily_shots:
            return

        # Min interval check
        if (self._last_shot_start_time is not None
                and (current_time_s - self._last_shot_start_time) < self.config.min_interval_s):
            return

        # Theta trigger
        if theta <= self.theta_trigger:
            self._state = IrrState.FIRING
            self._shot_start_time = current_time_s
            self._last_shot_start_time = current_time_s
            self._daily_shots += 1
            self.current_rate = self._irr_rate

    def _handle_firing(self, current_time_s):
        self.current_rate = self._irr_rate
        elapsed = current_time_s - self._shot_start_time
        if elapsed >= self.config.shot_duration_s:
            # Shot complete — log it
            self.shot_log.append((self._shot_start_time, current_time_s))
            self._state = IrrState.COOLDOWN
            self._cooldown_start_time = current_time_s
            self.current_rate = 0.0

    def _handle_cooldown(self, current_time_s):
        self.current_rate = 0.0
        elapsed = current_time_s - self._cooldown_start_time
        if elapsed >= self.config.drain_settle_s:
            self._state = IrrState.WAITING

    def _in_irrigation_window(self, time_s, lights_on):
        """Check if current time is within the irrigation window.

        Window opens first_shot_delay_s after lights-on,
        closes last_shot_cutoff_s before lights-off.
        """
        if not lights_on:
            return False

        hour = (time_s / 3600.0) % 24.0
        on_h = self.light_schedule.on_hour
        off_h = self.light_schedule.off_hour

        # Hours since lights-on
        hours_since_on = (hour - on_h) % 24.0
        secs_since_on = hours_since_on * 3600.0

        # Hours until lights-off
        hours_until_off = (off_h - hour) % 24.0
        secs_until_off = hours_until_off * 3600.0

        return (secs_since_on >= self.config.first_shot_delay_s
                and secs_until_off >= self.config.last_shot_cutoff_s)

    def _maybe_reset_daily(self, time_s):
        """Reset daily shot count on lights-on transition."""
        lights_on = self.light_schedule.is_on(time_s)
        if self._last_lights_on is not None and lights_on and not self._last_lights_on:
            self._daily_shots = 0
        self._last_lights_on = lights_on

    def summary(self):
        """Return a summary dict for end-of-sim reporting."""
        total_shots = len(self.shot_log)
        if total_shots == 0:
            return {
                'total_shots': 0,
                'avg_duration_s': 0.0,
                'total_irrigation_s': 0.0,
                'theta_trigger': self.theta_trigger,
                'shots': [],
            }
        durations = [end - start for start, end in self.shot_log]
        return {
            'total_shots': total_shots,
            'avg_duration_s': np.mean(durations),
            'total_irrigation_s': sum(durations),
            'theta_trigger': self.theta_trigger,
            'shots': [(round(s / 3600, 2), round(e / 3600, 2))
                      for s, e in self.shot_log],
        }
