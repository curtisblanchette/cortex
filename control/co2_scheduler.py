"""CO2 Scheduler — bang-bang solenoid controller with deadband hysteresis.

Replaces MPC-optimized continuous CO2 injection with a binary state machine
that opens/closes a solenoid based on CO2 level vs. target. The MPC retains
the u_co2 column (locked to 0) for trajectory prediction accuracy.

State machine:
    IDLE ──[lights on]──> READY ──[CO2 < target - deadband]──> INJECTING ──[CO2 ≥ target]──> HOLDOFF
      ^                     |                                                                    |
      |                     |<───────────────────────[holdoff elapsed]────────────────────────────┘
      |<───[lights off]─────┘
"""

from collections import deque
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

from mpc.reference import REF_CO2


class CO2State(Enum):
    IDLE = auto()
    READY = auto()
    INJECTING = auto()
    HOLDOFF = auto()


@dataclass
class CO2Config:
    """CO2 scheduler parameters."""
    deadband_ppm: float = 50.0
    min_off_s: float = 30.0
    expected_flow_ppm_s: float = 10.0


class CO2Scheduler:
    """Bang-bang CO2 solenoid controller.

    Ticks each sim step, reads CO2 from EKF state, and writes
    u_co2 = 0 or 100 into the control vector in-place.
    """

    def __init__(self, config, registry, light_schedule, reference):
        """
        Args:
            config: CO2Config with deadband and holdoff parameters.
            registry: ActuatorRegistry (for u_co2 channel index).
            light_schedule: LightSchedule instance.
            reference: ReferenceGenerator (for CO2 target lookup).
        """
        self.config = config
        self.light_schedule = light_schedule
        self.reference = reference

        # Find u_co2 index in the control vector
        names = registry.channel_names
        if 'u_co2' not in names:
            raise ValueError("CO2Scheduler requires u_co2 in registry")
        self._co2_idx = names.index('u_co2')

        # State machine
        self._state = CO2State.IDLE
        self._inject_start_time = None
        self._holdoff_start_time = None

        # Telemetry
        self.injection_log = []  # list of (start_time_s, end_time_s)
        self._current_inject_start = None

        # Flow-rate diagnostics: rolling buffer of (time_s, co2_ppm) during injection
        self._injection_readings = deque(maxlen=60)
        self._recent_dc_dt = []  # computed dC/dt values from recent injections

    @property
    def state(self):
        return self._state

    def _get_co2_target(self, current_time_s):
        """Get the current CO2 target from the reference generator."""
        targets = self.reference._targets_at_time(current_time_s, self.light_schedule)
        return targets[REF_CO2]

    def tick(self, x_hat, current_time_s, u_vector):
        """Main loop entry — call each sim tick.

        Reads CO2 from x_hat[2], modifies u_vector[co2_idx] in-place.
        """
        lights_on = self.light_schedule.is_on(current_time_s)
        co2 = x_hat[2]

        # Lights off → force to IDLE
        if not lights_on:
            if self._state == CO2State.INJECTING:
                self._end_injection(current_time_s)
            self._state = CO2State.IDLE
            u_vector[self._co2_idx] = 0.0
            return

        target = self._get_co2_target(current_time_s)

        if self._state == CO2State.IDLE:
            self._handle_idle(lights_on, u_vector)
        elif self._state == CO2State.READY:
            self._handle_ready(co2, target, current_time_s, u_vector)
        elif self._state == CO2State.INJECTING:
            self._handle_injecting(co2, target, current_time_s, u_vector)
        elif self._state == CO2State.HOLDOFF:
            self._handle_holdoff(current_time_s, u_vector)

    def _handle_idle(self, lights_on, u_vector):
        u_vector[self._co2_idx] = 0.0
        if lights_on:
            self._state = CO2State.READY

    def _handle_ready(self, co2, target, current_time_s, u_vector):
        u_vector[self._co2_idx] = 0.0
        trigger = target - self.config.deadband_ppm
        if co2 < trigger:
            self._state = CO2State.INJECTING
            self._inject_start_time = current_time_s
            self._current_inject_start = current_time_s
            self._injection_readings.clear()
            self._injection_readings.append((current_time_s, co2))
            u_vector[self._co2_idx] = 100.0

    def _handle_injecting(self, co2, target, current_time_s, u_vector):
        u_vector[self._co2_idx] = 100.0
        self._injection_readings.append((current_time_s, co2))

        if co2 >= target:
            self._end_injection(current_time_s)
            self._state = CO2State.HOLDOFF
            self._holdoff_start_time = current_time_s
            u_vector[self._co2_idx] = 0.0

    def _handle_holdoff(self, current_time_s, u_vector):
        u_vector[self._co2_idx] = 0.0
        elapsed = current_time_s - self._holdoff_start_time
        if elapsed >= self.config.min_off_s:
            self._state = CO2State.READY

    def _end_injection(self, end_time_s):
        """Log completed injection and compute dC/dt."""
        if self._current_inject_start is not None:
            self.injection_log.append((self._current_inject_start, end_time_s))

            # Compute observed dC/dt from readings
            if len(self._injection_readings) >= 2:
                readings = list(self._injection_readings)
                t0, c0 = readings[0]
                t1, c1 = readings[-1]
                dt = t1 - t0
                if dt > 0:
                    dc_dt = (c1 - c0) / dt
                    self._recent_dc_dt.append(dc_dt)
                    # Keep last 20 measurements
                    if len(self._recent_dc_dt) > 20:
                        self._recent_dc_dt.pop(0)

        self._current_inject_start = None
        self._injection_readings.clear()

    def observed_flow_rate(self):
        """Return average observed dC/dt (ppm/s) during recent injections.

        Returns None if no data available.
        """
        if not self._recent_dc_dt:
            return None
        return float(np.mean(self._recent_dc_dt))

    def summary(self):
        """Return a summary dict for end-of-sim reporting."""
        total_cycles = len(self.injection_log)
        if total_cycles == 0:
            return {
                'total_cycles': 0,
                'avg_cycle_s': 0.0,
                'total_on_time_s': 0.0,
                'observed_flow_ppm_s': None,
                'cycles': [],
            }
        durations = [end - start for start, end in self.injection_log]
        return {
            'total_cycles': total_cycles,
            'avg_cycle_s': float(np.mean(durations)),
            'total_on_time_s': sum(durations),
            'observed_flow_ppm_s': self.observed_flow_rate(),
            'cycles': [(round(s / 3600, 2), round(e / 3600, 2))
                       for s, e in self.injection_log],
        }
