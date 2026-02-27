"""Disturbance forecast for MPC prediction horizon.

Predicts disturbances over the horizon so OSQP can account for
known future events (like lights turning on/off).

Some disturbances are known exactly (light schedule).
Some are forecasted (outdoor weather).
Some are effectively constant (plant count, phase).
"""

import numpy as np

from core.dynamics import N_DISTURBANCES


class DisturbanceForecast:
    """Generates disturbance forecasts for the MPC solver."""

    def __init__(self, room_config):
        room = room_config if isinstance(room_config, dict) and 'light_schedule' in room_config else room_config.get('room', room_config)

        ls = room.get('light_schedule', {'on_hour': 6.0, 'off_hour': 18.0})
        self.on_hour = ls['on_hour']
        self.off_hour = ls['off_hour']
        self.light_power_W = room.get('light_power_W', 50000.0)
        self.par_efficiency = room.get('par_efficiency', 0.4)
        self.n_d = N_DISTURBANCES

        # Ramp duration matching simulation physics (avoid step discontinuity)
        self._ramp_s = 60.0

    def generate(self, current_time_s, current_d, horizon_steps=60, step_s=60.0):
        """Generate disturbance forecast.

        Args:
            current_time_s: Current time in seconds.
            current_d: Current disturbance vector (5,).
            horizon_steps: Number of steps in prediction horizon.
            step_s: Seconds per prediction step.

        Returns:
            forecast: Disturbance forecast (horizon_steps, 5).
        """
        forecast = np.zeros((horizon_steps, self.n_d))

        for k in range(horizon_steps):
            future_time = current_time_s + k * step_s

            # Lighting: persistence forecast (hold current level).
            # The transpiration model creates a strong nonlinearity at
            # Q_light=0 that makes linearization across light transitions
            # produce wrong-signed Jacobians.  Holding the current level
            # keeps linearization valid; the cold start at the actual
            # transition re-linearizes at the new operating point.
            forecast[k, 0] = current_d[0]

            # Outdoor weather: persistence forecast (hold current)
            forecast[k, 1] = current_d[1]  # T_out
            forecast[k, 2] = current_d[2]  # w_out

            # Plant/phase: constant over horizon
            forecast[k, 3] = current_d[3]  # N_plants
            forecast[k, 4] = current_d[4]  # phase

        return forecast

    def _light_heat(self, time_s):
        """Compute lighting heat load with smooth ramp transitions.

        Matches the simulation's DisturbanceScheduler ramp to avoid
        plant/model mismatch at light transitions.
        """
        hour = (time_s / 3600.0) % 24.0
        Q_max = self.light_power_W * (1.0 - self.par_efficiency)
        ramp_hr = self._ramp_s / 3600.0

        if self.on_hour <= self.off_hour:
            if hour < self.on_hour - ramp_hr or hour > self.off_hour + ramp_hr:
                return 0.0
            elif hour < self.on_hour + ramp_hr:
                frac = (hour - self.on_hour + ramp_hr) / (2.0 * ramp_hr)
                return Q_max * np.clip(frac, 0.0, 1.0)
            elif hour > self.off_hour - ramp_hr:
                frac = (self.off_hour + ramp_hr - hour) / (2.0 * ramp_hr)
                return Q_max * np.clip(frac, 0.0, 1.0)
            else:
                return Q_max
        else:
            if self.off_hour + ramp_hr < hour < self.on_hour - ramp_hr:
                return 0.0
            elif self.on_hour - ramp_hr <= hour <= self.on_hour + ramp_hr:
                frac = (hour - self.on_hour + ramp_hr) / (2.0 * ramp_hr)
                return Q_max * np.clip(frac, 0.0, 1.0)
            elif self.off_hour - ramp_hr <= hour <= self.off_hour + ramp_hr:
                frac = (self.off_hour + ramp_hr - hour) / (2.0 * ramp_hr)
                return Q_max * np.clip(frac, 0.0, 1.0)
            else:
                return Q_max
