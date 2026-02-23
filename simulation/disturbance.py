"""Disturbance scheduler for simulation.

Generates realistic disturbance profiles: lighting schedule, outdoor
weather, plant count, and growth phase. These are the things that
happen TO the room that the controller must respond to.

Disturbance vector d (5):
    d[0] Q_light   Lighting heat load          W
    d[1] T_out     Outdoor temperature          °C
    d[2] w_out     Outdoor absolute humidity    g/kg
    d[3] N_plants  Plant count                  -
    d[4] phase     Growth phase (int enum)      -
"""

import numpy as np


# Phase enum
PHASES = {
    'clone': 0,
    'veg': 1,
    'early_flower': 2,
    'mid_flower': 3,
    'late_flower': 4,
    'dry': 5,
}


class DisturbanceScheduler:
    """Generates disturbance profiles for simulation."""

    def __init__(self, config):
        room = config if isinstance(config, dict) and 'light_schedule' in config else config.get('room', config)

        ls = room.get('light_schedule', {'on_hour': 6.0, 'off_hour': 18.0})
        self.on_hour = ls['on_hour']
        self.off_hour = ls['off_hour']
        self.light_power_W = room.get('light_power_W', 50000.0)
        self.par_efficiency = room.get('par_efficiency', 0.4)

        outdoor = room.get('outdoor', {})
        self.T_out_mean = outdoor.get('T_out_mean_C', 28.0)
        self.T_out_amplitude = outdoor.get('T_out_amplitude_C', 5.0)
        self.w_out = outdoor.get('w_out_gkg', 12.0)

        self.N_plants = room.get('N_plants', 100)
        self.phase = room.get('phase', 'mid_flower')
        self._phase_int = PHASES.get(self.phase, 3)

        # Light transition (ramp over 60s to avoid step discontinuity)
        self._ramp_s = 60.0

    def get(self, time_s):
        """Return disturbance vector d at simulation time.

        Args:
            time_s: Simulation time in seconds from start.

        Returns:
            d: Disturbance vector (5,).
        """
        hour = (time_s / 3600.0) % 24.0

        # Light schedule with ramp
        Q_light = self._light_heat(hour)

        # Outdoor temperature (sinusoidal daily cycle, peak at 2pm)
        T_out = self.T_out_mean + self.T_out_amplitude * np.sin(
            2.0 * np.pi * (hour - 14.0) / 24.0
        )

        return np.array([Q_light, T_out, self.w_out, self.N_plants, self._phase_int])

    def _light_heat(self, hour):
        """Compute lighting heat load with smooth ramp transitions."""
        # Heat portion of lighting power
        Q_max = self.light_power_W * (1.0 - self.par_efficiency)

        # Distance from on/off transitions in hours
        ramp_hr = self._ramp_s / 3600.0

        if self.on_hour <= self.off_hour:
            # Normal schedule (e.g., 6am-6pm)
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
            # Overnight schedule (e.g., 6pm-6am)
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

    def forecast(self, time_s, horizon_s, step_s=60.0):
        """Generate disturbance forecast over a prediction horizon.

        Args:
            time_s: Current simulation time in seconds.
            horizon_s: Forecast horizon in seconds.
            step_s: Step size in seconds.

        Returns:
            d_forecast: Array of shape (N_steps, 5).
        """
        n_steps = int(horizon_s / step_s)
        forecast = np.zeros((n_steps, 5))
        for k in range(n_steps):
            forecast[k] = self.get(time_s + k * step_s)
        return forecast

    def is_lights_on(self, time_s):
        """Check if lights are on at given time."""
        hour = (time_s / 3600.0) % 24.0
        if self.on_hour <= self.off_hour:
            return self.on_hour <= hour < self.off_hour
        else:
            return hour >= self.on_hour or hour < self.off_hour
