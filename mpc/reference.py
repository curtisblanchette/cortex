"""Reference trajectory generator for MPC.

Produces the target trajectory r[k] over the prediction horizon.
Changes when growth phase changes, light state changes, or grower
overrides a target. Pre-computed and cached.
"""

import numpy as np
import yaml


# Output indices in the reference trajectory
# These are the controlled variables that MPC tracks
REF_VPD = 0
REF_T = 1
REF_CO2 = 2
REF_DRYBACK = 3
REF_THETA = 4
REF_RH = 5

N_OUTPUTS = 6

OUTPUT_NAMES = ['VPD', 'T', 'CO2', 'dryback_rate', 'theta', 'RH']


class LightSchedule:
    """Simple light on/off schedule."""

    def __init__(self, on_hour, off_hour):
        self.on_hour = on_hour
        self.off_hour = off_hour

    def is_on(self, time_s):
        """Check if lights are on at given time."""
        hour = (time_s / 3600.0) % 24.0
        if self.on_hour <= self.off_hour:
            return self.on_hour <= hour < self.off_hour
        else:
            return hour >= self.on_hour or hour < self.off_hour


class ReferenceGenerator:
    """Produces target trajectories for MPC.

    Each step in the trajectory contains targets for the 5 controlled outputs.
    Handles day/night transitions with configurable ramp rates.
    """

    def __init__(self, phase_config, cultivar_config=None, registry=None,
                 substrate=None):
        """Initialize from phase and cultivar configs.

        Args:
            phase_config: Dict from phases/*.yaml with day/night targets.
            cultivar_config: Dict from cultivar/*.yaml with offsets.
            registry: ActuatorRegistry — used to build R/S weights that
                      match the active profile's control vector layout.
            substrate: SubstrateProfile — theta targets are clamped to
                       [theta_wp, theta_fc] so unreachable setpoints
                       don't cause constant irrigation and RH spikes.
        """
        self.n_outputs = N_OUTPUTS

        # Parse targets
        day = phase_config.get('day', {})
        night = phase_config.get('night', {})

        # Resolve theta targets: fractional (if present + substrate) > absolute > default
        if substrate is not None and 'theta_target_frac' in day:
            frac = float(day['theta_target_frac'])
            awc = substrate.theta_fc - substrate.theta_wp
            day_theta = substrate.theta_wp + frac * awc
        else:
            day_theta = day.get('theta_target', 0.35)

        if substrate is not None and 'theta_target_frac' in night:
            frac = float(night['theta_target_frac'])
            awc = substrate.theta_fc - substrate.theta_wp
            night_theta = substrate.theta_wp + frac * awc
        else:
            night_theta = night.get('theta_target', 0.30)

        self.day_targets = np.array([
            day.get('VPD_target', 1.2),
            day.get('T_target', 26.0),
            day.get('CO2_target', 1200.0),
            day.get('dryback_rate_target', 0.0),
            day_theta,
            day.get('RH_target', 60.0),
        ])

        self.night_targets = np.array([
            night.get('VPD_target', 0.8),
            night.get('T_target', 22.0),
            night.get('CO2_target', 800.0),
            night.get('dryback_rate_target', 0.002),
            night_theta,
            night.get('RH_target', 55.0),
        ])

        # Clamp theta targets to substrate capacity so the MPC doesn't
        # chase an unreachable setpoint (e.g., theta=0.35 on perlite
        # with theta_fc=0.25), which causes constant irrigation + RH spikes.
        if substrate is not None:
            theta_lo = substrate.theta_wp
            theta_hi = substrate.theta_fc
            self.day_targets[REF_THETA] = np.clip(
                self.day_targets[REF_THETA], theta_lo, theta_hi,
            )
            self.night_targets[REF_THETA] = np.clip(
                self.night_targets[REF_THETA], theta_lo, theta_hi,
            )

        self.ramp_minutes = phase_config.get('ramp_minutes', 15)
        self.ramp_s = self.ramp_minutes * 60.0

        # Cultivar offsets
        if cultivar_config:
            vpd_offset = cultivar_config.get('vpd_offset', 0.0)
            temp_offset = cultivar_config.get('temp_offset', 0.0)
            self.day_targets[REF_VPD] += vpd_offset
            self.night_targets[REF_VPD] += vpd_offset
            self.day_targets[REF_T] += temp_offset
            self.night_targets[REF_T] += temp_offset

        # MPC weights
        weights = phase_config.get('weights', {})
        Q_weights = weights.get('Q', {})
        self.Q_diag = np.array([
            Q_weights.get('VPD', 10.0),
            Q_weights.get('T', 5.0),
            Q_weights.get('CO2', 1.0),
            Q_weights.get('dryback', 3.0),
            Q_weights.get('theta', 2.0),
            Q_weights.get('RH', 5.0),
        ])

        # Build R and S from registry channel order, with phase overrides
        R_weights = weights.get('R', {})
        S_weights = weights.get('S', {})

        if registry is not None:
            # Registry-aware: iterate actual channels, look up by name
            R_defaults = registry.default_R_diag
            S_defaults = registry.default_S_diag
            self.R_diag = np.array([
                R_weights.get(name, R_defaults[i])
                for i, name in enumerate(registry.channel_names)
            ])
            self.S_diag = np.array([
                S_weights.get(name, S_defaults[i])
                for i, name in enumerate(registry.channel_names)
            ])
        else:
            # Fallback for tests or standalone usage without registry
            self.R_diag = np.array([
                R_weights.get('T_sp', 0.1),
                R_weights.get('RH_sp', 0.1),
                R_weights.get('u_hum', 0.5),
                R_weights.get('u_co2', 0.3),
                R_weights.get('u_fan', 0.2),
                R_weights.get('u_irr', 1.0),
            ])
            self.S_diag = np.array([
                S_weights.get('T_sp', 1.0),
                S_weights.get('RH_sp', 1.0),
                S_weights.get('u_hum', 0.5),
                S_weights.get('u_co2', 0.3),
                S_weights.get('u_fan', 0.5),
                S_weights.get('u_irr', 2.0),
            ])

        # Override storage
        self._overrides = {}

    def generate(self, current_time_s, light_schedule, horizon_steps=60, step_s=60.0):
        """Generate reference trajectory over prediction horizon.

        Args:
            current_time_s: Current time in seconds.
            light_schedule: LightSchedule instance.
            horizon_steps: Number of steps in prediction horizon.
            step_s: Seconds per prediction step.

        Returns:
            ref: Reference trajectory (horizon_steps, N_OUTPUTS).
        """
        ref = np.zeros((horizon_steps, self.n_outputs))

        for k in range(horizon_steps):
            future_time = current_time_s + k * step_s
            ref[k] = self._targets_at_time(future_time, light_schedule)

        # Apply any active overrides
        for output_idx, value in self._overrides.items():
            ref[:, output_idx] = value

        return ref

    def _targets_at_time(self, time_s, light_schedule):
        """Get interpolated targets at a specific time.

        Handles smooth ramping between day/night targets.
        During the ramp period after a light transition, linearly
        interpolates from the old targets to the new targets.
        """
        lights_on = light_schedule.is_on(time_s)

        if self.ramp_s <= 0:
            return self.day_targets.copy() if lights_on else self.night_targets.copy()

        hour = (time_s / 3600.0) % 24.0

        if lights_on:
            # Lights ON — last transition was at on_hour
            hours_since = (hour - light_schedule.on_hour) % 24.0
            seconds_since = hours_since * 3600.0
            if seconds_since < self.ramp_s:
                alpha = seconds_since / self.ramp_s
                return (1.0 - alpha) * self.night_targets + alpha * self.day_targets
            return self.day_targets.copy()
        else:
            # Lights OFF — last transition was at off_hour
            hours_since = (hour - light_schedule.off_hour) % 24.0
            seconds_since = hours_since * 3600.0
            if seconds_since < self.ramp_s:
                alpha = seconds_since / self.ramp_s
                return (1.0 - alpha) * self.day_targets + alpha * self.night_targets
            return self.night_targets.copy()

    def set_override(self, output_name, value):
        """Grower override: set a fixed target for an output."""
        idx = OUTPUT_NAMES.index(output_name)
        self._overrides[idx] = value

    def clear_override(self, output_name):
        """Remove a grower override."""
        idx = OUTPUT_NAMES.index(output_name)
        self._overrides.pop(idx, None)

    def clear_all_overrides(self):
        self._overrides.clear()
