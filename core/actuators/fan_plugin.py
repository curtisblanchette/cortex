"""Fan plugin — publishes fan_pct to room_props for other plugins.

Controls: u_fan (fan speed 0-100%).
Physics: No direct ODE contribution. Instead, sets room_props['fan_pct']
         which other plugins (HVACD, AC) use to scale airflow.
"""

import numpy as np

from .base import (
    ActuatorPlugin, ControlChannel, PhysicsEffect,
    DispatchSpec, IOMapping,
)


class FanPlugin(ActuatorPlugin):

    def __init__(self, equipment_config):
        actuators = equipment_config.get('actuators', {})
        fan_config = actuators.get('fan', {})
        self.min_pct = fan_config.get('min_pct', 20.0)
        self.max_pct = fan_config.get('max_pct', 100.0)
        self.rated_power_W = fan_config.get('power_draw_W', 750.0)

    def name(self) -> str:
        return 'fan'

    def control_channels(self):
        return [
            ControlChannel(
                name='u_fan', default=50.0, unit='%',
                bound_low=self.min_pct, bound_high=self.max_pct,
                du_low=-10.0, du_high=10.0,
                jac_perturbation=0.5,
            ),
        ]

    def compute_physics(self, x, u_local, d, params, room_props):
        # Fan doesn't directly affect ODEs — it modulates airflow for
        # other plugins via room_props['fan_pct'].
        # The registry pre-computes fan_pct before calling other plugins.
        return PhysicsEffect()

    def get_fan_pct(self, u_local):
        """Compute normalized fan fraction (0-1) from control input."""
        u_fan = min(max(u_local[0], self.min_pct), self.max_pct)
        return u_fan / 100.0

    def dispatch_specs(self):
        return [DispatchSpec('u_fan', 'fan', 'u_fan', 50.0)]

    def io_mappings(self):
        return [IOMapping('u_fan', 'u_fan')]

    def default_R_weights(self):
        return [0.2]

    def default_S_weights(self):
        return [0.5]

    def power_draw_W(self, u_local):
        u_fan = min(max(u_local[0], 0.0), 100.0) / 100.0
        return u_fan * self.rated_power_W
