"""Humidifier plugin — additive moisture.

Controls: u_hum (humidifier output 0-100%).
Physics: positive dw_dt.
"""

import numpy as np

from .base import (
    ActuatorPlugin, ControlChannel, PhysicsEffect,
    DispatchSpec, IOMapping,
)


class HumidifierPlugin(ActuatorPlugin):

    def __init__(self, equipment_config):
        actuators = equipment_config.get('actuators', {})
        hum_config = actuators.get('humidifier', {})
        self.capacity_g_hr = hum_config.get('capacity_g_hr', 2000.0)

    def name(self) -> str:
        return 'humidifier'

    def control_channels(self):
        return [
            ControlChannel(
                name='u_hum', default=0.0, unit='%',
                bound_low=0.0, bound_high=100.0,
                du_low=-20.0, du_high=20.0,
                jac_perturbation=0.5,
            ),
        ]

    def compute_physics(self, x, u_local, d, params, room_props):
        u_hum = min(max(u_local[0], 0.0), 100.0) / 100.0
        V = room_props['V']
        rho_air = room_props.get('rho_air', 1.2)

        # g/hr -> g/kg/s
        w_humidifier = (self.capacity_g_hr * u_hum) / (rho_air * V) / 3600.0

        return PhysicsEffect(dw_dt=w_humidifier)

    def dispatch_specs(self):
        return [DispatchSpec('u_hum', 'humidifier', 'u_hum', 0.0)]

    def io_mappings(self):
        return [IOMapping('u_hum', 'u_hum')]

    def default_R_weights(self):
        return [0.5]

    def default_S_weights(self):
        return [0.5]
