"""CO2 injector plugin — purely additive CO2.

Controls: u_co2 (injection output 0-100%).
Physics: positive dC_dt (ppm/s).
"""

import numpy as np

from .base import (
    ActuatorPlugin, ControlChannel, PhysicsEffect,
    DispatchSpec, IOMapping,
)

CO2_INJECT_RATE = 50.0  # ppm/s at 100% injection (default)


class CO2InjectorPlugin(ActuatorPlugin):

    def __init__(self, equipment_config):
        actuators = equipment_config.get('actuators', {})
        co2_config = actuators.get('co2', {})
        self.inject_rate = co2_config.get('flow_rate_ppm_s', CO2_INJECT_RATE)

    def name(self) -> str:
        return 'co2_injector'

    def control_channels(self):
        return [
            ControlChannel(
                name='u_co2', default=0.0, unit='%',
                bound_low=0.0, bound_high=100.0,
                du_low=-30.0, du_high=30.0,
                jac_perturbation=0.5,
            ),
        ]

    def compute_physics(self, x, u_local, d, params, room_props):
        u_co2 = np.clip(u_local[0], 0.0, 100.0) / 100.0
        CO2_injection = self.inject_rate * u_co2  # ppm/s

        return PhysicsEffect(dC_dt=CO2_injection)

    def dispatch_specs(self):
        return [DispatchSpec('u_co2', 'co2', 'u_co2', 0.0)]

    def io_mappings(self):
        return [IOMapping('u_co2', 'u_co2')]

    def default_R_weights(self):
        return [0.3]

    def default_S_weights(self):
        return [0.3]
