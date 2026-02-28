"""Standalone heater plugin — resistive/electric space heating.

Controls: u_heat (heater output 0-100%).
Physics: positive Q_sensible (direct room heating, not ducted).

Equipment config section: actuators.heater
  capacity_W    — max heating power (W)
  max_pct       — MPC upper bound (%)
"""

from .base import (
    ActuatorPlugin, ControlChannel, PhysicsEffect,
    DispatchSpec, IOMapping,
)


class HeaterPlugin(ActuatorPlugin):

    def __init__(self, equipment_config):
        actuators = equipment_config.get('actuators', {})
        heater_config = actuators.get('heater', {})
        self.capacity_W = heater_config.get('capacity_W', 5000.0)
        self.max_pct = heater_config.get('max_pct', 100.0)

    def name(self) -> str:
        return 'heater'

    def control_channels(self):
        return [
            ControlChannel(
                name='u_heat', default=0.0, unit='%',
                bound_low=0.0, bound_high=self.max_pct,
                du_low=-20.0, du_high=20.0,
                jac_perturbation=0.5,
            ),
        ]

    def compute_physics(self, x, u_local, d, params, room_props):
        u_heat = min(max(u_local[0], 0.0), 100.0) / 100.0

        Q_sensible = u_heat * self.capacity_W * params.eta_hvac

        return PhysicsEffect(Q_sensible=Q_sensible)

    def dispatch_specs(self):
        return [DispatchSpec('u_heat', 'heat', 'u_heat', 0.0)]

    def io_mappings(self):
        return [IOMapping('u_heat', 'u_heat')]

    def default_R_weights(self):
        return [0.01]

    def default_S_weights(self):
        return [0.5]

    def power_draw_W(self, u_local):
        u_heat = min(max(u_local[0], 0.0), 100.0) / 100.0
        return u_heat * self.capacity_W
