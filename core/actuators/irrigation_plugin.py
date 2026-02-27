"""Irrigation plugin — water to substrate + surface evaporation.

Controls: u_irr (irrigation output 0-100%).
Physics: positive dtheta_dt (substrate water addition),
         positive dw_dt (surface evaporation to room air).
"""

import numpy as np

from .base import (
    ActuatorPlugin, ControlChannel, PhysicsEffect,
    DispatchSpec, IOMapping,
)

class IrrigationPlugin(ActuatorPlugin):

    def __init__(self, equipment_config):
        actuators = equipment_config.get('actuators', {})
        irr_config = actuators.get('irrigation', {})
        self.flow_rate_ml_s = irr_config.get('flow_rate_ml_s', 50.0)
        # Fraction of irrigation that evaporates directly to air as humidity
        self.evap_fraction = irr_config.get('evap_fraction', 0.05)

    def name(self) -> str:
        return 'irrigation'

    def control_channels(self):
        return [
            ControlChannel(
                name='u_irr', default=0.0, unit='%',
                bound_low=0.0, bound_high=100.0,
                du_low=-50.0, du_high=50.0,
                jac_perturbation=0.5,
            ),
        ]

    def compute_physics(self, x, u_local, d, params, room_props):
        u_irr = min(max(u_local[0], 0.0), 100.0) / 100.0
        A_canopy = room_props['A_canopy']
        V = room_props['V']
        rho_air = room_props.get('rho_air', 1.2)

        # ml/s -> m3/s, then / (A_canopy * depth) -> m3/m3/s
        irr_vol = self.flow_rate_ml_s * u_irr / 1e6  # m3/s
        substrate_depth = room_props.get('substrate_depth_m', 0.15)
        irr_rate_theta = irr_vol / (A_canopy * substrate_depth)

        # Surface evaporation: fraction of irrigation volume -> humidity
        # irr_vol is m3/s of water = 1e6 g/s, convert to g/kg/s in air
        dw_evap = (self.evap_fraction * irr_vol * 1e6) / (rho_air * V) / 3600.0

        return PhysicsEffect(
            dtheta_dt=irr_rate_theta,
            dw_dt=dw_evap,
        )

    def dispatch_specs(self):
        return [DispatchSpec('u_irr', 'irrigation', 'u_irr', 0.0)]

    def io_mappings(self):
        return [IOMapping('u_irr', 'u_irr')]

    def default_R_weights(self):
        return [1.0]

    def default_S_weights(self):
        return [2.0]
