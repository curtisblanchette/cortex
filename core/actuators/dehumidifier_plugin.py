"""Standalone dehumidifier plugin — moisture removal + waste heat.

Controls: u_dehum (dehumidifier output 0-100%).
Physics: negative dw_dt (moisture removal), positive Q_sensible (waste heat).
"""

import numpy as np

from .base import (
    ActuatorPlugin, ControlChannel, PhysicsEffect,
    DispatchSpec, IOMapping,
)


class StandaloneDehumPlugin(ActuatorPlugin):

    def __init__(self, equipment_config):
        dehum_config = equipment_config.get('standalone_dehumidifier', {})
        # L/hr at full capacity
        self.dehum_max_L_hr = dehum_config.get('capacity_L_hr', 6.0)
        # Waste heat factor: W per L/hr of moisture removed
        self.waste_heat_W_per_L_hr = dehum_config.get('waste_heat_W_per_L_hr', 500.0)

    def name(self) -> str:
        return 'standalone_dehumidifier'

    def control_channels(self):
        return [
            ControlChannel(
                name='u_dehum', default=0.0, unit='%',
                bound_low=0.0, bound_high=100.0,
                du_low=-20.0, du_high=20.0,
                jac_perturbation=0.5,
            ),
        ]

    def compute_physics(self, x, u_local, d, params, room_props):
        u_dehum = np.clip(u_local[0], 0.0, 100.0) / 100.0
        V = room_props['V']
        rho_air = room_props.get('rho_air', 1.2)

        # Moisture removal: L/hr -> g/kg/s
        # 1 L water = 1000 g, air mass = rho * V kg
        dehum_L_hr = self.dehum_max_L_hr * u_dehum
        w_removal = dehum_L_hr * 1000.0 / (rho_air * V) / 3600.0  # g/kg/s

        # Waste heat from compressor
        Q_waste = self.waste_heat_W_per_L_hr * dehum_L_hr

        return PhysicsEffect(
            Q_sensible=Q_waste,
            dw_dt=-w_removal,
        )

    def dispatch_specs(self):
        return [DispatchSpec('u_dehum', 'dehumidifier', 'u_dehum', 0.0)]

    def io_mappings(self):
        return [IOMapping('u_dehum', 'u_dehum')]

    def default_R_weights(self):
        return [0.3]

    def default_S_weights(self):
        return [0.5]
