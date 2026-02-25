"""Standalone AC plugin — cooling only, no dehumidification.

Controls: u_cool (cooling percentage 0-100%).
Physics: Q_sensible (negative = cooling), T_supply_target.
Drives the T_supply state.
"""

import numpy as np

from .base import (
    ActuatorPlugin, ControlChannel, PhysicsEffect,
    DispatchSpec, IOMapping,
)


class StandaloneACPlugin(ActuatorPlugin):

    def __init__(self, equipment_config):
        ac_config = equipment_config.get('standalone_ac', {})
        self.Q_cool_max = ac_config.get('cooling_capacity_W', 25000.0)
        self.airflow_cfm = ac_config.get('supply_airflow_cfm', 2000.0)
        self.airflow_m3s = self.airflow_cfm * 0.000471947

    def name(self) -> str:
        return 'standalone_ac'

    def control_channels(self):
        return [
            ControlChannel(
                name='u_cool', default=0.0, unit='%',
                bound_low=0.0, bound_high=100.0,
                du_low=-20.0, du_high=20.0,
                jac_perturbation=0.5,
            ),
        ]

    def compute_physics(self, x, u_local, d, params, room_props):
        T_a = x[0]
        fan_pct = room_props.get('fan_pct', 0.5)
        eta = params.eta_hvac

        u_cool = np.clip(u_local[0], 0.0, 100.0) / 100.0
        airflow = self.airflow_m3s * fan_pct

        Q_sensible = -u_cool * self.Q_cool_max * eta

        # Compute supply temp target from sensible heat
        rho_cp_flow = 1.2 * 1005.0 * airflow
        if rho_cp_flow > 0:
            T_supply_target = T_a + Q_sensible / rho_cp_flow
        else:
            T_supply_target = T_a

        return PhysicsEffect(
            Q_sensible=Q_sensible,
            T_supply_target=T_supply_target,
        )

    def dispatch_specs(self):
        return [DispatchSpec('u_cool', 'cool', 'u_cool', 0.0)]

    def io_mappings(self):
        return [IOMapping('u_cool', 'u_cool')]

    def requires_supply_temp_state(self) -> bool:
        return True

    def default_R_weights(self):
        return [0.1]

    def default_S_weights(self):
        return [1.0]
