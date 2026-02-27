"""Standalone dehumidifier plugin — moisture removal + waste heat.

Controls: u_dehum (dehumidifier output 0-100%).
Physics: negative dw_dt (moisture removal), positive Q_sensible (waste heat).

Equipment config section: standalone_dehumidifier
  capacity_L_hr        — max moisture removal rate (L/hr)
  power_draw_W         — rated electrical power (W)
  waste_heat_W_per_L_hr — sensible heat added per L/hr removed (W)
  limits.min_pct       — MPC lower bound (%)
  limits.max_pct       — MPC upper bound (%)
  limits.min_room_temp_C  — coil-freeze lockout temperature (°C)
  limits.min_room_rh_pct  — low-RH lockout (%)
"""

import numpy as np

from .base import (
    ActuatorPlugin, ControlChannel, PhysicsEffect,
    DispatchSpec, IOMapping,
)


class StandaloneDehumPlugin(ActuatorPlugin):

    def __init__(self, equipment_config):
        dehum_config = equipment_config.get('standalone_dehumidifier', {})
        limits = dehum_config.get('limits', {})

        # Capacity
        self.dehum_max_L_hr = dehum_config.get('capacity_L_hr', 6.0)
        self.power_draw_W = dehum_config.get('power_draw_W', 2800.0)
        self.waste_heat_W_per_L_hr = dehum_config.get('waste_heat_W_per_L_hr', 280.0)

        # MPC bounds from limits
        self.min_pct = limits.get('min_pct', 0.0)
        self.max_pct = limits.get('max_pct', 100.0)

        # Safety lockouts
        self.min_room_temp_C = limits.get('min_room_temp_C', 1.0)
        self.min_room_rh_pct = limits.get('min_room_rh_pct', 30.0)

    def name(self) -> str:
        return 'standalone_dehumidifier'

    def control_channels(self):
        return [
            ControlChannel(
                name='u_dehum', default=0.0, unit='%',
                bound_low=self.min_pct, bound_high=self.max_pct,
                du_low=-20.0, du_high=20.0,
                jac_perturbation=0.5,
            ),
        ]

    def compute_physics(self, x, u_local, d, params, room_props):
        T_a = x[0]
        w_a = x[1]
        u_dehum = min(max(u_local[0], 0.0), 100.0) / 100.0
        V = room_props['V']
        rho_air = room_props.get('rho_air', 1.2)

        # Safety lockouts: zero output if room conditions are outside range
        # Convert w_a (g/kg) to approximate RH for lockout check
        # Simplified: RH ≈ w_a / w_sat(T_a), using Magnus formula
        e_sat = 0.6108 * np.exp(17.27 * T_a / (T_a + 237.3))  # kPa
        w_sat = 622.0 * e_sat / (101.325 - e_sat)  # g/kg at 1 atm
        rh_approx = (w_a / w_sat) * 100.0 if w_sat > 0 else 100.0

        if T_a < self.min_room_temp_C or rh_approx < self.min_room_rh_pct:
            u_dehum = 0.0

        # Moisture removal: L/hr -> g/kg/s
        # 1 L water = 1000 g, air mass = rho * V kg
        dehum_L_hr = self.dehum_max_L_hr * u_dehum
        w_removal = dehum_L_hr * 1000.0 / (rho_air * V) / 3600.0  # g/kg/s

        # Waste heat from compressor (proportional to output)
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