"""HVACD actuator plugin — wraps the existing HVACDModel.

Controls: T_sp (temperature setpoint), RH_sp (humidity setpoint).
Physics: Q_sensible, moisture removal (w_removal), T_supply_target,
         and fresh air exchange (CO2 dilution, thermal/moisture load).

Drives the T_supply state via first-order lag.

Fresh air exchange
------------------
Commercial HVACDs (Desert Aire, etc.) have a fresh air damper that mixes
outdoor air into the return airstream before the conditioning coil. The
``fresh_air_fraction`` parameter (0.0–1.0) sets what proportion of the
total supply airflow is drawn from outdoors.

Effects modeled:
  - **CO2 dilution**: outdoor air at ~420 ppm displaces enriched room air.
    At 10% fresh air on 3000 CFM, this can require 25-30% more CO2
    injection to maintain 1200 ppm during lights-on.
  - **Sensible load**: outdoor air temperature adds/removes heat from the
    room, increasing the coil's cooling or heating demand.
  - **Moisture load**: outdoor humidity enters the room, increasing
    dehumidification demand in humid climates.

The fresh air T/w effects are modeled as unconditioned exchange (outdoor
air displaces room air directly). In reality the coil partially conditions
the mixed air before delivery, so this slightly overestimates the room-
side perturbation — a conservative approximation.

Typical values:
  - 0.00: Sealed grow room with CO2 enrichment (default)
  - 0.05: Minimum code-compliant ventilation
  - 0.10: Standard Desert Aire fresh air damper setting
  - 0.15: High ventilation (odor control, positive pressure)
"""

import numpy as np

from .base import (
    ActuatorPlugin, ControlChannel, PhysicsEffect,
    DispatchSpec, IOMapping,
)


class HVACDPlugin(ActuatorPlugin):

    def __init__(self, equipment_config):
        from core.hvacd import HVACDModel
        hvacd_config = equipment_config.get('hvacd', equipment_config)
        self.hvacd = HVACDModel(hvacd_config)
        self.COP = hvacd_config.get('COP', 3.5)
        self.fresh_air_fraction = hvacd_config.get('fresh_air_fraction', 0.0)
        limits = hvacd_config.get('limits', {})
        self.fan_min = limits.get('fan_min_pct', 20.0)
        self.fan_max = limits.get('fan_max_pct', 100.0)
        self._last_effort = {}

    def name(self) -> str:
        return 'hvacd'

    def control_channels(self):
        return [
            ControlChannel(
                name='T_sp', default=24.0, unit='C',
                bound_low=15.0, bound_high=32.0,
                du_low=-2.0, du_high=2.0,
                jac_perturbation=0.1,
            ),
            ControlChannel(
                name='RH_sp', default=55.0, unit='%',
                bound_low=40.0, bound_high=80.0,
                du_low=-5.0, du_high=5.0,
                jac_perturbation=0.1,
            ),
        ]

    def compute_physics(self, x, u_local, d, params, room_props):
        T_a, w_a, C, T_l, theta, T_s, T_w = x

        # HVACD blower speed is managed by the unit's onboard controller
        # (e.g., Desert Aire CM3550), not by the MPC. The blower runs at
        # full speed for maximum total capacity; the real hardware modulates
        # fan speed internally to shift sensible heat ratio (SHR) between
        # cooling and dehumidification via VPDsync-style logic.
        # TODO: model internal SHR modulation (fan speed + hot gas reheat)
        fan_pct = 1.0

        # Build a full-format u vector that HVACDModel expects:
        # [T_sp/u_cool, RH_sp/u_heat, _, _, u_fan_pct, _]
        # Only indices 0, 1, 4 are used by HVACDModel
        u_hvacd = np.array([u_local[0], u_local[1], 0.0, 0.0,
                            fan_pct * 100.0, 0.0])

        Q_sens, Q_lat, w_removal_model, T_s_target = self.hvacd.compute_heat_exchange(
            T_a, w_a, T_s, u_hvacd, params
        )

        # Clamp T_supply_target to physical coil limits
        T_s_target = max(5.0, min(T_s_target, 50.0))

        # Convective coupling: heat enters room through supply air, not directly.
        # Q_supply = rho * cp * airflow * (T_supply - T_air) is computed in dynamics.py.
        airflow = self.hvacd.airflow_m3s * fan_pct
        rho_air = room_props.get('rho_air', 1.2)
        supply_airflow_W_per_K = rho_air * 1005.0 * airflow

        # Recompute moisture removal using room volume.
        # HVACDModel divides by airflow mass (rho*airflow*1000) instead of
        # room air mass (rho*V), making dehumidification ~5x too weak.
        # Use the same formula as StandaloneDehumPlugin for consistency.
        V = room_props['V']
        RH_sp = min(max(u_local[1], self.hvacd.RH_sp_min), self.hvacd.RH_sp_max)
        RH = self.hvacd._compute_rh(T_a, w_a)
        RH_error = RH - RH_sp
        if RH_error > 0:
            dehum_fraction = min(RH_error / self.hvacd.dehum_band, 1.0)
            dehum_L_hr = self.hvacd.dehum_max * dehum_fraction * fan_pct
            w_removal = dehum_L_hr * 1000.0 / (rho_air * V) / 3600.0
        else:
            dehum_fraction = 0.0
            w_removal = 0.0

        # Cache effort metrics for telemetry
        # Estimate cooling load from supply-air delta-T vs max capacity
        delta_T = T_a - T_s_target
        Q_estimate = supply_airflow_W_per_K * delta_T
        cooling_load_pct = abs(Q_estimate) / self.hvacd.Q_cool_max * 100.0 if delta_T > 0 else 0.0
        self._last_effort = {
            'dehum_fraction': dehum_fraction,
            'cooling_load_pct': min(cooling_load_pct, 100.0),
        }

        # Fresh air intake — fraction of supply airflow drawn from outdoors.
        # Models the fresh air damper on commercial HVACDs. Outdoor air at
        # ~420 ppm displaces enriched room air (CO2 dilution), and carries
        # outdoor temperature and humidity into the room (additional load).
        # At 10% on 3000 CFM: ~0.14 m³/s outdoor air, replacing room air
        # at the same rate — equivalent to ~1.8 ACH of fresh air exchange.
        dC_fresh = 0.0
        dw_fresh = 0.0
        Q_fresh = 0.0
        if self.fresh_air_fraction > 0:
            fresh_airflow = airflow * self.fresh_air_fraction  # m³/s outdoor air
            T_out = d[1]
            w_out = d[2]
            CO2_ambient = 420.0
            dC_fresh = (fresh_airflow / V) * (CO2_ambient - C)
            dw_fresh = (fresh_airflow / V) * (w_out - w_a)
            Q_fresh = rho_air * 1005.0 * fresh_airflow * (T_out - T_a)

        return PhysicsEffect(
            Q_sensible=Q_fresh,
            supply_airflow_W_per_K=supply_airflow_W_per_K,
            dw_dt=-w_removal + dw_fresh,
            dC_dt=dC_fresh,
            T_supply_target=T_s_target,
        )

    def get_effort_metrics(self):
        """Return cached effort metrics from the last compute_physics() call."""
        return dict(self._last_effort)

    def dispatch_specs(self):
        return [
            DispatchSpec('T_sp', 'cool', 'u_T_sp', 24.0),
            DispatchSpec('RH_sp', 'heat', 'u_RH_sp', 55.0),
        ]

    def io_mappings(self):
        return [
            IOMapping('T_sp', 'T_sp'),
            IOMapping('RH_sp', 'RH_sp'),
        ]

    def requires_supply_temp_state(self) -> bool:
        return True

    def default_R_weights(self):
        return [0.1, 0.1]

    def default_S_weights(self):
        return [1.0, 1.0]

    def power_draw_W(self, u_local):
        # HVACD uses setpoint control — we can't infer load from setpoints alone.
        # Report rated compressor power as a constant (the HVACD is always running).
        return self.hvacd.Q_cool_max / self.COP
