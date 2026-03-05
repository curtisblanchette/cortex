"""HVAC dynamics model for integrated dehumidification units.

Models a commercial HVACD (e.g., Desert Aire) that can cool, heat,
and dehumidify. Supports two control interfaces:

  - Setpoint mode: Controller sends T_sp, RH_sp; HVACD's internal
    PID handles compressor/reheat staging.
  - Direct mode: Controller sends cool_pct, heat_pct directly.

The model captures thermal lag via first-order dynamics (tau_hvac),
so supply air temperature doesn't change instantaneously.
"""

import numpy as np


class HVACDModel:
    """Multi-mode HVAC dynamics.

    Maintains internal state for supply air temperature and computes
    the sensible/latent heat exchange with the room air at each step.
    """

    def __init__(self, config):
        self.Q_cool_max = config.get('cooling_capacity_W', 35000.0)
        self.Q_heat_max = config.get('heating_capacity_W', 15000.0)
        self.dehum_max = config.get('dehumidification_L_hr', 8.0)
        self.airflow_cfm = config.get('supply_airflow_cfm', 3000.0)
        self.mode = config.get('mode', 'setpoint')

        limits = config.get('limits', {})
        self.T_sp_min = limits.get('T_sp_min', 15.0)
        self.T_sp_max = limits.get('T_sp_max', 32.0)
        self.RH_sp_min = limits.get('RH_sp_min', 40.0)
        self.RH_sp_max = limits.get('RH_sp_max', 80.0)
        self.fan_min = limits.get('fan_min_pct', 20.0)
        self.fan_max = limits.get('fan_max_pct', 100.0)
        self.dehum_band = config.get('dehum_proportional_band', 5.0)

        # Airflow in m³/s (from CFM)
        self.airflow_m3s = self.airflow_cfm * 0.000471947

    def compute_heat_exchange(self, T_air, w_air, T_supply, u, params):
        """Compute sensible and latent heat exchange for current step.

        This does NOT advance internal state — it computes instantaneous
        heat/moisture exchange given the current supply air state.

        Args:
            T_air: Room air temperature in °C.
            w_air: Room absolute humidity in g/kg.
            T_supply: Current supply air temperature in °C (state x[5]).
            u: Control vector. In setpoint mode: [T_sp, RH_sp, u_hum, u_co2, u_fan, u_irr].
                              In direct mode: [u_cool, u_heat, u_hum, u_co2, u_fan, u_irr].
            params: CalibrationParams with eta_hvac.

        Returns:
            Q_sensible: Sensible heat exchange in W (positive = heating room).
            Q_latent: Latent heat exchange in W (positive = adding moisture).
            w_removal: Moisture removal rate in g/kg/s.
            T_supply_target: Target supply air temperature in °C.
        """
        eta = params.eta_hvac
        fan_pct = min(max(u[4], self.fan_min), self.fan_max) / 100.0
        airflow = self.airflow_m3s * fan_pct

        if self.mode == 'setpoint':
            return self._setpoint_mode(T_air, w_air, T_supply, u, eta, airflow)
        else:
            return self._direct_mode(T_air, w_air, T_supply, u, eta, airflow)

    def _setpoint_mode(self, T_air, w_air, T_supply, u, eta, airflow):
        """HVACD internal logic determines cooling/heating from setpoints."""
        T_sp = min(max(u[0], self.T_sp_min), self.T_sp_max)
        RH_sp = min(max(u[1], self.RH_sp_min), self.RH_sp_max)

        # Determine what the HVACD needs to do
        T_error = T_air - T_sp      # positive = too warm, need cooling
        RH_error = self._compute_rh(T_air, w_air) - RH_sp  # positive = too humid

        # Sensible: proportional response to temperature error
        # HVACD cools by sending cold supply air through the coil
        if T_error > 0:
            # Cooling needed — proportional, capped at capacity
            Q_sensible_raw = -min(T_error * 3000.0, self.Q_cool_max)
        else:
            # Heating needed
            Q_sensible_raw = min(-T_error * 2000.0, self.Q_heat_max)

        Q_sensible = Q_sensible_raw * eta

        # Latent: dehumidification when RH too high
        # Dehumidification is coupled to cooling — the coil condenses moisture
        if RH_error > 0:
            # Convert L/hr to g/kg/s removed from air
            dehum_fraction = min(RH_error / 20.0, 1.0)
            # dehum_max is in L/hr, 1L water = 1000g
            w_removal = (self.dehum_max * dehum_fraction * 1000.0 /
                         3600.0 / (1.2 * self.airflow_m3s * 1000.0))
        else:
            w_removal = 0.0

        # Latent heat from dehumidification (negative = removing moisture energy)
        Q_latent = -w_removal * 1.2 * airflow * 1000.0 * 2.45e6 / 1000.0

        # Target supply temperature based on what HVACD is doing
        rho_cp_flow = 1.2 * 1005.0 * airflow
        if rho_cp_flow > 0:
            T_supply_target = T_air + Q_sensible / rho_cp_flow
        else:
            T_supply_target = T_air

        return Q_sensible, Q_latent, w_removal, T_supply_target

    def _direct_mode(self, T_air, w_air, T_supply, u, eta, airflow):
        """Direct control of cooling/heating percentages."""
        u_cool = min(max(u[0], 0.0), 100.0) / 100.0
        u_heat = min(max(u[1], 0.0), 100.0) / 100.0

        Q_sensible = (-u_cool * self.Q_cool_max + u_heat * self.Q_heat_max) * eta

        # Dehumidification proportional to cooling
        if u_cool > 0.1:
            w_removal = (self.dehum_max * u_cool * 1000.0 /
                         3600.0 / (1.2 * self.airflow_m3s * 1000.0))
        else:
            w_removal = 0.0

        Q_latent = -w_removal * 1.2 * airflow * 1000.0 * 2.45e6 / 1000.0

        rho_cp_flow = 1.2 * 1005.0 * airflow
        if rho_cp_flow > 0:
            T_supply_target = T_air + Q_sensible / rho_cp_flow
        else:
            T_supply_target = T_air

        return Q_sensible, Q_latent, w_removal, T_supply_target

    def supply_temp_dynamics(self, T_supply, T_supply_target, tau_hvac, dt):
        """First-order lag for supply air temperature.

        The HVAC coil doesn't change supply temperature instantly.
        This models the thermal lag.

        dT_s/dt = (T_supply_target - T_supply) / tau_hvac

        Args:
            T_supply: Current supply air temp in °C.
            T_supply_target: Target supply air temp in °C.
            tau_hvac: Time constant in seconds.
            dt: Time step in seconds.

        Returns:
            Rate of change dT_s/dt in °C/s.
        """
        return (T_supply_target - T_supply) / tau_hvac

    @staticmethod
    def _compute_rh(T, w):
        """Compute RH from temperature and absolute humidity."""
        SVP = 0.6108 * np.exp(17.27 * T / (T + 237.3))
        w_sat = 622.0 * SVP / (101.325 - SVP)
        return (w / w_sat) * 100.0
