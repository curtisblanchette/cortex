"""GrowRoomDynamics — THE core physics model.

7 coupled ODEs describing the thermodynamics of an indoor grow room.
This is the single source of truth consumed by EKF, OSQP, SLSQP, and
the simulation environment.

State vector x (7):
    x[0] T_a    Air temperature              °C
    x[1] w_a    Absolute humidity             g/kg
    x[2] C      CO2 concentration             ppm
    x[3] T_l    Leaf temperature              °C
    x[4] θ      Substrate water content       m³/m³
    x[5] T_s    Supply air temperature        °C
    x[6] T_w    Wall/thermal mass temperature °C

Control vector u (6):
    u[0] T_sp or u_cool   Temperature setpoint or cooling %
    u[1] RH_sp or u_heat  RH setpoint or heating %
    u[2] u_hum            Humidifier output 0-100%
    u[3] u_co2            CO2 injection output 0-100%
    u[4] u_fan            Fan speed 0-100%
    u[5] u_irr            Irrigation output 0-100%

Disturbance vector d (5):
    d[0] Q_light   Lighting heat load          W
    d[1] T_out     Outdoor temperature          °C
    d[2] w_out     Outdoor absolute humidity    g/kg
    d[3] N_plants  Plant count                  -
    d[4] phase     Growth phase (int enum)      -
"""

import numpy as np
from scipy.linalg import expm

from .thermodynamics import (
    saturation_vapor_pressure, saturation_humidity, vpd,
    d_svp_dT, d_wsat_dT, P_ATM,
)
from .transpiration import (
    stomatal_conductance, transpiration_rate,
    transpiration_latent_heat, d_gs_dVPD, d_gs_dtheta,
)
from .hvacd import HVACDModel
from calibration.params import CalibrationParams


# State indices
I_TA = 0   # Air temperature
I_WA = 1   # Absolute humidity
I_CO2 = 2  # CO2 concentration
I_TL = 3   # Leaf temperature
I_TH = 4   # Substrate water content (theta)
I_TS = 5   # Supply air temperature
I_TW = 6   # Wall temperature

N_STATES = 7
N_CONTROLS = 6
N_DISTURBANCES = 5

# Physical constants
RHO_AIR = 1.2       # kg/m³
CP_AIR = 1005.0     # J/(kg·K)
LAMBDA_V = 2.45e6   # J/kg latent heat of vaporization
H_CONV = 5.0        # W/(m²·K) convective heat transfer coefficient (leaf-air)
SIGMA = 5.67e-8     # W/(m²·K⁴) Stefan-Boltzmann constant
EPSILON_LEAF = 0.95  # leaf emissivity

# CO2 constants
CO2_AMBIENT = 420.0  # ppm outdoor CO2
CO2_INJECT_RATE = 50.0  # ppm/s at 100% injection

# Infiltration
ACH_INFILTRATION = 0.1  # air changes per hour from leaks

# Substrate
RHO_SUBSTRATE = 400.0   # kg/m³ dry bulk density
SUBSTRATE_DEPTH = 0.15   # m
IRR_RATE_M3_S = 5e-6     # m³/s at 100% irrigation (per m² canopy)


class GrowRoomDynamics:
    """The core physics model. Everything else wraps or consumes this.

    Owns:
    - The 7 coupled ODEs (continuous-time dynamics)
    - Analytical Jacobian computation (for linearization)
    - Discretization (continuous → discrete for OSQP)
    - HVACD sub-model
    - Derived output calculations (VPD, RH, dryback rate)
    """

    def __init__(self, room_config, equipment_config, params=None):
        # Handle nested YAML structure: {'room': {...}} or flat {...}
        room = room_config.get('room', room_config) if isinstance(room_config, dict) else room_config

        self.V = room['volume_m3']
        self.A_floor = room['floor_area_m2']
        self.A_canopy = room.get('canopy_area_m2', room['floor_area_m2'] * 0.8)
        self.A_wall = room.get('wall_area_m2', 136.0)
        self.rho_air = room.get('rho_air', RHO_AIR)
        self.cp_air = room.get('cp_air', CP_AIR)

        # Thermal mass of room air
        self.C_air = self.rho_air * self.cp_air * self.V  # J/K

        # HVACD model
        hvacd_config = equipment_config.get('hvacd', equipment_config)
        self.hvacd = HVACDModel(hvacd_config)

        # Actuator configs
        actuators = equipment_config.get('actuators', {})
        self.co2_inject_rate = actuators.get('co2', {}).get('flow_rate_ppm_s', CO2_INJECT_RATE)
        self.hum_capacity = actuators.get('humidifier', {}).get('capacity_g_hr', 2000.0)
        self.irr_rate = actuators.get('irrigation', {}).get('flow_rate_ml_s', 50.0)

        # Parameters (can be SharedParams or CalibrationParams)
        self._params_source = params

        # Jacobian cache — avoids recomputing when EKF + MPC call at similar operating points
        self._jac_cache = None

    @property
    def params(self):
        """Get current calibration parameters."""
        if self._params_source is None:
            return CalibrationParams()
        if hasattr(self._params_source, 'current'):
            return self._params_source.current()
        return self._params_source

    def nonlinear_step(self, x, u, d, params, dt):
        """Integrate ODEs forward by dt seconds using RK4.

        This is the GROUND TRUTH model including all nonlinearities.

        Args:
            x: State vector (7,).
            u: Control vector (6,).
            d: Disturbance vector (5,).
            params: CalibrationParams instance.
            dt: Time step in seconds.

        Returns:
            x_next: State vector at t + dt.
        """
        k1 = self._derivatives(x, u, d, params)
        k2 = self._derivatives(x + 0.5 * dt * k1, u, d, params)
        k3 = self._derivatives(x + 0.5 * dt * k2, u, d, params)
        k4 = self._derivatives(x + dt * k3, u, d, params)

        x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # Physical clipping
        x_next[I_TA] = np.clip(x_next[I_TA], 5.0, 50.0)
        x_next[I_WA] = np.clip(x_next[I_WA], 0.5, 30.0)
        x_next[I_CO2] = np.clip(x_next[I_CO2], 200.0, 5000.0)
        x_next[I_TL] = np.clip(x_next[I_TL], 5.0, 50.0)
        x_next[I_TH] = np.clip(x_next[I_TH], 0.0, 0.6)
        x_next[I_TS] = np.clip(x_next[I_TS], 0.0, 50.0)
        x_next[I_TW] = np.clip(x_next[I_TW], 5.0, 50.0)

        return x_next

    def _derivatives(self, x, u, d, params):
        """The 7 coupled ODEs. Heart of the entire system.

        Args:
            x: State vector (7,).
            u: Control vector (6,).
            d: Disturbance vector (5,).
            params: CalibrationParams.

        Returns:
            dx/dt: Time derivatives (7,).
        """
        T_a, w_a, C, T_l, theta, T_s, T_w = x
        Q_light = d[0]
        T_out = d[1]
        w_out = d[2]

        UA_wall = params.UA_wall
        UA_env = params.UA_env
        tau_hvac = params.tau_hvac
        g_s_max = params.g_s_max
        LAI = params.LAI
        K_drain = params.K_drain
        C_wall = params.C_wall
        eta_hvac = params.eta_hvac

        # ---- HVAC heat exchange ----
        Q_sens, Q_lat, w_removal, T_s_target = self.hvacd.compute_heat_exchange(
            T_a, w_a, T_s, u, params
        )

        # ---- Transpiration ----
        # PAR from lighting: roughly 2.0 µmol/J for LEDs, only when lights on
        PAR = Q_light * 2.0 / self.A_canopy if Q_light > 0 else 0.0

        E_rate = transpiration_rate(
            T_l, T_a, w_a, PAR, theta, C, LAI, g_s_max
        )
        Q_transpiration = transpiration_latent_heat(E_rate, self.A_canopy)

        # ---- Infiltration ----
        # Air exchange with outdoors (leakage)
        ach_s = ACH_INFILTRATION / 3600.0  # per second
        Q_infiltration = self.rho_air * self.cp_air * self.V * ach_s * (T_out - T_a)
        w_infiltration = ach_s * (w_out - w_a)

        # ---- Humidifier ----
        u_hum = np.clip(u[2], 0.0, 100.0) / 100.0
        # g/hr → g/kg/s: divide by (rho * V * 1000) to get g/kg, then /3600 for /s
        w_humidifier = (self.hum_capacity * u_hum) / (self.rho_air * self.V) / 3600.0

        # ---- CO2 injection & plant uptake ----
        u_co2 = np.clip(u[3], 0.0, 100.0) / 100.0
        CO2_injection = self.co2_inject_rate * u_co2  # ppm/s

        # Plant CO2 uptake (simplified Michaelis-Menten)
        if PAR > 0 and theta > 0.05:
            # µmol/m²/s of CO2 fixation, rough: 10-30 µmol at peak
            A_photo = 20.0 * PAR / (PAR + 300.0) * LAI  # µmol/m²/s
            # Convert to ppm/s: µmol/s / (V_m3 * 1e3 / 24.45) * 1e6
            # 1 ppm in V m³ = V * 1e-3 / 24.45e-3 µmol = V/24.45e-3 * 1e-3 µmol
            # Simpler: 1 ppm = V * 1.2 * 44.01 / (28.97 * 1e6) kg...
            # Approximate: 1 µmol CO2/s in V m³ → 24.45/V ppm/s (at STP)
            CO2_uptake = A_photo * self.A_canopy * 24.45e-3 / self.V  # ppm/s
        else:
            CO2_uptake = 0.0

        CO2_infiltration = ach_s * (CO2_AMBIENT - C)  # ppm/s

        # ---- Irrigation ----
        u_irr = np.clip(u[5], 0.0, 100.0) / 100.0
        # ml/s → m³/s, then / (A_canopy * substrate_depth) → m³/m³/s
        irr_vol = self.irr_rate * u_irr / 1e6  # m³/s
        irr_rate_theta = irr_vol / (self.A_canopy * SUBSTRATE_DEPTH)

        # Transpiration water removal from substrate
        # E_rate in g/m²/hr → m³/m³/s
        E_m3 = E_rate * self.A_canopy / (1e6 * self.A_canopy * SUBSTRATE_DEPTH * 3600.0)

        # Drainage
        drain_rate = K_drain / 3600.0 * max(theta - 0.15, 0.0)  # only above FC residual

        # ==========================================
        # ODE 1: dT_a/dt — Air temperature
        # ==========================================
        dTa = (1.0 / self.C_air) * (
            Q_sens +                                    # HVAC sensible
            Q_light * 0.6 +                             # 60% of light heat to air (rest to canopy)
            Q_infiltration +                            # Outdoor air leakage
            UA_wall * (T_w - T_a) +                     # Wall-air exchange
            H_CONV * self.A_canopy * (T_l - T_a) -      # Leaf-air convection
            Q_transpiration                              # Evaporative cooling
        )

        # ==========================================
        # ODE 2: dw_a/dt — Absolute humidity
        # ==========================================
        dwa = (
            E_rate * self.A_canopy / (self.rho_air * self.V * 3600.0) +  # Transpiration (g/hr → g/kg/s)
            w_humidifier +                               # Humidifier
            w_infiltration -                             # Outdoor air
            w_removal                                    # HVAC dehumidification
        )

        # ==========================================
        # ODE 3: dC/dt — CO2 concentration
        # ==========================================
        dC = (
            CO2_injection -                              # CO2 solenoid
            CO2_uptake +                                 # Plant photosynthesis
            CO2_infiltration                             # Air exchange
        )

        # ==========================================
        # ODE 4: dT_l/dt — Leaf temperature
        # ==========================================
        # Leaf thermal mass (thin, so low capacitance)
        C_leaf = 500.0 * LAI * self.A_canopy  # J/K (approx)

        # Absorbed radiation (PAR + thermal from lights)
        Q_rad_leaf = Q_light * 0.4 * 0.9  # 40% hits canopy, 90% absorbed

        # Longwave radiation exchange (linearized)
        Q_lw = EPSILON_LEAF * SIGMA * self.A_canopy * LAI * (
            4.0 * ((T_a + 273.15) ** 3) * (T_a - T_l)
        )

        # Transpiration cooling on leaf
        Q_evap_leaf = Q_transpiration

        dTl = (1.0 / C_leaf) * (
            Q_rad_leaf +                                 # Radiation absorbed
            H_CONV * self.A_canopy * (T_a - T_l) +      # Convection from air
            Q_lw -                                       # Longwave exchange
            Q_evap_leaf                                  # Evaporative cooling
        ) if C_leaf > 0 else 0.0

        # ==========================================
        # ODE 5: dθ/dt — Substrate water content
        # ==========================================
        dtheta = (
            irr_rate_theta -                             # Irrigation in
            E_m3 -                                       # Transpiration out
            drain_rate                                   # Gravity drainage
        )

        # ==========================================
        # ODE 6: dT_s/dt — Supply air temperature
        # ==========================================
        dTs = self.hvacd.supply_temp_dynamics(T_s, T_s_target, tau_hvac, 1.0)

        # ==========================================
        # ODE 7: dT_w/dt — Wall/thermal mass temperature
        # ==========================================
        dTw = (1.0 / C_wall) * (
            UA_wall * (T_a - T_w) +                      # Air-wall exchange
            UA_env * (T_out - T_w)                       # Wall-outdoor exchange
        )

        return np.array([dTa, dwa, dC, dTl, dtheta, dTs, dTw])

    def linearize(self, x0, u0, d0, params, dt=60.0):
        """Compute affine discrete-time model around operating point.

        Returns A_d, B_d, Bd_d, c_d such that:
            x[k+1] ≈ A_d @ x[k] + B_d @ u[k] + Bd_d @ d[k] + c_d

        The affine offset c_d captures the constant term from linearizing
        nonlinear dynamics:
            dx/dt = f(x,u,d) ≈ A_c·x + B_c·u + Bd_c·d + c_0
        where c_0 = f(x0,u0,d0) - A_c·x0 - B_c·u0 - Bd_c·d0.

        Uses analytical Jacobians + ZOH discretization via matrix exponential.

        Args:
            x0: Operating point state (7,).
            u0: Operating point control (6,).
            d0: Operating point disturbance (5,).
            params: CalibrationParams.
            dt: Discretization step in seconds (default 60s for MPC).

        Returns:
            A_d: Discrete state matrix (7, 7).
            B_d: Discrete control input matrix (7, 6).
            Bd_d: Discrete disturbance input matrix (7, 5).
            c_d: Discrete affine offset (7,).
        """
        A_c, B_c, Bd_c = self._analytical_jacobians(x0, u0, d0, params)
        A_d, B_d, Bd_d = self._discretize_pade2(A_c, B_c, Bd_c, dt)

        # Affine offset: captures the constant term from the linearization.
        # c_0 = f(x0,u0,d0) - A_c·x0 - B_c·u0 - Bd_c·d0
        f0 = self._derivatives(x0, u0, d0, params)
        c_0 = f0 - A_c @ x0 - B_c @ u0 - Bd_c @ d0

        # Discretize c_0 with the same integral: c_d = ∫₀ᵈᵗ expm(A·τ) dτ · c_0
        n = A_c.shape[0]
        try:
            A_inv = np.linalg.solve(A_c, np.eye(n))
            integral = A_inv @ (A_d - np.eye(n))
            c_d = integral @ c_0
        except np.linalg.LinAlgError:
            c_d = dt * c_0

        return A_d, B_d, Bd_d, c_d

    def _analytical_jacobians(self, x0, u0, d0, params):
        """Compute continuous-time Jacobians via finite differences.

        For production, these would be analytically derived and Numba JIT
        compiled. For initial development, we use central finite differences
        which are accurate to O(h²).

        Results are cached so that back-to-back calls at similar operating
        points (e.g. EKF predict then MPC linearize in the same tick) skip
        the expensive 36× _derivatives() evaluations.

        Args:
            x0: State operating point (7,).
            u0: Control operating point (6,).
            d0: Disturbance operating point (5,).
            params: CalibrationParams.

        Returns:
            A_c: df/dx (7, 7) — state Jacobian.
            B_c: df/du (7, 6) — control Jacobian.
            Bd_c: df/dd (7, 5) — disturbance Jacobian.
        """
        # Check cache — reuse if operating point hasn't changed significantly
        if self._jac_cache is not None:
            cx, cu, cd = self._jac_cache['x'], self._jac_cache['u'], self._jac_cache['d']
            if (np.allclose(x0, cx, atol=0.05, rtol=0.01)
                    and np.array_equal(u0, cu)
                    and np.array_equal(d0, cd)):
                return self._jac_cache['A'], self._jac_cache['B'], self._jac_cache['Bd']

        n_x = N_STATES
        n_u = N_CONTROLS
        n_d = N_DISTURBANCES

        # Perturbation sizes (scaled to state magnitudes)
        h_x = np.array([0.01, 0.01, 1.0, 0.01, 0.001, 0.01, 0.01])
        h_u = np.array([0.1, 0.1, 0.5, 0.5, 0.5, 0.5])
        h_d = np.array([100.0, 0.1, 0.01, 1.0, 0.1])

        # A_c = df/dx
        A_c = np.zeros((n_x, n_x))
        for j in range(n_x):
            x_plus = x0.copy()
            x_minus = x0.copy()
            x_plus[j] += h_x[j]
            x_minus[j] -= h_x[j]
            f_plus = self._derivatives(x_plus, u0, d0, params)
            f_minus = self._derivatives(x_minus, u0, d0, params)
            A_c[:, j] = (f_plus - f_minus) / (2.0 * h_x[j])

        # B_c = df/du
        B_c = np.zeros((n_x, n_u))
        for j in range(n_u):
            u_plus = u0.copy()
            u_minus = u0.copy()
            u_plus[j] += h_u[j]
            u_minus[j] -= h_u[j]
            f_plus = self._derivatives(x0, u_plus, d0, params)
            f_minus = self._derivatives(x0, u_minus, d0, params)
            B_c[:, j] = (f_plus - f_minus) / (2.0 * h_u[j])

        # Bd_c = df/dd
        Bd_c = np.zeros((n_x, n_d))
        for j in range(n_d):
            d_plus = d0.copy()
            d_minus = d0.copy()
            d_plus[j] += h_d[j]
            d_minus[j] -= h_d[j]
            f_plus = self._derivatives(x0, u0, d_plus, params)
            f_minus = self._derivatives(x0, u0, d_minus, params)
            Bd_c[:, j] = (f_plus - f_minus) / (2.0 * h_d[j])

        # Update cache
        self._jac_cache = {
            'x': x0.copy(), 'u': u0.copy(), 'd': d0.copy(),
            'A': A_c, 'B': B_c, 'Bd': Bd_c,
        }

        return A_c, B_c, Bd_c

    def _discretize_pade2(self, A_c, B_c, Bd_c, dt):
        """2nd-order Padé approximation for discretization.

        More accurate than simple Euler or ZOH for stiff systems.

        Computes:
            A_d = expm(A_c * dt)
            B_d = A_c^{-1} (A_d - I) B_c  (zero-order hold)
            Bd_d = A_c^{-1} (A_d - I) Bd_c

        Falls back to dt * B_c if A_c is singular.
        """
        n = A_c.shape[0]
        A_d = expm(A_c * dt)

        # ZOH discretization for B and Bd
        try:
            # Use the matrix exponential integral: ∫₀ᵈᵗ expm(A*τ) dτ * B
            # Computed via augmented matrix:
            # [A  B; 0  0] → expm gives [expm(A*dt)  ∫; 0  I]
            # But simpler: if A is invertible, ∫ = A^{-1}(expm(A*dt) - I)
            A_inv = np.linalg.solve(A_c, np.eye(n))
            integral = A_inv @ (A_d - np.eye(n))
            B_d = integral @ B_c
            Bd_d = integral @ Bd_c
        except np.linalg.LinAlgError:
            # Fallback: first-order approximation
            B_d = dt * B_c
            Bd_d = dt * Bd_c

        return A_d, B_d, Bd_d

    def derived_outputs(self, x):
        """Compute derived outputs from state vector.

        Args:
            x: State vector (7,).

        Returns:
            Dict with VPD, RH, T_leaf, theta.
        """
        T_a, w_a, C, T_l, theta, T_s, T_w = x

        VPD = vpd(T_l, w_a)
        SVP_a = saturation_vapor_pressure(T_a)
        w_sat = saturation_humidity(T_a)
        RH = np.clip((w_a / w_sat) * 100.0, 0.0, 100.0)

        return {
            'VPD': VPD,
            'RH': RH,
            'T_leaf': T_l,
            'theta': theta,
            'CO2': C,
            'T_air': T_a,
            'T_supply': T_s,
            'T_wall': T_w,
        }

    @staticmethod
    def default_initial_state():
        """Reasonable initial state for simulation startup."""
        return np.array([
            24.0,   # T_a: 24°C air temp
            10.0,   # w_a: 10 g/kg absolute humidity (~55% RH at 24°C)
            800.0,  # C: 800 ppm CO2
            23.0,   # T_l: 23°C leaf temp (slightly below air)
            0.35,   # θ: 0.35 m³/m³ substrate water
            20.0,   # T_s: 20°C supply air
            23.0,   # T_w: 23°C wall temp
        ])

    @staticmethod
    def default_control():
        """Reasonable default control vector."""
        return np.array([
            24.0,  # T_sp or u_cool: 24°C setpoint
            55.0,  # RH_sp or u_heat: 55% RH setpoint
            0.0,   # u_hum: humidifier off
            0.0,   # u_co2: CO2 injection off
            50.0,  # u_fan: 50% fan speed
            0.0,   # u_irr: irrigation off
        ])

    @staticmethod
    def default_disturbance():
        """Typical mid-flower day disturbance."""
        return np.array([
            30000.0,  # Q_light: 30kW lights on
            28.0,     # T_out: 28°C outdoor
            12.0,     # w_out: 12 g/kg outdoor humidity
            100.0,    # N_plants
            3.0,      # phase: mid_flower
        ])
