"""GrowRoomDynamics — THE core physics model.

7 coupled ODEs describing the thermodynamics of an indoor grow room.
This is the single source of truth consumed by EKF, OSQP, SLSQP, and
the simulation environment.

State vector x (7):
    x[0] T_a    Air temperature              C
    x[1] w_a    Absolute humidity             g/kg
    x[2] C      CO2 concentration             ppm
    x[3] T_l    Leaf temperature              C
    x[4] theta  Substrate water content       m3/m3
    x[5] T_s    Supply air temperature        C
    x[6] T_w    Wall/thermal mass temperature C

Control vector u (n_u, determined by ActuatorRegistry):
    Default hvacd_full profile: [T_sp, RH_sp, u_hum, u_co2, u_fan, u_irr]

Disturbance vector d (5):
    d[0] Q_light   Lighting heat load          W
    d[1] T_out     Outdoor temperature          C
    d[2] w_out     Outdoor absolute humidity    g/kg
    d[3] N_plants  Plant count                  -
    d[4] phase     Growth phase (int enum)      -
"""

import numpy as np
from scipy.linalg import expm

from .thermodynamics import (
    saturation_vapor_pressure, saturation_humidity, vpd,
    P_ATM,
)
from .transpiration import (
    transpiration_rate, transpiration_latent_heat,
)
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
N_DISTURBANCES = 5

# Physical constants
RHO_AIR = 1.2       # kg/m3
CP_AIR = 1005.0     # J/(kg K)
LAMBDA_V = 2.45e6   # J/kg latent heat of vaporization
H_CONV = 5.0        # W/(m2 K) convective heat transfer coefficient (leaf-air)
SIGMA = 5.67e-8     # W/(m2 K4) Stefan-Boltzmann constant
EPSILON_LEAF = 0.95  # leaf emissivity

# CO2 constants
CO2_AMBIENT = 420.0  # ppm outdoor CO2

# Infiltration
ACH_INFILTRATION = 0.1  # air changes per hour from leaks

# Substrate
SUBSTRATE_DEPTH = 0.15   # m


class GrowRoomDynamics:
    """The core physics model. Everything else wraps or consumes this.

    Owns:
    - The 7 coupled ODEs (continuous-time dynamics)
    - Finite-difference Jacobian computation (for linearization)
    - Discretization (continuous -> discrete for OSQP)
    - Derived output calculations (VPD, RH, dryback rate)

    Actuator physics are delegated to plugins via the ActuatorRegistry.
    """

    def __init__(self, room_config, equipment_config, params=None,
                 registry=None):
        if registry is None:
            raise ValueError(
                "GrowRoomDynamics requires an ActuatorRegistry. "
                "Build one with core.actuators.factory.build_registry() "
                "and pass it as the 'registry' argument."
            )

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

        self.registry = registry

        # Parameters (can be SharedParams or CalibrationParams)
        self._params_source = params

        # Jacobian cache
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

        Args:
            x: State vector (7,).
            u: Control vector (n_u,).
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

        Actuator effects come from registry plugins. Core physics
        (transpiration, infiltration, wall dynamics, leaf energy balance,
        CO2 uptake, drainage) remain here.

        Args:
            x: State vector (7,).
            u: Control vector (n_u,).
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

        # ---- Plugin actuator effects ----
        fx = self.registry.compute_all_physics(x, u, d, params)

        # ---- Transpiration (core physics) ----
        PAR = Q_light * 2.0 / self.A_canopy if Q_light > 0 else 0.0

        E_rate = transpiration_rate(
            T_l, T_a, w_a, PAR, theta, C, LAI, g_s_max
        )
        Q_transpiration = transpiration_latent_heat(E_rate, self.A_canopy)

        # ---- Infiltration (core physics) ----
        ach_s = ACH_INFILTRATION / 3600.0
        Q_infiltration = self.rho_air * self.cp_air * self.V * ach_s * (T_out - T_a)
        w_infiltration = ach_s * (w_out - w_a)

        # ---- CO2 plant uptake (core physics) ----
        if PAR > 0 and theta > 0.05:
            A_photo = 20.0 * PAR / (PAR + 300.0) * LAI
            CO2_uptake = A_photo * self.A_canopy * 24.45e-3 / self.V
        else:
            CO2_uptake = 0.0
        CO2_infiltration = ach_s * (CO2_AMBIENT - C)

        # ---- Substrate dynamics (core physics) ----
        E_m3 = E_rate * self.A_canopy / (1e6 * self.A_canopy * SUBSTRATE_DEPTH * 3600.0)
        drain_rate = K_drain / 3600.0 * max(theta - 0.15, 0.0)

        # ODE 1: dT_a/dt
        dTa = (1.0 / self.C_air) * (
            fx.Q_sensible +
            Q_light * 0.6 +
            Q_infiltration +
            UA_wall * (T_w - T_a) +
            H_CONV * self.A_canopy * (T_l - T_a) -
            Q_transpiration
        )

        # ODE 2: dw_a/dt
        dwa = (
            E_rate * self.A_canopy / (self.rho_air * self.V * 3600.0) +
            fx.dw_dt +
            w_infiltration
        )

        # ODE 3: dC/dt
        dC = (
            fx.dC_dt -
            CO2_uptake +
            CO2_infiltration
        )

        # ODE 4: dT_l/dt
        C_leaf = 500.0 * LAI * self.A_canopy
        Q_rad_leaf = Q_light * 0.4 * 0.9
        Q_lw = EPSILON_LEAF * SIGMA * self.A_canopy * LAI * (
            4.0 * ((T_a + 273.15) ** 3) * (T_a - T_l)
        )

        dTl = (1.0 / C_leaf) * (
            Q_rad_leaf +
            H_CONV * self.A_canopy * (T_a - T_l) +
            Q_lw -
            Q_transpiration
        ) if C_leaf > 0 else 0.0

        # ODE 5: dtheta/dt
        dtheta = (
            fx.dtheta_dt -
            E_m3 -
            drain_rate
        )

        # ODE 6: dT_s/dt
        if fx.T_supply_target is not None:
            dTs = (fx.T_supply_target - T_s) / tau_hvac
        else:
            dTs = 0.0

        # ODE 7: dT_w/dt
        dTw = (1.0 / C_wall) * (
            UA_wall * (T_a - T_w) +
            UA_env * (T_out - T_w)
        )

        return np.array([dTa, dwa, dC, dTl, dtheta, dTs, dTw])

    def linearize(self, x0, u0, d0, params, dt=60.0):
        """Compute affine discrete-time model around operating point.

        Returns A_d, B_d, Bd_d, c_d such that:
            x[k+1] ~ A_d @ x[k] + B_d @ u[k] + Bd_d @ d[k] + c_d

        Args:
            x0: Operating point state (7,).
            u0: Operating point control (n_u,).
            d0: Operating point disturbance (5,).
            params: CalibrationParams.
            dt: Discretization step in seconds (default 60s for MPC).

        Returns:
            A_d: Discrete state matrix (7, 7).
            B_d: Discrete control input matrix (7, n_u).
            Bd_d: Discrete disturbance input matrix (7, 5).
            c_d: Discrete affine offset (7,).
        """
        A_c, B_c, Bd_c = self._analytical_jacobians(x0, u0, d0, params)
        A_d, B_d, Bd_d = self._discretize_pade2(A_c, B_c, Bd_c, dt)

        f0 = self._derivatives(x0, u0, d0, params)
        c_0 = f0 - A_c @ x0 - B_c @ u0 - Bd_c @ d0

        n = A_c.shape[0]
        try:
            A_inv = np.linalg.solve(A_c, np.eye(n))
            integral = A_inv @ (A_d - np.eye(n))
            c_d = integral @ c_0
        except np.linalg.LinAlgError:
            c_d = dt * c_0

        return A_d, B_d, Bd_d, c_d

    def _analytical_jacobians(self, x0, u0, d0, params):
        """Compute continuous-time Jacobians via central finite differences.

        Returns:
            A_c: df/dx (7, 7).
            B_c: df/du (7, n_u).
            Bd_c: df/dd (7, 5).
        """
        if self._jac_cache is not None:
            cx, cu, cd = self._jac_cache['x'], self._jac_cache['u'], self._jac_cache['d']
            if (np.allclose(x0, cx, atol=0.05, rtol=0.01)
                    and np.array_equal(u0, cu)
                    and np.array_equal(d0, cd)):
                return self._jac_cache['A'], self._jac_cache['B'], self._jac_cache['Bd']

        n_x = N_STATES
        n_u = self.registry.n_u
        n_d = N_DISTURBANCES

        h_x = np.array([0.01, 0.01, 1.0, 0.01, 0.001, 0.01, 0.01])
        h_u = self.registry.h_u
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

        self._jac_cache = {
            'x': x0.copy(), 'u': u0.copy(), 'd': d0.copy(),
            'A': A_c, 'B': B_c, 'Bd': Bd_c,
        }

        return A_c, B_c, Bd_c

    def _discretize_pade2(self, A_c, B_c, Bd_c, dt):
        """ZOH discretization via matrix exponential."""
        n = A_c.shape[0]
        A_d = expm(A_c * dt)

        try:
            A_inv = np.linalg.solve(A_c, np.eye(n))
            integral = A_inv @ (A_d - np.eye(n))
            B_d = integral @ B_c
            Bd_d = integral @ Bd_c
        except np.linalg.LinAlgError:
            B_d = dt * B_c
            Bd_d = dt * Bd_c

        return A_d, B_d, Bd_d

    def derived_outputs(self, x):
        """Compute derived outputs from state vector."""
        T_a, w_a, C, T_l, theta, T_s, T_w = x

        VPD = vpd(T_l, w_a)
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
            24.0,   # T_a: 24C air temp
            10.0,   # w_a: 10 g/kg absolute humidity (~55% RH at 24C)
            800.0,  # C: 800 ppm CO2
            23.0,   # T_l: 23C leaf temp (slightly below air)
            0.35,   # theta: 0.35 m3/m3 substrate water
            20.0,   # T_s: 20C supply air
            23.0,   # T_w: 23C wall temp
        ])

    @staticmethod
    def default_disturbance():
        """Typical mid-flower day disturbance."""
        return np.array([
            30000.0,  # Q_light: 30kW lights on
            28.0,     # T_out: 28C outdoor
            12.0,     # w_out: 12 g/kg outdoor humidity
            100.0,    # N_plants
            3.0,      # phase: mid_flower
        ])
