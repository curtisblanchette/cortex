"""Penman-Monteith transpiration model with stomatal conductance.

Models plant water loss from the canopy. The stomatal conductance
sub-model uses a Jarvis-type multiplicative approach where environmental
stresses (VPD, light, soil moisture) reduce conductance from its maximum.

Units:
  - VPD: kPa
  - PAR: µmol/m²/s (photosynthetically active radiation)
  - theta: m³/m³ (volumetric water content)
  - CO2: ppm
  - conductance: mol/m²/s
  - transpiration: g/m²/hr
"""

import numpy as np

from .thermodynamics import saturation_vapor_pressure, d_svp_dT, P_ATM


# Constants
LAMBDA_V = 2.45e6    # J/kg - latent heat of vaporization at ~20°C
GAMMA = 0.0667       # kPa/°C - psychrometric constant
R_AIR = 287.0        # J/(kg·K) - specific gas constant for dry air
CP_AIR = 1005.0      # J/(kg·K) - specific heat of air at constant pressure
RHO_AIR = 1.2        # kg/m³ - air density
MW_WATER = 18.015    # g/mol - molecular weight of water
R_GAS = 8.314        # J/(mol·K) - universal gas constant

# Stomatal model defaults (Jarvis-type) — used when substrate not provided
THETA_WP = 0.10      # m³/m³ - wilting point (stomata close below this)
THETA_FC = 0.40      # m³/m³ - field capacity (no water stress above this)
VPD_CLOSE = 3.5      # kPa - VPD at which stomata fully close
PAR_HALF = 200.0     # µmol/m²/s - PAR at half-maximum conductance
CO2_REF = 400.0      # ppm - reference CO2 for conductance scaling
G_CUTICULAR = 0.02   # mol/m²/s - cuticular conductance (~5% of g_s_max for cannabis)


def stomatal_conductance(PAR, VPD, theta, CO2, g_s_max,
                         theta_wp=THETA_WP, theta_fc=THETA_FC):
    """Jarvis-type stomatal conductance model with cuticular floor.

    g_s = max(g_s_max * f_light * f_vpd * f_water * f_co2, g_cuticular * f_water)

    Each stress function returns a value in [0, 1] representing the
    fraction of maximum conductance under that stress. Cuticular
    conductance provides a VPD-driven floor even when stomata are
    fully closed (lights off), enabling overnight dryback.

    Args:
        PAR: Photosynthetically active radiation in µmol/m²/s.
        VPD: Vapor pressure deficit in kPa.
        theta: Volumetric water content in m³/m³.
        CO2: CO2 concentration in ppm.
        g_s_max: Maximum stomatal conductance in mol/m²/s.
        theta_wp: Wilting point from substrate profile.
        theta_fc: Field capacity from substrate profile.

    Returns:
        Stomatal conductance in mol/m²/s.
    """
    f_light = PAR / (PAR + PAR_HALF) if PAR > 0 else 0.0
    f_vpd = min(max(1.0 - VPD / VPD_CLOSE, 0.05), 1.0)
    f_water = min(max((theta - theta_wp) / (theta_fc - theta_wp), 0.0), 1.0)
    f_co2 = min(max(1.0 - 0.1 * (CO2 - CO2_REF) / CO2_REF, 0.5), 1.2)

    g_stomatal = g_s_max * f_light * f_vpd * f_water * f_co2
    g_cuticular = G_CUTICULAR * f_water  # cuticle still needs water in substrate

    return max(g_stomatal, g_cuticular)


def transpiration_rate(T_leaf, T_air, w_air, PAR, theta, CO2, LAI, g_s_max,
                       R_net=0.0, theta_wp=THETA_WP, theta_fc=THETA_FC):
    """Penman-Monteith transpiration rate.

    Simplified form for an indoor environment where wind speed is
    replaced by forced convection from the HVAC system.

    Args:
        T_leaf: Leaf temperature in °C.
        T_air: Air temperature in °C.
        w_air: Air absolute humidity in g/kg.
        PAR: PAR in µmol/m²/s.
        theta: Substrate volumetric water content in m³/m³.
        CO2: CO2 concentration in ppm.
        LAI: Leaf area index (m²/m²).
        g_s_max: Maximum stomatal conductance in mol/m²/s.
        R_net: Net radiation absorbed by canopy in W/m².
        theta_wp: Wilting point from substrate profile.
        theta_fc: Field capacity from substrate profile.

    Returns:
        Transpiration rate in g/m²/hr (per unit ground area).
    """
    # Compute VPD
    SVP_leaf = saturation_vapor_pressure(T_leaf)
    e_air = w_air * P_ATM / (622.0 + w_air)
    VPD = max(SVP_leaf - e_air, 0.0)

    # Stomatal conductance
    g_s = stomatal_conductance(PAR, VPD, theta, CO2, g_s_max,
                               theta_wp=theta_wp, theta_fc=theta_fc)

    if g_s < 1e-8:
        return 0.0

    # Convert stomatal conductance from mol/m²/s to m/s
    # g_s [mol/m²/s] * R*T / P → m/s
    T_abs = T_air + 273.15
    g_s_ms = g_s * R_GAS * T_abs / (P_ATM * 1000.0)  # P in Pa

    # Boundary layer conductance (forced convection, typical indoor)
    g_b = 0.02  # m/s — typical for ~0.5 m/s air movement over leaves

    # Total conductance (series: stomatal + boundary layer)
    g_total = 1.0 / (1.0 / g_s_ms + 1.0 / g_b)

    # Slope of saturation vapor pressure curve
    delta = d_svp_dT(T_air)  # kPa/°C

    # Penman-Monteith equation (simplified for indoor)
    # E = (delta * R_net + rho * cp * VPD * g_b) / (delta + gamma * (1 + g_b/g_s_ms))
    numerator = delta * R_net + RHO_AIR * CP_AIR * VPD * g_b
    denominator = LAMBDA_V * (delta + GAMMA * (1.0 + g_b / g_s_ms))

    # E in kg/m²/s per unit leaf area
    E_leaf = max(numerator / denominator, 0.0)

    # Scale by LAI to get per ground area, convert to g/m²/hr
    E_ground = E_leaf * LAI * 1000.0 * 3600.0

    return E_ground


def transpiration_latent_heat(E_rate_g_m2_hr, canopy_area_m2):
    """Convert transpiration rate to latent heat flux.

    Args:
        E_rate_g_m2_hr: Transpiration in g/m²/hr.
        canopy_area_m2: Canopy ground area in m².

    Returns:
        Latent heat flux in W (removed from air as cooling).
    """
    # g/m²/hr → kg/s
    E_kg_s = E_rate_g_m2_hr * canopy_area_m2 / (1000.0 * 3600.0)
    return E_kg_s * LAMBDA_V


def d_gs_dVPD(PAR, VPD, theta, CO2, g_s_max,
              theta_wp=THETA_WP, theta_fc=THETA_FC):
    """Partial derivative of stomatal conductance w.r.t. VPD.

    Used in Jacobian computation. When cuticular conductance dominates
    (lights off), d(g_s)/d(VPD) = 0 since cuticular conductance is
    VPD-independent.
    """
    # Check if stomatal > cuticular at this operating point
    f_light = PAR / (PAR + PAR_HALF) if PAR > 0 else 0.0
    f_vpd = min(max(1.0 - VPD / VPD_CLOSE, 0.05), 1.0)
    f_water = min(max((theta - theta_wp) / (theta_fc - theta_wp), 0.0), 1.0)
    f_co2 = min(max(1.0 - 0.1 * (CO2 - CO2_REF) / CO2_REF, 0.5), 1.2)

    g_stomatal = g_s_max * f_light * f_vpd * f_water * f_co2
    g_cuticular = G_CUTICULAR * f_water

    # If cuticular dominates, derivative w.r.t. VPD is zero
    if g_cuticular >= g_stomatal:
        return 0.0

    if VPD >= VPD_CLOSE:
        return 0.0

    df_vpd = -1.0 / VPD_CLOSE
    return g_s_max * f_light * df_vpd * f_water * f_co2


def d_gs_dtheta(PAR, VPD, theta, CO2, g_s_max,
                theta_wp=THETA_WP, theta_fc=THETA_FC):
    """Partial derivative of stomatal conductance w.r.t. theta."""
    f_light = PAR / (PAR + PAR_HALF) if PAR > 0 else 0.0
    f_vpd = min(max(1.0 - VPD / VPD_CLOSE, 0.05), 1.0)
    f_water = min(max((theta - theta_wp) / (theta_fc - theta_wp), 0.0), 1.0)
    f_co2 = min(max(1.0 - 0.1 * (CO2 - CO2_REF) / CO2_REF, 0.5), 1.2)

    g_stomatal = g_s_max * f_light * f_vpd * f_water * f_co2
    g_cuticular = G_CUTICULAR * f_water

    if theta_wp <= theta <= theta_fc:
        df_water = 1.0 / (theta_fc - theta_wp)
    else:
        df_water = 0.0

    # Derivative depends on which branch is active
    if g_cuticular >= g_stomatal:
        return G_CUTICULAR * df_water
    return g_s_max * f_light * f_vpd * df_water * f_co2
