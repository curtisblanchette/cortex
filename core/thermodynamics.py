"""Shared thermodynamic utility functions.

All functions operate on scalar or numpy array inputs.
Units follow the convention in the architecture doc:
  - Temperature: °C
  - Humidity: g/kg (absolute), % (relative)
  - Pressure: kPa
"""

import numpy as np


# Constants
P_ATM = 101.325  # kPa, standard atmospheric pressure


def saturation_vapor_pressure(T):
    """Magnus formula for saturation vapor pressure.

    Args:
        T: Temperature in °C.

    Returns:
        Saturation vapor pressure in kPa.
    """
    return 0.6108 * np.exp(17.27 * T / (T + 237.3))


def saturation_humidity(T):
    """Absolute humidity at saturation.

    Args:
        T: Temperature in °C.

    Returns:
        Saturation absolute humidity in g/kg dry air.
    """
    SVP = saturation_vapor_pressure(T)
    return 622.0 * SVP / (P_ATM - SVP)


def relative_humidity(T, w):
    """Compute relative humidity from temperature and absolute humidity.

    Args:
        T: Air temperature in °C.
        w: Absolute humidity in g/kg.

    Returns:
        Relative humidity in %.
    """
    w_sat = saturation_humidity(T)
    return np.clip((w / w_sat) * 100.0, 0.0, 100.0)


def absolute_humidity_from_rh(T, RH):
    """Convert RH to absolute humidity.

    Args:
        T: Air temperature in °C.
        RH: Relative humidity in %.

    Returns:
        Absolute humidity in g/kg.
    """
    w_sat = saturation_humidity(T)
    return w_sat * RH / 100.0


def vpd(T_leaf, w_air):
    """Vapor pressure deficit between leaf and air.

    VPD = SVP(T_leaf) - actual_vapor_pressure(w_air)

    Args:
        T_leaf: Leaf surface temperature in °C.
        w_air: Air absolute humidity in g/kg.

    Returns:
        VPD in kPa.
    """
    SVP_leaf = saturation_vapor_pressure(T_leaf)
    # Actual vapor pressure from absolute humidity
    # w = 622 * e / (P - e)  =>  e = w * P / (622 + w)
    e_air = w_air * P_ATM / (622.0 + w_air)
    return SVP_leaf - e_air


def dew_point_from_w(w):
    """Compute dew point temperature from absolute humidity.

    Inverts the Magnus formula.

    Args:
        w: Absolute humidity in g/kg.

    Returns:
        Dew point temperature in °C.
    """
    # e = w * P / (622 + w)
    e = w * P_ATM / (622.0 + w)
    # Invert Magnus: T = 237.3 * ln(e/0.6108) / (17.27 - ln(e/0.6108))
    ln_ratio = np.log(e / 0.6108)
    return 237.3 * ln_ratio / (17.27 - ln_ratio)


def d_svp_dT(T):
    """Derivative of saturation vapor pressure with respect to temperature.

    Used in Jacobian computations.

    Args:
        T: Temperature in °C.

    Returns:
        dSVP/dT in kPa/°C.
    """
    SVP = saturation_vapor_pressure(T)
    return SVP * 17.27 * 237.3 / (T + 237.3) ** 2


def d_wsat_dT(T):
    """Derivative of saturation humidity with respect to temperature.

    Args:
        T: Temperature in °C.

    Returns:
        dw_sat/dT in (g/kg)/°C.
    """
    SVP = saturation_vapor_pressure(T)
    dSVP = d_svp_dT(T)
    # w_sat = 622 * SVP / (P - SVP)
    # dw_sat/dT = 622 * P * dSVP / (P - SVP)^2
    return 622.0 * P_ATM * dSVP / (P_ATM - SVP) ** 2
