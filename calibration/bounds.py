"""Physical bounds for tunable model parameters.

Used by SLSQP to constrain the optimization search space.
Each bound represents a physically reasonable range.
"""

# (lower, upper) for each parameter
PARAM_BOUNDS = {
    'UA_wall':  (10.0, 200.0),     # W/K
    'UA_env':   (5.0, 100.0),      # W/K
    'tau_hvac': (30.0, 600.0),     # s
    'g_s_max':  (0.05, 1.0),       # mol/m²/s
    'LAI':      (0.5, 8.0),        # m²/m²
    'K_drain':  (0.005, 0.5),      # 1/hr
    'C_wall':   (50000.0, 5e6),    # J/K
    'eta_hvac': (0.3, 1.0),        # -
}


def bounds_as_tuples():
    """Return bounds as list of (lower, upper) tuples in canonical order."""
    from .params import PARAM_NAMES
    return [PARAM_BOUNDS[name] for name in PARAM_NAMES]


def clip_params(params_array):
    """Clip parameter array to physical bounds."""
    import numpy as np
    from .params import PARAM_NAMES
    clipped = params_array.copy()
    for i, name in enumerate(PARAM_NAMES):
        lo, hi = PARAM_BOUNDS[name]
        clipped[i] = np.clip(clipped[i], lo, hi)
    return clipped


def bounds_with_substrate(substrate=None):
    """Return PARAM_BOUNDS with K_drain bounds from substrate profile.

    Args:
        substrate: SubstrateProfile or None.

    Returns:
        Dict of parameter bounds.
    """
    bounds = dict(PARAM_BOUNDS)
    if substrate is not None:
        bounds['K_drain'] = substrate.K_drain_bounds
    return bounds
