"""Substrate profile — physical properties of the growing medium.

A frozen dataclass describing the hydraulic and physical characteristics
of the substrate. Loaded from config/substrate.yaml at startup and
passed to dynamics, transpiration, and the irrigation plugin.

Not a plugin (no control vector, no ABC). Just a parameter set.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SubstrateProfile:
    """Physical properties of a substrate type.

    Attributes:
        name: Human-readable profile name.
        depth_m: Substrate slab/container depth in meters.
        theta_fc: Field capacity (m3/m3) — max water held against gravity.
        theta_wp: Wilting point (m3/m3) — stomata fully close below this.
        theta_drain: Drainage threshold (m3/m3) — drainage starts above this.
                     Typically equals theta_fc for most substrates.
        K_drain_default: Default drainage coefficient (1/hr) for this substrate.
        K_drain_bounds: (lower, upper) bounds for SLSQP calibration of K_drain.
        porosity: Total porosity (m3/m3) — physical upper limit of theta.
    """
    name: str
    depth_m: float
    theta_fc: float
    theta_wp: float
    theta_drain: float
    K_drain_default: float
    K_drain_bounds: tuple  # (lower, upper)
    porosity: float


# ---- Built-in profiles ----
SUBSTRATE_PROFILES = {
    'coco_coir': SubstrateProfile(
        name='coco_coir',
        depth_m=0.15,
        theta_fc=0.40,
        theta_wp=0.10,
        theta_drain=0.40,
        K_drain_default=0.05,
        K_drain_bounds=(0.005, 0.5),
        porosity=0.95,
    ),
    'rockwool': SubstrateProfile(
        name='rockwool',
        depth_m=0.10,
        theta_fc=0.55,
        theta_wp=0.15,
        theta_drain=0.55,
        K_drain_default=0.10,
        K_drain_bounds=(0.01, 1.0),
        porosity=0.97,
    ),
    'perlite': SubstrateProfile(
        name='perlite',
        depth_m=0.20,
        theta_fc=0.25,
        theta_wp=0.05,
        theta_drain=0.25,
        K_drain_default=0.20,
        K_drain_bounds=(0.05, 2.0),
        porosity=0.90,
    ),
}

DEFAULT_PROFILE_NAME = 'coco_coir'


def load_substrate_profile(substrate_config=None,
                           profile_override=None):
    """Load a SubstrateProfile from config dict and/or CLI override.

    Resolution order:
    1. profile_override (CLI --substrate flag)
    2. substrate_config['profile'] from YAML
    3. DEFAULT_PROFILE_NAME fallback

    If the resolved name matches a built-in profile, use that as the base
    and overlay any explicit fields from the YAML config.

    Args:
        substrate_config: Dict from config/substrate.yaml (may be None).
        profile_override: CLI --substrate override string.

    Returns:
        SubstrateProfile instance.
    """
    config = substrate_config or {}

    # Resolve profile name
    if profile_override:
        profile_name = profile_override
    else:
        profile_name = config.get('profile', DEFAULT_PROFILE_NAME)

    if profile_name not in SUBSTRATE_PROFILES:
        raise ValueError(
            f"Unknown substrate profile: {profile_name}. "
            f"Available: {list(SUBSTRATE_PROFILES.keys())}"
        )

    base = SUBSTRATE_PROFILES[profile_name]

    # Overlay any explicit config values
    overrides = config.get('properties', {})
    if not overrides:
        return base

    return SubstrateProfile(
        name=profile_name,
        depth_m=overrides.get('depth_m', base.depth_m),
        theta_fc=overrides.get('theta_fc', base.theta_fc),
        theta_wp=overrides.get('theta_wp', base.theta_wp),
        theta_drain=overrides.get('theta_drain', base.theta_drain),
        K_drain_default=overrides.get('K_drain_default', base.K_drain_default),
        K_drain_bounds=tuple(overrides.get('K_drain_bounds',
                                           list(base.K_drain_bounds))),
        porosity=overrides.get('porosity', base.porosity),
    )
