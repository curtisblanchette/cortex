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
        container_volume_L: Per-plant container volume in liters.
        theta_fc: Field capacity (m3/m3) — max water held against gravity.
        theta_wp: Wilting point (m3/m3) — stomata fully close below this.
        theta_drain: Drainage threshold (m3/m3) — drainage starts above this.
                     Typically equals theta_fc for most substrates.
        K_drain_default: Default drainage coefficient (1/hr) for this substrate.
        K_drain_bounds: (lower, upper) bounds for SLSQP calibration of K_drain.
        porosity: Total porosity (m3/m3) — physical upper limit of theta.
    """
    name: str
    container_volume_L: float
    theta_fc: float
    theta_wp: float
    theta_drain: float
    K_drain_default: float
    K_drain_bounds: tuple  # (lower, upper)
    porosity: float
    irr_flow_ml_s: float = 50.0         # per-plant irrigation flow rate (ml/s)
    shot_duration_s: float = 15.0       # seconds per irrigation event
    min_interval_min: float = 45.0      # minimum minutes between events
    drain_settle_min: float = 5.0       # cooldown after shot for drainage
    max_daily_shots: int = 12           # hard daily cap


# ---- Built-in profiles ----
SUBSTRATE_PROFILES = {
    'coco_coir': SubstrateProfile(
        name='coco_coir',
        container_volume_L=15.0,
        theta_fc=0.40,
        theta_wp=0.10,
        theta_drain=0.40,
        K_drain_default=0.05,
        K_drain_bounds=(0.005, 0.5),
        porosity=0.95,
        irr_flow_ml_s=50.0,        # drip stakes, ~18 L/hr per plant
        shot_duration_s=15.0,
        min_interval_min=45.0,
        drain_settle_min=5.0,
        max_daily_shots=12,
    ),
    'rockwool': SubstrateProfile(
        name='rockwool',
        container_volume_L=4.0,
        theta_fc=0.55,
        theta_wp=0.15,
        theta_drain=0.55,
        K_drain_default=0.10,
        K_drain_bounds=(0.01, 1.0),
        porosity=0.97,
        irr_flow_ml_s=15.0,        # precision drip, ~4 L/hr per plant
        shot_duration_s=8.0,
        min_interval_min=30.0,
        drain_settle_min=3.0,
        max_daily_shots=16,
    ),
    'perlite': SubstrateProfile(
        name='perlite',
        container_volume_L=15.0,
        theta_fc=0.25,
        theta_wp=0.05,
        theta_drain=0.25,
        K_drain_default=0.20,
        K_drain_bounds=(0.05, 2.0),
        porosity=0.90,
        irr_flow_ml_s=45.0,        # drip emitters, ~16 L/hr per plant
        shot_duration_s=10.0,
        min_interval_min=20.0,
        drain_settle_min=2.0,
        max_daily_shots=20,
    ),
    'soil_bed': SubstrateProfile(
        name='soil_bed',
        container_volume_L=250.0,  # deep bed allocation per plant
        theta_fc=0.35,          # loamy growing mix field capacity
        theta_wp=0.15,          # permanent wilting point
        theta_drain=0.35,       # drainage above field capacity
        K_drain_default=0.03,   # slow percolation through deep soil column
        K_drain_bounds=(0.005, 0.2),
        porosity=0.55,          # mineral soil has less air space than soilless
        irr_flow_ml_s=125.0,       # flood/high-flow drip, ~450 L/hr per plant
        shot_duration_s=60.0,
        min_interval_min=120.0,
        drain_settle_min=15.0,
        max_daily_shots=4,
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
        container_volume_L=overrides.get('container_volume_L', base.container_volume_L),
        theta_fc=overrides.get('theta_fc', base.theta_fc),
        theta_wp=overrides.get('theta_wp', base.theta_wp),
        theta_drain=overrides.get('theta_drain', base.theta_drain),
        K_drain_default=overrides.get('K_drain_default', base.K_drain_default),
        K_drain_bounds=tuple(overrides.get('K_drain_bounds',
                                           list(base.K_drain_bounds))),
        porosity=overrides.get('porosity', base.porosity),
        irr_flow_ml_s=overrides.get('irr_flow_ml_s', base.irr_flow_ml_s),
        shot_duration_s=overrides.get('shot_duration_s', base.shot_duration_s),
        min_interval_min=overrides.get('min_interval_min', base.min_interval_min),
        drain_settle_min=overrides.get('drain_settle_min', base.drain_settle_min),
        max_daily_shots=overrides.get('max_daily_shots', base.max_daily_shots),
    )
