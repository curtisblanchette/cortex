"""Advisory data model and severity constants."""

import uuid
import time
from dataclasses import dataclass, field, asdict
from typing import Optional


# Severity tiers — based on time-to-violation
SEVERITY_INFO = 'info'          # 30-60 min out, low confidence
SEVERITY_WARNING = 'warning'    # 10-30 min out, actionable
SEVERITY_CRITICAL = 'critical'  # <10 min or currently violating

SEVERITY_ORDER = {SEVERITY_INFO: 0, SEVERITY_WARNING: 1, SEVERITY_CRITICAL: 2}

# Advisory types
TYPE_ENV_BREACH = 'env_breach'
TYPE_ENV_TREND = 'env_trend'            # trend extrapolation beyond MPC horizon
TYPE_LIGHT_TRANSITION = 'light_transition'
TYPE_MODEL_DRIFT = 'model_drift'
TYPE_SOLVER_DEGRADED = 'solver_degraded'
TYPE_COMPLIANCE_DECLINING = 'compliance_declining'
TYPE_CO2_FLOW = 'co2_flow'
TYPE_LEAF_TEMP_DEFICIT = 'leaf_temp_deficit'
TYPE_INTEGRAL_SATURATED = 'integral_saturated'

# Channels
CHANNELS = ('VPD', 'T', 'RH', 'CO2', 'theta')

# Default TTLs (seconds)
DEFAULT_TTLS = {
    TYPE_ENV_BREACH: 600,
    TYPE_ENV_TREND: 900,
    TYPE_LIGHT_TRANSITION: 3600,
    TYPE_MODEL_DRIFT: 300,
    TYPE_SOLVER_DEGRADED: 300,
    TYPE_COMPLIANCE_DECLINING: 900,
    TYPE_CO2_FLOW: 600,
    TYPE_LEAF_TEMP_DEFICIT: 600,
    TYPE_INTEGRAL_SATURATED: 900,
}

# Cooldown before re-firing same (type, channel) pair
DEFAULT_COOLDOWN_S = 300


def severity_from_eta(eta_s):
    """Map time-to-violation to severity tier."""
    if eta_s is None or eta_s < 600:
        return SEVERITY_CRITICAL
    elif eta_s < 1800:
        return SEVERITY_WARNING
    else:
        return SEVERITY_INFO


def confidence_from_step(step_index, horizon_steps):
    """Confidence degrades linearly with horizon distance."""
    return max(0.3, 1.0 - 0.5 * (step_index / max(horizon_steps, 1)))


@dataclass
class Advisory:
    type: str
    severity: str
    title: str
    body: str
    recommendation: str
    channel: Optional[str] = None
    eta_s: Optional[float] = None
    current_value: Optional[float] = None
    predicted_value: Optional[float] = None
    target_value: Optional[float] = None
    tolerance: Optional[float] = None
    confidence: float = 1.0
    room_id: str = 'room1'
    phase: str = ''
    sim_time_s: Optional[float] = None  # simulation time in seconds
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    created_at: float = field(default_factory=time.time)
    expires_at: float = 0.0
    resolved: bool = False

    def __post_init__(self):
        if self.expires_at == 0.0:
            ttl = DEFAULT_TTLS.get(self.type, 600)
            self.expires_at = self.created_at + ttl

    def to_dict(self):
        return asdict(self)

    def is_expired(self, now=None):
        return (now or time.time()) > self.expires_at

    def is_higher_severity(self, other):
        return SEVERITY_ORDER.get(self.severity, 0) > SEVERITY_ORDER.get(other.severity, 0)
