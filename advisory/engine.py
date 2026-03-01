"""Advisory engine — scans MPC predictions and system state to produce
early-warning advisories for growers.

Consumes solver output each cycle, detects upcoming compliance breaches,
light transitions, model drift, and solver health issues. Manages
deduplication and advisory lifecycle.
"""

import time
import logging
import numpy as np

from .models import (
    Advisory, SEVERITY_ORDER, DEFAULT_COOLDOWN_S,
    TYPE_ENV_BREACH, TYPE_ENV_TREND, TYPE_LIGHT_TRANSITION, TYPE_MODEL_DRIFT,
    TYPE_SOLVER_DEGRADED, TYPE_COMPLIANCE_DECLINING,
    severity_from_eta, confidence_from_step,
)
from . import templates

logger = logging.getLogger(__name__)

# Trajectory column indices (matching dynamics state vector)
_STATE_T = 0
_STATE_WA = 1
_STATE_CO2 = 2
_STATE_TL = 3
_STATE_THETA = 4

# Channels and their mapping to trajectory data
# VPD and RH are derived (not direct state columns), handled separately
_DIRECT_CHANNELS = {
    'T': _STATE_T,
    'CO2': _STATE_CO2,
    'theta': _STATE_THETA,
}

# Model drift detection
_INNOVATION_THRESHOLD = 2.0
_INNOVATION_SUSTAIN_S = 60

# Solver degradation
_SOLVER_FAIL_THRESHOLD = 3

# Compliance floor
_COMPLIANCE_FLOOR_PCT = 90.0


class AdvisoryEngine:
    """Scans MPC trajectories and system state, produces Advisory objects.

    Call evaluate() after each MPC solve. The engine handles deduplication,
    cooldowns, severity upgrades, and auto-expiry internally.
    """

    def __init__(self, phase_config, reference, dynamics,
                 room_id='room1', cooldown_s=DEFAULT_COOLDOWN_S):
        self.room_id = room_id
        self.cooldown_s = cooldown_s
        self.dynamics = dynamics
        self.reference = reference

        # Phase metadata
        self.phase_name = phase_config.get('phase', '')
        self.ramp_minutes = phase_config.get('ramp_minutes', 15)

        # Compliance tolerances
        compliance = phase_config.get('compliance', {})
        self.tolerances = {
            'VPD': compliance.get('VPD_tolerance', 0.15),
            'T': compliance.get('T_tolerance', 1.0),
            'CO2': compliance.get('CO2_tolerance', 100.0),
            'theta': compliance.get('theta_tolerance', 0.05),
            'RH': compliance.get('RH_tolerance', 5.0),
        }

        # Day/night targets for light transition advisories
        day = phase_config.get('day', {})
        night = phase_config.get('night', {})
        self._day_targets = {
            'T': day.get('T_target', 26.0),
            'VPD': day.get('VPD_target', 1.2),
            'RH': day.get('RH_target', 60.0),
            'CO2': day.get('CO2_target', 1200.0),
        }
        self._night_targets = {
            'T': night.get('T_target', 22.0),
            'VPD': night.get('VPD_target', 0.8),
            'RH': night.get('RH_target', 55.0),
            'CO2': night.get('CO2_target', 800.0),
        }

        # Active advisories: key = (type, channel) → Advisory
        self._active = {}
        # Full history (append-only, for post-sim summary and API)
        self._history = []
        # Cooldown tracker: key = (type, channel) → expiry timestamp
        self._cooldowns = {}

        # Model drift state
        self._drift_start = None
        self._innovation_history = []  # [(time, rms), ...]
        self._innovation_window = 300  # 5 min window for trend detection

        # Solver degradation state
        self._consecutive_bad_solves = 0

        # Compliance history for slope prediction: channel → [(time, pct), ...]
        self._compliance_history = {ch: [] for ch in self.tolerances}
        self._compliance_window = 600  # 10 min rolling window for slope

        # Light transition tracking — last bracket fired per transition direction
        self._last_light_bracket = None
        self._last_light_direction = None

    def evaluate(self, trajectory, x_hat, ref_targets, solve_info,
                 current_time, light_schedule, step_s=60.0,
                 compliance_report=None, innovation_rms=0.0,
                 innovation_vector=None):
        """Run all advisory checks against current solver output.

        Args:
            trajectory: Predicted state trajectory (Np, 7).
            x_hat: Current state estimate (7,).
            ref_targets: Dict with current reference targets (VPD, T, CO2, theta, RH).
            solve_info: MPC solve diagnostics dict.
            current_time: Simulation/wall time in seconds.
            light_schedule: LightSchedule instance.
            step_s: Prediction step size in seconds.
            compliance_report: Optional dict from ComplianceTracker.report().
            innovation_rms: EKF innovation magnitude.
            innovation_vector: Raw EKF innovation per sensor (6,), for diagnostics.
        """
        now = time.time()
        self._current_sim_time = current_time
        self._expire_old(now)

        if trajectory is not None and ref_targets is not None:
            self._check_env_breach(
                trajectory, x_hat, ref_targets, step_s, current_time, now,
            )

        self._check_light_transition(
            current_time, light_schedule, now,
        )

        self._check_model_drift(innovation_rms, current_time, now, innovation_vector)

        self._check_solver_health(solve_info, now)

        if compliance_report:
            self._check_compliance(compliance_report, now)

    # -- Public API --

    def active_advisories(self):
        """Return list of active (non-expired, non-resolved) advisories."""
        now = time.time()
        return [a for a in self._active.values()
                if not a.is_expired(now) and not a.resolved]

    def all_advisories(self):
        """Return full advisory history."""
        return list(self._history)

    def resolve(self, advisory_type, channel=None):
        """Mark an advisory as resolved."""
        key = (advisory_type, channel)
        if key in self._active:
            self._active[key].resolved = True

    # -- Detection logic --

    def _check_env_breach(self, trajectory, x_hat, ref_targets, step_s,
                          current_time, now):
        """Scan predicted trajectory for compliance breaches.

        Two layers of detection:
        1. Direct breach: trajectory crosses tolerance within MPC horizon (30 min).
        2. Trend extrapolation: no breach in horizon, but end-of-horizon trend
           is heading toward the boundary. Linear extrapolation estimates when
           breach would occur (lower confidence, but much earlier warning).
        """
        Np = len(trajectory)
        horizon_s = Np * step_s

        # Compute derived outputs (VPD, RH) for each trajectory step
        traj_vpd = np.empty(Np)
        traj_rh = np.empty(Np)
        for i in range(Np):
            derived = self.dynamics.derived_outputs(trajectory[i])
            traj_vpd[i] = derived['VPD']
            traj_rh[i] = derived['RH']

        current_derived = self.dynamics.derived_outputs(x_hat)

        channel_data = {
            'VPD': (traj_vpd, current_derived['VPD']),
            'T': (trajectory[:, _STATE_T], x_hat[_STATE_T]),
            'CO2': (trajectory[:, _STATE_CO2], x_hat[_STATE_CO2]),
            'theta': (trajectory[:, _STATE_THETA], x_hat[_STATE_THETA]),
            'RH': (traj_rh, current_derived['RH']),
        }

        for channel, (traj_values, current_val) in channel_data.items():
            target = ref_targets.get(channel)
            tol = self.tolerances.get(channel)
            if target is None or tol is None:
                continue

            # --- Layer 1: direct breach within MPC horizon ---
            breach_step = None
            breach_value = None
            for k in range(Np):
                if abs(traj_values[k] - target) > tol:
                    breach_step = k
                    breach_value = float(traj_values[k])
                    break

            breach_key = (TYPE_ENV_BREACH, channel)
            trend_key = (TYPE_ENV_TREND, channel)

            if breach_step is not None:
                eta_s = breach_step * step_s
                severity = severity_from_eta(eta_s)
                conf = confidence_from_step(breach_step, Np)

                trend = None
                if Np >= 5:
                    trend = float(
                        (traj_values[4] - traj_values[0]) / (4 * step_s / 60.0)
                    )

                template_fn = templates.ENV_BREACH_TEMPLATES.get(channel)
                if template_fn is None:
                    continue

                title, body, rec = template_fn(
                    current=current_val,
                    predicted=breach_value,
                    target=target,
                    tolerance=tol,
                    eta_s=eta_s,
                    trend=trend,
                )

                advisory = Advisory(
                    type=TYPE_ENV_BREACH,
                    severity=severity,
                    channel=channel,
                    title=title,
                    body=body,
                    recommendation=rec,
                    eta_s=eta_s,
                    current_value=round(current_val, 4),
                    predicted_value=round(breach_value, 4),
                    target_value=target,
                    tolerance=tol,
                    confidence=conf,
                    room_id=self.room_id,
                    phase=self.phase_name,
                )
                self._emit(breach_key, advisory, now)
                # Clear trend advisory if we now have a direct breach
                if trend_key in self._active:
                    self._active[trend_key].resolved = True
                continue

            # No direct breach — resolve active breach advisory
            if breach_key in self._active and not self._active[breach_key].resolved:
                self._active[breach_key].resolved = True

            # --- Layer 2: trend extrapolation beyond horizon ---
            # Use the last quarter of the trajectory to compute a stable trend.
            quarter = max(5, Np // 4)
            tail = traj_values[-quarter:]
            tail_times = np.arange(quarter) * step_s / 60.0  # minutes
            if len(tail) < 2:
                continue

            # Linear regression on tail
            trend_per_min = float(
                (tail[-1] - tail[0]) / (tail_times[-1] - tail_times[0])
            ) if tail_times[-1] > tail_times[0] else 0.0

            end_value = float(traj_values[-1])
            end_error = end_value - target

            # Determine which boundary we'd cross
            if trend_per_min > 0:
                boundary = target + tol
            else:
                boundary = target - tol

            distance_to_boundary = boundary - end_value
            if abs(trend_per_min) < 1e-9:
                continue  # flat trend, no extrapolation

            # Time from end of horizon to boundary crossing
            extra_min = distance_to_boundary / trend_per_min
            if extra_min <= 0:
                continue  # trend heading away from boundary

            # Cap extrapolation at 4h beyond horizon (total ~4.5h)
            if extra_min > 240:
                continue

            total_eta_s = horizon_s + extra_min * 60.0
            projected_value = float(boundary)

            # Confidence drops with extrapolation distance
            conf = max(0.2, 0.6 - 0.3 * (extra_min / 240.0))

            title, body, rec = templates.env_trend(
                channel=channel,
                current=current_val,
                projected=projected_value,
                target=target,
                tolerance=tol,
                eta_s=total_eta_s,
                trend_per_min=trend_per_min,
            )

            advisory = Advisory(
                type=TYPE_ENV_TREND,
                severity=severity_from_eta(total_eta_s),
                channel=channel,
                title=title,
                body=body,
                recommendation=rec,
                eta_s=total_eta_s,
                current_value=round(current_val, 4),
                predicted_value=round(projected_value, 4),
                target_value=target,
                tolerance=tol,
                confidence=conf,
                room_id=self.room_id,
                phase=self.phase_name,
            )
            self._emit(trend_key, advisory, now)

    # Escalating brackets: fire once per threshold as ETA shrinks.
    # 4h → prep equipment, 2h → verify readiness, 1h → final check,
    # 30m → imminent, 10m → last call.
    _LIGHT_BRACKETS_S = [600, 1800, 3600, 2 * 3600, 4 * 3600]  # ascending

    def _check_light_transition(self, current_time, light_schedule, now):
        """Check for upcoming light transitions with escalating warnings."""
        lights_on = light_schedule.is_on(current_time)
        hour = (current_time / 3600.0) % 24.0

        if lights_on:
            target_hour = light_schedule.off_hour
            turning_on = False
        else:
            target_hour = light_schedule.on_hour
            turning_on = True

        hours_until = (target_hour - hour) % 24.0
        eta_s = hours_until * 3600.0

        # Reset bracket tracking on direction change (new transition approaching)
        direction = 'on' if turning_on else 'off'
        if direction != self._last_light_direction:
            self._last_light_bracket = None
            self._last_light_direction = direction

        # Find the tightest bracket eta_s has crossed into.
        # Ascending order: first match is the smallest bracket >= eta_s.
        bracket = None
        for threshold in self._LIGHT_BRACKETS_S:
            if eta_s <= threshold:
                bracket = threshold
                break

        if bracket is None:
            return  # more than 4h away

        if bracket == self._last_light_bracket:
            return  # already fired this bracket
        self._last_light_bracket = bracket

        # Severity escalates as transition approaches
        if eta_s > 3600:
            severity = 'info'
        elif eta_s > 600:
            severity = 'warning'
        else:
            severity = 'critical'

        key = (TYPE_LIGHT_TRANSITION, None)

        title, body, rec = templates.light_transition(
            lights_turning_on=turning_on,
            eta_s=eta_s,
            day_targets=self._day_targets,
            night_targets=self._night_targets,
            ramp_min=self.ramp_minutes,
        )

        advisory = Advisory(
            type=TYPE_LIGHT_TRANSITION,
            severity=severity,
            title=title,
            body=body,
            recommendation=rec,
            eta_s=eta_s,
            confidence=1.0,
            room_id=self.room_id,
            phase=self.phase_name,
            sim_time_s=current_time,
        )
        self._active[key] = advisory
        self._history.append(advisory)

    # Innovation thresholds
    _INNOVATION_WARN = 2.0    # sustained threshold for warning
    _INNOVATION_RISING = 1.2  # onset detection — early info advisory

    _SENSOR_NAMES = ['T_air', 'w_a', 'CO2', 'T_leaf', 'theta', 'T_supply']
    _SENSOR_UNITS = ['°C', 'g/kg', 'ppm', '°C', 'm³/m³', '°C']

    def _innovation_detail(self, innovation_vector):
        """Format per-sensor innovation breakdown for diagnostics."""
        if innovation_vector is None:
            return ''
        lines = []
        for i, (name, unit) in enumerate(zip(self._SENSOR_NAMES, self._SENSOR_UNITS)):
            if i < len(innovation_vector) and not np.isnan(innovation_vector[i]):
                val = innovation_vector[i]
                if abs(val) > 0.01:
                    lines.append(f'  {name}: {val:+.3f} {unit}')
        if not lines:
            return ''
        return '\nPer-sensor residuals (predicted - measured):\n' + '\n'.join(lines)

    def _check_model_drift(self, innovation_rms, current_time, now,
                           innovation_vector=None):
        """Detect model/sensor disagreement with two layers:

        1. Rising innovation trend: moving average crossing 1.2 → info advisory.
        2. Sustained high innovation (>2.0 for 60s) → warning advisory.

        Both layers include per-sensor innovation breakdown to identify
        which sensor or state is driving the disagreement.
        """
        key = (TYPE_MODEL_DRIFT, None)

        # Track innovation history
        self._innovation_history.append((current_time, innovation_rms))
        cutoff = current_time - self._innovation_window
        while self._innovation_history and self._innovation_history[0][0] < cutoff:
            self._innovation_history.pop(0)

        # Compute moving average
        if self._innovation_history:
            avg_innov = sum(v for _, v in self._innovation_history) / len(self._innovation_history)
        else:
            avg_innov = 0.0

        sensor_detail = self._innovation_detail(innovation_vector)

        # Layer 2: sustained high innovation → warning
        if innovation_rms > self._INNOVATION_WARN:
            if self._drift_start is None:
                self._drift_start = current_time
            duration = current_time - self._drift_start
            if duration >= _INNOVATION_SUSTAIN_S:
                title, body, rec = templates.model_drift(
                    innovation_rms, duration,
                )
                body += sensor_detail
                advisory = Advisory(
                    type=TYPE_MODEL_DRIFT,
                    severity='warning',
                    title=title,
                    body=body,
                    recommendation=rec,
                    confidence=0.8,
                    room_id=self.room_id,
                    phase=self.phase_name,
                )
                self._emit(key, advisory, now)
                return
        else:
            self._drift_start = None

        # Layer 1: rising innovation trend → early info
        if avg_innov > self._INNOVATION_RISING and innovation_rms > 1.0:
            hist = self._innovation_history
            if len(hist) >= 5:
                dt = hist[-1][0] - hist[0][0]
                if dt > 30:
                    slope = (hist[-1][1] - hist[0][1]) / dt
                    if slope > 0.001:
                        eta_s = (self._INNOVATION_WARN - avg_innov) / slope
                        if 0 < eta_s < 1800:
                            title = 'Model predictions starting to diverge'
                            body = (
                                f'EKF innovation RMS trending upward '
                                f'(avg: {avg_innov:.2f}, current: {innovation_rms:.2f}).\n'
                                f'Predicted to reach warning threshold in '
                                f'~{int(eta_s / 60)} minutes if trend continues.'
                            )
                            body += sensor_detail
                            rec = ('Monitor sensor readings. Early sign of '
                                   'changing room conditions or sensor drift.')
                            advisory = Advisory(
                                type=TYPE_MODEL_DRIFT,
                                severity='info',
                                title=title,
                                body=body,
                                recommendation=rec,
                                eta_s=eta_s,
                                confidence=0.5,
                                room_id=self.room_id,
                                phase=self.phase_name,
                            )
                            self._emit(key, advisory, now)
                            return

        # Healthy — resolve
        if avg_innov < 0.8:
            if key in self._active:
                self._active[key].resolved = True

    def _check_solver_health(self, solve_info, now):
        """Track consecutive bad solves with escalating severity.

        1 bad solve → info (heads up, may be transient cold-start).
        3+ consecutive → warning (inaccurate) or critical (failed).
        """
        key = (TYPE_SOLVER_DEGRADED, None)
        status = solve_info.get('status', 'solved')

        if status not in ('solved',):
            self._consecutive_bad_solves += 1
        else:
            self._consecutive_bad_solves = 0
            if key in self._active:
                self._active[key].resolved = True
            return

        count = self._consecutive_bad_solves

        if count == 1:
            # First bad solve — info level
            title, body, rec = templates.solver_degraded(status, count, 0.9)
            advisory = Advisory(
                type=TYPE_SOLVER_DEGRADED,
                severity='info',
                title=title,
                body=body,
                recommendation=rec,
                confidence=0.9,
                room_id=self.room_id,
                phase=self.phase_name,
            )
            self._emit(key, advisory, now)
        elif count >= _SOLVER_FAIL_THRESHOLD:
            # Sustained — escalate
            confidence = 0.7 if 'inaccurate' in status else 0.3
            title, body, rec = templates.solver_degraded(status, count, confidence)
            advisory = Advisory(
                type=TYPE_SOLVER_DEGRADED,
                severity='warning' if 'inaccurate' in status else 'critical',
                title=title,
                body=body,
                recommendation=rec,
                confidence=confidence,
                room_id=self.room_id,
                phase=self.phase_name,
            )
            self._emit(key, advisory, now)

    def _check_compliance(self, report, now):
        """Check for declining compliance — both current and predicted.

        Tracks compliance slope over a 10-min window. If compliance is above
        the floor but trending down, extrapolates when it will cross 90%
        and fires an early warning.
        """
        for channel, data in report.items():
            pct = data.get('compliance_pct', 100.0)
            key = (TYPE_COMPLIANCE_DECLINING, channel)

            # Track compliance history for slope detection
            if channel in self._compliance_history:
                hist = self._compliance_history[channel]
                hist.append((now, pct))
                # Trim to window
                cutoff = now - self._compliance_window
                while hist and hist[0][0] < cutoff:
                    hist.pop(0)

            # Already below floor — fire immediately
            if pct < _COMPLIANCE_FLOOR_PCT:
                title, body, rec = templates.compliance_declining(
                    channel=channel,
                    pct=pct,
                    mean=data.get('mean', 0),
                    std=data.get('std', 0),
                    min_val=data.get('min', 0),
                    max_val=data.get('max', 0),
                    tolerance=data.get('tolerance', 0),
                )
                advisory = Advisory(
                    type=TYPE_COMPLIANCE_DECLINING,
                    severity='warning' if pct >= 80 else 'critical',
                    channel=channel,
                    title=title,
                    body=body,
                    recommendation=rec,
                    confidence=1.0,
                    room_id=self.room_id,
                    phase=self.phase_name,
                )
                self._emit(key, advisory, now)
                continue

            # Above floor — check slope for early prediction
            hist = self._compliance_history.get(channel, [])
            if len(hist) >= 3:
                dt = hist[-1][0] - hist[0][0]
                if dt > 60:  # need at least 60s of data
                    slope_per_s = (hist[-1][1] - hist[0][1]) / dt
                    if slope_per_s < -0.001:  # declining
                        distance = pct - _COMPLIANCE_FLOOR_PCT
                        eta_s = distance / abs(slope_per_s)
                        # Only warn if predicted breach within 30 min
                        if eta_s < 1800:
                            title, body, rec = templates.compliance_declining(
                                channel=channel,
                                pct=pct,
                                mean=data.get('mean', 0),
                                std=data.get('std', 0),
                                min_val=data.get('min', 0),
                                max_val=data.get('max', 0),
                                tolerance=data.get('tolerance', 0),
                            )
                            body += (f'\nCompliance declining at '
                                     f'{abs(slope_per_s * 60):.1f}%/min. '
                                     f'Predicted to reach {_COMPLIANCE_FLOOR_PCT:.0f}% '
                                     f'in ~{int(eta_s / 60)} minutes.')
                            advisory = Advisory(
                                type=TYPE_COMPLIANCE_DECLINING,
                                severity='info',
                                channel=channel,
                                title=f'{channel} compliance trending down',
                                body=body,
                                recommendation=rec,
                                eta_s=eta_s,
                                confidence=0.6,
                                room_id=self.room_id,
                                phase=self.phase_name,
                            )
                            self._emit(key, advisory, now)
                            continue

            # Healthy — resolve
            if key in self._active:
                self._active[key].resolved = True

    # -- Lifecycle management --

    def _emit(self, key, advisory, now):
        """Emit an advisory, respecting cooldowns and severity upgrades."""
        advisory.sim_time_s = self._current_sim_time
        existing = self._active.get(key)

        # Severity upgrade bypasses cooldown
        if existing and not existing.resolved and advisory.is_higher_severity(existing):
            self._active[key] = advisory
            self._history.append(advisory)
            self._cooldowns[key] = now + self.cooldown_s
            logger.info(f'[Advisory] UPGRADE {key}: {advisory.severity} — {advisory.title}')
            return

        # Cooldown check
        if key in self._cooldowns and now < self._cooldowns[key]:
            return

        # Emit new advisory
        self._active[key] = advisory
        self._history.append(advisory)
        self._cooldowns[key] = now + self.cooldown_s
        logger.info(f'[Advisory] {advisory.severity.upper()} {key}: {advisory.title}')

    def _expire_old(self, now):
        """Remove expired advisories and stale cooldowns."""
        expired_keys = [k for k, a in self._active.items() if a.is_expired(now)]
        for k in expired_keys:
            del self._active[k]

        stale_cooldowns = [k for k, t in self._cooldowns.items() if now > t]
        for k in stale_cooldowns:
            del self._cooldowns[k]
