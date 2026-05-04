"""Advisory message templates.

Each template function returns (title, body, recommendation) given
the detection context. Templates are grower-facing — plain language,
actionable, with enough context for remote monitoring.
"""


def _fmt_eta(eta_s):
    """Format ETA as human-readable string."""
    if eta_s is None:
        return 'now'
    m = int(eta_s / 60)
    if m < 1:
        return '<1 minute'
    if m == 1:
        return '1 minute'
    return f'{m} minutes'


def _direction(current, target):
    """Return 'rising' or 'dropping' relative to target."""
    return 'rising above' if current > target else 'dropping below'


def _trend_str(trend, unit):
    """Format trend as signed rate string."""
    if trend is None:
        return ''
    return f'Trend: {trend:+.3f} {unit}/min'


# --- env_breach templates per channel ---

def vpd_breach(current, predicted, target, tolerance, eta_s, trend=None):
    direction = _direction(predicted, target)
    title = f'VPD {direction} target'
    body = (
        f'VPD predicted to reach {predicted:.3f} kPa in {_fmt_eta(eta_s)}.\n'
        f'Current: {current:.3f} kPa  \u2192  Target: {target:.2f} \u00b1 {tolerance:.2f} kPa'
    )
    if trend is not None:
        body += f'\n{_trend_str(trend, "kPa")}'
    if predicted > target:
        rec = 'Increase dehumidification or lower room temperature 1-2\u00b0C to reduce VPD.'
    else:
        rec = 'Increase humidification or raise room temperature to increase VPD.'
    return title, body, rec


def temperature_breach(current, predicted, target, tolerance, eta_s, trend=None):
    direction = 'rising' if predicted > target else 'dropping'
    title = f'Room temperature {direction}'
    body = (
        f'Temperature predicted to reach {predicted:.1f}\u00b0C in {_fmt_eta(eta_s)}.\n'
        f'Current: {current:.1f}\u00b0C  \u2192  Target: {target:.1f} \u00b1 {tolerance:.1f}\u00b0C'
    )
    if trend is not None:
        body += f'\n{_trend_str(trend, "\u00b0C")}'
    if predicted > target:
        rec = 'Increase cooling capacity. Verify AC is operating normally.'
    else:
        rec = 'Increase heating. Check for unexpected ventilation or open doors.'
    return title, body, rec


def rh_breach(current, predicted, target, tolerance, eta_s, trend=None):
    direction = _direction(predicted, target)
    title = f'Humidity {direction} target'
    body = (
        f'RH predicted to reach {predicted:.1f}% in {_fmt_eta(eta_s)}.\n'
        f'Current: {current:.1f}%  \u2192  Target: {target:.1f} \u00b1 {tolerance:.1f}%'
    )
    if trend is not None:
        body += f'\n{_trend_str(trend, "%")}'
    if predicted > target:
        rec = 'Increase dehumidification. High RH increases mold and pathogen risk.'
    else:
        rec = 'Reduce dehumidification or add humidification.'
    return title, body, rec


def co2_breach(current, predicted, target, tolerance, eta_s, trend=None):
    direction = _direction(predicted, target)
    title = f'CO2 {direction} target'
    body = (
        f'CO2 predicted to reach {predicted:.0f} ppm in {_fmt_eta(eta_s)}.\n'
        f'Current: {current:.0f} ppm  \u2192  Target: {target:.0f} \u00b1 {tolerance:.0f} ppm'
    )
    if trend is not None:
        body += f'\n{_trend_str(trend, "ppm")}'
    if predicted > target:
        rec = 'Reduce CO2 injection rate.'
    else:
        rec = 'Increase CO2 injection. Verify tank level and regulator.'
    return title, body, rec


def theta_breach(current, predicted, target, tolerance, eta_s, trend=None):
    direction = 'drying' if predicted < target else 'saturating'
    title = f'Substrate moisture {direction}'
    body = (
        f'Substrate moisture predicted to reach {predicted:.3f} m\u00b3/m\u00b3 in {_fmt_eta(eta_s)}.\n'
        f'Current: {current:.3f}  \u2192  Target: {target:.3f} \u00b1 {tolerance:.3f}'
    )
    if trend is not None:
        body += f'\n{_trend_str(trend, "m\u00b3/m\u00b3")}'
    if predicted < target:
        rec = 'Schedule irrigation. Substrate approaching dry-back threshold.'
    else:
        rec = 'Reduce irrigation frequency. Substrate is over-saturated.'
    return title, body, rec


ENV_BREACH_TEMPLATES = {
    'VPD': vpd_breach,
    'T': temperature_breach,
    'RH': rh_breach,
    'CO2': co2_breach,
    'theta': theta_breach,
}

# Channel display units for trend extrapolation
_CHANNEL_UNITS = {
    'VPD': 'kPa', 'T': '\u00b0C', 'RH': '%', 'CO2': 'ppm', 'theta': 'm\u00b3/m\u00b3',
}
_CHANNEL_FMT = {
    'VPD': '.3f', 'T': '.1f', 'RH': '.1f', 'CO2': '.0f', 'theta': '.3f',
}


def env_trend(channel, current, projected, target, tolerance, eta_s, trend_per_min):
    """Trend extrapolation advisory — breach predicted beyond MPC horizon."""
    unit = _CHANNEL_UNITS.get(channel, '')
    fmt = _CHANNEL_FMT.get(channel, '.2f')
    direction = _direction(projected, target)
    title = f'{channel} trending toward breach'
    body = (
        f'{channel} trending {direction} target at {trend_per_min:{fmt}} {unit}/min.\n'
        f'Current: {current:{fmt}} {unit}  \u2192  Target: {target:{fmt}} \u00b1 {tolerance:{fmt}} {unit}\n'
        f'Estimated breach in {_fmt_eta(eta_s)} (trend extrapolation, lower confidence).'
    )
    # Reuse the same recommendations as env_breach
    template_fn = ENV_BREACH_TEMPLATES.get(channel)
    if template_fn:
        _, _, rec = template_fn(current, projected, target, tolerance, eta_s, trend_per_min)
    else:
        rec = f'Monitor {channel} closely.'
    return title, body, rec


# --- light_transition ---

def light_transition(lights_turning_on, eta_s, day_targets, night_targets, ramp_min):
    if lights_turning_on:
        title = f'Lights on in {_fmt_eta(eta_s)}'
        rec = 'Prepare for heat load increase. Verify cooling capacity is available.'
    else:
        title = f'Lights off in {_fmt_eta(eta_s)}'
        rec = ('Prepare for temperature drop and VPD shift. '
               'Verify heating and dehumidification are ready.')
    body = (
        f'Day targets:   T={day_targets["T"]:.1f}\u00b0C, VPD={day_targets["VPD"]:.2f}, '
        f'RH={day_targets["RH"]:.1f}%, CO2={day_targets["CO2"]:.0f} ppm\n'
        f'Night targets: T={night_targets["T"]:.1f}\u00b0C, VPD={night_targets["VPD"]:.2f}, '
        f'RH={night_targets["RH"]:.1f}%, CO2={night_targets["CO2"]:.0f} ppm\n'
        f'Ramp duration: {ramp_min} minutes'
    )
    return title, body, rec


# --- model_drift ---

def model_drift(innovation_rms, duration_s):
    title = 'Sensor readings diverging from model'
    body = (
        f'EKF innovation RMS at {innovation_rms:.2f} (normal < 1.0) '
        f'for {int(duration_s)} seconds.\n'
        f'The model\'s predictions are not matching sensor observations.'
    )
    rec = ('Check sensors for drift or obstruction. If readings look correct, '
           'room conditions may have changed (door open, equipment failure, '
           'unexpected load).')
    return title, body, rec


# --- solver_degraded ---

def solver_degraded(status, consecutive_count, confidence):
    title = 'Control model accuracy degraded'
    body = (
        f'MPC solver returning "{status}" for {consecutive_count} '
        f'consecutive solves.\n'
        f'Predictions may be unreliable. Confidence: {confidence:.0%}'
    )
    rec = ('Advisory predictions may be less accurate. '
           'Monitor conditions more closely. '
           'If persistent, the system may need recalibration.')
    return title, body, rec


# --- compliance_declining ---

def co2_flow_low(actual_rate, expected_rate):
    pct = (actual_rate / expected_rate) * 100 if expected_rate > 0 else 0
    title = 'CO2 flow below expected'
    body = (
        f'CO2 injection rate measured at {actual_rate:.2f} ppm/s '
        f'({pct:.0f}% of expected {expected_rate:.1f} ppm/s).\n'
        f'Tank pressure may be low or regulator may need adjustment.'
    )
    rec = 'Check CO2 tank gauge and regulator output pressure.'
    return title, body, rec


def co2_flow_none():
    title = 'CO2 injection not responding'
    body = (
        'Solenoid is commanded ON but CO2 level is not rising.\n'
        'Possible causes: empty tank, closed valve, solenoid failure.'
    )
    rec = ('Immediately check CO2 tank level and solenoid operation. '
           'Verify regulator is not frozen or blocked.')
    return title, body, rec


def leaf_temp_deficit(deficit, traj_mean_deficit, lights_on):
    """Leaf temperature deficit advisory.

    deficit         = T_air - T_leaf (°C), current estimate
    traj_mean_deficit = mean deficit over prediction horizon (°C)
    lights_on       = whether lights are currently on
    """
    if deficit > 0:
        direction = 'high'
        body_cause = (
            'A large leaf-to-air temperature gap indicates excessive transpiration '
            'or poor canopy airflow — leaves are over-cooling relative to air.'
        )
        if deficit > 7.0:
            severity = 'critical'
            title = f'Leaf temperature deficit critical ({deficit:.1f}°C)'
            rec = (
                'Check canopy fan operation immediately. '
                'Consider reducing VPD target or increasing airflow. '
                'Severe deficit can indicate stomatal stress or light burn.'
            )
        elif deficit > 5.0:
            severity = 'warning'
            title = f'Leaf temperature deficit elevated ({deficit:.1f}°C)'
            rec = (
                'Verify canopy fans are running at target speed. '
                'Consider a small VPD reduction (0.1–0.2 kPa) to ease transpiration load.'
            )
        else:
            severity = 'info'
            title = f'Leaf temperature deficit above normal ({deficit:.1f}°C)'
            rec = 'Monitor airflow and VPD. Deficit may be transient during high-transpiration periods.'
    else:
        direction = 'reversed'
        body_cause = (
            'Leaf temperature is warmer than air — stomata may be partially closed '
            '(drought or salt stress) or a radiant heat source is warming leaves directly.'
        )
        severity = 'warning'
        title = f'Leaf temperature reversed — leaf warmer than air ({deficit:.1f}°C)'
        rec = (
            'Check substrate moisture and root zone. '
            'Verify no direct radiant heat source near canopy. '
            'May indicate stomatal closure from drought or high EC.'
        )

    phase_str = 'lights-on' if lights_on else 'lights-off'
    body = (
        f'Current T_air − T_leaf = {deficit:.1f}°C ({phase_str}). '
        f'Healthy range: 1–3°C.\n'
        f'Predicted mean over MPC horizon: {traj_mean_deficit:.1f}°C.\n'
        f'{body_cause}'
    )
    return severity, title, body, rec


def integral_saturated(channel, value, clamp, duration_s, unit):
    """Reference-bias integral has been pegged at clamp for an extended period.

    A saturated integral means the MPC cannot reach the setpoint with its
    in-horizon control authority and is leaning on the bias to compensate.
    The bias is bounded by clamp, so once it pegs, residual tracking error
    is absorbed by the operator instead of the controller.

    channel    : 'T' or 'VPD'
    value      : current integral magnitude
    clamp      : configured clamp magnitude (positive)
    duration_s : seconds the integral has been at clamp
    unit       : '°C' or 'kPa' for human-readable body
    """
    sign = '+' if value > 0 else '-'
    minutes = max(1, int(duration_s / 60))
    title = f'{channel} reference-bias saturated ({sign}{abs(value):.2f}{unit})'
    body = (
        f'{channel} integral has been at its ±{clamp:g}{unit} clamp for {minutes} '
        f'minute{"s" if minutes != 1 else ""}.\n'
        f'The MPC is requesting more bias than configured to track the setpoint, '
        f'which means an in-horizon control gap is being absorbed by the bias.'
    )
    if channel == 'T':
        rec = (
            'Likely causes: outdoor load exceeds equipment capacity, wall thermal '
            'mass is mistuned, or the room.yaml outdoor preset does not match '
            'reality. Check actual vs. modeled T_out, equipment effort, and '
            'consider raising integral_action.t_clamp_C if equipment has headroom.'
        )
    else:
        rec = (
            'Likely causes: dehumidification capacity exceeded, outdoor humidity '
            'higher than modeled, or transpiration model under-calibrated. Check '
            'dehumidifier effort and outdoor w_out vs. expected.'
        )
    return title, body, rec


def compliance_declining(channel, pct, mean, std, min_val, max_val, tolerance):
    title = f'{channel} compliance declining'
    body = (
        f'{channel} compliance at {pct:.1f}% over the last hour (target: >95%).\n'
        f'Mean: {mean:.3f}, Std: {std:.3f}, Range: [{min_val:.3f}, {max_val:.3f}]\n'
        f'Tolerance: \u00b1{tolerance}'
    )
    rec = (f'Review {channel} setpoints and equipment capacity. '
           f'Sustained low compliance indicates the system cannot maintain '
           f'target conditions.')
    return title, body, rec
