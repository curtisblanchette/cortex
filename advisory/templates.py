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
