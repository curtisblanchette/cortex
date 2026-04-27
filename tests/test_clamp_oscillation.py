"""Regression tests for the exclusive-pair clamp soft hysteresis.

The QP returns plans that use both pair members on most solves (linearization
split — see docs/decisions-recent.md). The post-solve clamp picks a winner.
Without hysteresis, when the two pre-clamp values are nearly equal the
winner can flip across solves; soft hysteresis (`hysteresis_margin`) keeps
the previous winner inside the close band.

These tests pin three invariants:
  1. Legacy behaviour preserved when `hysteresis_margin = 0.0`.
  2. With margin > 0, equal-ish values don't flip across solves.
  3. Per-pair fire and flip counters update correctly.
"""

from unittest.mock import MagicMock

import numpy as np

from mpc.constraints import ConstraintBuilder


def _registry_with_pair(channels):
    """Build a minimal registry mock exposing channel_names + bounds."""
    n = len(channels)
    reg = MagicMock()
    reg.channel_names = channels
    reg.n_u = n
    reg.u_min = np.zeros(n)
    reg.u_max = np.full(n, 100.0)
    reg.du_min = np.full(n, -100.0)
    reg.du_max = np.full(n, 100.0)
    return reg


def _builder(margin):
    reg = _registry_with_pair(['u_heat', 'u_cool'])
    cb = ConstraintBuilder(reg, clamp_config={'hysteresis_margin': margin})
    # Sanity: the heat/cool pair is detected
    assert len(cb.exclusive_pairs) == 1
    return cb


def test_no_hysteresis_when_margin_is_zero():
    """Margin=0 → legacy behaviour: always pick the larger value."""
    cb = _builder(0.0)
    u = np.array([40.0, 41.0])  # heat=40, cool=41 → cool wins
    cb.clamp_exclusive(u)
    assert u[0] == 0.0 and u[1] == 41.0

    u = np.array([41.0, 40.0])  # values flip → heat wins now
    cb.clamp_exclusive(u)
    assert u[0] == 41.0 and u[1] == 0.0


def test_hysteresis_holds_winner_inside_close_band():
    """With margin=5%, values within 5% of each other keep the previous winner."""
    cb = _builder(0.05)

    # First solve: heat=40, cool=41 → cool wins (1/41 = 2.4%, within margin
    # but no prior winner so larger wins).
    u = np.array([40.0, 41.0])
    cb.clamp_exclusive(u)
    assert u[1] == 41.0 and u[0] == 0.0

    # Second solve: heat=41, cool=40 (within margin). Hysteresis should hold
    # cool as winner despite heat now being marginally larger.
    u = np.array([41.0, 40.0])
    cb.clamp_exclusive(u)
    assert u[1] == 40.0 and u[0] == 0.0  # cool still wins


def test_hysteresis_releases_outside_close_band():
    """When the gap exceeds margin, the larger member wins regardless."""
    cb = _builder(0.05)
    # Lock in cool as winner.
    cb.clamp_exclusive(np.array([40.0, 41.0]))
    # Now heat dominates by 20% — well outside margin → heat wins.
    u = np.array([50.0, 40.0])
    cb.clamp_exclusive(u)
    assert u[0] == 50.0 and u[1] == 0.0


def test_metrics_track_fires_and_flips():
    """Counter invariants for clamp_metrics()."""
    cb = _builder(0.0)
    # Three solves, all with both > 0:
    cb.clamp_exclusive(np.array([10.0, 20.0]))  # cool wins (fire 1)
    cb.clamp_exclusive(np.array([10.0, 20.0]))  # cool wins (fire 2, no flip)
    cb.clamp_exclusive(np.array([20.0, 10.0]))  # heat wins (fire 3, flip 1)

    metrics = cb.clamp_metrics()
    assert 'u_heat__u_cool' in metrics
    m = metrics['u_heat__u_cool']
    assert m['fires'] == 3
    assert m['flips'] == 1
    assert m['winner'] == 'a'  # heat


def test_no_fire_when_only_one_member_active():
    """Single-member solves don't increment the counter."""
    cb = _builder(0.0)
    cb.clamp_exclusive(np.array([0.0, 30.0]))
    cb.clamp_exclusive(np.array([30.0, 0.0]))
    m = cb.clamp_metrics()['u_heat__u_cool']
    assert m['fires'] == 0
    assert m['flips'] == 0


def test_reset_clears_state_and_counters():
    cb = _builder(0.05)
    cb.clamp_exclusive(np.array([40.0, 41.0]))
    cb.clamp_exclusive(np.array([20.0, 30.0]))
    cb.reset_clamp_metrics()
    m = cb.clamp_metrics()['u_heat__u_cool']
    assert m['fires'] == 0
    assert m['flips'] == 0
    assert m['winner'] is None


def test_pair_inactive_when_channels_absent_from_registry():
    """Profiles without u_heat/u_cool/u_hum/u_dehum get no pairs."""
    reg = _registry_with_pair(['u_cool'])  # ac_minimal-style
    cb = ConstraintBuilder(reg, clamp_config={'hysteresis_margin': 0.05})
    assert cb.exclusive_pairs == []
    # No-op clamp
    u = np.array([50.0])
    cb.clamp_exclusive(u)
    assert u[0] == 50.0
    assert cb.clamp_metrics() == {}
