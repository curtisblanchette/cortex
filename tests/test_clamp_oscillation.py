"""Regression tests for the exclusive-pair clamp.

The clamp enforces non-convex mutual exclusivity by zeroing the smaller
pair member after the QP returns. Measurement on a 12hr ac_dehum sim
showed no sawtooth — winners are naturally stable because the QP runs
~95/5 (median pre-clamp gap 0.93), and flips are regime shifts not
tie-breaking. So no hysteresis ships; what does ship is per-pair fire/
flip counters that surface real-world clamp behaviour.

These tests pin three invariants:
  1. The legacy "always pick larger" decision is preserved.
  2. Per-pair fire and flip counters update correctly.
  3. Profiles without exclusive-pair channels are a clean no-op.
"""

from unittest.mock import MagicMock

import numpy as np

from mpc.constraints import ConstraintBuilder


def _registry_with(channels):
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


def _builder():
    cb = ConstraintBuilder(_registry_with(['u_heat', 'u_cool']))
    # The heat/cool pair is detected
    pair_channels = {(cb._channel_names[a], cb._channel_names[b])
                     for a, b, _ in cb.exclusive_pairs}
    assert ('u_heat', 'u_cool') in pair_channels
    return cb


def test_clamp_picks_larger_value():
    """Legacy behaviour: the larger pre-clamp value wins; tie → first member (a)."""
    cb = _builder()
    u = np.array([40.0, 41.0])  # heat=40, cool=41 → cool wins
    cb.clamp_exclusive(u)
    assert u[0] == 0.0 and u[1] == 41.0

    u = np.array([41.0, 40.0])  # values flip → heat wins now
    cb.clamp_exclusive(u)
    assert u[0] == 41.0 and u[1] == 0.0

    u = np.array([30.0, 30.0])  # tie → first member (heat) wins
    cb.clamp_exclusive(u)
    assert u[0] == 30.0 and u[1] == 0.0


def test_clamp_no_op_when_only_one_member_active():
    """When one pair member is already 0, the clamp leaves the other alone."""
    cb = _builder()
    u = np.array([0.0, 30.0])
    cb.clamp_exclusive(u)
    assert u[0] == 0.0 and u[1] == 30.0


def test_metrics_track_fires_and_flips():
    """Counter invariants for clamp_metrics()."""
    cb = _builder()
    cb.clamp_exclusive(np.array([10.0, 20.0]))  # cool wins (fire 1)
    cb.clamp_exclusive(np.array([10.0, 20.0]))  # cool wins (fire 2, no flip)
    cb.clamp_exclusive(np.array([20.0, 10.0]))  # heat wins (fire 3, flip 1)

    metrics = cb.clamp_metrics()
    assert 'u_heat__u_cool' in metrics
    m = metrics['u_heat__u_cool']
    assert m['fires'] == 3
    assert m['flips'] == 1
    assert m['winner'] == 'a'  # heat


def test_no_fire_counted_when_only_one_member_active():
    """Single-member solves don't increment the fire counter."""
    cb = _builder()
    cb.clamp_exclusive(np.array([0.0, 30.0]))
    cb.clamp_exclusive(np.array([30.0, 0.0]))
    m = cb.clamp_metrics()['u_heat__u_cool']
    assert m['fires'] == 0
    assert m['flips'] == 0


def test_reset_clears_state_and_counters():
    cb = _builder()
    cb.clamp_exclusive(np.array([40.0, 41.0]))
    cb.clamp_exclusive(np.array([20.0, 30.0]))
    cb.reset_clamp_metrics()
    m = cb.clamp_metrics()['u_heat__u_cool']
    assert m['fires'] == 0
    assert m['flips'] == 0
    assert m['winner'] is None


def test_reset_after_accumulated_state_zeros_winner():
    """After reset, the next clamp call has no prior winner and treats the
    decision as a fresh "pick larger" — important because the solver calls
    reset_clamp_metrics() on every light transition."""
    cb = _builder()
    cb.clamp_exclusive(np.array([10.0, 20.0]))  # cool wins, fire 1
    cb.clamp_exclusive(np.array([20.0, 10.0]))  # heat wins, flip 1
    assert cb.clamp_metrics()['u_heat__u_cool']['flips'] == 1

    cb.reset_clamp_metrics()

    # Same heat→cool transition that was a flip before is no longer a flip
    # post-reset (no prior winner to compare against).
    cb.clamp_exclusive(np.array([10.0, 20.0]))  # cool wins, fire 1, no flip
    m = cb.clamp_metrics()['u_heat__u_cool']
    assert m['fires'] == 1
    assert m['flips'] == 0
    assert m['winner'] == 'b'


def test_pair_inactive_when_channels_absent_from_registry():
    """Profiles without u_heat/u_cool/u_hum/u_dehum get no pairs."""
    reg = _registry_with(['u_cool'])  # ac_minimal-style
    cb = ConstraintBuilder(reg)
    assert cb.exclusive_pairs == []
    # No-op clamp
    u = np.array([50.0])
    cb.clamp_exclusive(u)
    assert u[0] == 50.0
    assert cb.clamp_metrics() == {}
