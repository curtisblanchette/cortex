"""Measure exclusive-pair clamp activity across profiles.

The clamp at mpc/constraints.py:clamp_exclusive enforces non-convex mutual
exclusivity by zeroing the smaller pair member after the QP returns. This
script monkey-patches the clamp to record (pre, post) values per solve so
we can quantify how often the QP genuinely wants both members and how
hard the clamp is fighting it.

Outputs per profile:
- docs/tasks/02-results/clamp_ticks_<profile>.csv  — per-solve pre/post values
- docs/tasks/02-results/clamp_summary.md           — aggregated statistics

Run: python scripts/measure_clamp_oscillation.py
"""

import argparse
import csv
import os
import sys
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mpc.constraints import ConstraintBuilder
from cli.commands.simulate import _run_decoupled


PROFILES = ['hvacd_full', 'ac_dehum', 'ac_minimal']
DURATION_S = 43200  # 12 hours
RESULTS_DIR = ROOT / 'docs' / 'tasks' / '02-results'


# Module-level recorder ------------------------------------------------------
_records = []          # list of dicts, one per clamp call
_solve_count = [0]     # mutable counter so closure can increment
_channel_names = []    # populated from constraints on first call


_original_clamp = ConstraintBuilder.clamp_exclusive


def _instrumented_clamp(self, u):
    """Drop-in wrapper for clamp_exclusive that captures pre/post state.

    Records (pre, post) per pair before delegating to the real clamp so the
    built-in fire/flip counters still update correctly. The script can run
    against either the pre-hysteresis or post-hysteresis implementation.
    """
    if not _channel_names:
        _channel_names.extend(self._channel_names)
    pre = u.copy()
    _original_clamp(self, u)
    post = u
    rec = {'solve': _solve_count[0]}
    for idx_a, idx_b, max_sum in self.exclusive_pairs:
        name_a = self._channel_names[idx_a]
        name_b = self._channel_names[idx_b]
        pair_key = f'{name_a}__{name_b}'
        rec[f'{pair_key}__a_pre'] = float(pre[idx_a])
        rec[f'{pair_key}__b_pre'] = float(pre[idx_b])
        rec[f'{pair_key}__a_post'] = float(post[idx_a])
        rec[f'{pair_key}__b_post'] = float(post[idx_b])
        clamp_fired = (pre[idx_a] > 0 and pre[idx_b] > 0)
        rec[f'{pair_key}__clamp_fired'] = clamp_fired
    _records.append(rec)
    _solve_count[0] += 1
    return u


def _make_args(profile):
    return SimpleNamespace(
        fast=True,
        duration=DURATION_S,
        config='config',
        profile=profile,
        phase=None,
        substrate=None,
        chart=False,
        chart_output='/tmp/unused.png',
        weather=None,
        T_out_mean=None,
        T_out_amp=None,
        w_out=None,
        dashboard=False,
        wait=False,
        api_port=8080,
    )


def _flush_records(profile):
    """Write per-solve CSV and clear the buffer."""
    if not _records:
        return [], 0
    path = RESULTS_DIR / f'clamp_ticks_{profile}.csv'
    keys = sorted({k for r in _records for k in r.keys()})
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(_records)
    out = list(_records)
    n = len(_records)
    _records.clear()
    _solve_count[0] = 0
    return out, n


def _summarize(profile, records):
    """Compute clamp activity stats per pair from the captured records."""
    if not records:
        return {'profile': profile, 'pairs': {}, 'n_solves': 0}

    pair_keys = sorted({
        k.rsplit('__', 1)[0]
        for k in records[0].keys()
        if k.endswith('__clamp_fired')
    })

    sim_seconds_per_solve = DURATION_S / max(1, len(records))

    pair_stats = {}
    for pair in pair_keys:
        fires = [int(r[f'{pair}__clamp_fired']) for r in records]
        n_fires = sum(fires)

        # Worst-case 60s window: ~ ceil(60 / sim_seconds_per_solve) solves
        win = max(1, int(round(60.0 / sim_seconds_per_solve)))
        worst = 0
        for i in range(0, len(fires) - win + 1):
            s = sum(fires[i:i + win])
            if s > worst:
                worst = s

        # Hours where clamp fired ≥1× per minute (saturation indicator)
        bucket = max(1, int(round(60.0 / sim_seconds_per_solve)))  # 1-min buckets
        per_min_active = 0
        per_hr_active = 0
        for i in range(0, len(fires), bucket):
            chunk = fires[i:i + bucket]
            if sum(chunk) >= 1:
                per_min_active += 1
        # Aggregate to "hours where ≥1 fire/min sustained for 60 min"
        per_hr = max(1, 60)  # we count minute-buckets directly
        pair_stats[pair] = {
            'n_solves': len(records),
            'n_fires': n_fires,
            'fires_per_hour': n_fires / (DURATION_S / 3600.0),
            'worst_60s_window_fires': worst,
            'minutes_with_at_least_one_fire': per_min_active,
            'pct_minutes_active': 100.0 * per_min_active / max(1, len(fires) // bucket),
        }
    return {'profile': profile, 'pairs': pair_stats, 'n_solves': len(records)}


def _write_summary(all_stats):
    md = ['# Exclusive-pair clamp measurement results',
          '',
          f'Duration: {DURATION_S}s ({DURATION_S/3600:.1f}h) per profile.',
          'Methodology: monkey-patch `ConstraintBuilder.clamp_exclusive` to '
          'capture (pre, post) per solve. A "fire" is a solve where '
          'both pair members were > 0 before the clamp ran (i.e., the '
          'QP genuinely wanted both).',
          '']
    for s in all_stats:
        md.append(f'## Profile: `{s["profile"]}`')
        md.append('')
        md.append(f'- Solves recorded: {s["n_solves"]}')
        if not s['pairs']:
            md.append('- No exclusive pairs in this profile (constraint inactive).')
            md.append('')
            continue
        md.append('')
        md.append('| Pair | Fires | Fires/hr | Worst 60s | Minutes active | % active |')
        md.append('|---|---|---|---|---|---|')
        for pair, p in s['pairs'].items():
            md.append(
                f"| `{pair}` | {p['n_fires']} | {p['fires_per_hour']:.2f} | "
                f"{p['worst_60s_window_fires']} | {p['minutes_with_at_least_one_fire']} | "
                f"{p['pct_minutes_active']:.1f}% |"
            )
        md.append('')
    (RESULTS_DIR / 'clamp_summary.md').write_text('\n'.join(md))


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    ConstraintBuilder.clamp_exclusive = _instrumented_clamp
    try:
        all_stats = []
        for profile in PROFILES:
            print(f'\n=== Measuring profile: {profile} ===\n')
            args = _make_args(profile)
            try:
                _run_decoupled(args)
            except SystemExit:
                pass  # the sim may sys.exit on chart fallback paths
            recs, n = _flush_records(profile)
            print(f'\n[clamp-measure] {profile}: {n} solves recorded')
            all_stats.append(_summarize(profile, recs))
    finally:
        ConstraintBuilder.clamp_exclusive = _original_clamp

    _write_summary(all_stats)
    print(f'\nResults written to {RESULTS_DIR}')


if __name__ == '__main__':
    main()
