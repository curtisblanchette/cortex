# Exclusive-pair clamp measurement results

Duration: 43200s (12.0h) per profile.
Methodology: monkey-patch `ConstraintBuilder.clamp_exclusive` to capture (pre, post) per solve. A "fire" is a solve where both pair members were > 0 before the clamp ran (i.e., the QP genuinely wanted both). A "flip" is a solve where the clamp's winner changed from the previous fire.

This file holds the **post-fix** measurement (with `hysteresis_margin=0.05`). The pre-fix numbers are quoted in `docs/decisions-recent.md` for comparison.

## Profile: `hvacd_full`

- Solves recorded: 1399
- No exclusive pairs in this profile (constraint inactive).

## Profile: `ac_dehum`

- Solves recorded: 1392

| Pair | Fires | Fires/hr | Flips (winner change) | Worst 60s | Minutes active | % active |
|---|---|---|---|---|---|---|
| `u_heat__u_cool` | 1111 | 92.58 | **51** | 2 | 652 | 93.7% |
| `u_hum__u_dehum` | 1108 | 92.33 | **4** | 2 | 627 | 90.1% |

## Profile: `ac_minimal`

- Solves recorded: 1392
- No exclusive pairs in this profile (constraint inactive).

## Pre-vs-post-fix comparison (ac_dehum)

| Pair | Pre-fix flips | Post-fix flips (margin=0.05) | Delta |
|---|---|---|---|
| `u_heat__u_cool` | 47 | 51 | +4 (within run-to-run noise) |
| `u_hum__u_dehum` | 4 | 4 | 0 |

The 5% hysteresis margin had **no statistically meaningful effect** on flip count.

## Why the hysteresis didn't move the needle

Diagnostic on the heat/cool pair fire distribution:

- Median pre-clamp gap `|a-b|/max(a,b)`: **0.93** — when the clamp fires, the larger value is typically ~14× the smaller. The QP is not splitting close values; it runs ~95/5 with a tiny tail of the other actuator.
- Only **0.9% of fires** (10 of 1111) fall within the 5% margin where hysteresis engages.
- At the 51 flip moments, the median gap is **0.74**. **Zero** of the 51 flips occurred within the 5% margin.

**Interpretation:** the flips are genuine *regime shifts* (cool dominating → heat dominating, both values move significantly), not *tie-breaking noise* (close values flickering). The 5% hysteresis is the right tool for the wrong problem.

A 20% margin would catch ~17/51 flips but starts interfering with valid load-change responses. Not justified by current data.

**The real value of this measurement** is establishing the baseline: now we know the clamp activity, the absence of sawtooth, and the regime-shift nature of flips. The hysteresis is harmless insurance kept in place at the conservative 5% default; the telemetry counters surface real-world deviations from this baseline if they appear in production.
