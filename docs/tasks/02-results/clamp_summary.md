# Exclusive-pair clamp measurement results

Duration: 43200s (12.0h) per profile.
Methodology: monkey-patch `ConstraintBuilder.clamp_exclusive` to capture (pre, post) per solve. A "fire" is a solve where both pair members were > 0 before the clamp ran (i.e., the QP genuinely wanted both).

## Profile: `hvacd_full`

- Solves recorded: 1399
- No exclusive pairs in this profile (constraint inactive).

## Profile: `ac_dehum`

- Solves recorded: 1392

| Pair | Fires | Fires/hr | Worst 60s | Minutes active | % active |
|---|---|---|---|---|---|
| `u_heat__u_cool` | 1084 | 90.33 | 2 | 638 | 91.7% |
| `u_hum__u_dehum` | 1080 | 90.00 | 2 | 622 | 89.4% |

## Profile: `ac_minimal`

- Solves recorded: 1392
- No exclusive pairs in this profile (constraint inactive).
