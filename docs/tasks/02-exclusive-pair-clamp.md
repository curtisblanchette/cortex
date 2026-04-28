# Task 2 — Quantify and address exclusive-pair clamp oscillation

**Status:** done (2026-04-28) — see `docs/decisions-recent.md` "Exclusive-pair clamp measurement"

## Outcome

**The "sawtooth oscillation" the docs warned about doesn't exist in measurement.**
12hr sims across all three profiles (instrumented `clamp_exclusive` capturing pre/post per solve):

- `hvacd_full` and `ac_minimal` — the registry doesn't expose the channels the
  pair constraints key off (HVACD uses setpoint mode; ac_minimal has a single
  cooling actuator). Clamp is dead code (0 fires in 1399 / 1392 solves).
- `ac_dehum` — the QP plans both pair members on 78% of solves (linearization
  split, since `R` penalises `‖Δu‖²` not `‖u‖²`), but post-clamp the winner is
  **stable for long runs**. Heat/cool flips ~4×/hr, hum/dehum is essentially
  single-mode (dehum wins virtually always). Worst 60s window is 2 fires.

**Diagnostic on the heat/cool pair:**
- Median pre-clamp gap `|a-b|/max(a,b)` = **0.93** — when the clamp fires, the
  larger value is typically ~14× the smaller. The QP runs ~95/5 with a tiny
  tail, not 51/49 tie-breaking.
- At the 51 flip moments, median gap = **0.74**. The flips are genuine regime
  shifts (e.g. cool 60→5 while heat 4→25), not numerical noise.
- The MPC's existing structure — re-linearization every solve, `R` on `‖Δu‖²`,
  1 Hz re-clamp — already produces stable winners.

## Shipped: telemetry only, no mitigation

- `mpc/constraints.py:ConstraintBuilder` — per-pair `fires` / `flips` / `winner`
  counters via new `clamp_metrics()` method, plus `reset_clamp_metrics()` for
  test/script harness use.
- `control/solver_process.py` — puts `clamp_metrics()` into the per-tick `info`
  dict under `clamp_metrics` for dashboards / production observability.
- `tests/test_clamp_oscillation.py` — 6 regression tests pinning legacy clamp
  behaviour (always pick larger), counter invariants, no-pair no-op profile.
- `scripts/measure_clamp_oscillation.py` + `docs/tasks/02-results/` — the
  measurement that drove the decision; re-runnable as a regression baseline.
- `docs/mpc-solver.md` — corrected the "sawtooth" framing with measured numbers.

## Skipped (data-driven negatives)

- **MIQP** — 80–400ms solve on Pi 5 vs. 1Hz target. Not feasible without a
  solver swap (Gurobi/SCIP) the project doesn't depend on.
- **Switching-cost q-bias on the QP** — overkill for ~4 flips/hr; would require
  augmented variables and risk breaking the OSQP PSD invariant.
- **Hard hysteresis lockout** — risks trapping the wrong actuator during load
  steps (lights-on, door openings).
- **Soft hysteresis (5% close-band)** — first prototyped, then removed:
  re-measurement showed flip count unchanged (47 → 51, within run-to-run
  noise). The flips happen at gaps far outside any reasonable close-band.
  Worse, hysteresis introduces a silent failure mode (stale-winner holds
  don't show up in `flips` or `fires` telemetry) — exactly the kind of debt
  this hardening pass is supposed to remove. The right answer is to expose
  the behaviour with counters and design a targeted fix only if real data
  ever demands one.
