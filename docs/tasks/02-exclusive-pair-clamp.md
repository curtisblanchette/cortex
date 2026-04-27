# Task 2 — Quantify and address exclusive-pair clamp oscillation

**Status:** done (2026-04-27) — see `docs/decisions-recent.md` "Exclusive-pair clamp soft hysteresis"

## Outcome

**Two findings, one expected, one not:**

1. **The "sawtooth oscillation" the docs warned about doesn't exist** in measurement.
12hr sims across all three profiles (instrumented `clamp_exclusive` capturing
pre/post per solve):

- `hvacd_full` and `ac_minimal` — the registry doesn't expose the channels the
  pair constraints key off (they use HVACD setpoint mode or have a single
  cooling actuator), so the clamp is dead code on these profiles.
- `ac_dehum` — the QP plans both pair members on 78% of solves (linearization
  split, since R penalises `‖Δu‖²` not `‖u‖²`), but post-clamp the winner is
  stable for long runs. Heat/cool flips ~4×/hr, hum/dehum is essentially
  single-mode (dehum wins virtually always). Worst 60s window is 2 fires.

2. **The hysteresis fix didn't move the needle** on flip count
   (47 → 51 post-fix, within noise). Diagnostic: at the 51 flip moments the
   median pre-clamp gap is 0.74; **zero** flips occurred within the 5% margin.
   When the clamp fires, the median gap is 0.93 — the QP runs ~95/5 with a
   tiny tail, not 51/49. The flips are **genuine regime shifts** (e.g. cool
   60→5 while heat 4→25), not tie-breaking noise. The 5% hysteresis is the
   right tool for the wrong problem.

The real win of this task is the **telemetry baseline and documented
measurement**, not the mitigation. The hysteresis is kept as harmless
insurance (changes 0.9% of fires) against future scenarios where
tie-breaking does appear; the per-pair counters surface real-world
deviations if they happen.

## Shipped fix

- `mpc/constraints.py` — soft hysteresis in `clamp_exclusive`. When the two
  pre-clamp values are within `hysteresis_margin` of each other (default 5%),
  keep the previous winner. Per-pair `fires` / `flips` / `winner` counters
  exposed via `clamp_metrics()`.
- `config/room.yaml` — new `room.clamp_hysteresis.hysteresis_margin` knob.
- `cli/bootstrap.py:build_mpc` — reads the config and passes to `ConstraintBuilder`.
- `control/solver_process.py` — puts `clamp_metrics()` into the per-tick `info`
  dict under `clamp_metrics` for dashboards / production observability.
- `tests/test_clamp_oscillation.py` — 7 regression tests pinning legacy and
  hysteresis behaviour, counter invariants, and the no-pair-no-op profile.
- `scripts/measure_clamp_oscillation.py` + `docs/tasks/02-results/` — the
  measurement that drove the decision; re-runnable for future regressions.
- `docs/mpc-solver.md` — corrected the "sawtooth" framing with measured numbers.

## Skipped (data-driven negatives)

- **MIQP** — 80–400ms solve on Pi 5 vs. 1Hz target. Not feasible without a
  solver swap (Gurobi/SCIP) the project doesn't depend on.
- **Switching-cost q-bias on the QP** — overkill for ~4 flips/hr; would require
  augmented variables and break the OSQP PSD invariant if not done carefully.
- **Hard hysteresis lockout** — risks trapping the wrong actuator during load
  steps (lights-on, door openings). Soft hysteresis avoids this by releasing
  whenever the gap exceeds the margin.
