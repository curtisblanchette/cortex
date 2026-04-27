# Task 2 — Quantify and address exclusive-pair clamp oscillation

**Status:** done (2026-04-27) — see `docs/decisions-recent.md` "Exclusive-pair clamp soft hysteresis"

## Outcome

The "sawtooth oscillation" the docs warned about doesn't exist in measurement.
12hr sims across all three profiles (instrumented `clamp_exclusive` capturing
pre/post per solve):

- `hvacd_full` and `ac_minimal` — the registry doesn't expose the channels the
  pair constraints key off (they use HVACD setpoint mode or have a single
  cooling actuator), so the clamp is dead code on these profiles.
- `ac_dehum` — the QP plans both pair members on 78% of solves (linearization
  split, since R penalises `‖Δu‖²` not `‖u‖²`), but post-clamp the winner is
  stable for long runs. Heat/cool flips ~4×/hr, hum/dehum is essentially
  single-mode (dehum wins virtually always). Worst 60s window is 2 fires.

The real cost was occasional regime-shift flips on `ac_dehum` heat/cool,
not solve-by-solve sawtooth.

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
