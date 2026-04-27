# Task 2 — Quantify and address exclusive-pair clamp oscillation

**Status:** pending

## The punch

`mpc/constraints.py:137-150` post-solves the QP and zeroes the smaller actuator in each exclusive pair (heat/cool, hum/dehum). The convex QP optimizes against `u_a + u_b ≤ 100`; the clamp then enforces a non-convex constraint *after the fact*. `docs/mpc-solver.md:30` admits this "can cause saw-tooth oscillation when the QP plans to use both actuators (e.g., dehum for waste heat)".

This admission has never been quantified in production. It might be benign, or it might be the dominant source of actuator wear and compliance noise. Either way, "we know it's a problem" is not a plan.

## Goal

1. **Measure** the oscillation: across all profiles and growth phases, how often per hour is the *non-clamped* actuator zeroed by the post-solve clamp? What is the mean toggle frequency on each affected channel?
2. **Decide** the fix:
   - **Hysteresis lockout** — once a pair is clamped to favor (say) cooling, lock heating to zero for N seconds before reconsidering.
   - **Warm-start bias** — add a small cost to switching the active member of a pair across solves.
   - **MIQP** — model exclusivity as a binary, accept the solve-time hit.
   - **Status quo + monitoring** — if the toggle frequency is genuinely low, add an advisory and move on.

## Acceptance criteria

- A `scripts/measure_clamp_oscillation.py` that runs sims across `hvacd_full`, `ac_dehum`, `ac_minimal` and reports per-channel toggle frequency, mean dwell time, and worst-case toggle in any 60s window.
- One regression test in `tests/` that fails if toggle frequency exceeds an agreed threshold on a canonical 12-hour mid-flower run.
- Implementation of the chosen fix, with the clamp's rationale and the chosen mitigation written into `docs/decisions-recent.md`.

## Research questions for the team

**Thermodynamics specialist**
- Is there a physical scenario where using both heat and cool simultaneously is *correct* (e.g., dehumidification via reheat coil)? If so, the exclusivity constraint itself is wrong, not the clamp.
- For the dehum/hum pair: is there ever a steady-state where the QP genuinely wants both at low duty cycle, indicating a model deficiency rather than an exclusivity violation?

**MPC specialist**
- Why does the QP plan to use both pair members? It has the constraint `u_a + u_b ≤ 100`, but no penalty for splitting. A small cross-cost (added linearly via slack) might break ties without making P indefinite.
- Compare a hysteresis-lockout fix versus an MIQP formulation. What is the OSQP-vs-Gurobi/SCIP solve-time delta on a Pi 5? Is MIQP feasible at 1 Hz?
- Does the integral action from Task 1 mask or amplify the clamp toggling?

**SWE architecture**
- The clamp lives in `mpc/constraints.py:clamp_exclusive`. Is it called every solve, or only on certain code paths? Are there places where the QP output is consumed before the clamp runs?
- Where would per-channel toggle telemetry go: solver_process logs, advisory engine, or new metrics? It should be visible without recompiling.
