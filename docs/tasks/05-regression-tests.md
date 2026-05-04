# Task 5 — Foundational regression test suite

**Status:** pending

## The punch

`tests/` contains only `__init__.py`. The project's stated validation strategy is "simulation compliance >97%", but compliance is integration testing — it tells you the system worked end-to-end on a 12-hour run, not that any specific module behaves correctly under edge conditions. There is no protection against:
- A plugin author shadowing an existing channel name.
- A constraint refactor breaking exclusive-pair invariants.
- An EKF change introducing numerical instability under a NaN burst.
- A Jacobian rewrite quietly violating the 0.01% accuracy claim.

## Goal

Build the *thinnest* regression layer that catches all four classes above. Not a coverage push — a load-bearing safety net.

Initial test families:

1. **Plugin contract tests** (`tests/test_plugins.py`)
   - Channel name uniqueness across all built-in profiles.
   - Dispatch spec validity (every channel has a destination).
   - Bound monotonicity (lower < upper, defaults inside bounds).
2. **Constraint invariants** (`tests/test_constraints.py`)
   - Exclusive pair clamp: post-clamp, never both > 0.
   - Bounds enforcement: post-clamp, every channel within registry bounds.
3. **EKF numerical** (`tests/test_ekf.py`)
   - NaN per channel: each isolated NaN does not crash, fallback path executes.
   - Singular covariance: synthetic R=0 case routes through pseudo-inverse.
4. **Jacobian accuracy** (`tests/test_jacobian.py`)
   - Compare analytical B-matrix to finite-difference at multiple operating points across all profiles. Fail if max relative error > 0.1%.

## Acceptance criteria

- `pytest tests/` runs in <30 seconds.
- All four test files exist and pass on `main`.
- A note in `docs/testing.md` describing how regression tests complement compliance simulation, and what each family covers.

## Research questions for the team

**Thermodynamics specialist**
- Where in the dynamics is finite-difference Jacobian likely to disagree with the analytical one? (Discontinuities in CO2 hysteresis, light schedule, irrigation — these need to be tested *off* of those events.)
- What operating points should the Jacobian test span? At minimum: lights on / lights off, mid-phase steady state, post-irrigation transient.

**MPC specialist**
- Are there constraint invariants beyond exclusive pairs and bounds worth testing? E.g., that the rate constraint doesn't allow a channel to leap by more than its slew limit, that warm-start preserves feasibility.

**SWE architecture**
- Pick the test framework idioms the rest of the codebase will follow (fixtures for registry, parameterized profiles). One pattern, applied consistently — not four.
- Set up a CI hook (or document one) so the suite blocks merges. Without that, the tests rot.
