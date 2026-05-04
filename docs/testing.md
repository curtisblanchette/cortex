<!-- SUMMARY -->
> **Scope:** Test strategy, simulation-based validation, compliance metrics, regression baselines, current pytest coverage.
> **Key constraint:** Primary validation is simulation compliance (>97% target on 12 h fast sims) — not unit test coverage. The pytest suite is intentionally sparse and pins specific invariants only.

---

# Testing

## Two-Layer Strategy

Validation is layered. Most "did we break it?" answers come from the sim layer; the pytest layer pins specific invariants that sim-level tests would miss.

### Layer 1 — Simulation compliance

Run 12 h fast sims across active equipment profiles, measure % time within setpoint tolerance for T, RH, VPD, CO2, theta. Target: **>97% compliance** across all channels.

```bash
# Compliance check across profiles
python main.py simulate --fast --duration 43200 --chart --profile hvacd_full
python main.py simulate --fast --duration 43200 --chart --profile ac_dehum
python main.py simulate --fast --duration 43200 --chart --profile ac_minimal
```

This catches:
- Compliance regressions (the headline metric)
- Bang-bang behavior, oscillation, mode hunting
- Linearization mismatch issues
- Schedule / transition bugs (lights-on/off, phase changes)

### Layer 2 — Pytest invariants

```bash
python -m pytest tests/
```

Currently sparse by design — pins behaviour the sim wouldn't catch directly:

| Test | Pins |
|------|------|
| `tests/test_clamp_oscillation.py` | Exclusive-pair clamp legacy behaviour, counter invariants, no-op profile |

When adding tests, prefer locking down a *specific invariant or counter-example* over broad integration tests. The sim layer covers integration.

## What to Validate After Changes

| Change type | Run |
|-------------|-----|
| Physics model | All three profiles, 12 h each, check compliance doesn't regress |
| MPC / constraints | Check oscillation, exclusive-pair clamp metrics, solve-time distribution |
| Config | Phase/profile-specific sim, verify setpoint tracking |
| Advisory | Verify no false positives during lights-off (especially CO2) |
| Calibrator | Innovation-driven trigger fires; SharedParams update visible to controller |

## Regression Baselines

`docs/tasks/02-results/` — captured CSVs and summary from the exclusive-pair clamp measurement (12 h sims across all profiles). Re-runnable via `scripts/measure_clamp_oscillation.py`. Treat the CSVs as the regression baseline for clamp behaviour.

## What's Not Tested (Yet)

The pytest layer does not cover:
- EKF convergence with noisy sensors
- SLSQP convergence to known params
- Fail-safe ladder transitions
- IO adapter equivalence (RealIO vs SimulatedIO interface)
- Solver speed budget (sim run time is a proxy)

These are tracked under hardening tasks 03–06 (see `docs/tasks/STATUS.md`). Before the gap closes, treat sim compliance as the safety net.