<!-- SUMMARY -->
> **Scope:** Test strategy, simulation-based validation, compliance metrics.
> **Key constraint:** Primary validation is simulation compliance (>97% target) — not unit test coverage.

---

# Testing

## Running Tests

```bash
python -m pytest tests/
```

## Validation Strategy

The primary validation method is **simulation compliance testing**:
- Run 12hr fast simulations across all equipment profiles
- Measure % time within setpoint tolerances for T, RH, VPD, CO2, theta
- Target: >97% compliance across all channels

## Simulation-Based Testing

```bash
# Full compliance check across profiles
python main.py simulate --fast --duration 43200 --chart --profile hvacd_full
python main.py simulate --fast --duration 43200 --chart --profile ac_dehum
python main.py simulate --fast --duration 43200 --chart --profile ac_minimal
```

## What to Validate After Changes

- Physics model changes → run all profiles, check compliance doesn't regress
- MPC/constraint changes → check for oscillation, exclusive pair behavior
- Config changes → run affected phase, verify setpoint tracking
- Advisory changes → verify no false positives during lights-off (especially CO2)