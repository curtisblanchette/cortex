<!-- SUMMARY -->
> **Scope:** Hardening tasks distilled from the 3-angle review (UX, architecture, devil's advocate). Ordered by risk-to-correctness.
> **How the team works:** Each task is tackled by three specialists in parallel — a thermodynamics scientist, an MPC controls specialist, and a software architecture engineer. Findings are synthesized, then implementation lands in a single coherent change.

---

# Cortex Hardening Tasks

These came out of a structured review where reviewers were explicitly asked to push hard. They are not stylistic. Each one points at a place where the system either silently absorbs a real-world problem, or relies on simulation evidence that does not generalize to the field.

## Task index

| # | Task | Why it's on the list |
|---|------|----------------------|
| 1 | [Diagnose integral action root cause](./01-integral-action-root-cause.md) | A ±4°C integral correction contradicts a "0.01% accurate" linearization claim. Something unmodeled is being absorbed. |
| 2 | [Quantify and address exclusive-pair clamp oscillation](./02-exclusive-pair-clamp.md) | A known sawtooth source. Acknowledged in docs, never measured in production. |
| 3 | [Add SharedParams validation gate and rollback](./03-shared-params-rollback.md) | A bad SLSQP convergence currently poisons live control with no revert path. |
| 4 | [EKF observability monitor + sensor dropout advisory](./04-ekf-observability.md) | EKF can lose observability silently. Operators get "model diverging" instead of "check sensor X". |
| 5 | [Build foundational regression test suite](./05-regression-tests.md) | `tests/` is `__init__.py` only. Compliance sims are integration testing, not regression. |
| 6 | [Real-world validation strategy](./06-validation-strategy.md) | MPC is validated against the same physics it controls. Need model-reality gap injection. |

## Working agreement

- **One task at a time.** No parallel tasks. Each builds context the next will lean on.
- **Three specialists in parallel per task.** Thermo, MPC, SWE. Each brings their angle; one consolidated implementation lands.
- **Research → validate → implement.** No fix ships without a numerical or test-backed reason.
- **Each task closes with a short note appended to `docs/decisions-recent.md`** so the rationale survives.
