# Hardening Task Status

Snapshot of where the 6-task hardening workstream stands. The full task
descriptions and working agreement live in `README.md` (currently only on
`task/01-integral-action-diagnosis` — not yet merged to `main`).

| # | Task | State | Where it lives |
|---|------|-------|----------------|
| 01 | Diagnose integral action root cause | Done | Branch `task/01-integral-action-diagnosis` (commits `0239094`, `b508e99`) — **not merged to main** |
| 02 | Exclusive-pair clamp oscillation | Done | Branch `task/02-exclusive-pair-clamp` — see `02-exclusive-pair-clamp.md` |
| 03 | SharedParams validation gate + rollback | Not started | `03-shared-params-rollback.md` (on `task/01` branch) |
| 04 | EKF observability monitor + sensor dropout advisory | Not started | `04-ekf-observability.md` (on `task/01` branch) |
| 05 | Foundational regression test suite | Not started | `05-regression-tests.md` (on `task/01` branch) |
| 06 | Real-world validation strategy | Not started | `06-validation-strategy.md` (on `task/01` branch) |

## Branch / merge state

- `main` is at `0b5ce75` (last merge: leaf-temp advisory).
- `task/01` is 2 commits ahead of `main`. Tasks 01 and the README + tasks 03–06
  files all land together when it merges.
- `task/02` forked off `main` *before* `task/01` was created, so tasks 03–06
  files are not visible from this branch. They reappear when both branches are
  on `main`.

## Task closure pattern

Per the README's working agreement: each task closes with a short note appended
to `docs/decisions-recent.md`. Both completed tasks have followed this:

- Task 01 → "Integral action root cause" entry (in `task/01` branch's docs).
- Task 02 → "Exclusive-pair clamp measurement — telemetry, no mitigation needed".

## Caveats

- Snapshot date: 2026-05-04. Re-verify against `git log` and the task files
  before treating any of this as live state.
- The project-level `MEMORY.md` "Known Issues: None currently" line is
  misleading while tasks 03–06 are open — they're scoped issues, just not
  active bugs.