# Task 1 — Diagnose integral action root cause

**Status:** done (2026-04-27) — see `docs/decisions-recent.md` "Integral action diagnosis"

## Outcome

Root cause is **not** linearization mismatch. The integral absorbs persistent bias from
slow unmodeled dynamics the in-horizon QP cannot resolve:
- Wall thermal mass τ ≈ 10,000s vs. control horizon 450s (~22× mismatch)
- Outdoor-weather persistence-forecast bias
- HVAC supply lag (τ ≈ 120s)
- Affine VPD/RH linearization compounding over 15 steps (SVP is exponential in T)

Plus a structural cause: `R` penalises `‖Δu‖²` not `‖u‖²`, so the QP has no incentive
to drive u to a specific steady-state value — bias absorption has to live somewhere.

## Shipped fix (Option B — instrument & document)

- `config/room.yaml` adds `room.integral_action.{gain, t_clamp_C, vpd_clamp_kPa, saturation_warn_s}` (defaults match the prior hardcoded values).
- `control/solver_process.py` reads the config, tracks per-channel saturation onset, and surfaces integral state in the per-tick `info` dict and to the advisory engine.
- `advisory/{models,templates,engine}.py` adds `TYPE_INTEGRAL_SATURATED` with grower-facing copy that names physical causes.
- `tests/test_integral_action.py` pins the formula, config plumbing, and saturation transitions.

## Queued follow-up (Option A — structural)

Replace the reference-bias integral with an augmented-state output-disturbance
observer (EKF estimates a constant bias per channel, MPC sees it in dynamics so
steady-state tracking is achieved without the hack). Needs Task 5's regression
suite first as a safety net.

## The punch

`docs/decisions-recent.md:169-177` claims piecewise B-matrix linearization achieves 0.01% accuracy versus the nonlinear ground truth. Yet `control/solver_process.py` runs a persistent integral correction on T (gain 0.003, max ±4.0°C) and an analogous bias on VPD. If linearization were truly that accurate, a steady-state bias large enough to justify ±4°C of windup would not exist. The integral is compensating for *something* — and that something is not named anywhere.

Candidate culprits (not exclusive):
- Unmodeled actuator dynamics (HVACD startup ramp, blower thermal lag).
- Sensor measurement bias or low-pass filtering not represented in the EKF.
- Plant-model gap that is small at the linearization point but accumulates over the prediction horizon.
- EKF process-noise tuning that biases the state estimate during transients.
- Reference-generation mismatch (e.g., RH→VPD inversion drift).

## Goal

Identify *what* the integral is absorbing, with numerical evidence. Then decide:

1. **Structural fix** — model the missing dynamics, remove or shrink the integral.
2. **Documented compensator** — keep the integral but with a written justification, bounded gain rationale, and a regression test that fails if the integral starts saturating.

## Acceptance criteria

- A short report under this task's directory naming the dominant contributor with numbers (e.g., "1.8°C of the 2°C steady-state bias is HVACD supply-air thermal lag, identified by step-response comparison").
- A reproducible script in `tests/` or `scripts/` that produces those numbers from a sim run.
- Either: a code change that closes the gap (tracked here), or a documented decision in `docs/decisions-recent.md` justifying the integral as the right tool with bounded scope.

## Research questions for the team

**Thermodynamics specialist**
- Where in the 7-state ODE could the linearization Jacobian be locally accurate but globally biased? (E.g., bilinear terms in transpiration, T_supply lag, wall thermal coupling.)
- Is there a thermal time constant in the room/HVACD that exceeds the MPC control horizon (Nc·dt = 15·30s = 450s)?
- Run a step-response comparison: hold u constant, compare nonlinear sim trajectory vs. linearized prediction over 30 minutes. Which states diverge first, and by how much?

**MPC specialist**
- Is the Nc=15, dt=30s control horizon long enough for the QP to find the steady-state actuator level for current disturbances? Or is the integral compensating for a too-short horizon?
- Are reference-tracking weights (Q) and rate-penalty weights (R on Δu) balanced such that the QP under-uses control authority at steady state?
- Does the integral interact with EKF feedback (state correction loop), or is it bolted onto the reference only?

**SWE architecture**
- Trace exactly where `_t_integral` and `_vpd_integral` are computed and consumed in `control/solver_process.py`. Is the bias added to the reference, the state estimate, or the cost? This is documented inconsistently.
- Are the gain (0.003) and clamp (4.0) hardcoded? Where do they come from? Are they tuned to a specific room and silently wrong elsewhere?
- Is there any logging today that would tell us how often the integral saturates in real ops? If not, add it.
