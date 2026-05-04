# Task 3 — SharedParams validation gate and rollback

**Status:** pending

## The punch

`calibration/calibrator.py` runs SLSQP on a rolling telemetry window and atomically swaps new parameters into `SharedParams`. There is no validation gate: if SLSQP converges to a bad local minimum (sensor drift period, unusual disturbance, optimizer instability), the next MPC solve immediately uses the corrupted plant model. There is no rollback. The only recovery is a process restart, by which time the controller has driven actuators against a wrong model for many minutes.

## Goal

Before SharedParams accepts a new parameter set, it must clear three checks:

1. **Bound check** — every delta within an agreed-multiplicative range of the prior value (e.g., no parameter changes by more than 25% per cycle).
2. **Prediction-error gate** — replay the last K seconds of telemetry through both old and new parameters; if the new RMS is worse, reject.
3. **Probationary acceptance** — even if accepted, monitor live prediction RMS for the next M ticks. If it exceeds the previous baseline by a margin, roll back.

## Acceptance criteria

- New module (or addition to `calibration/`) that runs the gate.
- Unit test that synthesizes a "bad SLSQP result" and confirms the gate rejects it.
- Unit test that confirms a legitimately better parameter set is accepted.
- Live integration in `calibration/calibrator.py` and `calibration/params.py`.
- An advisory fired when a calibration update is rejected so operators are not in the dark.

## Research questions for the team

**Thermodynamics specialist**
- Which of the calibrated parameters (UA_wall, tau_hvac, g_s_max, K_drain, C_wall, eta_hvac, …) have *physically* tight bounds vs. broad ones? The bound check should be parameter-specific, not uniform.
- Are there parameter combinations that are individually plausible but jointly unphysical? If so, the gate needs joint constraints, not just per-parameter clamps.

**MPC specialist**
- The probationary window depends on how fast bad parameters become visible in prediction error. Is 5 minutes enough? An hour? This is dynamics-dependent.
- During rollback, does the integral action need to reset (analogous to the existing light-transition reset)? A snap parameter change during steady state could otherwise cause a spurious bias buildup.

**SWE architecture**
- `calibration/params.py` claims atomic reads via the GIL. Is that contract still honored if we add a "previous params" snapshot for rollback? Specifically, can the consumer see a torn read between a probationary param and its predecessor?
- Where does the prediction-error gate run — inside the calibrator process, or inside the solver process? The former keeps the solver lean but means the calibrator needs sensor-history access.
