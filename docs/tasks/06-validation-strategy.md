# Task 6 — Real-world validation strategy

**Status:** pending

## The punch

Compliance metrics measure MPC against the same 7-state physics it controls. EKF uses that physics, OSQP uses that physics, the calibrator uses that physics. When the model is systematically wrong — actual room thermal mass off by 20%, stomatal resistance varying with humidity in a way the transpiration model doesn't capture, an actuator with hysteresis — all three components agree and oscillate together. The 12-hour sims never see model-reality gap. The 97% number is a tautology.

## Goal

A formal **model-reality gap injection** protocol that any change to the controller, EKF, or calibrator must pass *in addition to* compliance.

Three injection families:

1. **Parameter mismatch.** Run the controller stack on plant parameters A; run the simulator on plant parameters B = A · (1 + ε) for a vector of perturbations ε. Measure compliance degradation as a function of ε for each parameter.
2. **Sensor non-idealities.** Add bias, drift, dropouts, and quantization beyond the current Gaussian noise model. The current `simulation/noise.py` is a starting point but does not represent realistic sensor failure modes.
3. **Unmodeled disturbances.** Door openings (sudden T_out coupling), lamp partial failure mid-cycle, irrigation valve stuck open, condensate drain blockage. None of these are in the disturbance vector.

## Acceptance criteria

- `docs/validation-strategy.md` documenting the protocol, perturbation magnitudes, pass/fail criteria, and how it relates to compliance.
- `scripts/perturbation_sweep.py` that runs the parameter-mismatch family across all profiles and outputs a CSV/heatmap of compliance vs. ε.
- A first round of results checked into `docs/tasks/06-results/` for the parameters with the largest sensitivity.
- A note in `docs/testing.md` adding model-reality validation alongside compliance and regression tests.

## Research questions for the team

**Thermodynamics specialist**
- Which parameters does the system likely *actually* not know to better than 20%? The honest list (UA_wall, plant transpiration coefficients, infiltration rate) is the priority for ε sweeps.
- What are realistic unmodeled disturbances at this facility? Door openings are obvious; what else?

**MPC specialist**
- How much robustness buffer is built into the current QP? (E.g., terminal cost, slack on tolerances.) If the QP is brittle, perturbation tests will fail catastrophically rather than gracefully — that is information, not failure.
- Which controller hyperparameters (Q, R, Np, Nc) are most sensitive to model-reality gap? The protocol should report this.

**SWE architecture**
- The simulator and controller currently share the parameter source (`SharedParams`). The perturbation harness needs to *break* that — give them separate parameter views without forking the dynamics module. Decide: dependency injection at construction, or a "shadow params" wrapper?
- The CSV/heatmap output needs to be diffable across runs so changes show up cleanly. Pick a stable numerical format (sorted keys, fixed precision) up front.
