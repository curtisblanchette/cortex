"""Regression tests for the reference-bias integral in SolverProcess.

The integral exists to absorb persistent biases the QP cannot resolve
in-horizon (wall thermal mass beyond the prediction horizon, outdoor
weather persistence-forecast gap, HVAC supply-air lag). It is NOT a
linearization patch — see docs/decisions-recent.md "Integral action
diagnosis".

These tests pin three invariants:
  1. Math — accumulator formula and clipping behaviour match the docs.
  2. Configurability — gain and clamps are read from integral_config and
     fall back to the historical hardcoded defaults when absent.
  3. Saturation tracking — saturation onset/clear transitions fire at
     the correct times so the advisory engine can warn the operator.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from control.solver_process import SolverProcess


def _make_solver(integral_config=None, target_T=22.0, x_hat_T=20.0,
                 target_VPD=1.0, current_VPD=0.5):
    """Build a SolverProcess whose dependencies are mocked so a single
    solve_once() exercises only the integral update path.

    The mocked plant has constant error: x_hat[T] is below target by
    (target_T - x_hat_T) and VPD is below target by (target_VPD - current_VPD).
    This forces both integrals to accumulate positive each call.
    """
    io = MagicMock()
    io.read_sensors.return_value = np.zeros(6)
    io.get_disturbance.return_value = np.zeros(6)
    io.get_time.return_value = 1000.0  # mid-day, away from light transition

    x_hat = np.array([x_hat_T, 10.0, 800.0, x_hat_T - 1.0, 0.5, x_hat_T, x_hat_T])
    ekf = MagicMock()
    ekf.update.return_value = (x_hat, np.eye(7), np.zeros(6))
    ekf.innovation = np.zeros(6)
    ekf.innovation_magnitude.return_value = 0.0

    dynamics = MagicMock()
    dynamics.derived_outputs.return_value = {
        'VPD': current_VPD, 'RH': 60.0, 'T_leaf': x_hat_T - 1.0,
    }

    Np = 6
    ref = MagicMock()
    # Reference: VPD target in col 0, T target in col 1, RH in col 5.
    # Production reference.generate() rebuilds the array every call; the
    # solver mutates it in place when applying the integral bias, so the
    # mock must hand back a fresh copy each time or biases would compound.
    def _fresh_ref(*_a, **_kw):
        traj = np.zeros((Np, 6))
        traj[:, 0] = target_VPD
        traj[:, 1] = target_T
        traj[:, 2] = 1000.0
        traj[:, 4] = 0.5
        return traj
    ref.generate.side_effect = _fresh_ref
    ref.Q_diag = np.ones(6)
    ref.R_diag = np.ones(2)
    ref.S_diag = np.ones(6)

    forecast = MagicMock()
    forecast.generate.return_value = np.zeros((Np, 6))

    mpc = MagicMock()
    mpc.Np = Np
    mpc.step_s = 60.0
    mpc.solve.return_value = (
        np.zeros(2),
        np.tile(x_hat, (Np, 1)),
        {'status': 'solved', 'iterations': 10},
    )
    mpc.force_cold_start = MagicMock()

    failsafe = MagicMock()
    failsafe.check_solve.return_value = (np.zeros(2), 'P1_NORMAL')
    failsafe.reset = MagicMock()

    registry = MagicMock()
    registry.default_control.return_value = np.zeros(2)
    registry.index_to_solution_field.return_value = {0: 'u_a', 1: 'u_b'}
    registry.get_all_effort_metrics.return_value = {}

    light_schedule = SimpleNamespace(is_on=lambda t: True)

    solver = SolverProcess(
        io=io, dynamics=dynamics, ekf=ekf, mpc=mpc,
        reference=ref, forecast=forecast, failsafe=failsafe,
        shared_params={}, light_schedule=light_schedule, registry=registry,
        integral_config=integral_config,
    )
    # Avoid touching the EKF light-transition reset path on the first tick.
    solver._last_lights_on = True
    return solver


def test_defaults_match_legacy_hardcoded_values():
    """No config → behaviour identical to the pre-config code."""
    solver = _make_solver(integral_config=None)
    assert solver._integral_gain == 0.003
    assert solver._t_clamp == 4.0
    assert solver._vpd_clamp == 0.3
    assert solver._integral_saturation_warn_s == 300.0


def test_config_overrides_apply():
    cfg = {
        'gain': 0.01,
        't_clamp_C': 2.0,
        'vpd_clamp_kPa': 0.5,
        'saturation_warn_s': 60.0,
    }
    solver = _make_solver(integral_config=cfg)
    assert solver._integral_gain == 0.01
    assert solver._t_clamp == 2.0
    assert solver._vpd_clamp == 0.5
    assert solver._integral_saturation_warn_s == 60.0


def test_integral_accumulates_per_documented_formula():
    """integral += gain × error × dt_solve, clipped at clamp.

    With gain=0.003, persistent T error of +2°C, dt_solve=1, after N
    solves the integral should equal min(N × 0.006, t_clamp).
    """
    solver = _make_solver(target_T=22.0, x_hat_T=20.0)
    for _ in range(100):
        solver.solve_once()
    # 100 ticks × 0.003 × 2.0 × 1 = 0.6 °C, well below the 4°C clamp
    assert abs(solver._t_integral - 0.6) < 1e-6


def test_integral_saturates_at_clamp():
    """A small clamp + persistent error pegs the integral to the clamp."""
    solver = _make_solver(
        integral_config={'gain': 0.01, 't_clamp_C': 0.05,
                         'vpd_clamp_kPa': 0.3, 'saturation_warn_s': 60.0},
        target_T=22.0, x_hat_T=20.0,
    )
    for _ in range(20):
        solver.solve_once()
    assert abs(solver._t_integral - 0.05) < 1e-9
    assert solver._t_saturated_since is not None


def test_saturation_clears_when_error_reverses():
    """Once the integral leaves the clamp, _t_saturated_since must reset."""
    solver = _make_solver(
        integral_config={'gain': 0.01, 't_clamp_C': 0.05,
                         'vpd_clamp_kPa': 0.3, 'saturation_warn_s': 60.0},
        target_T=22.0, x_hat_T=20.0,
    )
    for _ in range(20):
        solver.solve_once()
    assert solver._t_saturated_since is not None

    # Reverse the error gently — pull integral off the positive clamp into
    # the mid-band but not far enough to hit the negative clamp.
    Np = solver.mpc.Np

    def _reversed_ref(*_a, **_kw):
        traj = np.zeros((Np, 6))
        traj[:, 1] = 19.5  # gentle reversal: error = -0.5 per tick
        traj[:, 2] = 1000.0
        traj[:, 4] = 0.5
        return traj
    solver.reference.generate.side_effect = _reversed_ref

    for _ in range(5):
        solver.solve_once()
    assert solver._t_saturated_since is None
    assert -0.05 < solver._t_integral < 0.05


def test_saturation_advisory_state_exposed_in_telemetry():
    """The per-tick info dict must surface integral state for dashboards."""
    solver = _make_solver(
        integral_config={'gain': 0.01, 't_clamp_C': 0.05,
                         'vpd_clamp_kPa': 0.3, 'saturation_warn_s': 60.0},
        target_T=22.0, x_hat_T=20.0,
    )
    for _ in range(20):
        _, _, _, info = solver.solve_once()
    assert 't_integral' in info
    assert 't_integral_saturated' in info
    assert info['t_integral_saturated'] is True
    assert abs(info['t_integral'] - 0.05) < 1e-9
