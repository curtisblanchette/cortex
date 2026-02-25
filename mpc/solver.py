"""MPC Solver — OSQP-based quadratic program for optimal control.

Formulates and solves the MPC problem as a sparse QP:

    min  Σ ||y[k] - r[k]||²_Q  +  Σ ||u[k]||²_R  +  Σ ||Δu[k]||²_S
    s.t. x[k+1] = A_d x[k] + B_d u[k] + Bd_d d[k]   (dynamics)
         x_min ≤ x[k] ≤ x_max                          (state bounds)
         u_min ≤ u[k] ≤ u_max                          (actuator bounds)
         Δu_min ≤ u[k] - u[k-1] ≤ Δu_max              (rate limits)

Decision variables: [u[0], ..., u[Nc-1], x[1], ..., x[Np]]
"""

import numpy as np
import osqp
from osqp import SolverStatus
from scipy import sparse

from core.dynamics import GrowRoomDynamics, N_STATES, N_DISTURBANCES
from .constraints import ConstraintBuilder


# Output mapping: how to compute tracked outputs from state
# y = C_out @ x  where y = [VPD_approx, T_a, CO2, dθ/dt_approx, θ]
# For MPC, we linearize VPD and dryback around the operating point
N_OUTPUTS = 5


class MPCSolver:
    """OSQP-based MPC solver for grow room control.

    Horizon parameters:
        Np = 60: Prediction horizon (60 min at 1 min/step)
        Nc = 15: Control horizon (15 min, then hold last value)
    """

    def __init__(self, dynamics, shared_params, registry, Np=60, Nc=15,
                 constraints=None, step_s=60.0):
        """Initialize MPC solver.

        Args:
            dynamics: GrowRoomDynamics instance.
            shared_params: SharedParams or CalibrationParams.
            registry: ActuatorRegistry instance.
            Np: Prediction horizon steps.
            Nc: Control horizon steps.
            constraints: ConstraintBuilder instance.
            step_s: Seconds per MPC prediction step.
        """
        self.dynamics = dynamics
        self.shared_params = shared_params
        self.registry = registry
        self.Np = Np
        self.Nc = Nc
        self.n_x = N_STATES
        self.n_u = registry.n_u
        self.n_y = N_OUTPUTS
        self.step_s = step_s

        self.constraints = constraints or ConstraintBuilder(registry)

        # QP solver instance (created on first solve, reused via update())
        self._solver = None
        self._solver_initialized = False

        # CSC sparsity templates (row/col indices in CSC data order, set on first solve)
        self._P_row_idx = None   # row indices of P's structural nonzeros
        self._P_col_idx = None   # col indices (derived from indptr)
        self._A_row_idx = None   # same for A
        self._A_col_idx = None
        self._P_template = None  # CSC template for P (structural pattern)
        self._A_template = None  # CSC template for A (structural pattern)

        # Previous solution for warm-starting (primal + dual)
        self._prev_u = None
        self._prev_x = None
        self._prev_y = None

        # Solve statistics
        self.last_solve_info = {
            'status': 'not_solved',
            'iterations': 0,
            'solve_time_ms': 0.0,
            'setup_time_ms': 0.0,
        }

    def solve(self, x_hat, d_forecast, reference, u_prev=None,
              Q_diag=None, R_diag=None, S_diag=None):
        """Solve MPC QP and return optimal control.

        Args:
            x_hat: Current state estimate (7,).
            d_forecast: Disturbance forecast (Np, 5).
            reference: Reference trajectory (Np, 5).
            u_prev: Previous control action (6,). For Δu computation.
            Q_diag: Output tracking weights (5,).
            R_diag: Control effort weights (6,).
            S_diag: Control rate weights (6,).

        Returns:
            u_opt: Optimal first control move (6,).
            trajectory: Predicted state trajectory (Np, 7).
            info: Dict with solve diagnostics.
        """
        import time as time_mod
        t_start = time_mod.perf_counter()

        params = self._get_params()

        if u_prev is None:
            u_prev = self.registry.default_control()

        # Default weights
        if Q_diag is None:
            Q_diag = np.array([10.0, 5.0, 1.0, 3.0, 2.0])
        if R_diag is None:
            R_diag = self.registry.default_R_diag
        if S_diag is None:
            S_diag = self.registry.default_S_diag

        # Linearize around current operating point
        u0 = u_prev
        d0 = d_forecast[0] if len(d_forecast) > 0 else GrowRoomDynamics.default_disturbance()
        A_d, B_d, Bd_d, c_d = self.dynamics.linearize(x_hat, u0, d0, params, dt=self.step_s)

        # Affine output model: y = C_out @ x + d_offset
        C_out, d_offset = self._build_output_matrix(x_hat, params)

        # Shift reference so QP tracks (C @ x) toward (r - d_offset),
        # which makes the true output y = C@x + d track the original r.
        reference_adj = reference - d_offset[np.newaxis, :]

        # Build and solve QP
        try:
            u_opt_seq, x_pred_seq, info = self._solve_qp(
                x_hat, A_d, B_d, Bd_d, C_out,
                d_forecast, reference_adj, u_prev,
                Q_diag, R_diag, S_diag,
                c_d=c_d,
            )

            t_total = (time_mod.perf_counter() - t_start) * 1000.0

            self.last_solve_info = {
                'status': info.get('status', 'solved'),
                'iterations': info.get('iterations', 0),
                'solve_time_ms': t_total,
                'setup_time_ms': info.get('setup_time_ms', 0.0),
            }

            # Extract first control move
            u_opt = u_opt_seq[:self.n_u]

            # Store for warm-starting
            self._prev_u = u_opt_seq
            self._prev_x = x_pred_seq

            # Build trajectory
            trajectory = x_pred_seq.reshape(self.Np, self.n_x)

            return u_opt, trajectory, self.last_solve_info

        except Exception as e:
            t_total = (time_mod.perf_counter() - t_start) * 1000.0
            self.last_solve_info = {
                'status': f'error: {str(e)}',
                'iterations': 0,
                'solve_time_ms': t_total,
                'setup_time_ms': 0.0,
            }
            # Return previous control if available
            u_fallback = u_prev.copy()
            trajectory = np.tile(x_hat, (self.Np, 1))
            return u_fallback, trajectory, self.last_solve_info

    def _build_qp_matrices(self, x0, A_d, B_d, Bd_d, C_out, d_forecast,
                           reference, u_prev, Q_diag, R_diag, S_diag, c_d=None):
        """Assemble QP cost and constraint matrices (dense).

        Returns:
            P_dense: Hessian (n_dec, n_dec), symmetric, pre-regularized.
            q: Gradient (n_dec,).
            A_dense: Constraint matrix (n_con, n_dec).
            l_all: Lower bounds (n_con,).
            u_all: Upper bounds (n_con,).
        """
        Np, Nc = self.Np, self.Nc
        n_x, n_u, n_y = self.n_x, self.n_u, self.n_y

        n_dec = Nc * n_u + Np * n_x  # decision variables

        # ===== Cost function: P (Hessian) and q (gradient) =====
        P = np.zeros((n_dec, n_dec))
        q = np.zeros(n_dec)

        # Control cost (R) and rate cost (S)
        # R penalizes ||u[k] - u_prev||²  (deviation from previous control)
        # NOT ||u[k]||² which biases setpoints like T_sp, RH_sp toward zero
        for k in range(Nc):
            u_start = k * n_u
            u_end = u_start + n_u
            P[u_start:u_end, u_start:u_end] += np.diag(R_diag)
            q[u_start:u_end] -= R_diag * u_prev

            # Rate-of-change cost
            if k == 0:
                # Δu[0] = u[0] - u_prev
                P[u_start:u_end, u_start:u_end] += np.diag(S_diag)
                q[u_start:u_end] -= S_diag * u_prev
            else:
                prev_start = (k - 1) * n_u
                P[u_start:u_end, u_start:u_end] += np.diag(S_diag)
                P[prev_start:prev_start + n_u, prev_start:prev_start + n_u] += np.diag(S_diag)
                P[u_start:u_end, prev_start:prev_start + n_u] -= np.diag(S_diag)
                P[prev_start:prev_start + n_u, u_start:u_end] -= np.diag(S_diag)

        # State tracking cost (Q via output matrix)
        x_offset = Nc * n_u
        Q_full = C_out.T @ np.diag(Q_diag) @ C_out  # n_x × n_x

        for k in range(Np):
            x_start = x_offset + k * n_x
            x_end = x_start + n_x
            P[x_start:x_end, x_start:x_end] += Q_full

            # Reference tracking: q += -2 * Q * C_out^T * r[k]
            ref_k = reference[k] if k < len(reference) else reference[-1]
            q[x_start:x_end] -= C_out.T @ np.diag(Q_diag) @ ref_k

        # ===== Equality constraints: affine dynamics =====
        # x[k+1] = A_d @ x[k] + B_d @ u[k] + Bd_d @ d[k] + c_d
        # Rearranged: A_d @ x[k] + B_d @ u[k] - x[k+1] = -Bd_d @ d[k] - c_d

        if c_d is None:
            c_d = np.zeros(n_x)

        n_eq = Np * n_x
        A_eq = np.zeros((n_eq, n_dec))
        b_eq = np.zeros(n_eq)

        for k in range(Np):
            row_start = k * n_x
            row_end = row_start + n_x
            x_k1_start = x_offset + k * n_x  # x[k+1] position

            # -I on x[k+1]
            A_eq[row_start:row_end, x_k1_start:x_k1_start + n_x] = -np.eye(n_x)

            # A_d on x[k]
            if k == 0:
                # x[0] = x0 (known), so A_d @ x0 goes to RHS
                b_eq[row_start:row_end] = -A_d @ x0
            else:
                x_k_start = x_offset + (k - 1) * n_x
                A_eq[row_start:row_end, x_k_start:x_k_start + n_x] = A_d

            # B_d on u[k] (use u[Nc-1] for k >= Nc — hold last control)
            u_k = min(k, Nc - 1)
            u_k_start = u_k * n_u
            A_eq[row_start:row_end, u_k_start:u_k_start + n_u] = B_d

            # Disturbance + affine offset on RHS
            d_k = d_forecast[k] if k < len(d_forecast) else d_forecast[-1]
            b_eq[row_start:row_end] -= Bd_d @ d_k + c_d

        # ===== Inequality constraints: bounds =====
        x_lb, x_ub = self.constraints.get_state_bounds(Np)
        u_lb, u_ub = self.constraints.get_control_bounds(Nc)

        # Assemble full bounds
        lb = np.concatenate([u_lb, x_lb])
        ub = np.concatenate([u_ub, x_ub])

        # Rate-of-change constraints: du_min ≤ u[k] - u[k-1] ≤ du_max
        n_du = Nc * n_u
        A_du = np.zeros((n_du, n_dec))
        du_lower = np.zeros(n_du)
        du_upper = np.zeros(n_du)

        for k in range(Nc):
            row_start = k * n_u
            u_k_start = k * n_u
            A_du[row_start:row_start + n_u, u_k_start:u_k_start + n_u] = np.eye(n_u)

            if k == 0:
                du_lower[row_start:row_start + n_u] = self.constraints.du_min + u_prev
                du_upper[row_start:row_start + n_u] = self.constraints.du_max + u_prev
            else:
                prev_start = (k - 1) * n_u
                A_du[row_start:row_start + n_u, prev_start:prev_start + n_u] = -np.eye(n_u)
                du_lower[row_start:row_start + n_u] = self.constraints.du_min
                du_upper[row_start:row_start + n_u] = self.constraints.du_max

        # Stack all constraints
        A_dense = np.vstack([
            A_eq,                    # Equality (dynamics)
            np.eye(n_dec),           # Variable bounds
            A_du,                    # Rate constraints
        ])

        l_all = np.concatenate([b_eq, lb, du_lower])
        u_all = np.concatenate([b_eq, ub, du_upper])

        return P, q, A_dense, l_all, u_all

    def _solve_qp(self, x0, A_d, B_d, Bd_d, C_out, d_forecast, reference,
                  u_prev, Q_diag, R_diag, S_diag, c_d=None):
        """Formulate and solve the QP.

        Decision variables z = [u[0], ..., u[Nc-1], x[1], ..., x[Np]]
        Total variables: Nc * n_u + Np * n_x

        On the first call, creates the OSQP instance via setup(). On subsequent
        calls, reuses the instance via update() to skip the expensive symbolic
        KKT factorization.
        """
        Np, Nc = self.Np, self.Nc
        n_x, n_u = self.n_x, self.n_u
        n_dec = Nc * n_u + Np * n_x

        # Assemble dense QP matrices
        P_dense, q, A_dense, l_all, u_all = self._build_qp_matrices(
            x0, A_d, B_d, Bd_d, C_out, d_forecast, reference,
            u_prev, Q_diag, R_diag, S_diag, c_d=c_d,
        )

        # Symmetrize + regularize P, take upper triangle
        P_dense = 0.5 * (P_dense + P_dense.T)
        P_dense += 1e-6 * np.eye(n_dec)
        P_upper = np.triu(P_dense)

        if not self._solver_initialized:
            # === First solve: build structural templates + full setup ===
            self._P_template, self._A_template = self._build_structural_templates()

            # Save CSC-order index arrays (row/col for each entry in .data)
            self._P_row_idx = self._P_template.indices.copy()
            self._P_col_idx = np.repeat(
                np.arange(self._P_template.shape[1]),
                np.diff(self._P_template.indptr))
            self._A_row_idx = self._A_template.indices.copy()
            self._A_col_idx = np.repeat(
                np.arange(self._A_template.shape[1]),
                np.diff(self._A_template.indptr))

            # Fill template copies with actual values
            P_csc = self._P_template.copy()
            P_csc.data[:] = P_upper[self._P_row_idx, self._P_col_idx]
            A_csc = self._A_template.copy()
            A_csc.data[:] = A_dense[self._A_row_idx, self._A_col_idx]

            self._solver = osqp.OSQP()
            self._solver.setup(
                P=P_csc, q=q, A=A_csc, l=l_all, u=u_all,
                eps_abs=1e-3,
                eps_rel=1e-3,
                max_iter=4000,
                warm_start=True,
                verbose=False,
                polish=True,
                adaptive_rho=True,
                scaling=10,
            )
            self._solver_initialized = True
        else:
            # === Subsequent solves: update values only ===
            try:
                Px = P_upper[self._P_row_idx, self._P_col_idx]
                Ax = A_dense[self._A_row_idx, self._A_col_idx]
                self._solver.update(Px=Px, q=q, Ax=Ax, l=l_all, u=u_all)
            except Exception:
                # Dimension mismatch or other error — reset and retry
                self._solver_initialized = False
                self._solver = None
                return self._solve_qp(
                    x0, A_d, B_d, Bd_d, C_out, d_forecast, reference,
                    u_prev, Q_diag, R_diag, S_diag, c_d=c_d,
                )

        # Warm-start from previous solution (primal + dual)
        if self._prev_u is not None and self._prev_x is not None:
            try:
                x_warm = self._shift_warmstart()
                y_warm = self._shift_dual_warmstart() if self._prev_y is not None else None
                self._solver.warm_start(x=x_warm, y=y_warm)
            except (ValueError, IndexError):
                pass

        import time as _t
        _t0 = _t.perf_counter()
        result = self._solver.solve()
        _solve_ms = (_t.perf_counter() - _t0) * 1000

        info = {
            'status': result.info.status,
            'iterations': result.info.iter,
            'setup_time_ms': result.info.setup_time * 1000 if hasattr(result.info, 'setup_time') else 0,
            '_solve_only_ms': _solve_ms,
        }

        if result.info.status_val not in (
            SolverStatus.OSQP_SOLVED,
            SolverStatus.OSQP_SOLVED_INACCURATE,
        ):
            raise RuntimeError(f"OSQP failed: {result.info.status}")

        # Save dual variables for next warm-start
        self._prev_y = result.y

        u_opt_seq = result.x[:Nc * n_u]
        x_pred_seq = result.x[Nc * n_u:]

        return u_opt_seq, x_pred_seq, info

    def _build_output_matrix(self, x0, params):
        """Build affine output model: y = C @ x + d_offset.

        Maps state x to tracked outputs y:
          y[0] = VPD (linearized around x0 with affine correction)
          y[1] = T_a (direct)
          y[2] = CO2 (direct)
          y[3] = dryback rate (disabled — needs finite-difference formulation)
          y[4] = theta (direct)

        Returns:
            C: Output matrix (N_OUTPUTS, n_x).
            d_offset: Affine offset (N_OUTPUTS,) such that y(x0) = C @ x0 + d.
        """
        from core.thermodynamics import saturation_vapor_pressure, d_svp_dT, vpd, P_ATM

        C = np.zeros((N_OUTPUTS, self.n_x))
        d_offset = np.zeros(N_OUTPUTS)

        T_l = x0[3]
        w_a = x0[1]

        # VPD = SVP(T_l) - e_air where e_air = w_a * P / (622 + w_a)
        # d(VPD)/dT_l = dSVP/dT_l
        C[0, 3] = d_svp_dT(T_l)
        # d(VPD)/dw_a = -d(e_air)/dw_a = -P * 622 / (622 + w_a)^2
        C[0, 1] = -P_ATM * 622.0 / (622.0 + w_a) ** 2
        # Affine offset: VPD(x0) = C[0,:] @ x0 + d[0]  =>  d[0] = VPD(x0) - C[0,:] @ x0
        d_offset[0] = vpd(T_l, w_a) - C[0, :] @ x0

        # T_a (direct, no offset needed)
        C[1, 0] = 1.0

        # CO2 (direct, no offset needed)
        C[2, 2] = 1.0

        # Dryback rate: disabled. The previous C[3,4]=-1 mapped -theta (a value)
        # not dtheta/dt (a rate), which fought against theta tracking.
        # Proper dryback rate needs finite-difference across horizon steps.
        # C[3, :] = 0  (already zero)

        # Theta (direct, no offset needed)
        C[4, 4] = 1.0

        return C, d_offset

    def _shift_warmstart(self):
        """Shift previous solution forward by one step for warm-starting."""
        Nc, Np = self.Nc, self.Np
        n_u, n_x = self.n_u, self.n_x

        # Shift u: [u[1], ..., u[Nc-1], u[Nc-1]]
        u_warm = np.zeros(Nc * n_u)
        for k in range(Nc - 1):
            u_warm[k * n_u:(k + 1) * n_u] = self._prev_u[(k + 1) * n_u:(k + 2) * n_u]
        u_warm[(Nc - 1) * n_u:] = self._prev_u[(Nc - 1) * n_u:Nc * n_u]

        # Shift x: [x[2], ..., x[Np], x[Np]]
        x_warm = np.zeros(Np * n_x)
        for k in range(Np - 1):
            x_warm[k * n_x:(k + 1) * n_x] = self._prev_x[(k + 1) * n_x:(k + 2) * n_x]
        x_warm[(Np - 1) * n_x:] = self._prev_x[(Np - 1) * n_x:Np * n_x]

        return np.concatenate([u_warm, x_warm])

    def _shift_dual_warmstart(self):
        """Shift previous dual variables forward by one step.

        The dual (y) vector corresponds to constraints:
            [dynamics (Np*n_x), bounds (n_dec), rate (Nc*n_u)]
        Shift the dynamics block forward; bounds and rate blocks stay as-is.
        """
        Np, Nc = self.Np, self.Nc
        n_x, n_u = self.n_x, self.n_u
        n_dec = Nc * n_u + Np * n_x

        y = self._prev_y.copy()
        n_eq = Np * n_x

        # Shift dynamics dual: [y_eq[1], ..., y_eq[Np-1], y_eq[Np-1]]
        for k in range(Np - 1):
            y[k * n_x:(k + 1) * n_x] = self._prev_y[(k + 1) * n_x:(k + 2) * n_x]
        # Last block repeats
        y[(Np - 1) * n_x:Np * n_x] = self._prev_y[(Np - 1) * n_x:Np * n_x]

        # Bounds and rate duals (indices n_eq onward) are kept as-is
        return y

    def _build_structural_templates(self):
        """Build CSC template matrices covering all structurally-nonzero positions.

        The sparsity pattern of P and A varies across linearization points
        (e.g., A_d, B_d, Q_full entries can be zero at one operating point
        and nonzero at another). This method fills ALL block positions with
        1.0, ensuring the CSC structure has slots for every entry that could
        ever be nonzero. OSQP's update() can then set any value at those
        positions without hitting "not quasidefinite" errors.
        """
        Np, Nc = self.Np, self.Nc
        n_x, n_u = self.n_x, self.n_u
        n_dec = Nc * n_u + Np * n_x
        x_off = Nc * n_u

        # --- P template (upper triangle) ---
        P = np.zeros((n_dec, n_dec))
        np.fill_diagonal(P, 1.0)  # regularization diagonal
        for k in range(Nc):
            s = k * n_u
            P[s:s+n_u, s:s+n_u] = 1.0          # R + S diagonal blocks
            if k > 0:
                ps = (k - 1) * n_u
                P[ps:ps+n_u, s:s+n_u] = 1.0    # S off-diagonal (upper tri)
        for k in range(Np):
            s = x_off + k * n_x
            P[s:s+n_x, s:s+n_x] = 1.0          # Q_full diagonal blocks
        P = np.triu(P)
        P_csc = sparse.csc_matrix(P)

        # --- A template ---
        n_eq = Np * n_x
        n_du = Nc * n_u
        n_con = n_eq + n_dec + n_du
        A = np.zeros((n_con, n_dec))

        # Dynamics rows
        for k in range(Np):
            rs = k * n_x
            x_k1 = x_off + k * n_x
            A[rs:rs+n_x, x_k1:x_k1+n_x] = 1.0         # -I on x[k+1]
            if k > 0:
                x_k = x_off + (k - 1) * n_x
                A[rs:rs+n_x, x_k:x_k+n_x] = 1.0        # A_d on x[k] (dense)
            u_k = min(k, Nc - 1)
            u_s = u_k * n_u
            A[rs:rs+n_x, u_s:u_s+n_u] = 1.0             # B_d on u[k] (dense)

        # Bounds rows: identity
        A[n_eq:n_eq+n_dec, :] = np.eye(n_dec)

        # Rate rows: I on u[k], -I on u[k-1]
        for k in range(Nc):
            rs = n_eq + n_dec + k * n_u
            u_s = k * n_u
            A[rs:rs+n_u, u_s:u_s+n_u] = 1.0
            if k > 0:
                ps = (k - 1) * n_u
                A[rs:rs+n_u, ps:ps+n_u] = 1.0

        A_csc = sparse.csc_matrix(A)

        return P_csc, A_csc

    def _get_params(self):
        """Get current calibration parameters."""
        if hasattr(self.shared_params, 'current'):
            return self.shared_params.current()
        return self.shared_params
