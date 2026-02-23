"""Extended Kalman Filter for state estimation.

Fuses noisy sensor data with model predictions to produce the best
estimate of the room's true state. The EKF is what makes the linear
OSQP solver work despite nonlinearities — it keeps the operating
point accurate.

Runs every control cycle (1 Hz):
  PREDICT: propagate state estimate forward using nonlinear model
  UPDATE: correct prediction using sensor measurements
"""

import numpy as np

from core.dynamics import GrowRoomDynamics, N_STATES
from simulation.noise import N_MEASUREMENTS, STATE_TO_MEAS


class StateEstimator:
    """Extended Kalman Filter for 7-state grow room model.

    Predict step uses the full nonlinear model (same code as simulation).
    Update step uses linearized observation to fuse sensor data.
    """

    def __init__(self, dynamics, Q=None, R=None, P0=None):
        """Initialize EKF.

        Args:
            dynamics: GrowRoomDynamics instance.
            Q: Process noise covariance (7, 7). If None, uses defaults.
            R: Measurement noise covariance (6, 6). If None, uses defaults.
            P0: Initial state covariance (7, 7). If None, uses defaults.
        """
        self.dynamics = dynamics
        self.n_x = N_STATES
        self.n_z = N_MEASUREMENTS

        # Process noise covariance (model uncertainty)
        if Q is not None:
            self.Q = Q
        else:
            self.Q = np.diag([
                0.01,    # T_a: ±0.1°C per step uncertainty
                0.004,   # w_a: ±0.06 g/kg
                1.0,     # CO2: ±1 ppm
                0.01,    # T_l: ±0.1°C
                0.0001,  # θ: ±0.01 m³/m³
                0.01,    # T_s: ±0.1°C
                0.001,   # T_w: ±0.03°C (thermal mass changes slowly)
            ])

        # Measurement noise covariance (sensor uncertainty)
        if R is not None:
            self.R = R
        else:
            self.R = np.diag([
                0.09,    # T_air: σ² = 0.3²
                0.04,    # w_a: σ² = 0.2²
                900.0,   # CO2: σ² = 30²
                0.25,    # T_leaf: σ² = 0.5²
                0.0001,  # theta: σ² = 0.01²
                0.09,    # T_supply: σ² = 0.3²
            ])

        # State estimate and covariance
        self.x_hat = GrowRoomDynamics.default_initial_state()
        if P0 is not None:
            self.P = P0.copy()
        else:
            self.P = np.diag([
                1.0, 0.5, 100.0, 1.0, 0.01, 1.0, 1.0
            ])

        # Observation matrix: z = H @ x (linear, selects measured states)
        self.H = np.zeros((self.n_z, self.n_x))
        for i, j in enumerate(STATE_TO_MEAS):
            self.H[i, j] = 1.0

        # Track innovation for diagnostics
        self.innovation = np.zeros(self.n_z)
        self.innovation_covariance = np.zeros((self.n_z, self.n_z))

    def initialize(self, x0, P0=None):
        """Set initial state estimate.

        Args:
            x0: Initial state vector (7,).
            P0: Initial covariance (7, 7). Optional.
        """
        self.x_hat = np.array(x0, dtype=np.float64)
        if P0 is not None:
            self.P = P0.copy()

    def predict(self, u, d, params, dt=1.0):
        """EKF predict step.

        Propagate state estimate forward using nonlinear model.
        Propagate covariance using linearized model (Jacobian).

        Args:
            u: Control vector (6,).
            d: Disturbance vector (5,).
            params: CalibrationParams.
            dt: Time step in seconds.

        Returns:
            x_pred: Predicted state (7,).
            P_pred: Predicted covariance (7, 7).
        """
        # State prediction (nonlinear)
        x_pred = self.dynamics.nonlinear_step(self.x_hat, u, d, params, dt)

        # Linearize for covariance propagation
        A_c, _, _ = self.dynamics._analytical_jacobians(self.x_hat, u, d, params)
        # Simple discrete F ≈ I + A_c * dt (first-order)
        F = np.eye(self.n_x) + A_c * dt

        # Covariance prediction
        P_pred = F @ self.P @ F.T + self.Q

        return x_pred, P_pred

    def update(self, z_meas, u, d, params, dt=1.0):
        """Full EKF cycle: predict then update.

        Args:
            z_meas: Measurement vector (6,). May contain NaN for missing sensors.
            u: Control vector (6,).
            d: Disturbance vector (5,).
            params: CalibrationParams.
            dt: Time step in seconds.

        Returns:
            x_hat: Updated state estimate (7,).
            P: Updated covariance (7, 7).
            innovation: Measurement innovation (6,).
        """
        # Predict
        x_pred, P_pred = self.predict(u, d, params, dt)

        # Handle missing sensors (NaN values)
        H, R, z_valid, valid_mask = self._build_observation(z_meas)

        if len(z_valid) == 0:
            # No valid measurements — prediction only
            self.x_hat = x_pred
            self.P = P_pred
            self.innovation = np.full(self.n_z, np.nan)
            return self.x_hat.copy(), self.P.copy(), self.innovation.copy()

        # Innovation (measurement residual)
        z_pred = H @ x_pred
        innovation = z_valid - z_pred

        # Innovation covariance
        S = H @ P_pred @ H.T + R

        # Kalman gain
        try:
            K = P_pred @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Fallback: use pseudo-inverse
            K = P_pred @ H.T @ np.linalg.pinv(S)

        # State update
        self.x_hat = x_pred + K @ innovation

        # Covariance update (Joseph form for numerical stability)
        IKH = np.eye(self.n_x) - K @ H
        self.P = IKH @ P_pred @ IKH.T + K @ R @ K.T

        # Ensure symmetry
        self.P = 0.5 * (self.P + self.P.T)

        # Store full innovation (with NaN for missing)
        full_innovation = np.full(self.n_z, np.nan)
        valid_indices = np.where(valid_mask)[0]
        for i, idx in enumerate(valid_indices):
            full_innovation[idx] = innovation[i]
        self.innovation = full_innovation
        self.innovation_covariance = S

        return self.x_hat.copy(), self.P.copy(), self.innovation.copy()

    def _build_observation(self, z_meas):
        """Build observation matrices for available sensors.

        Handles missing sensors (NaN values) by reducing the
        observation matrix and measurement vector.

        Args:
            z_meas: Raw measurement vector (6,). NaN = missing.

        Returns:
            H: Observation matrix (n_valid, 7).
            R: Measurement noise covariance (n_valid, n_valid).
            z_valid: Valid measurements (n_valid,).
            valid_mask: Boolean mask (6,).
        """
        valid_mask = ~np.isnan(z_meas)
        valid_indices = np.where(valid_mask)[0]

        if len(valid_indices) == 0:
            return np.zeros((0, self.n_x)), np.zeros((0, 0)), np.array([]), valid_mask

        H = self.H[valid_indices, :]
        R = self.R[np.ix_(valid_indices, valid_indices)]
        z_valid = z_meas[valid_indices]

        return H, R, z_valid, valid_mask

    def innovation_magnitude(self):
        """RMS of non-NaN innovations. Used to trigger recalibration."""
        valid = self.innovation[~np.isnan(self.innovation)]
        if len(valid) == 0:
            return 0.0
        # Normalize by R diagonal for dimensionless comparison
        valid_idx = np.where(~np.isnan(self.innovation))[0]
        R_diag = np.sqrt(np.diag(self.R)[valid_idx])
        normalized = valid / R_diag
        return np.sqrt(np.mean(normalized ** 2))
