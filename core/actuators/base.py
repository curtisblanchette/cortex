"""Base classes for the actuator plugin system.

Every actuator plugin implements ActuatorPlugin and declares its control
channels, physics contributions, dispatch specs, and IO mappings.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class ControlChannel:
    """One control input that this plugin owns."""
    name: str                # e.g. 'T_sp', 'u_co2'
    default: float           # startup value
    unit: str                # display unit ('C', '%', etc.)
    bound_low: float         # MPC lower bound
    bound_high: float        # MPC upper bound
    du_low: float            # rate-of-change lower bound (per MPC step)
    du_high: float           # rate-of-change upper bound (per MPC step)
    jac_perturbation: float  # finite-diff step size for Jacobian


@dataclass
class PhysicsEffect:
    """Additive physics contribution from one plugin at one timestep.

    Each field maps to a state derivative:
      Q_sensible  -> dT_a/dt (via 1/C_air)
      dw_dt       -> dw_a/dt (g/kg/s)
      dC_dt       -> dCO2/dt (ppm/s)
      dtheta_dt   -> d(theta)/dt (m3/m3/s)
      T_supply_target -> target for supply temp first-order lag
    """
    Q_sensible: float = 0.0
    dw_dt: float = 0.0
    dC_dt: float = 0.0
    dtheta_dt: float = 0.0
    T_supply_target: Optional[float] = None


@dataclass
class DispatchSpec:
    """Maps a control channel to an actuator name for gateway dispatch."""
    channel_name: str        # control channel name (e.g. 'T_sp')
    actuator_name: str       # gateway actuator name (e.g. 'cool')
    solution_field: str      # Redis/solution dict field (e.g. 'u_T_sp')
    default_value: float     # fallback when no solution


@dataclass
class IOMapping:
    """Maps a control channel to a Redis/hardware field for RealIO."""
    channel_name: str        # control channel name
    io_field: str            # Redis hash field name (e.g. 'T_sp')


class ActuatorPlugin(ABC):
    """Base class for actuator plugins.

    Each plugin declares what control channels it owns, how those channels
    affect the physics, and how they map to dispatch and IO.
    """

    @abstractmethod
    def name(self) -> str:
        """Unique plugin identifier (e.g. 'hvacd', 'co2_injector')."""
        ...

    @abstractmethod
    def control_channels(self) -> List[ControlChannel]:
        """Declare the control channels this plugin owns."""
        ...

    @abstractmethod
    def compute_physics(self, x, u_local, d, params, room_props) -> PhysicsEffect:
        """Compute this plugin's physics contribution.

        Args:
            x: Full state vector (7,).
            u_local: This plugin's slice of the control vector.
            d: Disturbance vector (5,).
            params: CalibrationParams.
            room_props: Dict with room geometry (V, A_floor, A_canopy, etc.)
                        and 'fan_pct' (0-1) from FanPlugin.

        Returns:
            PhysicsEffect with additive contributions.
        """
        ...

    @abstractmethod
    def dispatch_specs(self) -> List[DispatchSpec]:
        """Map control channels to gateway actuator names."""
        ...

    @abstractmethod
    def io_mappings(self) -> List[IOMapping]:
        """Map control channels to Redis/hardware field names."""
        ...

    def requires_supply_temp_state(self) -> bool:
        """Whether this plugin drives the T_supply state (ODE 6)."""
        return False

    @abstractmethod
    def default_R_weights(self) -> List[float]:
        """Default MPC R (control effort) weights for each channel."""
        ...

    @abstractmethod
    def default_S_weights(self) -> List[float]:
        """Default MPC S (rate-of-change) weights for each channel."""
        ...
