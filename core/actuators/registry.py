"""ActuatorRegistry — single source of truth for control vector layout.

Assembled from plugins at startup. Every component queries the registry
instead of using hardcoded N_CONTROLS, CONTROL_NAMES, or u[i] indices.
"""

from typing import Dict, List, Optional, Tuple

import numpy as np

from .base import ActuatorPlugin, ControlChannel, PhysicsEffect, DispatchSpec, IOMapping


class ActuatorRegistry:
    """Assembled from plugins. Replaces every hardcoded control constant."""

    def __init__(self, plugins: List[ActuatorPlugin], room_props: dict):
        self._plugins: List[ActuatorPlugin] = list(plugins)
        self._room_props: dict = dict(room_props)

        # Build channel list and index maps
        self._channels: List[ControlChannel] = []
        self._plugin_slices: Dict[str, Tuple[int, int]] = {}  # name -> (start, end)
        self._channel_to_plugin: Dict[str, ActuatorPlugin] = {}

        offset = 0
        for plugin in self._plugins:
            channels = plugin.control_channels()
            start = offset
            for ch in channels:
                self._channels.append(ch)
                self._channel_to_plugin[ch.name] = plugin
                offset += 1
            self._plugin_slices[plugin.name()] = (start, offset)

        # Identify the fan plugin (if any) for pre-computing fan_pct
        self._fan_plugin = None
        self._fan_slice = None
        for plugin in self._plugins:
            if hasattr(plugin, 'get_fan_pct'):
                self._fan_plugin = plugin
                self._fan_slice = self._plugin_slices[plugin.name()]
                break

    # ---- Dimensions ----

    @property
    def n_u(self) -> int:
        """Total number of control channels (replaces N_CONTROLS)."""
        return len(self._channels)

    @property
    def channel_names(self) -> List[str]:
        """Ordered list of control channel names."""
        return [ch.name for ch in self._channels]

    @property
    def plugins(self) -> List[ActuatorPlugin]:
        return list(self._plugins)

    # ---- Default control vector ----

    def default_control(self) -> np.ndarray:
        """Default control vector (replaces GrowRoomDynamics.default_control())."""
        return np.array([ch.default for ch in self._channels])

    # ---- Bounds (for ConstraintBuilder) ----

    @property
    def u_min(self) -> np.ndarray:
        return np.array([ch.bound_low for ch in self._channels])

    @property
    def u_max(self) -> np.ndarray:
        return np.array([ch.bound_high for ch in self._channels])

    @property
    def du_min(self) -> np.ndarray:
        return np.array([ch.du_low for ch in self._channels])

    @property
    def du_max(self) -> np.ndarray:
        return np.array([ch.du_high for ch in self._channels])

    # ---- Jacobian perturbation sizes ----

    @property
    def h_u(self) -> np.ndarray:
        return np.array([ch.jac_perturbation for ch in self._channels])

    # ---- MPC weights ----

    @property
    def default_R_diag(self) -> np.ndarray:
        weights = []
        for plugin in self._plugins:
            weights.extend(plugin.default_R_weights())
        return np.array(weights)

    @property
    def default_S_diag(self) -> np.ndarray:
        weights = []
        for plugin in self._plugins:
            weights.extend(plugin.default_S_weights())
        return np.array(weights)

    # ---- Plugin slicing ----

    def local_u(self, plugin_name: str, u: np.ndarray) -> np.ndarray:
        """Extract a plugin's local control inputs from the global u vector."""
        start, end = self._plugin_slices[plugin_name]
        return u[start:end]

    def plugin_slice(self, plugin_name: str) -> Tuple[int, int]:
        """Return (start, end) indices for a plugin's channels in u."""
        return self._plugin_slices[plugin_name]

    # ---- Physics aggregation ----

    def compute_all_physics(self, x, u, d, params) -> PhysicsEffect:
        """Sum physics contributions from all plugins.

        Pre-computes fan_pct and passes it via room_props so that
        HVACD/AC plugins can use the current fan speed.

        Returns a single PhysicsEffect with summed contributions.
        """
        props = dict(self._room_props)

        # Pre-compute fan_pct if fan plugin is present
        if self._fan_plugin is not None:
            u_fan = u[self._fan_slice[0]:self._fan_slice[1]]
            props['fan_pct'] = self._fan_plugin.get_fan_pct(u_fan)

        total = PhysicsEffect()
        T_supply_targets = []

        for plugin in self._plugins:
            start, end = self._plugin_slices[plugin.name()]
            u_local = u[start:end]
            effect = plugin.compute_physics(x, u_local, d, params, props)

            total.Q_sensible += effect.Q_sensible
            total.supply_airflow_W_per_K += effect.supply_airflow_W_per_K
            total.dw_dt += effect.dw_dt
            total.dC_dt += effect.dC_dt
            total.dtheta_dt += effect.dtheta_dt

            if effect.T_supply_target is not None:
                T_supply_targets.append(effect.T_supply_target)

        # If any plugin drives supply temp, use weighted average
        # (in practice only one plugin should drive it)
        if T_supply_targets:
            total.T_supply_target = np.mean(T_supply_targets)

        return total

    def has_supply_temp_driver(self) -> bool:
        """Whether any plugin drives the T_supply state."""
        return any(p.requires_supply_temp_state() for p in self._plugins)

    # ---- Dispatch and IO mappings ----

    def all_dispatch_specs(self) -> List[DispatchSpec]:
        """Aggregated dispatch specs from all plugins."""
        specs = []
        for plugin in self._plugins:
            specs.extend(plugin.dispatch_specs())
        return specs

    def all_io_mappings(self) -> List[IOMapping]:
        """Aggregated IO mappings from all plugins."""
        mappings = []
        for plugin in self._plugins:
            mappings.extend(plugin.io_mappings())
        return mappings

    # ---- Solution field mappings (for solver_process) ----

    def index_to_solution_field(self) -> Dict[int, str]:
        """Map control index -> solution dict field name.

        Replaces dispatch.CONTROL_INDEX_TO_FIELD.
        """
        result = {}
        specs = self.all_dispatch_specs()
        for spec in specs:
            # Find the channel index
            for i, ch in enumerate(self._channels):
                if ch.name == spec.channel_name:
                    result[i] = spec.solution_field
                    break
        return result

    def solution_field_to_index(self) -> Dict[str, int]:
        """Map solution dict field name -> control index."""
        return {v: k for k, v in self.index_to_solution_field().items()}

    def solution_field_to_actuator(self) -> Dict[str, str]:
        """Map solution field -> actuator name (replaces SETPOINT_TO_ACTUATOR)."""
        result = {}
        for spec in self.all_dispatch_specs():
            result[spec.solution_field] = spec.actuator_name
        return result

    def actuator_order(self) -> List[str]:
        """Ordered list of actuator names for visualization."""
        return [spec.actuator_name for spec in self.all_dispatch_specs()]

    # ---- Effort metrics ----

    def get_all_effort_metrics(self) -> Dict[str, dict]:
        """Collect effort metrics from plugins that expose them.

        Returns:
            Dict mapping plugin name to its effort metrics dict.
        """
        result = {}
        for plugin in self._plugins:
            if hasattr(plugin, 'get_effort_metrics'):
                metrics = plugin.get_effort_metrics()
                if metrics:
                    result[plugin.name()] = metrics
        return result

    # ---- Power draw ----

    def power_draw_all(self, u: np.ndarray) -> Dict[str, float]:
        """Compute power draw for each plugin given the global control vector.

        Returns:
            Dict mapping plugin name to instantaneous power in watts.
        """
        result = {}
        for plugin in self._plugins:
            start, end = self._plugin_slices[plugin.name()]
            u_local = u[start:end]
            result[plugin.name()] = plugin.power_draw_W(u_local)
        return result
