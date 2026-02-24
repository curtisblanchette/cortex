"""Per-actuator dispatch rule engine.

Each actuator has its own dispatch rules: dead-band, minimum hold time,
rate limiting, conflict resolution, and anti-short-cycle protection.

The gateway evaluates these rules to decide WHETHER and WHEN to dispatch
a new command. It NEVER modifies the MPC setpoint — only suppresses or
delays it.
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class DispatchAction(Enum):
    DISPATCH = "dispatch"
    HOLD_DEADBAND = "hold_deadband"
    HOLD_MIN_TIME = "hold_min_time"
    HOLD_RATE_LIMIT = "hold_rate_limit"
    HOLD_CONFLICT = "hold_conflict"
    HOLD_ANTI_SHORT_CYCLE = "hold_anti_short_cycle"
    HOLD_LIGHTS_ONLY = "hold_lights_only"
    HOLD_MAX_DAILY = "hold_max_daily"


@dataclass
class DispatchResult:
    """Result of evaluating dispatch rules for one actuator."""
    action: DispatchAction
    command: float          # The command value (original or rate-limited)
    reason: str = ""

    @property
    def should_dispatch(self) -> bool:
        return self.action == DispatchAction.DISPATCH


@dataclass
class ActuatorState:
    """Tracks the current state of a single actuator."""
    last_command: float = 0.0
    last_change_time: float = 0.0
    last_on_time: float = 0.0       # When the actuator last turned on
    last_off_time: float = 0.0      # When the actuator last turned off
    daily_events: int = 0
    daily_reset_time: float = 0.0   # Start of current day for event counting


class DispatchRule:
    """Dispatch rules for a single actuator.

    Evaluates a proposed setpoint against the current actuator state
    and returns a DispatchResult indicating what to do.
    """

    def __init__(self, name, config):
        self.name = name
        self.dead_band = config.get('dead_band')
        self.min_hold_s = config.get('min_hold_s', 0)
        self.rate_limit = config.get('rate_limit')  # units/min, None = no limit
        self.anti_short_cycle_s = config.get('anti_short_cycle_s', 0)
        self.conflicts = config.get('conflicts', [])
        self.lights_only = config.get('lights_only', False)
        self.max_daily_events = config.get('max_daily_events')
        self.protocol = config.get('protocol', 'mqtt_relay')

    def evaluate(self, setpoint, state, now=None,
                 active_actuators=None, lights_on=True):
        """Evaluate whether to dispatch a new command.

        Args:
            setpoint: Proposed setpoint from MPC solution.
            state: ActuatorState for this actuator.
            now: Current time (time.time()). If None, uses wall clock.
            active_actuators: Set of actuator names currently active
                              (for conflict checking).
            lights_on: Whether lights are currently on.

        Returns:
            DispatchResult with action and command.
        """
        if now is None:
            now = time.time()

        if active_actuators is None:
            active_actuators = set()

        # Rule 1: Lights-only check (CO2 injection)
        if self.lights_only and not lights_on:
            return DispatchResult(
                action=DispatchAction.HOLD_LIGHTS_ONLY,
                command=0.0,
                reason=f"{self.name}: CO2 suppressed during dark period",
            )

        # Rule 2: Conflict resolution
        for conflict in self.conflicts:
            if conflict in active_actuators:
                return DispatchResult(
                    action=DispatchAction.HOLD_CONFLICT,
                    command=state.last_command,
                    reason=f"{self.name}: conflicts with active {conflict}",
                )

        # Rule 3: Dead-band
        if self.dead_band is not None:
            change = abs(setpoint - state.last_command)
            if change < self.dead_band:
                return DispatchResult(
                    action=DispatchAction.HOLD_DEADBAND,
                    command=state.last_command,
                    reason=f"{self.name}: change {change:.3f} < dead_band {self.dead_band}",
                )

        # Rule 4: Minimum hold time
        if self.min_hold_s > 0 and state.last_change_time > 0:
            elapsed = now - state.last_change_time
            if elapsed < self.min_hold_s:
                return DispatchResult(
                    action=DispatchAction.HOLD_MIN_TIME,
                    command=state.last_command,
                    reason=f"{self.name}: {elapsed:.0f}s < min_hold {self.min_hold_s}s",
                )

        # Rule 5: Anti-short-cycle (for compressors, binary actuators)
        if self.anti_short_cycle_s > 0:
            is_turning_on = setpoint > 0 and state.last_command == 0
            is_turning_off = setpoint == 0 and state.last_command > 0

            if is_turning_on and state.last_off_time > 0:
                off_elapsed = now - state.last_off_time
                if off_elapsed < self.anti_short_cycle_s:
                    return DispatchResult(
                        action=DispatchAction.HOLD_ANTI_SHORT_CYCLE,
                        command=state.last_command,
                        reason=f"{self.name}: off for {off_elapsed:.0f}s < "
                               f"anti_short_cycle {self.anti_short_cycle_s}s",
                    )

            if is_turning_off and state.last_on_time > 0:
                on_elapsed = now - state.last_on_time
                if on_elapsed < self.anti_short_cycle_s:
                    return DispatchResult(
                        action=DispatchAction.HOLD_ANTI_SHORT_CYCLE,
                        command=state.last_command,
                        reason=f"{self.name}: on for {on_elapsed:.0f}s < "
                               f"anti_short_cycle {self.anti_short_cycle_s}s",
                    )

        # Rule 6: Max daily events (irrigation safety cap)
        if self.max_daily_events is not None:
            # Reset counter at midnight
            if now - state.daily_reset_time > 86400:
                state.daily_events = 0
                state.daily_reset_time = now

            is_event = setpoint > 0 and state.last_command == 0
            if is_event and state.daily_events >= self.max_daily_events:
                return DispatchResult(
                    action=DispatchAction.HOLD_MAX_DAILY,
                    command=state.last_command,
                    reason=f"{self.name}: {state.daily_events} events >= "
                           f"max {self.max_daily_events}/day",
                )

        # Rule 7: Rate limiting
        command = setpoint
        if self.rate_limit is not None and state.last_change_time > 0:
            dt_min = (now - state.last_change_time) / 60.0
            if dt_min > 0:
                max_change = self.rate_limit * dt_min
                delta = setpoint - state.last_command
                if abs(delta) > max_change:
                    # Clamp to max rate
                    command = state.last_command + max_change * (1.0 if delta > 0 else -1.0)
                    return DispatchResult(
                        action=DispatchAction.HOLD_RATE_LIMIT,
                        command=command,
                        reason=f"{self.name}: rate-limited to {command:.2f} "
                               f"(wanted {setpoint:.2f})",
                    )

        # All rules passed — dispatch
        return DispatchResult(
            action=DispatchAction.DISPATCH,
            command=command,
            reason=f"{self.name}: dispatching {command:.3f}",
        )


# MPC control vector index → actuator name mapping
# u[0] T_sp → cool/heat, u[1] RH_sp → humidifier (indirect),
# u[2] u_hum, u[3] u_co2, u[4] u_fan, u[5] u_irr
SETPOINT_TO_ACTUATOR = {
    'u_T_sp': 'cool',
    'u_RH_sp': 'heat',
    'u_co2': 'co2',
    'u_hum': 'humidifier',
    'u_fan': 'fan',
    'u_irr': 'irrigation',
}

# Maps MPC control vector indices to Redis solution field names
CONTROL_INDEX_TO_FIELD = {
    0: 'u_T_sp',
    1: 'u_RH_sp',
    2: 'u_hum',
    3: 'u_co2',
    4: 'u_fan',
    5: 'u_irr',
}


def load_dispatch_rules(dispatch_config):
    """Load dispatch rules from config dict.

    Args:
        dispatch_config: The 'dispatch' section of equipment.yaml.

    Returns:
        Dict of {actuator_name: DispatchRule}.
    """
    rules = {}
    for name, cfg in dispatch_config.items():
        rules[name] = DispatchRule(name, cfg)
    return rules
