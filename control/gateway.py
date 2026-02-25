"""Actuator Gateway — reads MPC solutions, dispatches to hardware.

This is Process 2 in the decoupled architecture. It is the ONLY
process that writes actuator commands to hardware.

Responsibilities:
  - Read latest MPC solution from the solution store (Redis or in-memory)
  - Apply per-actuator dispatch rules (dead-band, hold, rate limit, etc.)
  - Write commands to hardware via MQTT/Modbus
  - Publish gateway status for monitoring

The gateway NEVER modifies MPC setpoints. It only decides WHETHER and
WHEN to dispatch them.
"""

import asyncio
import json
import time
import logging
from datetime import datetime, timezone

from control.dispatch import (
    DispatchRule, ActuatorState, DispatchResult, DispatchAction,
    load_dispatch_rules,
)

logger = logging.getLogger(__name__)


class ActuatorGateway:
    """Reads MPC solutions, dispatches to hardware with per-device rules.

    In simulation mode, reads from SolutionStore and applies dispatch
    filtering. The actual sim physics are still advanced by the solver
    process (which calls io.write_actuators). The gateway's role in sim
    mode is to validate dispatch logic and log decisions.

    In production mode, this is the sole writer to hardware (MQTT/Modbus).
    """

    def __init__(self, solution_store, dispatch_config, registry,
                 redis_client=None, mqtt_client=None, modbus_client=None,
                 room_id='room1', light_schedule=None,
                 stale_threshold_multiplier=2.0):
        self.solution_store = solution_store
        self.redis = redis_client
        self.mqtt = mqtt_client
        self.modbus = modbus_client
        self.room_id = room_id
        self.light_schedule = light_schedule
        self.registry = registry

        # Load dispatch rules from config
        self.dispatch_rules = load_dispatch_rules(dispatch_config)

        # Per-actuator state tracking
        self.actuator_state = {}
        for name in self.dispatch_rules:
            self.actuator_state[name] = ActuatorState()

        # Staleness detection
        self.stale_threshold_multiplier = stale_threshold_multiplier
        self.last_solution_id = None
        self.last_solution_time = 0.0

        # Running state
        self.running = False

        # Dispatch log buffer (for observability)
        self._dispatch_log = []
        self._max_log_entries = 1000

    def evaluate_solution(self, solution, current_time_s=0.0):
        """Evaluate a solution and return dispatch decisions for each actuator.

        This is the core logic, usable in both sync and async modes.

        Args:
            solution: Dict from SolutionStore/Redis.
            current_time_s: Current time for light schedule checks.

        Returns:
            List of (actuator_name, DispatchResult) tuples.
        """
        now = time.time()
        decisions = []

        # Determine which actuators are currently active (for conflict checks)
        active = set()
        for name, state in self.actuator_state.items():
            if state.last_command > 0:
                active.add(name)

        # Check lights status
        lights_on = True
        if self.light_schedule:
            lights_on = self.light_schedule.is_on(current_time_s)

        # Extract setpoints from solution and map to actuator names
        setpoint_map = self._extract_setpoints(solution)

        for actuator_name, setpoint in setpoint_map.items():
            if actuator_name not in self.dispatch_rules:
                continue

            rule = self.dispatch_rules[actuator_name]
            state = self.actuator_state[actuator_name]

            result = rule.evaluate(
                setpoint, state, now=now,
                active_actuators=active, lights_on=lights_on,
            )

            if result.should_dispatch:
                # Update actuator state
                old_command = state.last_command
                state.last_command = result.command
                state.last_change_time = now

                # Track on/off transitions
                if result.command > 0 and old_command == 0:
                    state.last_on_time = now
                    state.daily_events += 1
                elif result.command == 0 and old_command > 0:
                    state.last_off_time = now

            # Handle special case: lights_only forces command to 0
            if result.action == DispatchAction.HOLD_LIGHTS_ONLY:
                if state.last_command != 0.0:
                    state.last_command = 0.0
                    state.last_change_time = now
                    state.last_off_time = now
                    result = DispatchResult(
                        action=DispatchAction.DISPATCH,
                        command=0.0,
                        reason=f"{actuator_name}: forced off (lights off)",
                    )

            decisions.append((actuator_name, result))
            self._log_decision(actuator_name, setpoint, result)

        return decisions

    def _extract_setpoints(self, solution):
        """Extract actuator setpoints from an MPC solution dict.

        Uses registry dispatch specs to map solution fields to actuator names.
        """
        mapping = {}
        for spec in self.registry.all_dispatch_specs():
            value = float(solution.get(spec.solution_field, spec.default_value))
            mapping[spec.actuator_name] = value
        return mapping

    def read_solution(self):
        """Read latest MPC solution, check staleness.

        Returns:
            (solution, is_stale) or (None, True) if no solution.
        """
        solution = self.solution_store.read()

        if solution is None:
            return None, True

        # Check staleness
        solve_id = solution.get('solve_id', 0)
        next_solve_s = float(solution.get('next_solve_s', 60))
        stale_threshold = next_solve_s * self.stale_threshold_multiplier

        # In simulation, use sim_time; in production, use wall clock
        solution_time = solution.get('sim_time', 0)

        is_new = solve_id != self.last_solution_id
        if is_new:
            self.last_solution_id = solve_id
            self.last_solution_time = time.time()

        age_s = time.time() - self.last_solution_time
        is_stale = age_s > stale_threshold

        return solution, is_stale

    async def run(self, poll_interval=0.5):
        """Run the gateway loop (production async mode).

        Args:
            poll_interval: Seconds between solution reads (default 0.5 = 2 Hz).
        """
        self.running = True

        while self.running:
            solution, is_stale = self.read_solution()

            if solution is None or is_stale:
                if is_stale and solution is not None:
                    logger.warning(
                        f"[Gateway] Solution stale "
                        f"(id={self.last_solution_id})")
                await asyncio.sleep(1.0)
                continue

            self.evaluate_solution(solution)

            # In production, dispatch to hardware here
            if self.redis:
                self._publish_gateway_status()

            await asyncio.sleep(poll_interval)

    def evaluate_sync(self):
        """Single synchronous evaluation cycle.

        Used in simulation mode where the solver process calls this
        after each solve to get dispatch decisions.

        Returns:
            List of (actuator_name, DispatchResult) or empty list.
        """
        solution, is_stale = self.read_solution()
        if solution is None or is_stale:
            return []

        sim_time = float(solution.get('sim_time', 0))
        return self.evaluate_solution(solution, current_time_s=sim_time)

    def _publish_gateway_status(self):
        """Publish gateway status to Redis."""
        if not self.redis:
            return

        key = f'gateway:{self.room_id}:status'
        status = {
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'last_solution_id': str(self.last_solution_id or 0),
            'solution_age_s': str(
                round(time.time() - self.last_solution_time, 1)),
            'stale': str(
                (time.time() - self.last_solution_time) >
                self.stale_threshold_multiplier * 60),
        }

        # Per-actuator status
        for name, state in self.actuator_state.items():
            prefix = f'actuator:{name}'
            status[f'{prefix}:last_command'] = str(
                round(state.last_command, 3))
            status[f'{prefix}:last_change'] = (
                datetime.fromtimestamp(
                    state.last_change_time, tz=timezone.utc
                ).isoformat()
                if state.last_change_time > 0
                else 'never'
            )

        self.redis.hset(key, mapping=status)

    def _log_decision(self, actuator_name, setpoint, result):
        """Log a dispatch decision for observability."""
        entry = {
            'time': time.time(),
            'actuator': actuator_name,
            'setpoint': setpoint,
            'action': result.action.value,
            'command': result.command,
            'reason': result.reason,
        }

        self._dispatch_log.append(entry)
        if len(self._dispatch_log) > self._max_log_entries:
            self._dispatch_log = self._dispatch_log[-self._max_log_entries // 2:]

        if result.should_dispatch:
            logger.debug(f"[Gateway] DISPATCH {result.reason}")
        else:
            logger.debug(f"[Gateway] HOLD    {result.reason}")

    def get_dispatch_log(self, n=50):
        """Return last N dispatch decisions."""
        return self._dispatch_log[-n:]

    def stop(self):
        self.running = False
