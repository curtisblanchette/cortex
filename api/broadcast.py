"""WebSocket broadcaster — bridges sync telemetry callbacks to async WS sends."""

import asyncio
import json
import logging

logger = logging.getLogger(__name__)


class WebSocketBroadcaster:
    """Bridges SolverProcess telemetry (sync) to WebSocket broadcasts (async).

    Usage:
        broadcaster = WebSocketBroadcaster(app, dynamics, ...)
        solver.set_telemetry_callback(broadcaster.on_telemetry)
    """

    def __init__(self, app, dynamics, reference=None, compliance=None,
                 advisory_engine=None, registry=None,
                 mpc_step_s=60.0, mpc_interval=60, light_schedule=None):
        self._app = app
        self._dynamics = dynamics
        self._reference = reference
        self._compliance = compliance
        self._advisory_engine = advisory_engine
        self._registry = registry
        self._mpc_step_s = mpc_step_s
        self._mpc_interval = mpc_interval
        self._light_schedule = light_schedule
        self._loop = None
        self._tick_count = 0
        self._state_decimation = 5  # send state every 5 ticks
        self._last_u = None

        # Cached trajectory from last MPC solve
        self._last_trajectory = None
        self._last_solve_time_s = None

    def set_event_loop(self, loop):
        self._loop = loop

    def on_telemetry(self, x_hat, u, derived, info):
        """Telemetry callback — called synchronously from SolverProcess."""
        if self._loop is None:
            return

        self._last_u = u
        self._last_effort = derived.get('effort', {})
        self._tick_count += 1
        time_s = info.get('time_s', 0)

        # State update (decimated)
        if self._tick_count % self._state_decimation == 0:
            msg = self._build_state_msg(x_hat, derived, info)
            self._schedule(msg)

        # Trajectory + advisories (on MPC solve ticks)
        if self._tick_count % self._mpc_interval == 0:
            traj_msg = self._build_trajectory_msg(time_s)
            if traj_msg:
                self._schedule(traj_msg)

            adv_msg = self._build_advisories_msg()
            self._schedule(adv_msg)

    def notify_solve(self, trajectory, sim_time_s, ref_targets, tolerances):
        """Called from the solve path to cache trajectory data."""
        self._last_trajectory = trajectory
        self._last_solve_time_s = sim_time_s
        self._last_ref_targets = ref_targets
        self._last_tolerances = tolerances

    def _schedule(self, message):
        if self._loop is None:
            return
        broadcast = getattr(self._app.state, 'broadcast', None)
        if broadcast is None:
            return
        try:
            asyncio.run_coroutine_threadsafe(broadcast(message), self._loop)
        except RuntimeError:
            pass  # loop closed

    def _build_state_msg(self, x_hat, derived, info):
        compliance_data = {}
        if self._compliance:
            report = self._compliance.report()
            for ch, data in report.items():
                compliance_data[ch] = {'pct': round(data.get('compliance_pct', 0), 1)}

        targets = {}
        if hasattr(self, '_last_ref_targets') and self._last_ref_targets:
            targets = {k: round(float(v), 3) for k, v in self._last_ref_targets.items()}

        # Equipment values from last control vector
        equipment = []
        if self._registry and self._last_u is not None:
            for i in range(self._registry.n_u):
                equipment.append({'value': round(float(self._last_u[i]), 2)})

        time_s = info.get('time_s', 0)
        lights_on = False
        if self._light_schedule:
            lights_on = self._light_schedule.is_on(time_s)

        return {
            'type': 'cortex_state',
            'data': {
                'sim_time_s': round(time_s, 1),
                'lights_on': lights_on,
                'state': {
                    'VPD': round(float(derived.get('VPD', 0)), 3),
                    'T_air': round(float(x_hat[0]), 2),
                    'T_leaf': round(float(x_hat[3]), 2),
                    'RH': round(float(derived.get('RH', 0)), 1),
                    'CO2': round(float(x_hat[2]), 1),
                    'theta': round(float(x_hat[4]), 4),
                    'w_a': round(float(x_hat[1]), 2),
                },
                'targets': targets,
                'compliance': compliance_data,
                'solve_ms': round(info.get('solve_ms', 0), 1),
                'failsafe_level': info.get('failsafe_level', 'unknown'),
                'equipment': equipment,
                'effort': getattr(self, '_last_effort', {}),
            },
        }

    def _build_trajectory_msg(self, sim_time_s):
        trajectory = self._last_trajectory
        if trajectory is None:
            return None

        traj_VPD = []
        traj_T = []
        traj_T_leaf = []
        traj_RH = []
        traj_CO2 = []
        traj_w_a = []
        for i in range(len(trajectory)):
            d = self._dynamics.derived_outputs(trajectory[i])
            traj_VPD.append(round(d['VPD'], 3))
            traj_RH.append(round(d['RH'], 1))
            traj_T.append(round(float(trajectory[i, 0]), 2))
            traj_T_leaf.append(round(float(trajectory[i, 3]), 2))
            traj_CO2.append(round(float(trajectory[i, 2]), 1))
            traj_w_a.append(round(float(trajectory[i, 1]), 2))

        targets = {}
        tolerances = {}
        if hasattr(self, '_last_ref_targets') and self._last_ref_targets:
            targets = {k: round(float(v), 3) for k, v in self._last_ref_targets.items()}
        if hasattr(self, '_last_tolerances') and self._last_tolerances:
            tolerances = {k: round(float(v), 3) for k, v in self._last_tolerances.items()}

        return {
            'type': 'cortex_trajectory',
            'data': {
                'sim_time_s': round(self._last_solve_time_s or sim_time_s, 1),
                'step_s': self._mpc_step_s,
                'horizon_steps': len(trajectory),
                'trajectory_VPD': traj_VPD,
                'trajectory_T': traj_T,
                'trajectory_T_leaf': traj_T_leaf,
                'trajectory_RH': traj_RH,
                'trajectory_CO2': traj_CO2,
                'trajectory_w_a': traj_w_a,
                'targets': targets,
                'tolerances': tolerances,
            },
        }

    def notify_equipment_manifest(self):
        """Broadcast the equipment manifest so the UI can render channel cards."""
        if not self._registry:
            return
        idx_to_field = self._registry.index_to_solution_field()
        manifest = []
        for i, ch in enumerate(self._registry._channels):
            field = idx_to_field.get(i, ch.name)
            kind = 'setpoint' if ch.name.endswith('_sp') else 'percentage'
            manifest.append({
                'name': ch.name,
                'field': field,
                'unit': ch.unit,
                'bound_low': ch.bound_low,
                'bound_high': ch.bound_high,
                'default': ch.default,
                'kind': kind,
            })
        self._schedule({
            'type': 'cortex_equipment_manifest',
            'data': {'channels': manifest},
        })

    def notify_sim_progress(self, elapsed_s, duration_s, pct,
                            phase=None, profile=None):
        """Broadcast simulation progress to WS clients."""
        self._schedule({
            'type': 'cortex_sim_progress',
            'data': {
                'elapsed_s': round(elapsed_s, 1),
                'duration_s': duration_s,
                'pct': round(pct, 1),
                'phase': phase,
                'profile': profile,
            },
        })

    def notify_sim_complete(self, compliance_report, duration_s):
        """Broadcast simulation completion to WS clients."""
        self._schedule({
            'type': 'cortex_sim_complete',
            'data': {
                'status': 'completed',
                'duration_s': duration_s,
                'compliance': compliance_report,
            },
        })

    def _build_advisories_msg(self):
        active = []
        if self._advisory_engine:
            for a in self._advisory_engine.active_advisories():
                active.append(a.to_dict())

        return {
            'type': 'cortex_advisories',
            'data': {
                'active': active,
            },
        }
