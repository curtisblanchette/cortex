"""Component builders for CLI commands.

Each builder constructs exactly one component. Commands compose only
what they need — the solver never imports FastAPI; the API never imports OSQP.
"""

import os
import yaml
import redis


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def load_configs(config_dir='config'):
    """Load all YAML configs. Returns a dict of config dicts."""
    return {
        'room': load_yaml(os.path.join(config_dir, 'room.yaml')),
        'equipment': load_yaml(os.path.join(config_dir, 'equipment.yaml')),
        'phase': load_yaml(os.path.join(config_dir, 'phases', 'mid_flower.yaml')),
        'cultivar': load_yaml(os.path.join(config_dir, 'cultivar', 'default.yaml')),
    }


def make_redis(host='localhost', port=6379, db=0, decode_responses=True):
    """Create a Redis client and verify connectivity (fail-fast)."""
    r = redis.Redis(host=host, port=port, db=db,
                    decode_responses=decode_responses,
                    socket_connect_timeout=5)
    r.ping()
    return r


def build_shared_params():
    """Create initial CalibrationParams + SharedParams."""
    from calibration.params import CalibrationParams, SharedParams
    return SharedParams(CalibrationParams())


def build_dynamics(configs, shared_params):
    """Build GrowRoomDynamics from configs."""
    from core.dynamics import GrowRoomDynamics
    return GrowRoomDynamics(configs['room'], configs['equipment'], shared_params)


def build_ekf(dynamics):
    """Build and initialize the EKF."""
    from estimation.ekf import StateEstimator
    from core.dynamics import GrowRoomDynamics
    ekf = StateEstimator(dynamics)
    ekf.initialize(GrowRoomDynamics.default_initial_state())
    return ekf


def build_mpc(dynamics, shared_params, configs):
    """Build the MPC solver."""
    from mpc.solver import MPCSolver
    from mpc.constraints import ConstraintBuilder
    constraints = ConstraintBuilder(configs['equipment'])
    return MPCSolver(dynamics, shared_params, Np=60, Nc=15,
                     constraints=constraints)


def build_reference(configs):
    """Build ReferenceGenerator."""
    from mpc.reference import ReferenceGenerator
    return ReferenceGenerator(configs['phase'], configs['cultivar'])


def build_light_schedule(configs):
    """Build LightSchedule from room config."""
    from mpc.reference import LightSchedule
    room = configs['room'].get('room', configs['room'])
    ls = room.get('light_schedule', {'on_hour': 6.0, 'off_hour': 18.0})
    return LightSchedule(ls['on_hour'], ls['off_hour'])


def build_forecast(configs):
    """Build DisturbanceForecast."""
    from mpc.forecast import DisturbanceForecast
    return DisturbanceForecast(configs['room'])


def build_compliance(configs):
    """Build ComplianceTracker."""
    from control.compliance import ComplianceTracker
    return ComplianceTracker(configs['phase'])


def build_failsafe():
    """Build FailSafeManager."""
    from control.failsafe import FailSafeManager
    return FailSafeManager()


def build_simulation_io(configs):
    """Build SimulatedRoom + SimulatedIO. Returns (io, sim_room)."""
    from simulation.room import SimulatedRoom
    from control.io import SimulatedIO
    sim_room = SimulatedRoom(configs['room'], configs['equipment'],
                             noise_seed=42)
    io = SimulatedIO(sim_room)
    return io, sim_room


def build_real_io(redis_client, room_id='room1'):
    """Build RealIO for production."""
    from control.io import RealIO
    return RealIO(redis_client, room_id=room_id)
