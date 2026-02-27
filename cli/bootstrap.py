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
    substrate_path = os.path.join(config_dir, 'substrate.yaml')
    return {
        'room': load_yaml(os.path.join(config_dir, 'room.yaml')),
        'equipment': load_yaml(os.path.join(config_dir, 'equipment.yaml')),
        'phase': load_yaml(os.path.join(config_dir, 'phases', 'mid_flower.yaml')),
        'cultivar': load_yaml(os.path.join(config_dir, 'cultivar', 'default.yaml')),
        'substrate': load_yaml(substrate_path) if os.path.isfile(substrate_path) else {},
    }


def make_redis(host='localhost', port=6379, db=0, decode_responses=True):
    """Create a Redis client and verify connectivity (fail-fast)."""
    r = redis.Redis(host=host, port=port, db=db,
                    decode_responses=decode_responses,
                    socket_connect_timeout=5)
    r.ping()
    return r


def build_substrate(configs, substrate_override=None):
    """Build SubstrateProfile from configs."""
    from core.substrate import load_substrate_profile
    return load_substrate_profile(configs.get('substrate'),
                                  profile_override=substrate_override)


def build_registry(configs, profile_override=None, substrate=None):
    """Build ActuatorRegistry from configs."""
    from core.actuators.factory import build_registry as _build
    return _build(configs['equipment'], configs['room'],
                  profile_override=profile_override, substrate=substrate)


def build_shared_params(substrate=None):
    """Create initial CalibrationParams + SharedParams."""
    from calibration.params import CalibrationParams, SharedParams
    if substrate is not None:
        initial = CalibrationParams(K_drain=substrate.K_drain_default)
    else:
        initial = CalibrationParams()
    return SharedParams(initial)


def build_dynamics(configs, shared_params, registry, substrate=None):
    """Build GrowRoomDynamics from configs."""
    from core.dynamics import GrowRoomDynamics
    return GrowRoomDynamics(configs['room'], configs['equipment'],
                            shared_params, registry=registry,
                            substrate=substrate)


def build_ekf(dynamics):
    """Build and initialize the EKF."""
    from estimation.ekf import StateEstimator
    from core.dynamics import GrowRoomDynamics
    ekf = StateEstimator(dynamics)
    ekf.initialize(GrowRoomDynamics.default_initial_state())
    return ekf


def build_mpc(dynamics, shared_params, registry, configs):
    """Build the MPC solver."""
    from mpc.solver import MPCSolver
    from mpc.constraints import ConstraintBuilder
    constraints = ConstraintBuilder(registry)
    return MPCSolver(dynamics, shared_params, registry, Np=60, Nc=30,
                     constraints=constraints, step_s=30.0)


def build_reference(configs, registry=None):
    """Build ReferenceGenerator."""
    from mpc.reference import ReferenceGenerator
    return ReferenceGenerator(configs['phase'], configs['cultivar'],
                              registry=registry)


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


def build_simulation_io(configs, registry, substrate=None):
    """Build SimulatedRoom + SimulatedIO. Returns (io, sim_room)."""
    from simulation.room import SimulatedRoom
    from control.io import SimulatedIO
    sim_room = SimulatedRoom(configs['room'], configs['equipment'],
                              registry, noise_seed=42, substrate=substrate)
    io = SimulatedIO(sim_room)
    return io, sim_room


def build_real_io(redis_client, registry, room_id='room1'):
    """Build RealIO for production."""
    from control.io import RealIO
    return RealIO(redis_client, registry, room_id=room_id)
