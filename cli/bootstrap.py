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


def load_configs(config_dir='config', phase_override=None):
    """Load all YAML configs. Returns a dict of config dicts."""
    room = load_yaml(os.path.join(config_dir, 'room.yaml'))
    phase_name = phase_override or room.get('room', room).get('phase', 'mid_flower')
    substrate_path = os.path.join(config_dir, 'substrate.yaml')
    return {
        'room': room,
        'equipment': load_yaml(os.path.join(config_dir, 'equipment.yaml')),
        'phase': load_yaml(os.path.join(config_dir, 'phases', f'{phase_name}.yaml')),
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
    LAI_ref = configs['phase'].get('LAI', 3.0)
    return GrowRoomDynamics(configs['room'], configs['equipment'],
                            shared_params, registry=registry,
                            substrate=substrate, LAI_ref=LAI_ref)


def build_ekf(dynamics, substrate=None, theta_init=None):
    """Build and initialize the EKF."""
    from estimation.ekf import StateEstimator
    from core.dynamics import GrowRoomDynamics
    ekf = StateEstimator(dynamics)
    x0 = GrowRoomDynamics.default_initial_state()
    if theta_init is not None:
        x0[4] = theta_init
    elif substrate is not None:
        x0[4] = substrate.theta_fc * 0.92
    ekf.initialize(x0)
    return ekf


def build_mpc(dynamics, shared_params, registry, configs):
    """Build the MPC solver."""
    from mpc.solver import MPCSolver
    from mpc.constraints import ConstraintBuilder
    room_cfg = configs['room'].get('room', configs['room'])
    clamp_config = room_cfg.get('clamp_hysteresis')
    constraints = ConstraintBuilder(registry, clamp_config=clamp_config)
    return MPCSolver(dynamics, shared_params, registry, Np=60, Nc=30,
                     constraints=constraints, step_s=30.0)


def build_reference(configs, registry=None, substrate=None):
    """Build ReferenceGenerator."""
    from mpc.reference import ReferenceGenerator
    return ReferenceGenerator(configs['phase'], configs['cultivar'],
                              registry=registry, substrate=substrate)


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


def build_advisory_engine(configs, reference, dynamics, room_id='room1'):
    """Build AdvisoryEngine from configs."""
    from advisory.engine import AdvisoryEngine
    return AdvisoryEngine(configs['phase'], reference, dynamics,
                          room_id=room_id)


def build_simulation_io(configs, registry, substrate=None, theta_init=None):
    """Build SimulatedRoom + SimulatedIO. Returns (io, sim_room)."""
    from simulation.room import SimulatedRoom
    from control.io import SimulatedIO
    from core.dynamics import GrowRoomDynamics
    LAI_ref = configs['phase'].get('LAI', 3.0)
    initial_state = GrowRoomDynamics.default_initial_state()
    if theta_init is not None:
        initial_state[4] = theta_init
    elif substrate is not None:
        initial_state[4] = substrate.theta_fc * 0.92
    sim_room = SimulatedRoom(configs['room'], configs['equipment'],
                              registry, noise_seed=42, substrate=substrate,
                              LAI_ref=LAI_ref, initial_state=initial_state)
    io = SimulatedIO(sim_room)
    return io, sim_room


def build_real_io(redis_client, registry, room_id='room1'):
    """Build RealIO for production."""
    from control.io import RealIO
    return RealIO(redis_client, registry, room_id=room_id)


def build_co2_scheduler(configs, registry, light_schedule, reference):
    """Build CO2Scheduler from configs.

    Returns CO2Scheduler or None if u_co2 is not in the active profile.
    """
    if 'u_co2' not in registry.channel_names:
        return None

    from control.co2_scheduler import CO2Config, CO2Scheduler

    equip = configs['equipment']
    actuators = equip.get('actuators', {})
    co2_equip = actuators.get('co2', {})

    config = CO2Config(
        deadband_ppm=co2_equip.get('deadband_ppm', 75.0),
        overshoot_ppm=co2_equip.get('overshoot_ppm', 75.0),
        min_off_s=co2_equip.get('min_off_s', 30.0),
        expected_flow_ppm_s=co2_equip.get('flow_rate_ppm_s', 10.0),
    )

    return CO2Scheduler(
        config=config,
        registry=registry,
        light_schedule=light_schedule,
        reference=reference,
    )


def build_irrigation_scheduler(configs, substrate, light_schedule):
    """Build IrrigationScheduler from configs.

    Returns IrrigationScheduler or None if:
      - the phase config disables irrigation (enabled: false)
      - no irrigation_schedule block in phase config
    """
    phase_cfg = configs['phase']
    irr_cfg = phase_cfg.get('irrigation_schedule')
    if irr_cfg is None:
        return None

    if not irr_cfg.get('enabled', False):
        return None

    from control.irrigation_scheduler import IrrigationConfig, IrrigationScheduler

    # Resolve the day theta target so the trigger is target-relative
    day = phase_cfg.get('day', {})
    if 'theta_target_frac' in day:
        awc = substrate.theta_fc - substrate.theta_wp
        day_theta = substrate.theta_wp + float(day['theta_target_frac']) * awc
    else:
        day_theta = day.get('theta_target', 0.35)

    config = IrrigationConfig(
        enabled=irr_cfg['enabled'],
        first_shot_delay_s=irr_cfg.get('first_shot_delay_min', 30) * 60.0,
        last_shot_cutoff_s=irr_cfg.get('last_shot_cutoff_min', 180) * 60.0,
        min_interval_s=irr_cfg.get('min_interval_min', substrate.min_interval_min) * 60.0,
        theta_trigger_fraction=irr_cfg.get('theta_trigger_fraction', 0.15),
        shot_duration_s=irr_cfg.get('shot_duration_s', substrate.shot_duration_s),
        max_daily_shots=irr_cfg.get('max_daily_shots', substrate.max_daily_shots),
        drain_settle_s=irr_cfg.get('drain_settle_min', substrate.drain_settle_min) * 60.0,
    )

    room = configs['room'].get('room', configs['room'])
    canopy_area_m2 = room.get('canopy_area_m2', 56.0)
    N_plants = room.get('N_plants', 100)

    # Per-plant flow rate from substrate, scaled to total manifold flow
    flow_rate_ml_s = substrate.irr_flow_ml_s * N_plants

    return IrrigationScheduler(
        config=config,
        substrate=substrate,
        light_schedule=light_schedule,
        flow_rate_ml_s=flow_rate_ml_s,
        N_plants=N_plants,
        container_volume_L=substrate.container_volume_L,
        canopy_area_m2=canopy_area_m2,
        theta_target=day_theta,
    )
