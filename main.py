"""Cortex MPC — main entry point.

Usage:
    python main.py                    # Production mode (requires hardware)
    python main.py --simulate         # Simulation mode (no hardware)
    python main.py --simulate --fast  # Fast-forward simulation
"""

import argparse
import asyncio
import sys
import os
import yaml

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.dynamics import GrowRoomDynamics
from calibration.params import CalibrationParams, SharedParams
from estimation.ekf import StateEstimator
from mpc.solver import MPCSolver
from mpc.reference import ReferenceGenerator, LightSchedule
from mpc.forecast import DisturbanceForecast
from mpc.constraints import ConstraintBuilder
from control.loop import ControlLoop
from control.failsafe import FailSafeManager
from control.compliance import ComplianceTracker


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def build_simulation(config_dir='config', duration_s=86400, fast=True):
    """Build and run a simulation.

    Returns the control loop and simulation room for inspection.
    """
    from simulation.room import SimulatedRoom
    from control.io import SimulatedIO

    # Load configs
    room_config = load_yaml(os.path.join(config_dir, 'room.yaml'))
    equipment_config = load_yaml(os.path.join(config_dir, 'equipment.yaml'))
    phase_config = load_yaml(os.path.join(config_dir, 'phases', 'mid_flower.yaml'))
    cultivar_config = load_yaml(os.path.join(config_dir, 'cultivar', 'default.yaml'))

    # Initialize parameters
    initial_params = CalibrationParams()
    shared_params = SharedParams(initial_params)

    # Core physics model
    dynamics = GrowRoomDynamics(room_config, equipment_config, shared_params)

    # Simulation room (uses same dynamics, may have different params)
    sim_room = SimulatedRoom(room_config, equipment_config, noise_seed=42)
    io = SimulatedIO(sim_room)

    # State estimator
    ekf = StateEstimator(dynamics)
    ekf.initialize(GrowRoomDynamics.default_initial_state())

    # MPC solver
    constraints = ConstraintBuilder(equipment_config)
    mpc = MPCSolver(dynamics, shared_params, Np=60, Nc=15, constraints=constraints)

    # Reference + forecast
    reference = ReferenceGenerator(phase_config, cultivar_config)
    room = room_config.get('room', room_config)
    ls = room.get('light_schedule', {'on_hour': 6.0, 'off_hour': 18.0})
    light_schedule = LightSchedule(ls['on_hour'], ls['off_hour'])
    forecast = DisturbanceForecast(room_config)

    # Compliance + failsafe
    compliance = ComplianceTracker(phase_config)
    failsafe = FailSafeManager()

    # Control loop
    loop = ControlLoop(
        io=io,
        dynamics=dynamics,
        ekf=ekf,
        mpc=mpc,
        reference=reference,
        forecast=forecast,
        compliance=compliance,
        failsafe=failsafe,
        shared_params=shared_params,
        light_schedule=light_schedule,
    )

    return loop, sim_room, compliance


def main():
    parser = argparse.ArgumentParser(description='Cortex MPC Controller')
    parser.add_argument('--simulate', action='store_true', help='Run in simulation mode')
    parser.add_argument('--fast', action='store_true', help='Fast-forward simulation (no sleep)')
    parser.add_argument('--duration', type=int, default=86400, help='Simulation duration in seconds')
    parser.add_argument('--config', type=str, default='config', help='Config directory')
    parser.add_argument('--port', type=int, default=8000, help='API server port')
    args = parser.parse_args()

    if args.simulate:
        print(f"[Cortex] Starting simulation ({args.duration}s, fast={args.fast})")

        loop, sim_room, compliance = build_simulation(
            config_dir=args.config,
            duration_s=args.duration,
            fast=args.fast,
        )

        # Progress reporting
        report_interval = max(1, args.duration // 20)

        def telemetry_cb(x_hat, u, derived, info):
            tick = info['tick']
            if tick % report_interval == 0:
                hours = info['time_s'] / 3600.0
                print(f"  [{hours:6.1f}h] T={x_hat[0]:.1f}°C  "
                      f"RH={derived['RH']:.0f}%  VPD={derived['VPD']:.2f}kPa  "
                      f"CO2={x_hat[2]:.0f}ppm  θ={x_hat[4]:.3f}  "
                      f"solve={info['solve_ms']:.1f}ms  {info['failsafe_level']}")

        loop.set_telemetry_callback(telemetry_cb)

        print("[Cortex] Running control loop...")
        loop.run_sync(args.duration, realtime=not args.fast)

        print("\n[Cortex] Simulation complete. Compliance report:")
        report = compliance.report()
        for key, data in report.items():
            print(f"  {key}: {data['compliance_pct']:.1f}% compliant "
                  f"(mean={data['mean']:.3f}, std={data['std']:.3f})")

    else:
        print("[Cortex] Production mode not yet implemented.")
        print("  Use --simulate for simulation mode.")
        sys.exit(1)


if __name__ == '__main__':
    main()
