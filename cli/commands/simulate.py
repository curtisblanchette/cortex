"""cortex simulate — all-in-process simulation (existing behavior)."""

import time

from cli.bootstrap import (
    load_configs, build_shared_params, build_dynamics, build_ekf,
    build_mpc, build_reference, build_light_schedule, build_forecast,
    build_compliance, build_failsafe, build_simulation_io,
)


def register(subparsers):
    p = subparsers.add_parser('simulate', help='Run in-process simulation')
    p.add_argument('--fast', action='store_true',
                   help='Fast-forward (no sleep)')
    p.add_argument('--legacy', action='store_true',
                   help='Use legacy coupled control loop')
    p.add_argument('--duration', type=int, default=86400,
                   help='Simulation duration in seconds')
    p.add_argument('--config', type=str, default='config',
                   help='Config directory')
    p.set_defaults(func=run)


def run(args):
    if args.legacy:
        _run_legacy(args)
    else:
        _run_decoupled(args)


def _run_legacy(args):
    """Run the legacy coupled control loop."""
    from control.loop import ControlLoop

    print(f"[Cortex] Starting LEGACY simulation "
          f"({args.duration}s, fast={args.fast})")

    configs = load_configs(args.config)
    shared_params = build_shared_params()
    dynamics = build_dynamics(configs, shared_params)
    io, sim_room = build_simulation_io(configs)
    ekf = build_ekf(dynamics)
    mpc = build_mpc(dynamics, shared_params, configs)
    reference = build_reference(configs)
    light_schedule = build_light_schedule(configs)
    forecast = build_forecast(configs)
    compliance = build_compliance(configs)
    failsafe = build_failsafe()

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

    report_interval = max(1, args.duration // 20)

    def telemetry_cb(x_hat, u, derived, info):
        tick = info['tick']
        if tick % report_interval == 0:
            hours = info['time_s'] / 3600.0
            print(f"  [{hours:6.1f}h] T={x_hat[0]:.1f}\u00b0C  "
                  f"RH={derived['RH']:.0f}%  VPD={derived['VPD']:.2f}kPa  "
                  f"CO2={x_hat[2]:.0f}ppm  \u03b8={x_hat[4]:.3f}  "
                  f"solve={info['solve_ms']:.1f}ms  {info['failsafe_level']}")

    loop.set_telemetry_callback(telemetry_cb)

    print("[Cortex] Running control loop...")
    loop.run_sync(args.duration, realtime=not args.fast)

    _print_compliance(compliance)


def _run_decoupled(args):
    """Run the decoupled solver + gateway simulation."""
    from control.solver_process import SolverProcess, SolutionStore
    from control.gateway import ActuatorGateway

    print(f"[Cortex] Starting DECOUPLED simulation "
          f"({args.duration}s, fast={args.fast})")

    configs = load_configs(args.config)
    shared_params = build_shared_params()
    dynamics = build_dynamics(configs, shared_params)
    io, sim_room = build_simulation_io(configs)
    ekf = build_ekf(dynamics)
    mpc = build_mpc(dynamics, shared_params, configs)
    reference = build_reference(configs)
    light_schedule = build_light_schedule(configs)
    forecast = build_forecast(configs)
    compliance = build_compliance(configs)
    failsafe = build_failsafe()

    solution_store = SolutionStore()

    solver = SolverProcess(
        io=io,
        dynamics=dynamics,
        ekf=ekf,
        mpc=mpc,
        reference=reference,
        forecast=forecast,
        failsafe=failsafe,
        shared_params=shared_params,
        light_schedule=light_schedule,
        solution_store=solution_store,
        compliance=compliance,
    )

    dispatch_config = configs['equipment'].get('dispatch', {})
    gateway = ActuatorGateway(
        solution_store=solution_store,
        dispatch_config=dispatch_config,
        light_schedule=light_schedule,
    )

    report_interval = max(1, args.duration // 20)
    dispatch_summary = {'total': 0, 'dispatched': 0, 'held': 0}

    def telemetry_cb(x_hat, u, derived, info):
        tick = info['tick']
        if tick % report_interval == 0:
            hours = info['time_s'] / 3600.0
            pct = (dispatch_summary['dispatched'] /
                   max(1, dispatch_summary['total'])) * 100
            print(f"  [{hours:6.1f}h] T={x_hat[0]:.1f}\u00b0C  "
                  f"RH={derived['RH']:.0f}%  VPD={derived['VPD']:.2f}kPa  "
                  f"CO2={x_hat[2]:.0f}ppm  \u03b8={x_hat[4]:.3f}  "
                  f"solve={info['solve_ms']:.1f}ms  "
                  f"dispatch={pct:.0f}%  {info['failsafe_level']}")

    solver.set_telemetry_callback(telemetry_cb)

    print("[Cortex] Running solver + gateway...")

    solver.running = True
    n_steps = int(args.duration)

    for step in range(n_steps):
        if not solver.running:
            break

        try:
            solver.solve_once()

            decisions = gateway.evaluate_sync()
            for name, result in decisions:
                dispatch_summary['total'] += 1
                if result.should_dispatch:
                    dispatch_summary['dispatched'] += 1
                else:
                    dispatch_summary['held'] += 1

            solver.io.write_actuators(solver.last_u)

        except Exception as e:
            print(f"[Cortex] tick {step} error: {e}")
            solver.io.write_actuators(solver.last_u)

        if not args.fast:
            time.sleep(max(0, 1.0 - 0.025))

    _print_compliance(compliance)
    _print_dispatch_summary(dispatch_summary, gateway)


def _print_compliance(compliance):
    print("\n[Cortex] Simulation complete. Compliance report:")
    report = compliance.report()
    for key, data in report.items():
        print(f"  {key}: {data['compliance_pct']:.1f}% compliant "
              f"(mean={data['mean']:.3f}, std={data['std']:.3f})")


def _print_dispatch_summary(summary, gateway):
    total = summary['total']
    dispatched = summary['dispatched']
    held = summary['held']

    print(f"\n[Cortex] Dispatch summary:")
    print(f"  Total evaluations: {total}")
    print(f"  Dispatched:        {dispatched} "
          f"({dispatched/max(1,total)*100:.1f}%)")
    print(f"  Held (filtered):   {held} "
          f"({held/max(1,total)*100:.1f}%)")

    print(f"\n  Per-actuator final state:")
    for name, state in gateway.actuator_state.items():
        print(f"    {name:12s}: command={state.last_command:.2f}  "
              f"events={state.daily_events}")
