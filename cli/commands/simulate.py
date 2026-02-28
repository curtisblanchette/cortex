"""cortex simulate — all-in-process simulation (existing behavior)."""

import time

from cli.bootstrap import (
    load_configs, build_substrate, build_registry, build_shared_params,
    build_dynamics, build_ekf, build_mpc, build_reference,
    build_light_schedule, build_forecast, build_compliance, build_failsafe,
    build_simulation_io,
)


def register(subparsers):
    p = subparsers.add_parser('simulate', help='Run in-process simulation')
    p.add_argument('--fast', action='store_true',
                   help='Fast-forward (no sleep)')
    p.add_argument('--duration', type=int, default=86400,
                   help='Simulation duration in seconds')
    p.add_argument('--config', type=str, default='config',
                   help='Config directory')
    p.add_argument('--profile', type=str, default=None,
                   help='Equipment profile override (e.g. ac_dehum, ac_minimal)')
    p.add_argument('--substrate', type=str, default=None,
                   help='Substrate profile override (e.g. coco_coir, rockwool, perlite)')
    p.add_argument('--chart', action='store_true',
                   help='Generate visualization chart on completion')
    p.add_argument('--chart-output', type=str, default='output/simulation.png',
                   help='Output path for chart (default: output/simulation.png)')
    p.set_defaults(func=run)


def run(args):
    _run_decoupled(args)


def _maybe_recorder(args, registry=None, light_power_W=0.0):
    """Create a TelemetryRecorder if --chart was requested."""
    if not args.chart:
        return None
    from cli.visualization import TelemetryRecorder
    return TelemetryRecorder(registry=registry, light_power_W=light_power_W)


def _maybe_chart(args, recorder, compliance):
    """Generate the chart if --chart was requested."""
    if not args.chart or recorder is None:
        return
    from cli.visualization import plot_simulation
    print("\n[Cortex] Generating chart...")
    data = recorder.to_arrays()
    report = compliance.report()
    plot_simulation(data, report, args.duration, args.chart_output)


def _run_decoupled(args):
    """Run the decoupled solver + gateway simulation."""
    from control.solver_process import SolverProcess, SolutionStore
    from control.gateway import ActuatorGateway

    profile = getattr(args, 'profile', None)
    substrate_name = getattr(args, 'substrate', None)
    print(f"[Cortex] Starting DECOUPLED simulation "
          f"({args.duration}s, fast={args.fast}"
          f"{f', profile={profile}' if profile else ''}"
          f"{f', substrate={substrate_name}' if substrate_name else ''})")

    configs = load_configs(args.config)
    substrate = build_substrate(configs, substrate_override=substrate_name)
    registry = build_registry(configs, profile_override=profile, substrate=substrate)
    shared_params = build_shared_params(substrate=substrate)
    dynamics = build_dynamics(configs, shared_params, registry, substrate=substrate)
    io, sim_room = build_simulation_io(configs, registry, substrate=substrate)
    ekf = build_ekf(dynamics)
    mpc = build_mpc(dynamics, shared_params, registry, configs)
    reference = build_reference(configs, registry=registry)
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
        registry=registry,
        solution_store=solution_store,
        compliance=compliance,
    )

    dispatch_config = configs['equipment'].get('dispatch', {})
    gateway = ActuatorGateway(
        solution_store=solution_store,
        dispatch_config=dispatch_config,
        registry=registry,
        light_schedule=light_schedule,
    )

    room_config = configs.get('room', {})
    disturbances_config = room_config.get('disturbances', room_config)
    light_power_W = disturbances_config.get('light_power_W', 18700.0)
    recorder = _maybe_recorder(args, registry=registry, light_power_W=light_power_W)
    report_interval = max(1, args.duration // 20)
    dispatch_summary = {'total': 0, 'dispatched': 0, 'held': 0}

    def telemetry_cb(x_hat, u, derived, info):
        if recorder:
            ref = reference.generate(
                info['time_s'], light_schedule, horizon_steps=1, step_s=60.0)
            ref_targets = {
                'VPD': ref[0, 0], 'T': ref[0, 1],
                'CO2': ref[0, 2], 'theta': ref[0, 4],
                'RH': ref[0, 5],
            }
            recorder.record(x_hat, u, derived, info, sim_room, ref_targets)

        tick = info['tick']
        if tick % report_interval == 0:
            hours = info['time_s'] / 3600.0
            pct = (dispatch_summary['dispatched'] /
                   max(1, dispatch_summary['total'])) * 100
            print(f"  [{hours:6.1f}h] T={x_hat[0]:.1f}C  "
                  f"RH={derived['RH']:.0f}%  VPD={derived['VPD']:.2f}kPa  "
                  f"CO2={x_hat[2]:.0f}ppm  theta={x_hat[4]:.3f}  "
                  f"solve={info['solve_ms']:.1f}ms  "
                  f"dispatch={pct:.0f}%  {info['failsafe_level']}")

    solver.set_telemetry_callback(telemetry_cb)

    # Match MPC solve cadence to prediction step size.
    # Without this, the MPC re-solves every 1s tick but the du constraints
    # are sized for 60s steps, causing 60x faster rate-of-change than intended.
    solver.set_mpc_interval(int(mpc.step_s))

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

            if recorder and decisions:
                recorder.record_dispatch(decisions)

            solver.io.write_actuators(solver.last_u)

        except Exception as e:
            print(f"[Cortex] tick {step} error: {e}")
            solver.io.write_actuators(solver.last_u)

        if not args.fast:
            time.sleep(max(0, 1.0 - 0.025))

    _print_compliance(compliance)
    _print_dispatch_summary(dispatch_summary, gateway)
    _maybe_chart(args, recorder, compliance)


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
