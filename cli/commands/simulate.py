"""cortex simulate — all-in-process simulation (existing behavior)."""

import time

from cli.bootstrap import (
    load_configs, build_substrate, build_registry, build_shared_params,
    build_dynamics, build_ekf, build_mpc, build_reference,
    build_light_schedule, build_forecast, build_compliance, build_failsafe,
    build_simulation_io, build_irrigation_scheduler, build_co2_scheduler,
    build_advisory_engine,
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
    p.add_argument('--phase', type=str, default=None,
                   help='Growth phase override (e.g. clone, veg, early_flower, late_flower, dry)')
    p.add_argument('--substrate', type=str, default=None,
                   help='Substrate profile override (e.g. coco_coir, rockwool, perlite)')
    p.add_argument('--chart', action='store_true',
                   help='Generate visualization chart on completion')
    p.add_argument('--chart-output', type=str, default='output/simulation.png',
                   help='Output path for chart (default: output/simulation.png)')
    p.add_argument('--dashboard', action='store_true',
                   help='Start API server for live VPD dashboard')
    p.add_argument('--api-port', type=int, default=8080,
                   help='API server port for dashboard (default: 8080)')
    p.add_argument('--wait', action='store_true',
                   help='Start API server idle — simulations triggered from UI')
    p.set_defaults(func=run)


def run(args):
    if getattr(args, 'wait', False) and getattr(args, 'dashboard', False):
        _run_wait_mode(args)
    else:
        _run_decoupled(args)


def _run_wait_mode(args):
    """Start the API server idle — simulations are triggered from the UI."""
    import uvicorn
    from api.server import create_api

    port = getattr(args, 'api_port', 8080)
    app = create_api()

    print(f"[Cortex] Dashboard API started on http://localhost:{port}")
    print(f"[Cortex] WebSocket at ws://localhost:{port}/ws/dashboard")
    print("[Cortex] Waiting for simulation trigger from UI...")

    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


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

    phase = getattr(args, 'phase', None)
    profile = getattr(args, 'profile', None)
    substrate_name = getattr(args, 'substrate', None)

    configs = load_configs(args.config, phase_override=phase)
    substrate = build_substrate(configs, substrate_override=substrate_name)

    resolved_phase = configs['phase'].get('phase', phase or 'mid_flower')
    print(f"[Cortex] Starting DECOUPLED simulation "
          f"({args.duration}s, fast={args.fast}, "
          f"phase={resolved_phase}, "
          f"profile={profile or 'hvacd_full'}, "
          f"substrate={substrate.name})")
    registry = build_registry(configs, profile_override=profile, substrate=substrate)
    shared_params = build_shared_params(substrate=substrate)
    dynamics = build_dynamics(configs, shared_params, registry, substrate=substrate)

    # Resolve night theta target for initial state (sim starts at midnight)
    night_cfg = configs['phase'].get('night', {})
    if substrate is not None and 'theta_target_frac' in night_cfg:
        awc = substrate.theta_fc - substrate.theta_wp
        theta_init = substrate.theta_wp + float(night_cfg['theta_target_frac']) * awc
    else:
        theta_init = night_cfg.get('theta_target', 0.30)

    io, sim_room = build_simulation_io(configs, registry, substrate=substrate,
                                        theta_init=theta_init)
    ekf = build_ekf(dynamics, substrate=substrate, theta_init=theta_init)
    mpc = build_mpc(dynamics, shared_params, registry, configs)
    reference = build_reference(configs, registry=registry, substrate=substrate)
    light_schedule = build_light_schedule(configs)
    forecast = build_forecast(configs)
    compliance = build_compliance(configs)
    failsafe = build_failsafe()

    # Build irrigation scheduler (disturbance-driven, not MPC-controlled)
    irr_scheduler = build_irrigation_scheduler(
        configs, substrate, light_schedule)
    if irr_scheduler:
        print(f"[Cortex] Irrigation scheduler active — "
              f"theta trigger={irr_scheduler.theta_trigger:.3f}, "
              f"shot={irr_scheduler.config.shot_duration_s}s, "
              f"max={irr_scheduler.config.max_daily_shots}/day")
    else:
        print("[Cortex] Irrigation scheduler disabled")

    # Build CO2 scheduler (before MPC so we can lock the channel)
    co2_scheduler = build_co2_scheduler(configs, registry, light_schedule, reference)
    if co2_scheduler:
        mpc.constraints.lock_channel('u_co2', 0.0)
        print(f"[Cortex] CO2 scheduler active — "
              f"deadband={co2_scheduler.config.deadband_ppm:.0f}ppm, "
              f"overshoot={co2_scheduler.config.overshoot_ppm:.0f}ppm, "
              f"holdoff={co2_scheduler.config.min_off_s:.0f}s")
    else:
        print("[Cortex] CO2 scheduler disabled (no u_co2 channel)")

    advisory_engine = build_advisory_engine(configs, reference, dynamics)
    if co2_scheduler:
        advisory_engine.co2_scheduler = co2_scheduler

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
        advisory_engine=advisory_engine,
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

    # Dashboard API + WebSocket broadcaster
    broadcaster = None
    if getattr(args, 'dashboard', False):
        broadcaster = _start_dashboard(
            args, solver, dynamics, reference, compliance,
            advisory_engine, mpc, light_schedule, registry)

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

        if broadcaster:
            broadcaster.on_telemetry(x_hat, u, derived, info)

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

            # Feed trajectory to broadcaster when a solve happened
            if broadcaster and solver._cached_trajectory is not None:
                ref_targets = solver._cached_ref_targets or {}
                tolerances = {}
                if compliance:
                    tolerances = compliance.tolerances
                broadcaster.notify_solve(
                    solver._cached_trajectory,
                    solver.io.get_time(),
                    ref_targets,
                    tolerances,
                )

            if irr_scheduler:
                x_hat = solver.ekf.x_hat
                irr_scheduler.tick(x_hat, solver.io.get_time())
                sim_room.disturbance_schedule.irr_rate = irr_scheduler.current_rate

            if co2_scheduler:
                co2_scheduler.tick(solver.ekf.x_hat, solver.io.get_time(), solver.last_u)

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
    _print_advisory_summary(advisory_engine)
    _print_dispatch_summary(dispatch_summary, gateway)
    if co2_scheduler:
        _print_co2_summary(co2_scheduler)
    if irr_scheduler:
        _print_irrigation_summary(irr_scheduler)
        if recorder:
            recorder.irr_events = [
                (s / 3600.0, e / 3600.0) for s, e in irr_scheduler.shot_log
            ]
    if recorder:
        recorder.advisories = advisory_engine.all_advisories()
    _maybe_chart(args, recorder, compliance)


def _start_dashboard(args, solver, dynamics, reference, compliance,
                     advisory_engine, mpc, light_schedule, registry=None):
    """Start the dashboard API server in a background thread."""
    import asyncio
    import threading
    import uvicorn
    from api.server import create_api
    from api.broadcast import WebSocketBroadcaster

    port = getattr(args, 'api_port', 8080)

    app = create_api(
        control_loop=solver,
        compliance=compliance,
        advisory_engine=advisory_engine,
    )

    broadcaster = WebSocketBroadcaster(
        app=app,
        dynamics=dynamics,
        reference=reference,
        compliance=compliance,
        advisory_engine=advisory_engine,
        registry=registry,
        mpc_step_s=mpc.step_s,
        mpc_interval=int(mpc.step_s),
        light_schedule=light_schedule,
    )

    # Capture uvicorn's actual running event loop for async broadcasting
    ready = threading.Event()

    @app.on_event("startup")
    async def _capture_event_loop():
        broadcaster.set_event_loop(asyncio.get_running_loop())
        ready.set()

    def run_server():
        config = uvicorn.Config(
            app, host="0.0.0.0", port=port,
            log_level="warning",
        )
        server = uvicorn.Server(config)
        server.run()

    thread = threading.Thread(target=run_server, daemon=True)
    thread.start()
    ready.wait(timeout=10.0)
    print(f"[Cortex] Dashboard API started on http://localhost:{port}")
    print(f"[Cortex] WebSocket at ws://localhost:{port}/ws/dashboard")

    return broadcaster


def _print_advisory_summary(advisory_engine):
    all_advisories = advisory_engine.all_advisories()
    if not all_advisories:
        print("\n[Cortex] Advisories: none generated")
        return

    by_severity = {}
    by_type = {}
    for a in all_advisories:
        by_severity[a.severity] = by_severity.get(a.severity, 0) + 1
        key = f'{a.type}:{a.channel}' if a.channel else a.type
        by_type[key] = by_type.get(key, 0) + 1

    print(f"\n[Cortex] Advisory summary ({len(all_advisories)} total):")
    for sev in ('critical', 'warning', 'info'):
        if sev in by_severity:
            print(f"  {sev:10s}: {by_severity[sev]}")
    print("  By type:")
    for key, count in sorted(by_type.items()):
        print(f"    {key:30s}: {count}")

    print("  Timeline:")
    for a in all_advisories:
        # Simulation time
        if a.sim_time_s is not None:
            t_h = a.sim_time_s / 3600.0
            t_str = f'{t_h:5.1f}h'
        else:
            t_str = '  ?  '
        # ETA to predicted event
        if a.eta_s is not None and a.eta_s >= 3600:
            eta_str = f'ETA {a.eta_s/3600:.1f}h'
        elif a.eta_s is not None and a.eta_s > 0:
            eta_str = f'ETA {int(a.eta_s/60)}m'
        else:
            eta_str = 'now'
        print(f"    {t_str} [{a.severity:8s}] {a.title} ({eta_str})")


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


def _print_co2_summary(scheduler):
    s = scheduler.summary()
    print(f"\n[Cortex] CO2 injection summary:")
    print(f"  Total cycles:      {s['total_cycles']}")
    if s['total_cycles'] > 0:
        print(f"  Avg cycle duration: {s['avg_cycle_s']:.0f}s")
        print(f"  Total on-time:     {s['total_on_time_s']:.0f}s")
        obs = s['observed_flow_ppm_s']
        if obs is not None:
            expected = scheduler.config.expected_flow_ppm_s
            pct = (obs / expected) * 100 if expected > 0 else 0
            print(f"  Observed flow:     {obs:.2f} ppm/s ({pct:.0f}% of expected {expected:.1f})")


def _print_irrigation_summary(scheduler):
    s = scheduler.summary()
    print(f"\n[Cortex] Irrigation summary:")
    print(f"  Total shots:      {s['total_shots']}")
    print(f"  Theta trigger:    {s['theta_trigger']:.3f}")
    if s['total_shots'] > 0:
        print(f"  Avg duration:     {s['avg_duration_s']:.0f}s")
        print(f"  Total irrigation: {s['total_irrigation_s']:.0f}s")
        print(f"  Shot timeline (hours):")
        for start_h, end_h in s['shots']:
            print(f"    {start_h:.2f}h - {end_h:.2f}h")
