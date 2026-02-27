"""cortex all — run all four production services in one process.

Solver + gateway via asyncio.gather, calibrator in background thread,
API in a daemon thread (uvicorn blocks).
"""

import asyncio
import signal
import sys
import threading


def register(subparsers):
    p = subparsers.add_parser('all',
                              help='Run all services in one process')
    p.add_argument('--redis-host', default='localhost',
                   help='Redis host (default: localhost)')
    p.add_argument('--redis-port', type=int, default=6379,
                   help='Redis port (default: 6379)')
    p.add_argument('--room-id', default='room1',
                   help='Room identifier (default: room1)')
    p.add_argument('--config', type=str, default='config',
                   help='Config directory')
    p.add_argument('--substrate', type=str, default=None,
                   help='Substrate profile override (e.g. coco_coir, rockwool, perlite)')
    p.add_argument('--port', type=int, default=8000,
                   help='API server port (default: 8000)')
    p.set_defaults(func=run)


def run(args):
    from control.solver_process import SolverProcess
    from control.gateway import ActuatorGateway
    from calibration.calibrator import ModelCalibrator
    from api.server import create_api
    from cli.bootstrap import (
        load_configs, make_redis, build_substrate, build_registry,
        build_shared_params, build_dynamics, build_ekf, build_mpc,
        build_reference, build_light_schedule, build_forecast,
        build_compliance, build_failsafe, build_real_io,
    )
    from cli.commands.gateway import RedisSolutionReader
    from cli.commands.calibrator import RedisTelemetrySource

    print(f"[Cortex] Connecting to Redis at {args.redis_host}:{args.redis_port}")
    try:
        redis_client = make_redis(args.redis_host, args.redis_port)
    except Exception as e:
        print(f"[Cortex] Redis connection failed: {e}")
        sys.exit(1)
    print("[Cortex] Redis connected.")

    configs = load_configs(args.config)
    substrate_name = getattr(args, 'substrate', None)
    substrate = build_substrate(configs, substrate_override=substrate_name)
    registry = build_registry(configs, substrate=substrate)
    shared_params = build_shared_params(substrate=substrate)
    dynamics = build_dynamics(configs, shared_params, registry, substrate=substrate)
    io = build_real_io(redis_client, registry, room_id=args.room_id)
    ekf = build_ekf(dynamics)
    mpc = build_mpc(dynamics, shared_params, registry, configs)
    reference = build_reference(configs, registry=registry)
    light_schedule = build_light_schedule(configs)
    forecast = build_forecast(configs)
    compliance = build_compliance(configs)
    failsafe = build_failsafe()

    # --- Solver ---
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
        redis_client=redis_client,
        room_id=args.room_id,
        compliance=compliance,
    )

    # --- Gateway ---
    dispatch_config = configs['equipment'].get('dispatch', {})
    solution_reader = RedisSolutionReader(redis_client, room_id=args.room_id)
    gw = ActuatorGateway(
        solution_store=solution_reader,
        dispatch_config=dispatch_config,
        registry=registry,
        redis_client=redis_client,
        light_schedule=light_schedule,
        room_id=args.room_id,
    )

    # --- Calibrator ---
    telemetry_source = RedisTelemetrySource(redis_client, room_id=args.room_id)
    cal = ModelCalibrator(
        dynamics=dynamics,
        telemetry_source=telemetry_source,
        shared_params=shared_params,
        interval_s=600,
    )

    # --- API ---
    app = create_api(
        redis_client=redis_client,
        room_id=args.room_id,
        shared_params=shared_params,
        calibrator=cal,
    )

    # Signal handling — stop all services
    def _shutdown(signum, frame):
        print("\n[Cortex] Shutting down all services...")
        solver.stop()
        gw.stop()
        cal.stop()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # Start calibrator (background thread)
    cal.start()
    print("[Cortex] Calibrator started (background thread).")

    # Start API in a daemon thread
    def _run_api():
        import uvicorn
        uvicorn.run(app, host="0.0.0.0", port=args.port,
                    log_level="warning")

    api_thread = threading.Thread(target=_run_api, daemon=True)
    api_thread.start()
    print(f"[Cortex] API server started on port {args.port}.")

    # Run solver + gateway via asyncio
    print(f"[Cortex] All services running for room '{args.room_id}'. "
          f"Ctrl-C to stop.")

    async def _run_async():
        await asyncio.gather(
            solver.run(),
            gw.run(),
        )

    asyncio.run(_run_async())

    # Cleanup
    cal.stop()
    print("[Cortex] All services stopped.")
