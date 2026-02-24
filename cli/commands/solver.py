"""cortex solver — EKF + OSQP, publishes solutions to Redis."""

import asyncio
import signal
import sys

from cli.bootstrap import (
    load_configs, make_redis, build_shared_params, build_dynamics,
    build_ekf, build_mpc, build_reference, build_light_schedule,
    build_forecast, build_compliance, build_failsafe, build_real_io,
)


def register(subparsers):
    p = subparsers.add_parser('solver', help='Run MPC solver (production)')
    p.add_argument('--redis-host', default='localhost',
                   help='Redis host (default: localhost)')
    p.add_argument('--redis-port', type=int, default=6379,
                   help='Redis port (default: 6379)')
    p.add_argument('--room-id', default='room1',
                   help='Room identifier (default: room1)')
    p.add_argument('--config', type=str, default='config',
                   help='Config directory')
    p.set_defaults(func=run)


def run(args):
    from control.solver_process import SolverProcess

    print(f"[Cortex] Connecting to Redis at {args.redis_host}:{args.redis_port}")
    try:
        redis_client = make_redis(args.redis_host, args.redis_port)
    except Exception as e:
        print(f"[Cortex] Redis connection failed: {e}")
        sys.exit(1)
    print("[Cortex] Redis connected.")

    configs = load_configs(args.config)
    shared_params = build_shared_params()
    dynamics = build_dynamics(configs, shared_params)
    io = build_real_io(redis_client, room_id=args.room_id)
    ekf = build_ekf(dynamics)
    mpc = build_mpc(dynamics, shared_params, configs)
    reference = build_reference(configs)
    light_schedule = build_light_schedule(configs)
    forecast = build_forecast(configs)
    compliance = build_compliance(configs)
    failsafe = build_failsafe()

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
        redis_client=redis_client,
        room_id=args.room_id,
        compliance=compliance,
    )

    # Signal handling
    def _shutdown(signum, frame):
        print("\n[Cortex] Shutting down solver...")
        solver.stop()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print(f"[Cortex] Solver running for room '{args.room_id}'. Ctrl-C to stop.")
    asyncio.run(solver.run())
    print("[Cortex] Solver stopped.")
