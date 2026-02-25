"""cortex gateway — reads Redis solutions, dispatches to MQTT/Modbus."""

import asyncio
import signal
import sys

from cli.bootstrap import load_configs, make_redis, build_registry, build_light_schedule


class RedisSolutionReader:
    """Adapter that implements the SolutionStore.read() interface via Redis.

    The ActuatorGateway expects a solution_store with .read() and .publish().
    This adapter backs it with a Redis hash instead of in-memory state.
    """

    def __init__(self, redis_client, room_id='room1'):
        self.redis = redis_client
        self.room_id = room_id

    def read(self):
        """Read the latest MPC solution from Redis."""
        key = f'mpc:{self.room_id}:solution'
        data = self.redis.hgetall(key)
        if not data:
            return None
        return data

    def publish(self, solution):
        """Write solution to Redis (used if gateway also publishes)."""
        key = f'mpc:{self.room_id}:solution'
        str_solution = {k: str(v) for k, v in solution.items()}
        self.redis.hset(key, mapping=str_solution)
        self.redis.expire(key, 120)


def register(subparsers):
    p = subparsers.add_parser('gateway', help='Run actuator gateway (production)')
    p.add_argument('--redis-host', default='localhost',
                   help='Redis host (default: localhost)')
    p.add_argument('--redis-port', type=int, default=6379,
                   help='Redis port (default: 6379)')
    p.add_argument('--room-id', default='room1',
                   help='Room identifier (default: room1)')
    p.add_argument('--config', type=str, default='config',
                   help='Config directory')
    p.add_argument('--poll-interval', type=float, default=0.5,
                   help='Solution poll interval in seconds (default: 0.5)')
    p.set_defaults(func=run)


def run(args):
    from control.gateway import ActuatorGateway

    print(f"[Cortex] Connecting to Redis at {args.redis_host}:{args.redis_port}")
    try:
        redis_client = make_redis(args.redis_host, args.redis_port)
    except Exception as e:
        print(f"[Cortex] Redis connection failed: {e}")
        sys.exit(1)
    print("[Cortex] Redis connected.")

    configs = load_configs(args.config)
    registry = build_registry(configs)
    light_schedule = build_light_schedule(configs)
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

    # Signal handling
    def _shutdown(signum, frame):
        print("\n[Cortex] Shutting down gateway...")
        gw.stop()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print(f"[Cortex] Gateway running for room '{args.room_id}'. Ctrl-C to stop.")
    asyncio.run(gw.run(poll_interval=args.poll_interval))
    print("[Cortex] Gateway stopped.")
