"""cortex calibrator — background SLSQP parameter tuning via Redis telemetry."""

import json
import signal
import sys

import numpy as np

from cli.bootstrap import (
    load_configs, make_redis, build_registry, build_shared_params,
    build_dynamics,
)


class RedisTelemetrySource:
    """Adapter that reads telemetry history from a Redis list.

    The ModelCalibrator expects a telemetry_source with
    .get_recent_data(window_s) that returns {'x': ..., 'u': ..., 'd': ...}.

    The solver pushes JSON records to `telemetry:{room_id}:history`
    (rpush + ltrim). This adapter reads and decodes them.
    """

    def __init__(self, redis_client, room_id='room1'):
        self.redis = redis_client
        self.room_id = room_id

    def get_recent_data(self, window_s):
        """Read recent telemetry from Redis list.

        Args:
            window_s: Desired window in seconds (used to limit samples).

        Returns:
            Dict with 'x', 'u', 'd' numpy arrays, or None if insufficient.
        """
        key = f'telemetry:{self.room_id}:history'
        # Read all available entries (list is already trimmed by solver)
        raw = self.redis.lrange(key, 0, -1)
        if not raw:
            return None

        records = []
        for entry in raw:
            try:
                records.append(json.loads(entry))
            except (json.JSONDecodeError, TypeError):
                continue

        if len(records) < 2:
            return None

        # Limit to window_s most recent records (1 record ~= 1 second)
        max_records = int(window_s)
        if len(records) > max_records:
            records = records[-max_records:]

        x = np.array([r['x'] for r in records])
        u = np.array([r['u'] for r in records])
        d = np.array([r['d'] for r in records])

        return {'x': x, 'u': u, 'd': d}


def register(subparsers):
    p = subparsers.add_parser('calibrator',
                              help='Run background calibrator (production)')
    p.add_argument('--redis-host', default='localhost',
                   help='Redis host (default: localhost)')
    p.add_argument('--redis-port', type=int, default=6379,
                   help='Redis port (default: 6379)')
    p.add_argument('--room-id', default='room1',
                   help='Room identifier (default: room1)')
    p.add_argument('--config', type=str, default='config',
                   help='Config directory')
    p.add_argument('--interval', type=int, default=600,
                   help='Calibration interval in seconds (default: 600)')
    p.set_defaults(func=run)


def run(args):
    from calibration.calibrator import ModelCalibrator

    print(f"[Cortex] Connecting to Redis at {args.redis_host}:{args.redis_port}")
    try:
        redis_client = make_redis(args.redis_host, args.redis_port)
    except Exception as e:
        print(f"[Cortex] Redis connection failed: {e}")
        sys.exit(1)
    print("[Cortex] Redis connected.")

    configs = load_configs(args.config)
    registry = build_registry(configs)
    shared_params = build_shared_params()
    dynamics = build_dynamics(configs, shared_params, registry)

    telemetry_source = RedisTelemetrySource(redis_client, room_id=args.room_id)

    cal = ModelCalibrator(
        dynamics=dynamics,
        telemetry_source=telemetry_source,
        shared_params=shared_params,
        interval_s=args.interval,
    )

    # Signal handling
    def _shutdown(signum, frame):
        print("\n[Cortex] Shutting down calibrator...")
        cal.stop()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print(f"[Cortex] Calibrator running (interval={args.interval}s). "
          f"Ctrl-C to stop.")
    # Run in foreground (start() uses a daemon thread, so we block here)
    cal.start()

    # Block until stop signal
    import threading
    stop_event = threading.Event()

    original_shutdown = _shutdown

    def _shutdown_with_event(signum, frame):
        original_shutdown(signum, frame)
        stop_event.set()

    signal.signal(signal.SIGINT, _shutdown_with_event)
    signal.signal(signal.SIGTERM, _shutdown_with_event)

    stop_event.wait()
    print("[Cortex] Calibrator stopped.")
