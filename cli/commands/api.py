"""cortex api — FastAPI dashboard server (reads from Redis)."""

import signal
import sys


def register(subparsers):
    p = subparsers.add_parser('api', help='Run API dashboard (production)')
    p.add_argument('--redis-host', default='localhost',
                   help='Redis host (default: localhost)')
    p.add_argument('--redis-port', type=int, default=6379,
                   help='Redis port (default: 6379)')
    p.add_argument('--room-id', default='room1',
                   help='Room identifier (default: room1)')
    p.add_argument('--port', type=int, default=8000,
                   help='API server port (default: 8000)')
    p.add_argument('--host', default='0.0.0.0',
                   help='API server bind address (default: 0.0.0.0)')
    p.set_defaults(func=run)


def run(args):
    import uvicorn
    from cli.bootstrap import make_redis
    from api.server import create_api

    print(f"[Cortex] Connecting to Redis at {args.redis_host}:{args.redis_port}")
    try:
        redis_client = make_redis(args.redis_host, args.redis_port)
    except Exception as e:
        print(f"[Cortex] Redis connection failed: {e}")
        sys.exit(1)
    print("[Cortex] Redis connected.")

    app = create_api(redis_client=redis_client, room_id=args.room_id)

    print(f"[Cortex] API server starting on {args.host}:{args.port}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
