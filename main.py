"""Cortex MPC — main entry point (thin shim).

Rewrites legacy flags for backwards compatibility, then delegates
to the CLI subcommand system.

Usage:
    python main.py simulate [--fast] [--legacy] [--duration N]
    python main.py solver   [--redis-host HOST]
    python main.py gateway  [--redis-host HOST]
    python main.py api      [--redis-host HOST] [--port PORT]
    python main.py calibrator [--redis-host HOST]
    python main.py all      [--redis-host HOST]

Legacy (still works):
    python main.py --simulate [--fast] [--legacy] [--duration N]
"""

import sys
import os

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _rewrite_legacy_args(argv):
    """Rewrite --simulate flag to 'simulate' subcommand for backwards compat.

    python main.py --simulate --fast --duration 3600
      =>  python main.py simulate --fast --duration 3600
    """
    if '--simulate' in argv:
        argv = list(argv)
        argv.remove('--simulate')
        argv.insert(0, 'simulate')
    return argv


if __name__ == '__main__':
    argv = _rewrite_legacy_args(sys.argv[1:])
    from cli.main import main
    main(argv)
