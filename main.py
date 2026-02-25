"""Cortex MPC — main entry point.

Usage:
    python main.py simulate [--fast] [--duration N] [--chart] [--chart-output PATH]
    python main.py solver   [--redis-host HOST]
    python main.py gateway  [--redis-host HOST]
    python main.py api      [--redis-host HOST] [--port PORT]
    python main.py calibrator [--redis-host HOST]
    python main.py all      [--redis-host HOST]
"""

import sys
import os

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


if __name__ == '__main__':
    from cli.main import main
    main(sys.argv[1:])