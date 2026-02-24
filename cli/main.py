"""Cortex MPC — CLI entry point with subcommands.

Usage:
    cortex simulate [--fast] [--legacy] [--duration N]
    cortex solver   [--redis-host HOST] [--room-id ID]
    cortex gateway  [--redis-host HOST] [--room-id ID]
    cortex api      [--redis-host HOST] [--port PORT]
    cortex calibrator [--redis-host HOST] [--room-id ID]
    cortex all      [--redis-host HOST] [--room-id ID] [--port PORT]
"""

import argparse
import sys

from cli.commands import simulate


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='cortex',
        description='Cortex MPC Grow Room Controller',
    )
    subparsers = parser.add_subparsers(dest='command')

    # Register all subcommands
    simulate.register(subparsers)

    # Lazy-import production commands to avoid pulling in redis
    # when only running simulation
    _register_production_commands(subparsers)

    args = parser.parse_args(argv)

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    args.func(args)


def _register_production_commands(subparsers):
    """Register production subcommands (solver, gateway, api, calibrator, all)."""
    from cli.commands import solver, gateway, api, calibrator, all as all_cmd

    solver.register(subparsers)
    gateway.register(subparsers)
    api.register(subparsers)
    calibrator.register(subparsers)
    all_cmd.register(subparsers)
