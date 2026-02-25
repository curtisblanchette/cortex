"""Modular actuator plugin system.

Each actuator type is a self-contained plugin declaring its control channels,
physics effects, dispatch rules, and IO mappings. The ActuatorRegistry
assembles plugins from config and becomes the single source of truth for
control vector dimensions, bounds, and mappings.
"""

from .base import (
    ActuatorPlugin,
    ControlChannel,
    PhysicsEffect,
    DispatchSpec,
    IOMapping,
)
from .registry import ActuatorRegistry
from .factory import build_registry

__all__ = [
    'ActuatorPlugin',
    'ControlChannel',
    'PhysicsEffect',
    'DispatchSpec',
    'IOMapping',
    'ActuatorRegistry',
    'build_registry',
]
