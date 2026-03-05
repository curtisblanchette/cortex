"""Factory for building ActuatorRegistry from config.

Reads the equipment profile section and instantiates the right plugins.
When no profile section exists, defaults to 'hvacd_full' for backward
compatibility. Profile YAML files in config/profiles/ are loaded as
a fallback when a profile name isn't in the built-in dict.
"""

import os
from typing import Dict, List

from .base import ActuatorPlugin
from .registry import ActuatorRegistry


# Map profile actuator names to plugin classes
PLUGIN_CLASSES = {
    'hvacd': 'core.actuators.hvacd_plugin.HVACDPlugin',
    'standalone_ac': 'core.actuators.ac_plugin.StandaloneACPlugin',
    'standalone_dehumidifier': 'core.actuators.dehumidifier_plugin.StandaloneDehumPlugin',
    'humidifier': 'core.actuators.humidifier_plugin.HumidifierPlugin',
    'co2_injector': 'core.actuators.co2_plugin.CO2InjectorPlugin',
    'fan': 'core.actuators.fan_plugin.FanPlugin',
    'irrigation': 'core.actuators.irrigation_plugin.IrrigationPlugin',
    'heater': 'core.actuators.heater_plugin.HeaterPlugin',
}

# Built-in profiles
PROFILES = {
    'hvacd_full': [
        'hvacd', 'humidifier', 'co2_injector', 'fan', 'irrigation',
    ],
    'ac_dehum': [
        'standalone_ac', 'standalone_dehumidifier', 'humidifier',
        'heater', 'co2_injector', 'fan', 'irrigation',
    ],
    'ac_minimal': [
        'standalone_ac', 'co2_injector', 'fan',
    ],
    'hvacd_no_irr': [
        'hvacd', 'humidifier', 'co2_injector', 'fan',
    ],
}


def _import_plugin_class(dotted_path: str):
    """Import a plugin class from a dotted module path."""
    module_path, class_name = dotted_path.rsplit('.', 1)
    import importlib
    module = importlib.import_module(module_path)
    return getattr(module, class_name)


def _load_profile_yaml(profile_name: str) -> List[str]:
    """Load a profile from config/profiles/{name}.yaml."""
    import yaml
    search_dirs = ['config/profiles', 'config']
    for d in search_dirs:
        path = os.path.join(d, f'{profile_name}.yaml')
        if os.path.isfile(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            profile = data.get('profile', {})
            if 'actuators' in profile:
                return profile['actuators']
    raise ValueError(
        f"Unknown equipment profile: {profile_name}. "
        f"Not found in built-in profiles or config/profiles/{profile_name}.yaml"
    )


def build_registry(equipment_config: dict, room_config: dict,
                   profile_override: str = None,
                   substrate=None) -> ActuatorRegistry:
    """Build an ActuatorRegistry from config.

    Args:
        equipment_config: Equipment YAML dict (may contain 'profile' section).
        room_config: Room YAML dict (for room geometry props).
        profile_override: CLI --profile override (takes precedence over config).
        substrate: SubstrateProfile instance (for container_volume_L in room_props).

    Returns:
        Fully assembled ActuatorRegistry.
    """
    # Determine actuator list
    profile_section = equipment_config.get('profile', {})

    if profile_override:
        profile_name = profile_override
    elif profile_section:
        profile_name = profile_section.get('name', 'hvacd_full')
    else:
        profile_name = 'hvacd_full'

    # Get actuator list from profile section, built-in profiles, or YAML file
    if profile_section and 'actuators' in profile_section and not profile_override:
        actuator_names = profile_section['actuators']
    elif profile_name in PROFILES:
        actuator_names = PROFILES[profile_name]
    else:
        actuator_names = _load_profile_yaml(profile_name)

    # Instantiate plugins
    plugins: List[ActuatorPlugin] = []
    for name in actuator_names:
        if name not in PLUGIN_CLASSES:
            raise ValueError(f"Unknown actuator type: {name}")
        cls = _import_plugin_class(PLUGIN_CLASSES[name])
        plugins.append(cls(equipment_config))

    # Extract room properties
    room = room_config.get('room', room_config) if isinstance(room_config, dict) else room_config
    room_props = {
        'V': room['volume_m3'],
        'A_floor': room['floor_area_m2'],
        'A_canopy': room.get('canopy_area_m2', room['floor_area_m2'] * 0.8),
        'A_wall': room.get('wall_area_m2', 136.0),
        'rho_air': room.get('rho_air', 1.2),
        'cp_air': room.get('cp_air', 1005.0),
    }

    if substrate is not None:
        room_props['container_volume_L'] = substrate.container_volume_L

    return ActuatorRegistry(plugins, room_props)
