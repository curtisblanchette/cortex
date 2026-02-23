"""Pre-defined simulation scenarios for testing and development.

Each scenario defines:
  - Room and equipment config overrides
  - Initial state
  - Simulation parameters (can differ from controller params)
  - Duration
  - Description of what's being tested
"""

import numpy as np

from core.dynamics import GrowRoomDynamics
from calibration.params import CalibrationParams


def scenario_24h_mid_flower():
    """24 hours of mid-flower, lights 12/12, outdoor 28°C.

    Basic steady-state test. The controller should maintain
    targets with >95% compliance.
    """
    return {
        'name': '24h_mid_flower',
        'duration_s': 86400,
        'initial_state': GrowRoomDynamics.default_initial_state(),
        'sim_params': CalibrationParams(),
        'room_overrides': {},
        'equipment_overrides': {},
        'description': '24h steady mid-flower with 12/12 light cycle',
    }


def scenario_lights_transition():
    """Focus on lights-on transition with 30kW heat spike.

    Tests the controller's ability to handle the sudden thermal
    load when lights turn on. Starts 1 hour before lights-on.
    """
    initial = GrowRoomDynamics.default_initial_state()
    # Start at night conditions
    initial[0] = 22.0   # T_a: night temp
    initial[1] = 8.0    # w_a: lower humidity at night
    initial[2] = 600.0  # CO2: lower at night (no injection)
    initial[3] = 21.0   # T_l: follows air closely
    initial[5] = 20.0   # T_s: supply air near room temp

    return {
        'name': 'lights_transition',
        'duration_s': 7200,  # 2 hours
        'initial_state': initial,
        'sim_params': CalibrationParams(),
        'room_overrides': {
            'light_schedule': {'on_hour': 1.0, 'off_hour': 13.0},  # lights on at hour 1 of sim
        },
        'equipment_overrides': {},
        'description': 'Lights-on transition: 30kW heat spike test',
        'start_hour': 0.0,  # Start at midnight, lights on at 1am
    }


def scenario_hvac_failure():
    """HVAC compressor fails at hour 3, recovers at hour 5.

    Tests fail-safe degradation and recovery. With no cooling
    for 2 hours, temperature will rise significantly.
    """
    return {
        'name': 'hvac_failure',
        'duration_s': 28800,  # 8 hours
        'initial_state': GrowRoomDynamics.default_initial_state(),
        'sim_params': CalibrationParams(),
        'fault_injection': {
            'type': 'hvac_failure',
            'start_s': 10800,   # hour 3
            'end_s': 18000,     # hour 5
        },
        'description': 'HVAC failure from hour 3-5, test recovery',
    }


def scenario_hot_outdoor():
    """Extreme outdoor heat: 40°C with high humidity.

    Tests cooling capacity limits. The HVAC may not be able
    to maintain setpoints, so compliance will drop.
    """
    initial = GrowRoomDynamics.default_initial_state()
    initial[0] = 28.0  # Already warm
    initial[6] = 30.0  # Walls are warm

    return {
        'name': 'hot_outdoor',
        'duration_s': 43200,  # 12 hours
        'initial_state': initial,
        'sim_params': CalibrationParams(),
        'room_overrides': {
            'outdoor': {
                'T_out_mean_C': 40.0,
                'T_out_amplitude_C': 5.0,
                'w_out_gkg': 18.0,
            }
        },
        'description': 'Extreme outdoor heat 40°C, high humidity',
    }


def scenario_parameter_mismatch():
    """Controller has wrong parameters; sim has different ones.

    Tests robustness to model uncertainty. The SLSQP calibrator
    should eventually close this gap in production.
    """
    # "True" room parameters
    true_params = CalibrationParams(
        UA_wall=70.0,      # 40% higher than default
        UA_env=35.0,       # 40% higher
        tau_hvac=180.0,    # 50% slower HVAC
        g_s_max=0.5,       # 25% more transpiration
        LAI=4.0,           # bigger canopy
        K_drain=0.08,      # faster drainage
        C_wall=700000.0,   # more thermal mass
        eta_hvac=0.75,     # less efficient HVAC
    )

    return {
        'name': 'parameter_mismatch',
        'duration_s': 86400,
        'initial_state': GrowRoomDynamics.default_initial_state(),
        'sim_params': true_params,
        'description': 'Controller default params vs true params with 25-50% mismatch',
    }


def scenario_dryback():
    """Test substrate dryback management overnight.

    Start with well-watered substrate, lights off, monitor
    dryback rate. No irrigation during test.
    """
    initial = GrowRoomDynamics.default_initial_state()
    initial[4] = 0.45   # θ: well-watered

    return {
        'name': 'dryback',
        'duration_s': 43200,  # 12 hours overnight
        'initial_state': initial,
        'sim_params': CalibrationParams(),
        'room_overrides': {
            'light_schedule': {'on_hour': 18.0, 'off_hour': 6.0},  # lights off
        },
        'description': 'Overnight dryback test, no irrigation',
    }


ALL_SCENARIOS = {
    '24h_mid_flower': scenario_24h_mid_flower,
    'lights_transition': scenario_lights_transition,
    'hvac_failure': scenario_hvac_failure,
    'hot_outdoor': scenario_hot_outdoor,
    'parameter_mismatch': scenario_parameter_mismatch,
    'dryback': scenario_dryback,
}
