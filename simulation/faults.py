"""Fault injection for simulation testing.

Simulates hardware failures to test fail-safe degradation:
  - Sensor dropout (NaN readings)
  - HVAC compressor failure (no cooling/heating)
  - Stuck actuator (value frozen)
  - Network outage (stale readings)
"""

import numpy as np


class FaultInjector:
    """Manages active faults during simulation."""

    def __init__(self):
        self.faults = []

    def add_fault(self, fault_type, start_s, end_s, **kwargs):
        """Schedule a fault.

        Args:
            fault_type: One of 'sensor_dropout', 'hvac_failure',
                       'stuck_actuator', 'sensor_bias'.
            start_s: Fault start time in seconds.
            end_s: Fault end time in seconds.
            **kwargs: Fault-specific parameters.
        """
        self.faults.append({
            'type': fault_type,
            'start_s': start_s,
            'end_s': end_s,
            **kwargs,
        })

    def is_active(self, fault_type, time_s):
        """Check if any fault of given type is active at time_s."""
        for f in self.faults:
            if f['type'] == fault_type and f['start_s'] <= time_s <= f['end_s']:
                return True, f
        return False, None

    def corrupt_measurements(self, z_meas, time_s):
        """Apply sensor faults to measurements."""
        z = z_meas.copy()

        active, fault = self.is_active('sensor_dropout', time_s)
        if active:
            sensor_idx = fault.get('sensor_idx', 0)
            z[sensor_idx] = np.nan

        active, fault = self.is_active('sensor_bias', time_s)
        if active:
            sensor_idx = fault.get('sensor_idx', 0)
            bias = fault.get('bias', 5.0)
            z[sensor_idx] += bias

        return z

    def corrupt_control(self, u, time_s):
        """Apply actuator faults to control vector."""
        u_out = u.copy()

        active, fault = self.is_active('hvac_failure', time_s)
        if active:
            # Zero out HVAC-related controls
            u_out[0] = 0.0  # No cooling/heating
            u_out[1] = 0.0

        active, fault = self.is_active('stuck_actuator', time_s)
        if active:
            actuator_idx = fault.get('actuator_idx', 0)
            stuck_value = fault.get('stuck_value', u_out[actuator_idx])
            u_out[actuator_idx] = stuck_value

        return u_out
