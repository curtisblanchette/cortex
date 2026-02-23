"""Compliance tracker — rolling metrics for control performance.

Tracks what percentage of time each controlled variable stays within
its target tolerance. Reports every minute based on a rolling 1-hour window.
"""

import numpy as np
from collections import deque


class ComplianceTracker:
    """Rolling compliance metrics for control quality monitoring."""

    def __init__(self, phase_config=None, window_s=3600):
        """Initialize compliance tracker.

        Args:
            phase_config: Phase config with compliance thresholds.
            window_s: Rolling window size in seconds.
        """
        self.window_s = window_s

        # Parse thresholds from phase config
        compliance = {}
        if phase_config:
            compliance = phase_config.get('compliance', {})

        self.tolerances = {
            'VPD': compliance.get('VPD_tolerance', 0.15),
            'T': compliance.get('T_tolerance', 1.0),
            'CO2': compliance.get('CO2_tolerance', 100.0),
            'theta': compliance.get('theta_tolerance', 0.05),
        }

        # Rolling buffers: (timestamp, value, target, in_compliance)
        self.buffers = {key: deque() for key in self.tolerances}

        # Cumulative stats
        self.total_samples = {key: 0 for key in self.tolerances}
        self.compliant_samples = {key: 0 for key in self.tolerances}

    def update(self, x_hat, derived, reference_targets, time_s):
        """Record one sample.

        Args:
            x_hat: State estimate (7,).
            derived: Dict with 'VPD', 'RH', etc. from dynamics.derived_outputs().
            reference_targets: Current reference targets dict.
            time_s: Current time in seconds.
        """
        # Map values and targets
        values = {
            'VPD': derived.get('VPD', 0.0),
            'T': x_hat[0],
            'CO2': x_hat[2],
            'theta': x_hat[4],
        }

        targets = {
            'VPD': reference_targets.get('VPD', 1.2),
            'T': reference_targets.get('T', 26.0),
            'CO2': reference_targets.get('CO2', 1200.0),
            'theta': reference_targets.get('theta', 0.35),
        }

        for key in self.tolerances:
            error = abs(values[key] - targets[key])
            in_compliance = error <= self.tolerances[key]

            self.buffers[key].append((time_s, values[key], targets[key], in_compliance))
            self.total_samples[key] += 1
            if in_compliance:
                self.compliant_samples[key] += 1

            # Trim old samples outside window
            while self.buffers[key] and self.buffers[key][0][0] < time_s - self.window_s:
                old = self.buffers[key].popleft()
                self.total_samples[key] -= 1
                if old[3]:
                    self.compliant_samples[key] -= 1

    def report(self):
        """Generate compliance report.

        Returns:
            Dict with per-variable compliance percentage and stats.
        """
        report = {}
        for key in self.tolerances:
            total = self.total_samples[key]
            if total > 0:
                pct = (self.compliant_samples[key] / total) * 100.0
            else:
                pct = 100.0

            values = [entry[1] for entry in self.buffers[key]]
            report[key] = {
                'compliance_pct': round(pct, 1),
                'samples': total,
                'tolerance': self.tolerances[key],
                'mean': round(np.mean(values), 3) if values else 0.0,
                'std': round(np.std(values), 3) if values else 0.0,
                'min': round(min(values), 3) if values else 0.0,
                'max': round(max(values), 3) if values else 0.0,
            }

        return report

    def reset(self):
        """Clear all buffers and counters."""
        for key in self.tolerances:
            self.buffers[key].clear()
            self.total_samples[key] = 0
            self.compliant_samples[key] = 0
