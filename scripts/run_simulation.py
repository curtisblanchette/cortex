#!/usr/bin/env python3
"""Run a simulation scenario and print results.

Usage:
    python scripts/run_simulation.py                     # default 24h mid-flower
    python scripts/run_simulation.py --scenario lights_transition
    python scripts/run_simulation.py --duration 3600     # 1 hour
"""

import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import build_simulation


def main():
    parser = argparse.ArgumentParser(description='Run Cortex simulation')
    parser.add_argument('--scenario', type=str, default='24h_mid_flower',
                       help='Scenario name')
    parser.add_argument('--duration', type=int, default=None,
                       help='Override duration in seconds')
    parser.add_argument('--config', type=str, default='config')
    args = parser.parse_args()

    duration = args.duration or 86400

    print(f"=== Cortex Simulation: {args.scenario} ({duration}s) ===\n")

    loop, sim_room, compliance = build_simulation(
        config_dir=args.config,
        duration_s=duration,
    )

    report_interval = max(1, duration // 20)

    def telemetry_cb(x_hat, u, derived, info):
        tick = info['tick']
        if tick % report_interval == 0:
            hours = info['time_s'] / 3600.0
            print(f"  [{hours:6.1f}h] T={x_hat[0]:.1f}°C  "
                  f"RH={derived['RH']:.0f}%  VPD={derived['VPD']:.2f}kPa  "
                  f"CO2={x_hat[2]:.0f}ppm  θ={x_hat[4]:.3f}  "
                  f"solve={info['solve_ms']:.1f}ms")

    loop.set_telemetry_callback(telemetry_cb)

    print("Running...")
    loop.run_sync(duration, realtime=False)

    print("\n=== Compliance Report ===")
    report = compliance.report()
    for key, data in report.items():
        status = "PASS" if data['compliance_pct'] >= 95 else "WARN" if data['compliance_pct'] >= 80 else "FAIL"
        print(f"  [{status}] {key}: {data['compliance_pct']:.1f}%  "
              f"(mean={data['mean']:.3f}, range=[{data['min']:.3f}, {data['max']:.3f}])")

    # Summary stats
    hist = sim_room.get_history_arrays()
    if hist:
        print(f"\n=== Summary ===")
        print(f"  Total ticks: {loop.tick_count}")
        print(f"  Simulation time: {sim_room.time:.0f}s ({sim_room.time/3600:.1f}h)")


if __name__ == '__main__':
    main()
