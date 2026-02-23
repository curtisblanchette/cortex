#!/usr/bin/env python3
"""Run a simulation and generate comprehensive visualization charts.

Usage:
    python scripts/visualize.py                          # 4h default
    python scripts/visualize.py --duration 86400         # 24h
    python scripts/visualize.py --output charts/run1.png # custom output path
"""

import argparse
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Rectangle
from matplotlib.colors import LinearSegmentedColormap

from main import build_simulation


class TelemetryRecorder:
    """Captures per-tick data from the control loop for visualization."""

    def __init__(self):
        self.time = []
        self.x_hat = []
        self.x_true = []
        self.z_meas = []
        self.u = []
        self.d = []
        self.ref = []
        self.vpd = []
        self.rh = []
        self.vpd_target = []
        self.t_target = []
        self.co2_target = []
        self.theta_target = []
        self.solve_ms = []
        self.innovation_rms = []
        self.failsafe = []

    def record(self, x_hat, u, derived, info, sim_room, ref_targets):
        self.time.append(info['time_s'])
        self.x_hat.append(x_hat.copy())
        self.x_true.append(sim_room.x_true.copy())
        self.u.append(u.copy() if u is not None else np.zeros(6))
        self.d.append(sim_room.disturbance_schedule.get(sim_room.time))
        self.vpd.append(derived['VPD'])
        self.rh.append(derived['RH'])
        self.vpd_target.append(ref_targets.get('VPD', 1.2))
        self.t_target.append(ref_targets.get('T', 26.0))
        self.co2_target.append(ref_targets.get('CO2', 1200.0))
        self.theta_target.append(ref_targets.get('theta', 0.35))
        self.solve_ms.append(info.get('solve_ms', 0))
        self.innovation_rms.append(info.get('innovation_rms', 0))
        self.failsafe.append(info.get('failsafe_level', 'P1_NORMAL'))

    def to_arrays(self):
        return {
            'time_h': np.array(self.time) / 3600.0,
            'x_hat': np.array(self.x_hat),
            'x_true': np.array(self.x_true),
            'u': np.array(self.u),
            'd': np.array(self.d),
            'vpd': np.array(self.vpd),
            'rh': np.array(self.rh),
            'vpd_target': np.array(self.vpd_target),
            't_target': np.array(self.t_target),
            'co2_target': np.array(self.co2_target),
            'theta_target': np.array(self.theta_target),
            'solve_ms': np.array(self.solve_ms),
            'innovation_rms': np.array(self.innovation_rms),
            'failsafe': self.failsafe,
        }


def plot_simulation(data, compliance_report, duration_s, output_path):
    """Generate a comprehensive multi-panel visualization.

    Layout (8 panels):
      Row 1: Temperature tracking  |  VPD tracking
      Row 2: Humidity (RH + w_a)   |  CO2 tracking
      Row 3: Substrate (θ)         |  Control actions
      Row 4: MPC diagnostics       |  Compliance summary
    """
    t = data['time_h']
    x = data['x_hat']
    x_true = data['x_true']
    u = data['u']
    d = data['d']

    # Color palette
    C_ACTUAL = '#2196F3'      # blue
    C_TRUE = '#1565C0'        # dark blue
    C_TARGET = '#E53935'      # red
    C_TOLERANCE = '#FFCDD2'   # light red
    C_CONTROL = '#4CAF50'     # green
    C_CONTROL2 = '#FF9800'    # orange
    C_CONTROL3 = '#9C27B0'    # purple
    C_LIGHT_ON = '#FFF9C4'    # light yellow
    C_LIGHT_OFF = '#E8EAF6'   # light blue-grey
    C_GRID = '#E0E0E0'

    fig = plt.figure(figsize=(20, 22))
    fig.patch.set_facecolor('#FAFAFA')

    gs = GridSpec(4, 2, figure=fig, hspace=0.35, wspace=0.25,
                  left=0.06, right=0.97, top=0.95, bottom=0.04)

    # Helper: shade light on/off periods
    def shade_lights(ax, t_hours, d_array):
        """Shade background yellow when lights are on."""
        lights = d_array[:, 0] > 100  # Q_light > 100W means on
        if len(lights) == 0:
            return
        in_light = False
        start = 0
        for i in range(len(lights)):
            if lights[i] and not in_light:
                start = t_hours[i]
                in_light = True
            elif not lights[i] and in_light:
                ax.axvspan(start, t_hours[i], color=C_LIGHT_ON, alpha=0.3, zorder=0)
                in_light = False
        if in_light:
            ax.axvspan(start, t_hours[-1], color=C_LIGHT_ON, alpha=0.3, zorder=0)

    def style_axis(ax, ylabel, show_xlabel=False):
        ax.set_facecolor('white')
        ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.7)
        ax.set_ylabel(ylabel, fontsize=11, fontweight='bold')
        if show_xlabel:
            ax.set_xlabel('Time (hours)', fontsize=11)
        ax.tick_params(labelsize=9)
        ax.set_xlim(t[0], t[-1])

    # ===== Panel 1: Temperature =====
    ax1 = fig.add_subplot(gs[0, 0])
    shade_lights(ax1, t, d)
    ax1.plot(t, x[:, 0], color=C_ACTUAL, linewidth=1.2, label='T_air (EKF)', zorder=3)
    ax1.plot(t, x_true[:, 0], color=C_TRUE, linewidth=0.7, alpha=0.5, label='T_air (true)', zorder=2)
    ax1.plot(t, x[:, 3], color='#FF7043', linewidth=0.8, label='T_leaf', zorder=3)
    ax1.plot(t, data['t_target'], color=C_TARGET, linewidth=1.5, linestyle='--', label='Target', zorder=4)
    # Tolerance band
    tol = 1.0
    ax1.fill_between(t, data['t_target'] - tol, data['t_target'] + tol,
                      color=C_TOLERANCE, alpha=0.3, zorder=1, label=f'±{tol}°C')
    ax1.plot(t, x[:, 6], color='#795548', linewidth=0.6, alpha=0.6, label='T_wall')
    ax1.legend(loc='upper right', fontsize=8, ncol=3)
    ax1.set_title('Temperature Tracking', fontsize=13, fontweight='bold', pad=10)
    style_axis(ax1, 'Temperature (°C)')

    # ===== Panel 2: VPD =====
    ax2 = fig.add_subplot(gs[0, 1])
    shade_lights(ax2, t, d)
    ax2.plot(t, data['vpd'], color=C_ACTUAL, linewidth=1.2, label='VPD', zorder=3)
    ax2.plot(t, data['vpd_target'], color=C_TARGET, linewidth=1.5, linestyle='--', label='Target', zorder=4)
    tol_vpd = 0.15
    ax2.fill_between(t, data['vpd_target'] - tol_vpd, data['vpd_target'] + tol_vpd,
                      color=C_TOLERANCE, alpha=0.3, zorder=1, label=f'±{tol_vpd} kPa')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.set_title('VPD Tracking', fontsize=13, fontweight='bold', pad=10)
    style_axis(ax2, 'VPD (kPa)')

    # ===== Panel 3: Humidity =====
    ax3 = fig.add_subplot(gs[1, 0])
    shade_lights(ax3, t, d)
    ax3_rh = ax3
    ax3_wa = ax3.twinx()
    ln1 = ax3_rh.plot(t, data['rh'], color=C_ACTUAL, linewidth=1.2, label='RH (%)', zorder=3)
    ln2 = ax3_wa.plot(t, x[:, 1], color=C_CONTROL, linewidth=0.8, label='w_a (g/kg)', zorder=3)
    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    ax3.legend(lns, labs, loc='upper right', fontsize=8)
    ax3.set_title('Humidity', fontsize=13, fontweight='bold', pad=10)
    style_axis(ax3, 'Relative Humidity (%)')
    ax3_wa.set_ylabel('Absolute Humidity (g/kg)', fontsize=10, color=C_CONTROL)
    ax3_wa.tick_params(labelsize=9, colors=C_CONTROL)

    # ===== Panel 4: CO2 =====
    ax4 = fig.add_subplot(gs[1, 1])
    shade_lights(ax4, t, d)
    ax4.plot(t, x[:, 2], color=C_ACTUAL, linewidth=1.2, label='CO2 (EKF)', zorder=3)
    ax4.plot(t, data['co2_target'], color=C_TARGET, linewidth=1.5, linestyle='--', label='Target', zorder=4)
    tol_co2 = 100.0
    ax4.fill_between(t, data['co2_target'] - tol_co2, data['co2_target'] + tol_co2,
                      color=C_TOLERANCE, alpha=0.3, zorder=1)
    ax4.legend(loc='upper right', fontsize=8)
    ax4.set_title('CO2 Tracking', fontsize=13, fontweight='bold', pad=10)
    style_axis(ax4, 'CO2 (ppm)')

    # ===== Panel 5: Substrate Water =====
    ax5 = fig.add_subplot(gs[2, 0])
    shade_lights(ax5, t, d)
    ax5.plot(t, x[:, 4], color=C_ACTUAL, linewidth=1.2, label='θ (EKF)', zorder=3)
    ax5.plot(t, x_true[:, 4], color=C_TRUE, linewidth=0.7, alpha=0.5, label='θ (true)', zorder=2)
    ax5.plot(t, data['theta_target'], color=C_TARGET, linewidth=1.5, linestyle='--', label='Target', zorder=4)
    tol_th = 0.05
    ax5.fill_between(t, data['theta_target'] - tol_th, data['theta_target'] + tol_th,
                      color=C_TOLERANCE, alpha=0.3, zorder=1)
    ax5.legend(loc='upper right', fontsize=8)
    ax5.set_title('Substrate Water Content', fontsize=13, fontweight='bold', pad=10)
    style_axis(ax5, 'θ (m³/m³)', show_xlabel=True)

    # ===== Panel 6: Control Actions =====
    ax6 = fig.add_subplot(gs[2, 1])
    shade_lights(ax6, t, d)
    ax6.plot(t, u[:, 0], color=C_ACTUAL, linewidth=1.0, label='T_sp (°C)', zorder=3)
    ax6.plot(t, u[:, 1], color=C_CONTROL2, linewidth=1.0, label='RH_sp (%)', zorder=3)
    ax6.plot(t, u[:, 4], color=C_CONTROL, linewidth=0.8, label='Fan (%)', zorder=3)
    ax6.plot(t, u[:, 3], color=C_CONTROL3, linewidth=0.8, label='CO2 inj (%)', zorder=3)
    ax6.plot(t, u[:, 2], color='#00BCD4', linewidth=0.8, label='Humidifier (%)', zorder=3)
    ax6.plot(t, u[:, 5], color='#795548', linewidth=0.8, label='Irrigation (%)', zorder=3)
    ax6.legend(loc='upper right', fontsize=7, ncol=2)
    ax6.set_title('Control Actions', fontsize=13, fontweight='bold', pad=10)
    style_axis(ax6, 'Setpoint / Output', show_xlabel=True)

    # ===== Panel 7: MPC Diagnostics =====
    ax7 = fig.add_subplot(gs[3, 0])
    ax7_inn = ax7.twinx()
    ln1 = ax7.plot(t, data['solve_ms'], color=C_ACTUAL, linewidth=0.8, label='Solve time (ms)', zorder=3)
    ax7.axhline(100, color=C_TARGET, linewidth=1, linestyle='--', alpha=0.5, label='Budget (100ms)')
    ln2 = ax7_inn.plot(t, data['innovation_rms'], color=C_CONTROL2, linewidth=0.8, label='Innovation RMS', zorder=3)
    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    ax7.legend(lns, labs, loc='upper right', fontsize=8)
    ax7.set_title('MPC Diagnostics', fontsize=13, fontweight='bold', pad=10)
    style_axis(ax7, 'Solve Time (ms)', show_xlabel=True)
    ax7_inn.set_ylabel('Innovation RMS', fontsize=10, color=C_CONTROL2)
    ax7_inn.tick_params(labelsize=9, colors=C_CONTROL2)

    # ===== Panel 8: Compliance Summary =====
    ax8 = fig.add_subplot(gs[3, 1])
    ax8.set_facecolor('white')

    if compliance_report:
        keys = list(compliance_report.keys())
        vals = [compliance_report[k]['compliance_pct'] for k in keys]
        colors = ['#4CAF50' if v >= 95 else '#FF9800' if v >= 80 else '#E53935' for v in vals]

        bars = ax8.barh(keys, vals, color=colors, edgecolor='white', height=0.5)

        for bar, val in zip(bars, vals):
            ax8.text(bar.get_width() + 1, bar.get_y() + bar.get_height() / 2,
                     f'{val:.1f}%', va='center', fontsize=11, fontweight='bold')

        ax8.set_xlim(0, 110)
        ax8.axvline(95, color='#4CAF50', linewidth=1.5, linestyle='--', alpha=0.7, label='95% target')
        ax8.axvline(80, color='#FF9800', linewidth=1, linestyle=':', alpha=0.5, label='80% warning')

        # Add compliance details as text
        detail_y = -0.15
        for k in keys:
            r = compliance_report[k]
            ax8.text(55, detail_y, f"{k}: mean={r['mean']:.3f}  σ={r['std']:.3f}  [{r['min']:.3f}, {r['max']:.3f}]",
                     transform=ax8.get_yaxis_transform(), fontsize=7, color='#666',
                     va='top', fontfamily='monospace')
            detail_y -= 0.08

        ax8.legend(loc='lower right', fontsize=8)

    ax8.set_title('Compliance Summary', fontsize=13, fontweight='bold', pad=10)
    ax8.set_xlabel('Compliance (%)', fontsize=11)
    ax8.tick_params(labelsize=10)
    ax8.grid(True, axis='x', color=C_GRID, linewidth=0.5, alpha=0.7)

    # Title
    hours = duration_s / 3600
    fig.suptitle(f'Cortex MPC — {hours:.0f}h Simulation', fontsize=18,
                 fontweight='bold', y=0.98)

    # Save
    os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else '.', exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f'Chart saved to {output_path}')


def run_and_visualize(duration_s, output_path, config_dir='config'):
    """Run simulation and generate charts."""
    print(f'Building {duration_s}s simulation...')
    loop, sim_room, compliance = build_simulation(
        config_dir=config_dir, duration_s=duration_s,
    )

    recorder = TelemetryRecorder()
    report_interval = max(1, duration_s // 40)

    def telemetry_cb(x_hat, u, derived, info):
        # Get reference targets
        current_time = info['time_s']
        ref = loop.reference.generate(
            current_time, loop.light_schedule, horizon_steps=1, step_s=60.0
        )
        ref_targets = {
            'VPD': ref[0, 0], 'T': ref[0, 1],
            'CO2': ref[0, 2], 'theta': ref[0, 4],
        }
        recorder.record(x_hat, u, derived, info, sim_room, ref_targets)

        tick = info['tick']
        if tick % report_interval == 0:
            hours = current_time / 3600.0
            print(f'  [{hours:6.1f}h] T={x_hat[0]:.1f}°C  '
                  f'VPD={derived["VPD"]:.2f}kPa  '
                  f'CO2={x_hat[2]:.0f}ppm  '
                  f'θ={x_hat[4]:.3f}  '
                  f'solve={info["solve_ms"]:.0f}ms')

    loop.set_telemetry_callback(telemetry_cb)

    print('Running simulation...')
    t0 = time.perf_counter()
    loop.run_sync(duration_s, realtime=False)
    elapsed = time.perf_counter() - t0

    print(f'\n{loop.tick_count} ticks in {elapsed:.1f}s '
          f'({loop.tick_count / elapsed:.0f} ticks/s)')

    report = compliance.report()
    print('\nCompliance:')
    for key, dat in report.items():
        print(f'  {key}: {dat["compliance_pct"]:.1f}%')

    print('\nGenerating charts...')
    data = recorder.to_arrays()
    plot_simulation(data, report, duration_s, output_path)


def main():
    parser = argparse.ArgumentParser(description='Cortex simulation visualization')
    parser.add_argument('--duration', type=int, default=14400,
                        help='Simulation duration in seconds (default: 4h)')
    parser.add_argument('--output', type=str, default='output/simulation.png',
                        help='Output chart path')
    parser.add_argument('--config', type=str, default='config')
    args = parser.parse_args()

    run_and_visualize(args.duration, args.output, args.config)


if __name__ == '__main__':
    main()
