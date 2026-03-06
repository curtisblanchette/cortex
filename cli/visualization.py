"""Telemetry recording and chart generation for simulation runs."""

import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


class TelemetryRecorder:
    """Captures per-tick data from the control loop for visualization."""

    # Fallback actuator order (used when no registry provided)
    ACTUATOR_ORDER = ['cool', 'heat', 'co2', 'dehumidifier', 'fan', 'irrigation']

    def __init__(self, registry=None, light_power_W=0.0):
        self._registry = registry
        self._light_power_W = light_power_W
        if registry is not None:
            self.ACTUATOR_ORDER = registry.actuator_order()
            self.CHANNEL_NAMES = registry.channel_names
            # Map each dispatch actuator to its u column index
            specs = registry.all_dispatch_specs()
            channels = registry.channel_names
            self._dispatch_to_u = {}
            for spec in specs:
                if spec.channel_name in channels:
                    self._dispatch_to_u[spec.actuator_name] = channels.index(spec.channel_name)
        else:
            self.CHANNEL_NAMES = ['T_sp', 'RH_sp', 'u_hum', 'u_co2', 'u_fan', 'u_irr']
            self._dispatch_to_u = {}
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
        self.rh_target = []
        self.theta_target = []
        self.solve_ms = []
        self.innovation_rms = []
        self.failsafe = []
        self.dispatch_commands = []  # per-tick gateway dispatched values
        self.power = []             # per-tick power draw dicts
        self.effort = []            # per-tick effort metrics dicts
        self.irr_events = []        # list of (start_h, end_h) from scheduler
        self.advisories = []        # list of Advisory objects from engine

    def record(self, x_hat, u, derived, info, sim_room, ref_targets):
        self.time.append(info['time_s'])
        self.x_hat.append(x_hat.copy())
        self.x_true.append(sim_room.x_true.copy())
        n_u = len(self.ACTUATOR_ORDER) if u is None else len(u)
        self.u.append(u.copy() if u is not None else np.zeros(n_u))
        d = sim_room.disturbance_schedule.get(sim_room.time)
        self.d.append(d)
        self.vpd.append(derived['VPD'])
        self.rh.append(derived['RH'])
        self.vpd_target.append(ref_targets.get('VPD', 1.2))
        self.t_target.append(ref_targets.get('T', 26.0))
        self.co2_target.append(ref_targets.get('CO2', 1200.0))
        self.rh_target.append(ref_targets.get('RH', 55.0))
        self.theta_target.append(ref_targets.get('theta', 0.35))
        self.solve_ms.append(info.get('solve_ms', 0))
        self.innovation_rms.append(info.get('innovation_rms', 0))
        self.failsafe.append(info.get('failsafe_level', 'P1_NORMAL'))

        # Power draw per component
        if self._registry is not None and u is not None:
            power_dict = self._registry.power_draw_all(u)
        else:
            power_dict = {}
        # Add lighting power (disturbance-driven, not an actuator)
        lights_on = d[0] > 100  # Q_light > 100W means on
        power_dict['lights'] = self._light_power_W if lights_on else 0.0
        self.power.append(power_dict)

        # Effort metrics (e.g., HVACD dehum_fraction, cooling_load_pct)
        self.effort.append(derived.get('effort', {}))

    def record_dispatch(self, decisions):
        """Record gateway dispatch decisions for this tick.

        Args:
            decisions: List of (actuator_name, DispatchResult) from gateway.
        """
        tick_cmds = {}
        for name, result in decisions:
            tick_cmds[name] = result.command
        # Fill missing actuators with 0
        row = [tick_cmds.get(a, 0.0) for a in self.ACTUATOR_ORDER]
        self.dispatch_commands.append(row)

    def to_arrays(self):
        # Build power arrays: (n_ticks, n_components)
        if self.power:
            # Stable component order: lights first, then plugins alphabetically
            all_keys = set()
            for p in self.power:
                all_keys.update(p.keys())
            # Put lights first, then sort the rest
            power_labels = ['lights'] if 'lights' in all_keys else []
            power_labels += sorted(k for k in all_keys if k != 'lights')
            power_array = np.array(
                [[p.get(k, 0.0) for k in power_labels] for p in self.power]
            )
        else:
            power_labels = []
            power_array = None

        # Build effort arrays: flatten {plugin: {metric: val}} per tick
        effort_data = {}
        for tick_effort in self.effort:
            for plugin_name, metrics in tick_effort.items():
                for metric_name, val in metrics.items():
                    key = f'{plugin_name}_{metric_name}'
                    effort_data.setdefault(key, []).append(val)
        # Pad any short series (shouldn't happen, but be safe)
        n_ticks = len(self.time)
        effort_arrays = {}
        for key, vals in effort_data.items():
            arr = np.array(vals)
            if len(arr) < n_ticks:
                arr = np.pad(arr, (0, n_ticks - len(arr)), constant_values=0.0)
            effort_arrays[key] = arr

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
            'rh_target': np.array(self.rh_target),
            'theta_target': np.array(self.theta_target),
            'solve_ms': np.array(self.solve_ms),
            'innovation_rms': np.array(self.innovation_rms),
            'failsafe': self.failsafe,
            'dispatch': np.array(self.dispatch_commands) if self.dispatch_commands else None,
            'dispatch_labels': self.ACTUATOR_ORDER,
            'channel_names': self.CHANNEL_NAMES,
            'dispatch_to_u': self._dispatch_to_u,
            'power': power_array,
            'power_labels': power_labels,
            'effort': effort_arrays,
            'irr_events': self.irr_events,
            'advisories': self.advisories,
        }


def plot_simulation(data, compliance_report, duration_s, output_path):
    """Generate a comprehensive multi-panel visualization.

    Layout (dynamic rows):
      Row 1: Temperature tracking      |  VPD tracking
      Row 2: Humidity (RH + w_a)       |  CO2 tracking
      Row 3: Substrate (theta)         |  Energy usage (stacked area)
      Row 4: MPC control output        |  (spans full width if no dispatch)
      Row 5: Gateway dispatched cmds   |  Dispatch filtering delta
      Row 6: MPC diagnostics           |  Compliance summary
    """
    t = data['time_h']
    x = data['x_hat']
    x_true = data['x_true']
    u = data['u']
    d = data['d']
    dispatch = data.get('dispatch')
    dispatch_labels = data.get('dispatch_labels', [])
    power = data.get('power')
    power_labels = data.get('power_labels', [])

    # Color palette
    C_ACTUAL = '#2196F3'      # blue
    C_TRUE = '#1565C0'        # dark blue
    C_TARGET = '#E53935'      # red
    C_TOLERANCE = '#FFCDD2'   # light red
    C_CONTROL = '#4CAF50'     # green
    C_CONTROL2 = '#FF9800'    # orange
    C_CONTROL3 = '#9C27B0'    # purple
    C_LIGHT_ON = '#FFF9C4'    # light yellow
    C_GRID = '#E0E0E0'

    advisories = data.get('advisories', [])
    effort = data.get('effort', {})

    n_rows = 4  # base: temp, humidity/CO2, substrate/energy, diagnostics/compliance
    has_effort = len(effort) > 0
    if has_effort:
        n_rows += 1  # HVACD effort row
    if dispatch is not None:
        n_rows += 1  # gateway dispatch row
    n_rows += 1  # control output row
    if advisories:
        n_rows += 1  # advisory timeline row
    fig = plt.figure(figsize=(20, 4 * n_rows))
    fig.patch.set_facecolor('#FAFAFA')

    gs = GridSpec(n_rows, 2, figure=fig, hspace=0.28, wspace=0.25,
                  left=0.06, right=0.97, top=0.93, bottom=0.04)

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
    tol = 1.0
    ax1.fill_between(t, data['t_target'] - tol, data['t_target'] + tol,
                      color=C_TOLERANCE, alpha=0.3, zorder=1, label=f'\u00b1{tol}\u00b0C')
    ax1.plot(t, x[:, 6], color='#795548', linewidth=0.6, alpha=0.6, label='T_wall')
    ax1.legend(loc='upper right', fontsize=8, ncol=3)
    ax1.set_title('Temperature Tracking', fontsize=13, fontweight='bold', pad=6)
    style_axis(ax1, 'Temperature (\u00b0C)')

    # ===== Panel 2: VPD =====
    ax2 = fig.add_subplot(gs[0, 1])
    shade_lights(ax2, t, d)
    ax2.plot(t, data['vpd'], color=C_ACTUAL, linewidth=1.2, label='VPD', zorder=3)
    ax2.plot(t, data['vpd_target'], color=C_TARGET, linewidth=1.5, linestyle='--', label='Target', zorder=4)
    tol_vpd = 0.15
    ax2.fill_between(t, data['vpd_target'] - tol_vpd, data['vpd_target'] + tol_vpd,
                      color=C_TOLERANCE, alpha=0.3, zorder=1, label=f'\u00b1{tol_vpd} kPa')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.set_title('VPD Tracking', fontsize=13, fontweight='bold', pad=6)
    style_axis(ax2, 'VPD (kPa)')

    # ===== Panel 3: Humidity =====
    ax3 = fig.add_subplot(gs[1, 0])
    shade_lights(ax3, t, d)
    ax3_rh = ax3
    ax3_wa = ax3.twinx()
    rh_tgt = data.get('rh_target')
    if rh_tgt is not None:
        tol_rh = compliance_report.get('RH', {}).get('tolerance', 5.0) if compliance_report else 5.0
        ax3_rh.plot(t, rh_tgt, color=C_TARGET, linewidth=1.5, linestyle='--', label='RH target', zorder=4)
        ax3_rh.fill_between(t, rh_tgt - tol_rh, rh_tgt + tol_rh,
                             color=C_TOLERANCE, alpha=0.3, zorder=1)
    ln1 = ax3_rh.plot(t, data['rh'], color=C_ACTUAL, linewidth=1.2, label='RH (%)', zorder=3)
    ln2 = ax3_wa.plot(t, x[:, 1], color=C_CONTROL, linewidth=0.8, label='w_a (g/kg)', zorder=3)
    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    ax3.legend(lns, labs, loc='upper right', fontsize=8)
    ax3.set_title('Humidity', fontsize=13, fontweight='bold', pad=6)
    style_axis(ax3, 'Relative Humidity (%)')
    # Fix y-axis to prevent jitter from derived RH target
    rh_all = data['rh']
    rh_lo = max(20, np.floor(rh_all.min() / 5) * 5 - 5)
    rh_hi = min(100, np.ceil(rh_all.max() / 5) * 5 + 5)
    ax3_rh.set_ylim(rh_lo, rh_hi)
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
    ax4.set_title('CO2 Tracking', fontsize=13, fontweight='bold', pad=6)
    style_axis(ax4, 'CO2 (ppm)')

    # ===== Panel 5: Substrate Water =====
    ax5 = fig.add_subplot(gs[2, 0])
    shade_lights(ax5, t, d)
    ax5.plot(t, x[:, 4], color=C_ACTUAL, linewidth=1.2, label='\u03b8 (EKF)', zorder=3)
    ax5.plot(t, x_true[:, 4], color=C_TRUE, linewidth=0.7, alpha=0.5, label='\u03b8 (true)', zorder=2)
    ax5.plot(t, data['theta_target'], color=C_TARGET, linewidth=1.5, linestyle='--', label='Target', zorder=4)
    tol_th = 0.05
    ax5.fill_between(t, data['theta_target'] - tol_th, data['theta_target'] + tol_th,
                      color=C_TOLERANCE, alpha=0.3, zorder=1)
    # Irrigation shot markers
    irr_events = data.get('irr_events', [])
    for i, (start_h, end_h) in enumerate(irr_events):
        ax5.axvspan(start_h, end_h, color='#4CAF50', alpha=0.3, zorder=1,
                    label='Irrigation shot' if i == 0 else None)
    ax5.legend(loc='upper right', fontsize=8)
    ax5.set_title('Substrate Water Content', fontsize=13, fontweight='bold', pad=6)
    style_axis(ax5, '\u03b8 (m\u00b3/m\u00b3)')

    # ===== Panel 6: Energy Usage =====
    ax_energy = fig.add_subplot(gs[2, 1])
    shade_lights(ax_energy, t, d)
    if power is not None and len(power_labels) > 0:
        # Component colors for stacked area
        _energy_colors = {
            'lights': '#FFD600',
            'hvacd': '#2196F3',
            'standalone_ac': '#2196F3',
            'heater': '#E53935',
            'standalone_dehumidifier': '#00BCD4',
            'humidifier': '#4CAF50',
            'co2_injector': '#9C27B0',
            'fan': '#607D8B',
            'irrigation': '#795548',
        }
        _energy_labels = {
            'lights': 'Lights',
            'hvacd': 'HVACD',
            'standalone_ac': 'AC',
            'heater': 'Heater',
            'standalone_dehumidifier': 'Dehumidifier',
            'humidifier': 'Humidifier',
            'co2_injector': 'CO2',
            'fan': 'Fan',
            'irrigation': 'Irrigation',
        }
        # Convert W to kW for display
        power_kW = power / 1000.0
        colors = [_energy_colors.get(k, '#666') for k in power_labels]
        labels = [_energy_labels.get(k, k) for k in power_labels]
        ax_energy.stackplot(t, power_kW.T, labels=labels, colors=colors,
                            alpha=0.8, zorder=2)
        ax_energy.legend(loc='upper right', fontsize=7, ncol=2)

        # Compute cumulative kWh via trapezoidal integration
        time_h = data['time_h']
        total_power_kW = power_kW.sum(axis=1)
        total_kWh = np.trapezoid(total_power_kW, time_h)
        ax_energy.text(0.02, 0.95, f'Total: {total_kWh:.0f} kWh',
                       transform=ax_energy.transAxes, fontsize=10,
                       fontweight='bold', va='top',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                                 alpha=0.8, edgecolor='#999'))
    ax_energy.set_title('Energy Usage', fontsize=13, fontweight='bold', pad=6)
    style_axis(ax_energy, 'Power (kW)')

    # ===== HVACD Effort Panel =====
    next_row = 3
    if has_effort:
        ax_eff = fig.add_subplot(gs[next_row, :])
        shade_lights(ax_eff, t, d)
        _effort_colors = {
            'hvacd_dehum_fraction': '#00BCD4',
            'hvacd_cooling_load_pct': '#2196F3',
        }
        _effort_labels = {
            'hvacd_dehum_fraction': 'Dehum Fraction',
            'hvacd_cooling_load_pct': 'Cooling Load (%)',
        }
        for key, arr in effort.items():
            # Scale dehum_fraction (0–1) to percentage for display
            if key.endswith('_dehum_fraction'):
                arr = arr * 100.0
            ax_eff.plot(t[:len(arr)], arr,
                        color=_effort_colors.get(key, '#666'),
                        linewidth=1.0, label=_effort_labels.get(key, key), zorder=3)
        ax_eff.axhline(100, color=C_TARGET, linewidth=1.2, linestyle='--',
                        alpha=0.6, label='Saturation (100%)', zorder=2)
        ax_eff.set_ylim(-5, 115)
        ax_eff.legend(loc='upper right', fontsize=8)
        ax_eff.set_title('HVACD Effort', fontsize=13, fontweight='bold', pad=6)
        style_axis(ax_eff, 'Load (%)')
        next_row += 1

    # ===== Control Actions (full width) =====
    ax6 = fig.add_subplot(gs[next_row, :])
    shade_lights(ax6, t, d)
    channel_names = data.get('channel_names', [])
    _ctrl_colors = [C_ACTUAL, C_CONTROL2, C_CONTROL, C_CONTROL3,
                    '#00BCD4', '#795548', '#E91E63', '#607D8B']
    for i in range(u.shape[1]):
        label = channel_names[i] if i < len(channel_names) else f'u[{i}]'
        color = _ctrl_colors[i % len(_ctrl_colors)]
        ax6.plot(t, u[:, i], color=color, linewidth=0.9, label=label, zorder=3)
    ax6.legend(loc='upper right', fontsize=7, ncol=2)
    ax6.set_title('MPC Control Output', fontsize=13, fontweight='bold', pad=6)
    style_axis(ax6, 'Setpoint / Output', show_xlabel=True)
    next_row += 1

    # ===== Gateway Dispatch =====
    # Left: actual command sent to hardware per actuator per tick.
    #   On DISPATCH: result.command = MPC setpoint (possibly rate-limited).
    #   On HOLD_*:   result.command = state.last_command (previous value held).
    #   Flat segments indicate holds where dispatch rules refused a change.
    #
    # Right: delta = MPC_raw - Gateway_dispatched.
    #   Positive = gateway suppressed an increase (MPC wanted more).
    #   Negative = gateway is over-driving (holding a previous higher command
    #              due to min_hold_s, anti-short-cycle, or dead-band).
    if dispatch is not None:
        ax_gw = fig.add_subplot(gs[next_row, 0])
        shade_lights(ax_gw, t, d)
        t_disp = t[:len(dispatch)]

        gw_colors = {
            'cool': C_ACTUAL, 'heat': C_CONTROL2, 'co2': C_CONTROL3,
            'humidifier': '#00BCD4', 'fan': C_CONTROL, 'irrigation': '#795548',
        }
        gw_labels = {
            'cool': 'Cool', 'heat': 'Heat', 'co2': 'CO2 inj',
            'humidifier': 'Humidifier', 'fan': 'Fan', 'irrigation': 'Irrigation',
        }
        for i, name in enumerate(dispatch_labels):
            ax_gw.plot(t_disp, dispatch[:, i],
                       color=gw_colors.get(name, '#666'),
                       linewidth=1.0, label=gw_labels.get(name, name), zorder=3)
        ax_gw.legend(loc='upper right', fontsize=7, ncol=2)
        ax_gw.set_title('Gateway Dispatched Commands', fontsize=13, fontweight='bold', pad=6)
        style_axis(ax_gw, 'Command Value', show_xlabel=True)

        ax_gw_r = fig.add_subplot(gs[next_row, 1])
        shade_lights(ax_gw_r, t, d)
        # Show dispatch delta: difference between MPC raw and gateway output
        dispatch_to_u = data.get('dispatch_to_u', {})
        for i, name in enumerate(dispatch_labels):
            u_idx = dispatch_to_u.get(name)
            if u_idx is not None and u_idx < u.shape[1]:
                delta = u[:len(dispatch), u_idx] - dispatch[:, i]
            else:
                delta = -dispatch[:, i]
            ax_gw_r.plot(t_disp, delta,
                         color=gw_colors.get(name, '#666'),
                         linewidth=0.8, label=gw_labels.get(name, name), zorder=3)
        ax_gw_r.axhline(0, color='#999', linewidth=0.5, linestyle='-', zorder=1)
        ax_gw_r.legend(loc='upper right', fontsize=7, ncol=2)
        ax_gw_r.set_title('Dispatch Filtering (MPC - Gateway)', fontsize=13, fontweight='bold', pad=6)
        style_axis(ax_gw_r, 'Filtered Amount', show_xlabel=True)

        next_row += 1

    # ===== MPC Diagnostics =====
    ax7 = fig.add_subplot(gs[next_row, 0])
    ax7_inn = ax7.twinx()
    ln1 = ax7.plot(t, data['solve_ms'], color=C_ACTUAL, linewidth=0.8, label='Solve time (ms)', zorder=3)
    ax7.axhline(100, color=C_TARGET, linewidth=1, linestyle='--', alpha=0.5, label='Budget (100ms)')
    ln2 = ax7_inn.plot(t, data['innovation_rms'], color=C_CONTROL2, linewidth=0.8, label='Innovation RMS', zorder=3)
    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    ax7.legend(lns, labs, loc='upper right', fontsize=8)
    ax7.set_title('MPC Diagnostics', fontsize=13, fontweight='bold', pad=6)
    style_axis(ax7, 'Solve Time (ms)', show_xlabel=True)
    ax7_inn.set_ylabel('Innovation RMS', fontsize=10, color=C_CONTROL2)
    ax7_inn.tick_params(labelsize=9, colors=C_CONTROL2)

    # ===== Compliance Summary =====
    ax8 = fig.add_subplot(gs[next_row, 1])
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

        ax8.legend(loc='lower right', fontsize=8)

    ax8.set_title('Compliance Summary', fontsize=13, fontweight='bold', pad=6)
    ax8.set_xlabel('Compliance (%)', fontsize=11)
    ax8.tick_params(labelsize=10)
    ax8.grid(True, axis='x', color=C_GRID, linewidth=0.5, alpha=0.7)

    # ===== Advisory Timeline =====
    if advisories:
        next_row += 1
        ax_adv = fig.add_subplot(gs[next_row, :])
        ax_adv.set_facecolor('white')
        shade_lights(ax_adv, t, d)

        _sev_colors = {'critical': '#E53935', 'warning': '#FF9800', 'info': '#2196F3'}
        _sev_markers = {'critical': 'D', 'warning': 's', 'info': 'o'}
        _sev_y = {'critical': 3, 'warning': 2, 'info': 1}

        # Short type labels for the chart
        _type_labels = {
            'env_breach': 'breach',
            'env_trend': 'trend',
            'light_transition': 'lights',
            'model_drift': 'drift',
            'solver_degraded': 'solver',
            'compliance_declining': 'compl',
        }

        # Stagger overlapping labels — track occupied (y, time_bucket) slots
        _label_offsets = {}
        time_bucket_h = max(0.3, t[-1] / 40.0)  # bucket width

        for a in advisories:
            if a.sim_time_s is None:
                continue
            t_h = a.sim_time_s / 3600.0
            sev = a.severity
            y = _sev_y.get(sev, 1)
            color = _sev_colors.get(sev, '#999')
            marker = _sev_markers.get(sev, 'o')

            # Label: channel if present, else short type name
            label = a.channel if a.channel else _type_labels.get(a.type, a.type)

            ax_adv.scatter(t_h, y, color=color, marker=marker, s=60, zorder=4,
                           edgecolors='white', linewidths=0.5)

            # Stagger y-offset for overlapping labels at the same severity
            bucket = (y, round(t_h / time_bucket_h))
            count = _label_offsets.get(bucket, 0)
            _label_offsets[bucket] = count + 1
            y_off = 6 + count * 10

            ax_adv.annotate(label, (t_h, y), textcoords='offset points',
                            xytext=(4, y_off), fontsize=6.5, color=color,
                            alpha=0.9, rotation=30, ha='left', va='bottom')

        ax_adv.set_yticks([1, 2, 3])
        ax_adv.set_yticklabels(['Info', 'Warning', 'Critical'], fontsize=9)
        ax_adv.set_ylim(0.3, 3.7)
        ax_adv.set_title('Advisory Timeline', fontsize=13, fontweight='bold', pad=6)
        style_axis(ax_adv, '', show_xlabel=True)
        ax_adv.grid(True, axis='x', color=C_GRID, linewidth=0.5, alpha=0.7)
        ax_adv.grid(False, axis='y')

        # Legend
        from matplotlib.lines import Line2D
        legend_elems = [
            Line2D([0], [0], marker='D', color='w', markerfacecolor='#E53935',
                   markersize=8, label='Critical'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='#FF9800',
                   markersize=8, label='Warning'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#2196F3',
                   markersize=8, label='Info'),
        ]
        ax_adv.legend(handles=legend_elems, loc='upper right', fontsize=8, ncol=3)

    # Title
    hours = duration_s / 3600
    fig.suptitle(f'Cortex MPC \u2014 {hours:.0f}h Simulation', fontsize=18,
                 fontweight='bold', y=0.97)

    # Save
    os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else '.', exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f'Chart saved to {output_path}')