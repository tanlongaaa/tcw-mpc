#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_logs.py — 对比 MPC vs PID 飞行日志

用法:
  python3 analyze_logs.py mpc_log_*.csv pid_log_*.csv

输出:
  - 位置跟踪误差统计 (RMSE)
  - 功耗统计 (mean, std, total)
  - 推力使用统计
  - 姿态使用统计
  - 对比总结
"""

import sys, os
import numpy as np

def load_csv(path):
    """Load CSV and return dict of arrays"""
    import csv
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"Empty CSV: {path}")
    data = {}
    for key in rows[0].keys():
        try:
            data[key.strip()] = np.array([float(r[key]) for r in rows])
        except (ValueError, KeyError):
            pass  # skip non-numeric columns
    return data, len(rows)


def compute_metrics(d, label, t_min=5.0):
    """Compute standard metrics from log data"""
    # Trim: skip first 5s (takeoff transient)
    mask = d['t'] >= t_min
    
    metrics = {
        'label': label,
        'duration_s': d['t'][-1] - d['t'][0],
        'n_samples': np.sum(mask),
    }
    
    # Position tracking
    for ax, name in [('px', 'X'), ('py', 'Y'), ('pz', 'Z')]:
        err = d[ax][mask] - d[f'ref_{ax}'][mask]
        metrics[f'rmse_{name}'] = np.sqrt(np.mean(err**2))
    
    pos_err = np.sqrt(
        (d['px'][mask] - d['ref_px'][mask])**2 +
        (d['py'][mask] - d['ref_py'][mask])**2
    )
    pos_err_3d = np.sqrt(
        (d['px'][mask] - d['ref_px'][mask])**2 +
        (d['py'][mask] - d['ref_py'][mask])**2 +
        (d['pz'][mask] - d['ref_pz'][mask])**2
    )
    metrics['rmse_XY'] = np.sqrt(np.mean(pos_err**2))
    metrics['rmse_3D'] = np.sqrt(np.mean(pos_err_3d**2))
    metrics['max_err_XY'] = np.max(pos_err)
    metrics['max_err_3D'] = np.max(pos_err_3d)
    
    # Power
    if 'power_est' in d:
        p = d['power_est'][mask]
        metrics['power_mean_W'] = np.mean(p)
        metrics['power_std_W'] = np.std(p)
        metrics['power_max_W'] = np.max(p)
        metrics['energy_total_J'] = np.trapz(p, d['t'][mask])
    else:
        metrics['power_mean_W'] = float('nan')
    
    # Thrust (from u_thrust if available, else estimate)
    if 'u_thrust' in d:
        t_vec = d['u_thrust'][mask]
        metrics['thrust_mean'] = np.mean(t_vec)
        metrics['thrust_std'] = np.std(t_vec)
        metrics['thrust_max'] = np.max(t_vec)
        metrics['thrust_min'] = np.min(t_vec)
    
    # Wind
    if 'wspeed' in d:
        ws = d['wspeed'][mask]
        metrics['wind_mean'] = np.mean(ws)
        metrics['wind_max'] = np.max(ws)
    
    # Attitude
    if 'roll_deg' in d:
        r = np.abs(d['roll_deg'][mask])
        p = np.abs(d['pitch_deg'][mask])
        metrics['att_max_deg'] = max(np.max(r), np.max(p))
        metrics['att_mean_deg'] = np.mean(np.sqrt(r**2 + p**2))
    
    return metrics


def print_metrics(m):
    """Pretty print metrics"""
    print(f"\n{'='*60}")
    print(f"  {m['label']}")
    print(f"{'='*60}")
    print(f"  Duration: {m['duration_s']:.1f}s ({m['n_samples']} samples)")
    print(f"\n  📍 Position Tracking:")
    print(f"    RMSE X:    {m['rmse_X']:.3f} m")
    print(f"    RMSE Y:    {m['rmse_Y']:.3f} m")
    print(f"    RMSE Z:    {m['rmse_Z']:.3f} m")
    print(f"    RMSE XY:   {m['rmse_XY']:.3f} m")
    print(f"    RMSE 3D:   {m['rmse_3D']:.3f} m")
    print(f"    Max XY:    {m['max_err_XY']:.3f} m")
    
    if not np.isnan(m['power_mean_W']):
        print(f"\n  ⚡ Power:")
        print(f"    Mean:      {m['power_mean_W']:.1f} W")
        print(f"    Std:       {m['power_std_W']:.1f} W")
        print(f"    Max:       {m['power_max_W']:.1f} W")
        print(f"    Total:     {m['energy_total_J']:.1f} J")
    
    if 'thrust_mean' in m:
        print(f"\n  🔧 Thrust (normalized):")
        print(f"    Mean:      {m['thrust_mean']:.3f}")
        print(f"    Std:       {m['thrust_std']:.3f}")
    
    if 'wind_mean' in m:
        print(f"\n  🌬 Wind:")
        print(f"    Mean:      {m['wind_mean']:.1f} m/s")
        print(f"    Max:       {m['wind_max']:.1f} m/s")
    
    if 'att_max_deg' in m:
        print(f"\n  📐 Attitude:")
        print(f"    Max tilt:  {m['att_max_deg']:.1f}°")
        print(f"    Mean tilt: {m['att_mean_deg']:.1f}°")


def print_comparison(m1, m2, label1="MPC", label2="PID"):
    """Compare two metrics"""
    print(f"\n{'='*60}")
    print(f"  📊 {label1} vs {label2} COMPARISON")
    print(f"{'='*60}")
    
    comparisons = [
        ('rmse_XY', 'RMSE XY [m]', False),
        ('rmse_3D', 'RMSE 3D [m]', False),
        ('max_err_XY', 'Max XY Error [m]', False),
        ('power_mean_W', 'Mean Power [W]', False),
        ('power_max_W', 'Max Power [W]', False),
        ('energy_total_J', 'Total Energy [J]', False),
        ('thrust_mean', 'Mean Thrust', False),
        ('att_max_deg', 'Max Tilt [°]', False),
    ]
    
    print(f"\n  {'Metric':<20} {label1:>10} {label2:>10} {'Δ':>10} {'Δ%':>10}")
    print(f"  {'-'*20} {'-'*10} {'-'*10} {'-'*10} {'-'*10}")
    
    for key, name, _ in comparisons:
        if key in m1 and key in m2 and not np.isnan(m1[key]) and not np.isnan(m2[key]):
            delta = m1[key] - m2[key]
            if abs(m2[key]) > 1e-6:
                delta_pct = 100 * delta / abs(m2[key])
            else:
                delta_pct = 0
            
            good = "🟢" if (key in ('power_mean_W','power_max_W','energy_total_J','rmse_XY') and delta < 0) else "🔴" if delta > 0 else "⚪"
            if key in ('rmse_XY', 'rmse_3D') and delta < 0:
                good = "🟢"  # lower error is good
            
            print(f"  {name:<20} {m1[key]:>10.3f} {m2[key]:>10.3f} {delta:>10.3f} {delta_pct:>+9.1f}% {good}")


# ── Energy utilization stats (MPC only) ──
def print_energy_states(d):
    """Print energy module statistics"""
    print(f"\n{'='*60}")
    print(f"  ⚡ Energy Module States")
    print(f"{'='*60}")
    
    if 'coast_factor' in d:
        cf = d['coast_factor']
        coast_time = np.sum(cf > 0.05)
        print(f"  Coast time: {coast_time}/{len(cf)} steps ({100*coast_time/len(cf):.1f}%)")
        print(f"  Coast max:  {np.max(cf):.3f}")
    
    if 'cma_mode' in d:
        modes = d['cma_mode']
        # For string fields, count occurrences
        try:
            unique, counts = np.unique(modes, return_counts=True)
            print(f"  CMA modes:")
            for m, c in zip(unique, counts):
                print(f"    {m}: {c} ({100*c/len(modes):.1f}%)")
        except:
            pass
    
    if 'energy_mode' in d:
        modes = d['energy_mode']
        try:
            unique, counts = np.unique(modes, return_counts=True)
            print(f"  Energy modes:")
            for m, c in zip(unique, counts):
                print(f"    {m}: {c} ({100*c/len(modes):.1f}%)")
        except:
            pass


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_logs.py <mpc_log.csv> [pid_log.csv]")
        sys.exit(1)
    
    mpc_path = sys.argv[1]
    d_mpc, n_mpc = load_csv(mpc_path)
    m_mpc = compute_metrics(d_mpc, "⚡ Energy-Aware TCW-MPC")
    print_metrics(m_mpc)
    print_energy_states(d_mpc)
    
    if len(sys.argv) >= 3:
        pid_path = sys.argv[2]
        d_pid, n_pid = load_csv(pid_path)
        m_pid = compute_metrics(d_pid, "📐 PX4 PID")
        print_metrics(m_pid)
        print_comparison(m_mpc, m_pid, "MPC", "PID")
