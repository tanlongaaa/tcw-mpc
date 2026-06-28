#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
m0_motivation_fig.py — M0 motivation 图生成 (论文 Fig.1 候选)
==============================================================
两个面板:
  (a) 前飞 power curve: 功耗 vs 空速 → 经典 U 形, 标出 v* 与悬停功耗
  (b) 节能潜力 vs 风速: 最优巡航相对悬停的节能 %

物理基于 quadrotor_dynamics.IRIS + effective_CdA, 配平 (trim) 求解。
输出: reports/m0_power_curve.png
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from quadrotor_dynamics import IRIS, effective_CdA

P = IRIS
m, g, rho = P['mass'], P['g'], P['rho']
W = m * g
fom, eff, td, nr = P['figure_of_merit'], P['motor_eff'], P['total_disk'], P['n_rotors']


def trim_power(v_air):
    """给定空速求配平 (倾角/推力) 与完整功耗 [W]."""
    phi = 0.0
    for _ in range(80):
        CdA = effective_CdA(phi, 0.0)
        D = 0.5 * rho * CdA * v_air**2
        phi_new = np.arctan2(D, W)
        if abs(phi_new - phi) < 1e-7:
            phi = phi_new
            break
        phi = phi_new
    CdA = effective_CdA(phi, 0.0)
    D = 0.5 * rho * CdA * v_air**2
    T = np.sqrt(D**2 + W**2)
    v_h = np.sqrt(T / (2.0 * rho * td))
    v_i = v_h**2 / np.sqrt(v_air**2 + v_h**2 + 1e-6)
    P_ind = T * v_i / fom
    P_pro = nr * 8.0 * (T / W) ** 0.25
    P_par = D * v_air
    return np.degrees(phi), T, (P_ind + P_pro + P_par) / eff, \
        P_ind / eff, P_pro / eff, P_par / eff


def main():
    v_grid = np.arange(0, 15.1, 0.5)
    data = np.array([trim_power(v) for v in v_grid])
    phi, T, Ptot, Pind, Ppro, Ppar = data.T
    P_hover = Ptot[0]
    i_star = np.argmin(Ptot)
    v_star, P_star = v_grid[i_star], Ptot[i_star]
    save_pct = (P_hover - P_star) / P_hover * 100

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))

    # ── (a) power curve ──
    ax1.plot(v_grid, Ptot, 'b-', lw=2.2, label='Total power')
    ax1.plot(v_grid, Pind, '--', color='tab:green', lw=1.3, label='Induced (↓)')
    ax1.plot(v_grid, Ppar, '--', color='tab:red', lw=1.3, label='Parasitic (↑)')
    ax1.plot(v_grid, Ppro, ':', color='gray', lw=1.2, label='Profile')
    ax1.axhline(P_hover, color='k', ls=':', lw=1, alpha=0.6)
    ax1.annotate('Hover %.0f W' % P_hover, (0.3, P_hover + 4), fontsize=9)
    ax1.plot(v_star, P_star, 'r*', ms=16, zorder=5)
    ax1.annotate('v* = %.1f m/s\n%.0f W (-%.0f%%)' % (v_star, P_star, save_pct),
                 (v_star, P_star), textcoords='offset points', xytext=(12, -28),
                 fontsize=9, color='darkred',
                 arrowprops=dict(arrowstyle='->', color='darkred'))
    ax1.set_xlabel('Airspeed [m/s]')
    ax1.set_ylabel('Electrical power [W]')
    ax1.set_title('(a) Quadrotor power curve (U-shape)')
    ax1.legend(fontsize=8, loc='upper center')
    ax1.grid(alpha=0.3)
    ax1.set_ylim(0, max(Ptot) * 0.65)

    # ── (b) 节能潜力 vs 等效风速 ──
    # 死守原点(相对气流=vw) vs 借风巡航(逆风分量降至接近 v*)
    winds = np.arange(0, 13, 0.5)
    save_potential = []
    for vw in winds:
        _, _, P_hold, *_ = trim_power(vw)               # 死守: 全量相对气流
        # 借风: 允许顺风漂移, 使相对气流向 v* 靠拢(取与 vw 的最近可行点)
        v_eff = abs(vw - v_star) if vw > v_star else min(vw, v_star)
        _, _, P_drift, *_ = trim_power(v_eff)
        save_potential.append(max(0.0, (P_hold - P_drift) / P_hold * 100))
    ax2.plot(winds, save_potential, 'm-', lw=2.2)
    ax2.fill_between(winds, 0, save_potential, alpha=0.15, color='m')
    ax2.set_xlabel('Wind speed [m/s]')
    ax2.set_ylabel('Energy-saving potential [%]')
    ax2.set_title('(b) Hold-station vs wind-aware drift (model est.)')
    ax2.grid(alpha=0.3)

    fig.suptitle('M0 Motivation: attitude/velocity regulation enables energy saving in wind',
                 fontsize=11, y=1.02)
    fig.tight_layout()

    outdir = os.path.join(os.path.dirname(os.path.dirname(SCRIPT_DIR)), 'reports')
    # fallback: 项目根 reports/
    proj_reports = '/home/tan/catkin_ws/src/px4-ros-6dof_project-pid-eso-/reports'
    os.makedirs(proj_reports, exist_ok=True)
    out = os.path.join(proj_reports, 'm0_power_curve.png')
    fig.savefig(out, dpi=140, bbox_inches='tight')
    print('saved:', out)
    print('v*=%.1f m/s  P*=%.1f W  hover=%.1f W  saving=%.1f%%  trim_tilt@v*=%.1f deg'
          % (v_star, P_star, P_hover, save_pct, phi[i_star]))


if __name__ == '__main__':
    main()
