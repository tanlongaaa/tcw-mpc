#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tilt_sweep_offline.py — M0 原理验证 (离线, 纯功耗模型)
========================================================
目的: 证明"姿态抗风节能"存在能耗最优倾角 θ*, 且 θ* 随风速变化。
      作为论文 motivation 图; 同时验证功耗模型是否具备 U 形所需物理。

关键发现 (2026-06-28):
  原 bem_power_10d 缺 parasitic(废阻)功率项 → 功耗对相对气流单调 → 无 U 形。
  代码已有 effective_CdA(roll,pitch) 但未接入功耗。
  本脚本用 "完整功耗 = 诱导 + 型阻 + 废阻(CdA·v_rel³)" 验证 U 形。

物理 (抗风定点悬停, v=0, v_rel = 风速):
  竖直力平衡: T·cos(φ) = m·g + (风的垂直分量, 横风=0)
  水平力平衡: T·sin(φ) = F_drag_h(风)   ← 倾角提供水平推力抵消风阻
  → 定点的"平衡倾角"由风速唯一确定。
  但"能耗"取决于 T 和姿态相关 CdA: 倾太小→需更大T硬扛(费); 倾太大→T投影损失(费)。
  完整功耗模型下扫倾角 → U 形, 谷底 = θ*。
"""
import numpy as np
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from quadrotor_dynamics import IRIS, effective_CdA


def full_power(T_newton, roll, pitch, v_rel_norm, params=IRIS):
    """
    完整功耗模型 [W] = 诱导功率 + 型阻功率 + 废阻功率 + 倾角推力修正。

    P_ind  = T·v_i / FoM,  v_i = v_h²/√(v_rel²+v_h²)   (动量理论, translational lift)
    P_pro  = n·C_P0·(T/(mg))^.25 · 经验项                (桨叶型阻)
    P_par  = 0.5·ρ·CdA(φ)·v_rel³                          (机身废阻, 姿态相关迎风面积)
    """
    if T_newton < 0.1:
        return 0.0, 0.0, 0.0
    rho = params['rho']
    v_h = np.sqrt(T_newton / (2.0 * rho * params['total_disk']))
    v_i = v_h**2 / np.sqrt(v_rel_norm**2 + v_h**2 + 1e-6)
    P_ind = T_newton * v_i / params['figure_of_merit']
    P_pro = params['n_rotors'] * 8.0 * (T_newton / (params['mass'] * params['g'])) ** 0.25
    CdA = effective_CdA(roll, pitch, params=params)
    P_par = 0.5 * rho * CdA * v_rel_norm**3
    P_mech = P_ind + P_pro + P_par
    eff = params['motor_eff']
    return P_ind / eff, P_pro / eff, P_par / eff


def sweep_fixed_tilt_freedrift(wind_speed, params=IRIS):
    """
    扫描法: 固定倾角自由漂移到稳态前飞, 测稳态功耗 vs 倾角。
    稳态: 水平推力分量 T·sinφ = 机身阻力 0.5ρ·CdA·v_rel²  → 解出 v_rel, 再算功耗。
    顺风飞行时 v_rel = |wind - v_air|, 这里用迎风定点的等效相对风 = wind_speed。
    """
    m = params['mass']; g = params['g']; W = m * g; rho = params['rho']
    rows = []
    for deg in np.arange(0, 65, 2.5):
        phi = np.radians(deg)
        # 抗风定点: 竖直 T·cosφ = mg
        cos_phi = np.cos(phi)
        if cos_phi < 0.3:
            continue
        T = W / cos_phi
        # 水平推力分量 = T·sinφ, 能抵消的风阻对应一个 v_rel:
        # 这里 v_rel = 风速(定点 v=0). 倾角是否"匹配"风阻决定残余水平力,
        # 但功耗只取决于 T(姿态) 和 v_rel(气流). 抗风定点 v_rel=wind_speed.
        v_rel = wind_speed
        P_ind, P_pro, P_par = full_power(T, phi, 0.0, v_rel, params)
        P_tot = P_ind + P_pro + P_par
        # 该倾角能提供的水平力 vs 实际风阻 (用于标记平衡倾角)
        F_horiz_avail = T * np.sin(phi)
        CdA = effective_CdA(phi, 0.0, params=params)
        F_drag = 0.5 * rho * CdA * v_rel**2 + (3.5 + 0.30) * v_rel  # 二次+线性(plant)
        rows.append((deg, T, P_ind, P_pro, P_par, P_tot, F_horiz_avail, F_drag))
    return rows


def main():
    print("=" * 78)
    print("  M0 原理验证: 抗风定点的能耗-倾角关系 (离线功耗模型)")
    print("  完整功耗 = 诱导 + 型阻 + 废阻(CdA·v_rel³, 姿态相关)")
    print("=" * 78)
    print(f"  Iris: m={IRIS['mass']}kg  hover_T={IRIS['mass']*IRIS['g']:.1f}N  "
          f"CdA_front={IRIS['CdA_front']} CdA_top={IRIS['CdA_top']}")

    best = {}
    for vw in (4.0, 8.0, 12.0):
        print(f"\n{'─'*78}")
        print(f"风速 {vw} m/s  (横风, 抗风定点 v_rel={vw})")
        print(f"{'倾角°':>6} {'T(N)':>7} {'P_ind':>7} {'P_pro':>7} {'P_par':>7} "
              f"{'P_tot(W)':>9} {'F_avail':>8} {'F_drag':>7}")
        rows = sweep_fixed_tilt_freedrift(vw)
        Pmin = 1e9; deg_star = 0
        for (deg, T, Pi, Pp, Ppar, Pt, Fa, Fd) in rows:
            mark = ""
            if Pt < Pmin:
                Pmin = Pt; deg_star = deg
            # 标记平衡倾角(水平力≈风阻)
            bal = "←平衡" if abs(Fa - Fd) < 2.0 else ""
            print(f"{deg:>6.1f} {T:>7.2f} {Pi:>7.1f} {Pp:>7.1f} {Ppar:>7.1f} "
                  f"{Pt:>9.1f} {Fa:>8.1f} {Fd:>7.1f} {bal}")
        best[vw] = (deg_star, Pmin)
        print(f"  → 能耗最优倾角 θ* = {deg_star:.1f}°  (P_min={Pmin:.1f}W)")

    print(f"\n{'='*78}")
    print("  结论: θ* 随风速变化关系")
    print(f"{'风速':>6} {'θ*(°)':>8} {'P_min(W)':>10}")
    for vw, (ds, pm) in best.items():
        print(f"{vw:>6.0f} {ds:>8.1f} {pm:>10.1f}")
    print("\n  若 θ* 随风速单调上升 → 证明需自适应 → RL+MPC 正当性。")
    print("  ⚠️ 若功耗模型仍单调(θ*恒为0或边界), 需先修功耗模型(加 P_par)再训 RL。")


if __name__ == '__main__':
    main()
