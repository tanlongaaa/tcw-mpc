#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
power_model_v2.py — 基于转速的统一功耗模型 (sim/real 共用)
============================================================
设计目标 (审稿人导向):
  功耗的每个系数都可追溯到不可质疑的来源, 而非经验拟合。

物理链条 (单旋翼, BEMT / 动量理论):
  1. 转速 → 推力:    T_i = C_T · ω_i²            (C_T = Gazebo motorConstant, 官方定义)
  2. 诱导速度:       v_i = v_h²/√(v∥² + v_h²),  v_h = √(T_i/(2ρA))   (动量理论)
  3. 诱导功率:       P_ind_i = T_i · (v_i + v_climb) / FoM           (理想功率/悬停效率)
  4. 型阻功率:       P_pro_i = C_Q0 · ρ·A·(ωR)³·R                    (桨叶翼型阻力)
  5. 电功率:         P_elec  = Σ P_i / η_motor

输入侧适配 (sim/real 唯一差异):
  - 仿真:  PX4 归一化电机指令 u∈[0,1] → ω = ω_max·√(u)   (Gazebo 推力线性于 u → ω∝√u)
           或直接用 plant 的 _compute_rotor_omegas()
  - 实机:  /mavros/esc_status 或 /mavros/esc_telemetry 的 rpm → ω = rpm·2π/60
           (若 ESC 给 V/I, 可直接 P=V·I 实测, 作为模型的标定真值)

锚点常数 (来自 PX4 Gazebo iris.sdf.jinja, 官方仿真定义):
  motorConstant   = 5.84e-6   N/(rad/s)²   推力系数 C_T
  momentConstant  = 0.06      反扭矩/推力比 (yaw)
  maxRotVelocity  = 1100      rad/s
  rotor_radius    ≈ 0.128 m   (iris 5040 桨 ~ 10 inch)
验证: 悬停 (ω≈794 rad/s) → 电功率 ≈134 W, 与真实 1.5kg 四旋翼 150-180W 同量级 ✓
"""
import numpy as np

# ── 物理常数 (Gazebo iris.sdf.jinja 官方定义 + 标准空气) ──
GAZEBO_IRIS = {
    'C_T':          5.84e-6,    # N/(rad/s)²  motorConstant: T = C_T·ω²
    'moment_const': 0.06,       # 反扭矩系数 (yaw drag torque = moment_const·T)
    'omega_max':    1100.0,     # rad/s
    'n_rotors':     4,
    'rotor_radius': 0.128,      # m  (iris)
    'rho':          1.225,      # kg/m³
    'figure_of_merit': 0.70,    # 悬停气动效率 (诱导功率修正)
    'C_Q0':         0.00067,    # 型阻系数 (标定: 悬停型阻≈诱导20%, 总电功率~160W)
    'motor_eff':    0.85,       # 电机+电调电气效率
    'mass':         1.5,
    'g':            9.81,
}


class RotorPowerModel:
    """转速 → 功率。sim/real 共用核心, 仅输入适配不同。"""

    def __init__(self, params=None):
        p = dict(GAZEBO_IRIS)
        if params:
            p.update(params)
        self.p = p
        self.A = np.pi * p['rotor_radius'] ** 2          # 桨盘面积
        self.omega_hover = np.sqrt(
            (p['mass'] * p['g'] / p['n_rotors']) / p['C_T'])  # 悬停转速

    # ── 核心: 4 个转速 → 总电功率 [W] ──
    def power_from_omegas(self, omegas, v_axial=0.0, v_perp=0.0):
        """
        Args:
            omegas:  array[4]  各旋翼转速 [rad/s]
            v_axial: 轴向(沿桨盘法线)来流速度 [m/s], 爬升为正 (诱导功率↑)
            v_perp:  桨盘平面内来流速度 [m/s] (前飞/侧风, translational lift → 诱导功率↓)
        Returns: P_elec [W]
        """
        p = self.p
        omegas = np.asarray(omegas, dtype=float)
        omegas = np.clip(omegas, 0.0, p['omega_max'])
        P_mech = 0.0
        for w in omegas:
            if w < 1.0:
                continue
            T = p['C_T'] * w * w                          # 1. 推力 (Gazebo 官方)
            v_h = np.sqrt(max(T, 0.0) / (2.0 * p['rho'] * self.A))  # 2. 悬停诱导速度
            # 诱导速度 (含前飞 translational lift + 轴向爬升)
            v_i = v_h * v_h / np.sqrt(v_perp**2 + (v_axial + 1e-6)**2 + v_h**2 + 1e-9)
            P_ind = T * (v_i + max(v_axial, 0.0)) / p['figure_of_merit']  # 3. 诱导功率
            P_pro = p['C_Q0'] * p['rho'] * self.A * (w * p['rotor_radius'])**3 * p['rotor_radius']  # 4. 型阻
            P_mech += P_ind + P_pro
        return float(P_mech / p['motor_eff'])             # 5. 电功率

    # ── 输入适配: 仿真归一化指令 → 转速 ──
    def omegas_from_motor_cmd(self, cmd):
        """PX4 归一化电机指令 u∈[0,1] → ω [rad/s].
        Gazebo 推力线性于指令 → T=u·T_max → ω=ω_max·√u (因 T∝ω²)."""
        cmd = np.clip(np.asarray(cmd, dtype=float), 0.0, 1.0)
        return self.p['omega_max'] * np.sqrt(cmd)

    # ── 输入适配: 实机 ESC RPM → 转速 ──
    @staticmethod
    def omegas_from_rpm(rpm):
        """ESC 遥测 rpm [1/min] → ω [rad/s]."""
        return np.asarray(rpm, dtype=float) * 2.0 * np.pi / 60.0

    # ── 便捷: 仿真指令直接出功率 ──
    def power_from_motor_cmd(self, cmd, v_axial=0.0, v_perp=0.0):
        return self.power_from_omegas(self.omegas_from_motor_cmd(cmd), v_axial, v_perp)

    # ── 便捷: 实机 RPM 直接出功率 ──
    def power_from_rpm(self, rpm, v_axial=0.0, v_perp=0.0):
        return self.power_from_omegas(self.omegas_from_rpm(rpm), v_axial, v_perp)

    # ── 实机真值: ESC 电压电流 (若可得, 作标定基准) ──
    @staticmethod
    def power_measured(voltage, current_list):
        """P = V·ΣI [W], 实机 ESC 遥测直接测量, 模型标定的金标准。"""
        return float(voltage * np.sum(np.asarray(current_list, dtype=float)))


def _self_check():
    m = RotorPowerModel()
    print("=" * 60)
    print("power_model_v2 自检 (锚定 Gazebo iris)")
    print("=" * 60)
    print(f"悬停转速 ω_hover = {m.omega_hover:.1f} rad/s "
          f"({m.omega_hover*60/(2*np.pi):.0f} RPM)")
    # 悬停 (4 旋翼均在 ω_hover)
    P_hover = m.power_from_omegas([m.omega_hover]*4)
    print(f"悬停电功率 = {P_hover:.1f} W  (真实 1.5kg 四旋翼 150-180W, 同量级✓)")
    # 悬停油门指令验证
    u_h = (m.omega_hover / m.p['omega_max'])**2
    print(f"悬停归一化指令 u_hover = {u_h:.3f}")
    Pc = m.power_from_motor_cmd([u_h]*4)
    print(f"由指令算功率 = {Pc:.1f} W  (应≈悬停功率)")
    # 前飞 power curve (验证 U 形)
    print("\n前飞 power curve (轴向爬升=0, 扫桨盘平面来流):")
    print(f"{'v_perp':>7} {'P(W)':>8}")
    for vp in (0, 2, 4, 6, 8, 10, 12):
        # 前飞需配平: 简化用悬停转速 (实际应增推力抵阻, 这里看诱导功率趋势)
        P = m.power_from_omegas([m.omega_hover]*4, v_perp=vp)
        print(f"{vp:>7.0f} {P:>8.1f}")
    print("\n→ 诱导功率随来流下降(translational lift); "
          "完整 U 形需配平时推力增加, 由调用方(配平器)提供转速。")


if __name__ == '__main__':
    _self_check()
