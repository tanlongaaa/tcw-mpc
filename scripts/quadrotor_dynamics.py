#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
quadrotor_dynamics.py — 完整四旋翼非线性动力学
================================================
用于 SL-MPC: 全非线性模型 + 解析雅可比 + BEMT 功率

模型层级:
  1. 刚体动力学: 全 3D 旋转矩阵 (ZYX Euler, 无小角度假设)
  2. 气动阻力: 姿态相关 CdA(α), 相对风速
  3. 功率: 叶素动量理论 (BEMT) — 诱导 + 型阻 + 效率

参考文献:
  - Mahony et al., "Multirotor Aerial Vehicles", 2012
  - Bangura & Mahony, "Nonlinear Dynamic Modeling", 2014
  - Leishman, "Principles of Helicopter Aerodynamics", 2006
"""

import numpy as np


# ══════════════════════════════════════════════════════════
# 物理常数
# ══════════════════════════════════════════════════════════

# Iris 四旋翼参数
IRIS_PARAMS = {
    'mass': 1.5,             # kg
    'g': 9.81,               # m/s²
    'hover_thrust': 0.66,    # 归一化悬停推力
    'max_thrust': 22.3,      # N (hover_thrust * thrust_scale)
    
    # 空气动力学
    'rho': 1.225,            # kg/m³
    'CdA_front': 0.045,      # m² 前向迎风面积 (水平姿态)
    'CdA_top': 0.12,         # m² 顶部迎风面积 (90°倾角)
    'CdA_side': 0.05,        # m² 侧向迎风面积
    
    # BEMT 旋翼参数 (10" 桨)
    'n_rotors': 4,
    'rotor_radius': 0.127,   # m (5")
    'disk_area': 0.0507,     # m² per rotor (πR²)
    'C_P0': 0.012,           # 型阻功率系数
    'figure_of_merit': 0.7,  # 悬停效率
    'motor_eff': 0.85,       # 电机+电调效率
}

# 推导参数
def derived_params(p):
    p['thrust_scale'] = p['mass'] * p['g'] / p['hover_thrust']
    p['k_t'] = p['g'] / p['hover_thrust']  # 归一化推力→加速度
    p['total_disk'] = p['n_rotors'] * p['disk_area']
    p['k_drag_base'] = 0.5 * p['rho'] / p['mass']  # ½ρ/m, 乘以 CdA×|V|×V
    return p

IRIS = derived_params(IRIS_PARAMS.copy())


# ══════════════════════════════════════════════════════════
# 1. 姿态相关气动阻力 CdA(φ, θ, ψ)
# ══════════════════════════════════════════════════════════

def effective_CdA(roll, pitch, yaw=0.0, params=None):
    """
    姿态相关的有效阻力面积
    
    原理: 无人机倾斜时，螺旋桨盘面部分暴露在来流中，
         有效截面积 = 机体截面积在来流方向的投影
    
    简化椭圆模型:
      CdA(α, β) = CdA_front·|cos α·cos β| 
                 + CdA_side·|sin β|
                 + CdA_top·(1 - |cos α·cos β|)
      
      其中 α = pitch, β = roll (相对来流的角度)
    
    返回: 有效 CdA [m²]
    """
    if params is None:
        params = IRIS
    
    # 投影因子
    cos_proj = abs(np.cos(pitch) * np.cos(roll))
    sin_proj = abs(np.sin(roll))  # 侧滑分量
    
    # 前向/顶部插值
    CdA = (params['CdA_front'] * cos_proj 
           + params['CdA_top'] * (1.0 - cos_proj)
           + params['CdA_side'] * sin_proj)
    
    return CdA


# ══════════════════════════════════════════════════════════
# 2. 全非线性动力学 ẋ = f(x, u, w)
# ══════════════════════════════════════════════════════════

def quadrotor_dynamics(x, u, wind_enu, params=None):
    """
    完整四旋翼非线性动力学 (ENU 系, 连续时间)
    
    状态 x = [px, py, pz, vx, vy, vz]  (ENU: x=东 y=北 z=上)
    控制 u = [φ, θ, T_norm]  (机体: roll, pitch, 归一化推力)
    风   wind_enu = [we, wn, wu] (ENU)
    
    返回: ẋ = [vx, vy, vz, ax, ay, az]
    
    动力学:
      F_thrust_body = [0, 0, T]  (体轴系: z_b 向下, 推力向上 = -z_b → [0, 0, T] in 体轴)
      
      旋转矩阵 R = Rz(ψ)·Ry(θ)·Rx(φ), ψ≈0:
        R ≈ Ry(θ)·Rx(φ)
          = [[cθ,  sθ·sφ,  sθ·cφ],
             [0,   cφ,     -sφ],
             [-sθ, cθ·sφ,  cθ·cφ]]
      
      F_thrust_ENU = R · [0, 0, T]
        Fx = T · sin(θ) · cos(φ)
        Fy = -T · sin(φ)
        Fz = T · cos(θ) · cos(φ)
      
      F_drag = -½ρ·CdA(φ,θ)·|V_rel|·V_rel
      
      a = (F_thrust + F_drag)/m + [0, 0, -g]
    """
    if params is None:
        params = IRIS
    
    # 状态分量
    vx, vy, vz = x[3], x[4], x[5]
    roll, pitch = u[0], u[1]
    thrust_norm = u[2]
    
    # 推力 [N]
    T_newton = thrust_norm * params['thrust_scale']
    
    # ── 全非线性旋转矩阵 (无小角度近似!) ──
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    
    Fx_thrust = T_newton * sp * cr      # = T·sin(θ)·cos(φ)
    Fy_thrust = -T_newton * sr          # = -T·sin(φ)
    Fz_thrust = T_newton * cp * cr      # = T·cos(θ)·cos(φ)
    
    # ── 气动阻力 (姿态相关 CdA) ──
    CdA = effective_CdA(roll, pitch, params=params)
    
    V_rel = np.array([vx - wind_enu[0], 
                      vy - wind_enu[1], 
                      vz - wind_enu[2]])
    V_rel_norm = np.linalg.norm(V_rel)
    
    if V_rel_norm > 0.1:
        F_drag = -0.5 * params['rho'] * CdA * V_rel_norm * V_rel
    else:
        F_drag = np.zeros(3)
    
    m = params['mass']
    ax = Fx_thrust / m + F_drag[0] / m
    ay = Fy_thrust / m + F_drag[1] / m
    az = Fz_thrust / m + F_drag[2] / m - params['g']
    
    return np.array([vx, vy, vz, ax, ay, az])


# ══════════════════════════════════════════════════════════
# 3. 离散时间 + 解析雅可比 (用于 SL-MPC)
# ══════════════════════════════════════════════════════════

def discrete_linearize(x_op, u_op, wind, dt, params=None):
    """
    在操作点 (x_op, u_op) 处线性化 + 离散化
    
    A = I + ∂f/∂x · dt  (前向欧拉)
    B = ∂f/∂u · dt
    
    返回: A (6×6), B (6×3), g_offset (6,) 偏移项
    """
    if params is None:
        params = IRIS
    
    # 操作点分量
    roll, pitch = u_op[0], u_op[1]
    T_norm = u_op[2]
    T = T_norm * params['thrust_scale']
    vx, vy, vz = x_op[3], x_op[4], x_op[5]
    
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    m = params['mass']
    
    # ── 连续时间雅可比 ∂f/∂x ──
    # 位置对速度: 导数 = I (已在 A=I 中)
    # 速度对位置: 0 (无直接依赖)
    # 速度对速度: 来自气动阻尼
    
    # 气动阻尼 (线性化): F_drag ≈ -½ρ·CdA·|V0|·V (对 V 的一阶近似)
    # 在操作点 V_op = x_op[3:6] - wind 处
    V_op = x_op[3:6] - wind
    V_mag = np.linalg.norm(V_op)
    CdA = effective_CdA(roll, pitch, params=params)
    
    if V_mag > 0.1:
        # ∂(F_drag)/∂V = -½ρ·CdA·(|V|·I + V·V^T/|V|)
        k_damp = 0.5 * params['rho'] * CdA / m
        J_drag_v = -k_damp * (V_mag * np.eye(3) + np.outer(V_op, V_op) / max(V_mag, 0.01))
    else:
        J_drag_v = np.zeros((3, 3))
    
    # A 连续时间 = [[0, I], [0, J_drag_v]]
    A_cont = np.zeros((6, 6))
    A_cont[0:3, 3:6] = np.eye(3)
    A_cont[3:6, 3:6] = J_drag_v
    
    # 前向欧拉离散化
    A_disc = np.eye(6) + A_cont * dt
    
    # ── 连续时间雅可比 ∂f/∂u ──
    # u = [roll, pitch, thrust_norm]
    # 使用 thrust_scale 将归一化推力转牛顿
    k_s = params['thrust_scale']  # N per normalized thrust
    
    B_cont = np.zeros((6, 3))
    
    # ∂ax/∂roll, ∂ax/∂pitch, ∂ax/∂thrust
    B_cont[3, 0] = T * sp * (-sr) / m                    # = -T·sin(θ)·sin(φ)/m
    B_cont[3, 1] = T * cp * cr / m                        # = T·cos(θ)·cos(φ)/m
    B_cont[3, 2] = k_s * sp * cr / m                      # = k_s·sin(θ)·cos(φ)/m
    
    # ∂ay/∂roll, ∂ay/∂pitch, ∂ay/∂thrust
    B_cont[4, 0] = -T * cr / m                            # = -T·cos(φ)/m
    B_cont[4, 1] = 0.0
    B_cont[4, 2] = -k_s * sr / m                          # = -k_s·sin(φ)/m
    
    # ∂az/∂roll, ∂az/∂pitch, ∂az/∂thrust
    B_cont[5, 0] = T * cp * (-sr) / m                     # = -T·cos(θ)·sin(φ)/m
    B_cont[5, 1] = -T * sp * cr / m                       # = -T·sin(θ)·cos(φ)/m  
    B_cont[5, 2] = k_s * cp * cr / m                      # = k_s·cos(θ)·cos(φ)/m
    
    B_disc = B_cont * dt
    
    # ── 偏移项 ──
    # g_offset 主要来自重力: [0,0,0, 0,0,-g*dt]
    g_offset = np.zeros(6)
    g_offset[5] = -params['g'] * dt
    
    # 附加模型非线性残差 (操作点处 f(x,u) - A·x - B·u)
    f_op = quadrotor_dynamics(x_op, u_op, wind, params)
    f_lin = A_cont @ x_op
    f_lin[3:6] += B_cont[3:6, :] @ u_op
    residual = f_op - f_lin
    g_offset += residual * dt
    
    return A_disc, B_disc, g_offset


# ══════════════════════════════════════════════════════════
# 4. BEMT 功率模型
# ══════════════════════════════════════════════════════════

def bem_power(thrust_norm, roll=0.0, pitch=0.0, v_rel_norm=0.0, params=None):
    """
    叶素动量功率模型 [W]
    
    P_total = P_induced + P_profile + P_parasite
    
    P_induced = T · v_i / FoM           [理想诱导功率/效率]
    P_profile = N · ρ · A · (ΩR)³ · C_P0 [型阻功率]
    P_parasite = (P_ind + P_pro) · (1/η - 1)  [电机+电调损耗]
    
    输入:
      thrust_norm: 归一化推力 [0, 1]
      roll, pitch: 姿态角 (计算 cos 倾斜修正)
      v_rel_norm:  相对风速模长 (前飞时诱导速度减小)
    
    返回: P_total [W]
    """
    if params is None:
        params = IRIS
    
    # 推力 [N], 含倾斜修正
    cos_tilt = np.cos(roll) * np.cos(pitch)
    T = thrust_norm * params['thrust_scale'] / max(cos_tilt, 0.5)
    
    if T < 0.1:
        return 0.0
    
    # ── 诱导功率 ──
    # 悬停诱导速度: v_h = √(T/(2ρA))
    v_h = np.sqrt(T / (2.0 * params['rho'] * params['total_disk']))
    
    # 前飞修正 (Glauert): v_i = v_h² / √(V² + v_h²)
    # 降低前飞时的诱导功率
    v_i = v_h**2 / np.sqrt(v_rel_norm**2 + v_h**2 + 1e-6)
    
    P_ind = T * v_i / params['figure_of_merit']
    
    # ── 型阻功率 ──
    # 估算旋翼转速: ΩR ≈ √(T/(4ρA))  (hover)
    # P_profile ∝ Ω³ — 推力无关的常值分量
    # 简化: 每桨 ~8W 在悬停
    P_pro = params['n_rotors'] * 8.0 * (T / (params['mass'] * params['g'])) ** 0.25
    
    # ── 电机+电调损耗 ──
    P_mech = P_ind + P_pro
    P_total = P_mech / params['motor_eff']
    
    return float(P_total)


# ══════════════════════════════════════════════════════════
# 5. 风扰计算 (基于非线性模型)
# ══════════════════════════════════════════════════════════

def compute_wind_disturbance(x, u, wind_enu, params=None):
    """
    将风效应拆分为: 
      d_aero = 纯风推力 (进入 d_seq)
      阻尼部分由 A 矩阵的 J_drag_v 处理
    
    V_drone → 已有速度, A 矩阵通过 J_drag_v 处理阻尼
    V_wind → 风场速度, 产生额外推力 (进入 d_seq)
    
    分解: F_aero(V_rel) = F_aero(-V_wind) + [F_aero(V_rel) - F_aero(-V_wind)]
          风推力(d_seq)         阻尼(A矩阵)
    
    简化: d_aero = ½ρ·CdA·|W|·W  (风推力, 方向=风向)
    阻尼由 J_drag_v 处理
    """
    if params is None:
        params = IRIS
    
    w = np.asarray(wind_enu)
    w_mag = np.linalg.norm(w)
    
    if w_mag < 0.1:
        return np.zeros(6)
    
    # 姿态相关 CdA
    roll, pitch = u[0], u[1]
    CdA = effective_CdA(roll, pitch, params=params)
    
    # 风推力加速度
    a_wind = 0.5 * params['rho'] * CdA * w_mag * w / params['mass']
    
    d = np.zeros(6)
    d[3:6] = a_wind  # 连续时间加速度, MPC solve 中乘 dt
    return d
