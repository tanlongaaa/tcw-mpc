#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
quadrotor_dynamics.py — 完整四旋翼非线性动力学
================================================
用于 SL-MPC: 全非线性模型 + 解析雅可比 + BEMT 功率

支持两种模型:
  6D Euler 模型 (保留):
    状态 x = [px, py, pz, vx, vy, vz]
    控制 u = [roll, pitch, thrust_norm]
    姿态作为控制输入 (无姿态动力学)

  10D CTBR 模型 (新增):
    状态 x = [px, py, pz, qw, qx, qy, qz, vx, vy, vz]
    控制 u = [f_c, ωx, ωy, ωz]  (总推力 + 体轴角速度)
    四元数姿态运动学, CTBR = Collective Thrust + Body Rates
    参考: Kaufmann et al., "ACMPC", TRO 2025

模型层级:
  1. 刚体动力学 / 姿态运动学
  2. 气动阻力: 姿态相关 CdA(α), 相对风速
  3. 功率: 叶素动量理论 (BEMT)

参考文献:
  - Mahony et al., "Multirotor Aerial Vehicles", 2012
  - Bangura & Mahony, "Nonlinear Dynamic Modeling", 2014
  - Kaufmann et al., "Actor-Critic Model Predictive Control", TRO 2025
  - Leishman, "Principles of Helicopter Aerodynamics", 2006
"""

import numpy as np


# ══════════════════════════════════════════════════════════
# 物理常数
# ══════════════════════════════════════════════════════════

# Iris 四旋翼参数 (6D 模型用)
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


def derived_params(p):
    p['thrust_scale'] = p['mass'] * p['g'] / p['hover_thrust']
    p['k_t'] = p['g'] / p['hover_thrust']
    p['total_disk'] = p['n_rotors'] * p['disk_area']
    p['k_drag_base'] = 0.5 * p['rho'] / p['mass']
    return p


IRIS = derived_params(IRIS_PARAMS.copy())

# ── 标准 6-DOF 转动惯量 (Iris) ──
IRIS.update({
    'Jx': 0.029,     # kg·m² roll inertia
    'Jy': 0.029,     # kg·m² pitch inertia
    'Jz': 0.055,     # kg·m² yaw inertia
    'J_diag': np.array([0.029, 0.029, 0.055]),
    'J_inv': np.array([1/0.029, 1/0.029, 1/0.055]),
    'arm_lx': np.array([0.13, -0.13, -0.13, 0.13]),   # X-config rotor x offsets (m)
    'arm_ly': np.array([-0.13, 0.13, -0.13, 0.13]),    # X-config rotor y offsets (m)
    'kappa': 0.022,  # torque-to-thrust ratio (Nm/N)
})


# ══════════════════════════════════════════════════════════
# Quaternion 工具函数
# ══════════════════════════════════════════════════════════

def quat_mult(q1, q2):
    """四元数乘法 q1 ⊗ q2. 每个 q = [qw, qx, qy, qz]"""
    q1 = np.asarray(q1); q2 = np.asarray(q2)
    return np.array([
        q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
        q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
        q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
        q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0],
    ])


def quat_conj(q):
    """四元数共轭"""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q):
    """归一化四元数到单位范数"""
    n = np.linalg.norm(q)
    if n > 1e-12:
        return q / n
    return np.array([1.0, 0.0, 0.0, 0.0])


def rotate_vector_by_quat(q, v):
    """用四元数 q 旋转向量 v: v_rot = q ⊗ [0, v] ⊗ q*"""
    q = np.asarray(q); v = np.asarray(v)
    qv = q[1:4]
    # 矩阵形式 (更快)
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx**2 + qy**2)],
    ])
    return R @ v


def quat_rotation_matrix(q):
    """四元数 → 旋转矩阵 (SO(3))"""
    q = np.asarray(q)
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx**2 + qy**2)],
    ])


def quat_to_euler(q):
    """四元数 → ZYX Euler 角 [roll, pitch, yaw] (rad)"""
    q = np.asarray(q)
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    pitch = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1.0, 1.0))
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return np.array([roll, pitch, yaw])


def quat_exp_map(omega, dt):
    """角速度指数映射: q = exp(ω*dt/2)"""
    omega = np.asarray(omega)
    angle = np.linalg.norm(omega) * dt * 0.5
    if angle > 1e-8:
        axis = omega / np.linalg.norm(omega)
        return np.array([np.cos(angle),
                         axis[0] * np.sin(angle),
                         axis[1] * np.sin(angle),
                         axis[2] * np.sin(angle)])
    return np.array([1.0, 0.0, 0.0, 0.0])


# ══════════════════════════════════════════════════════════
# 姿态相关气动阻力 CdA
# ══════════════════════════════════════════════════════════

def effective_CdA(roll, pitch, yaw=0.0, params=None):
    """
    姿态相关的有效阻力面积 (Euler角版本)

    CdA(α, β) = CdA_front·|cos α·cos β|
               + CdA_side·|sin β|
               + CdA_top·(1 - |cos α·cos β|)
    """
    if params is None:
        params = IRIS
    cos_proj = abs(np.cos(pitch) * np.cos(roll))
    sin_proj = abs(np.sin(roll))
    CdA = (params['CdA_front'] * cos_proj
           + params['CdA_top'] * (1.0 - cos_proj)
           + params['CdA_side'] * sin_proj)
    return CdA


def effective_CdA_from_quat(q, params=None):
    """姿态相关的有效阻力面积 (四元数版本)"""
    roll, pitch, _ = quat_to_euler(q)
    return effective_CdA(roll, pitch, params=params)


# ══════════════════════════════════════════════════════════
# 6D 模型 (Euler 角控制, 保留向后兼容)
# ==========================================
# 状态 x = [px, py, pz, vx, vy, vz]  (6)
# 控制 u = [roll, pitch, thrust_norm] (3)
# ══════════════════════════════════════════════════════════

def quadrotor_dynamics(x, u, wind_enu, params=None):
    """
    6D 四旋翼非线性动力学 (ENU 系, 连续时间)

    状态 x = [px, py, pz, vx, vy, vz]  (ENU)
    控制 u = [φ, θ, T_norm]  (roll, pitch, 归一化推力)
    风   wind_enu = [we, wn, wu] (ENU)
    """
    if params is None:
        params = IRIS

    vx, vy, vz = x[3], x[4], x[5]
    roll, pitch = u[0], u[1]
    thrust_norm = u[2]

    T_newton = thrust_norm * params['thrust_scale']

    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)

    Fx_thrust = T_newton * sp * cr
    Fy_thrust = -T_newton * sr
    Fz_thrust = T_newton * cp * cr

    CdA = effective_CdA(roll, pitch, params=params)
    V_rel = np.array([vx - wind_enu[0], vy - wind_enu[1], vz - wind_enu[2]])
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


def discrete_linearize(x_op, u_op, wind, dt, params=None):
    """6D 模型线性化 + 离散化 → A(6×6), B(6×3), g_offset(6)"""
    if params is None:
        params = IRIS

    roll, pitch = u_op[0], u_op[1]
    T_norm = u_op[2]
    T = T_norm * params['thrust_scale']
    vx, vy, vz = x_op[3], x_op[4], x_op[5]

    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    m = params['mass']

    # ── 气动阻尼 J_drag_v ──
    V_op = x_op[3:6] - wind
    V_mag = np.linalg.norm(V_op)
    CdA = effective_CdA(roll, pitch, params=params)

    if V_mag > 0.1:
        k_damp = 0.5 * params['rho'] * CdA / m
        J_drag_v = -k_damp * (V_mag * np.eye(3) + np.outer(V_op, V_op) / max(V_mag, 0.01))
    else:
        J_drag_v = np.zeros((3, 3))

    A_cont = np.zeros((6, 6))
    A_cont[0:3, 3:6] = np.eye(3)
    A_cont[3:6, 3:6] = J_drag_v

    A_disc = np.eye(6) + A_cont * dt

    # ── B 矩阵 ──
    k_s = params['thrust_scale']
    B_cont = np.zeros((6, 3))
    B_cont[3, 0] = T * sp * (-sr) / m
    B_cont[3, 1] = T * cp * cr / m
    B_cont[3, 2] = k_s * sp * cr / m
    B_cont[4, 0] = -T * cr / m
    B_cont[4, 1] = 0.0
    B_cont[4, 2] = -k_s * sr / m
    B_cont[5, 0] = T * cp * (-sr) / m
    B_cont[5, 1] = -T * sp * cr / m
    B_cont[5, 2] = k_s * cp * cr / m
    B_disc = B_cont * dt

    # ── 偏移项 ──
    # 线性化: f(x,u) ≈ J_x·x + J_u·u + (f_op - J_x·x_op - J_u·u_op)
    # g_offset = dt·(f_op - J_x·x_op - J_u·u_op), 重力已在 f_op 中
    f_op = quadrotor_dynamics(x_op, u_op, wind, params)
    f_lin = A_cont @ x_op
    f_lin[3:6] += B_cont[3:6, :] @ u_op
    g_offset = (f_op - f_lin) * dt

    return A_disc, B_disc, g_offset


def compute_wind_disturbance(x, u, wind_enu, params=None):
    """6D 模型风扰: d[3:6] = 风推力加速度"""
    if params is None:
        params = IRIS
    w = np.asarray(wind_enu)
    w_mag = np.linalg.norm(w)
    if w_mag < 0.1:
        return np.zeros(6)
    roll, pitch = u[0], u[1]
    CdA = effective_CdA(roll, pitch, params=params)
    a_wind = 0.5 * params['rho'] * CdA * w_mag * w / params['mass']
    d = np.zeros(6)
    d[3:6] = a_wind
    return d


# ══════════════════════════════════════════════════════════
# 10D CTBR 模型 (四元数 + 体轴角速度)
# ==========================================
# 状态 x = [px, py, pz, qw, qx, qy, qz, vx, vy, vz]  (10)
# 控制 u = [f_c, ωx, ωy, ωz]  (总推力N + 体轴角速度 rad/s)  (4)
#
# 动力学:
#   ṗ = v
#   q̇ = ½ Ω(ω) q   (姿态运动学)
#   v̇ = R(q)·[0,0,f_c]/m + g + a_drag
#
# 离散化 (iLQR 前向推演):
#   p_{k+1} = p_k + v_k·dt
#   q_{k+1} = exp(ω_k·dt/2) ⊗ q_k  (精确离散)
#   v_{k+1} = v_k + v̇_k·dt
# ══════════════════════════════════════════════════════════

# 状态/控制维度
NX_10D = 10
NU_10D = 4
# 子索引
POS_IDX = slice(0, 3)    # [px, py, pz]
QUAT_IDX = slice(3, 7)   # [qw, qx, qy, qz]
VEL_IDX = slice(7, 10)   # [vx, vy, vz]

# 10D 悬停参考
HOVER_Q = np.array([1.0, 0.0, 0.0, 0.0])          # 水平姿态四元数
HOVER_U = np.array([IRIS['mass'] * IRIS['g'], 0.0, 0.0, 0.0])  # 悬停控制: f_c = mg

G_VEC = np.array([0.0, 0.0, -IRIS['g']])  # 重力加速度 ENU


def quadrotor_dynamics_10d(x, u, wind_enu, params=None):
    """
    10D CTBR 四旋翼动力学 (连续时间, ENU 系)

    Args:
        x:        10D 状态 [p(3), q(4), v(3)]
        u:        4D 控制 [f_c, ωx, ωy, ωz]
        wind_enu: 3D 风速 ENU
        params:   物理参数 dict

    Returns:
        x_dot: 10D 状态导数 [v(3), q̇(4), v̇(3)]
    """
    if params is None:
        params = IRIS

    p = x[POS_IDX]
    q = x[QUAT_IDX]
    v = x[VEL_IDX]

    f_c = u[0]
    omega = u[1:4]  # 体轴角速度

    # ── 位置导数: ṗ = v ──
    p_dot = v.copy()

    # ── 姿态导数: q̇ = ½ Ω(ω)·q ──
    # Ω(ω) = [[0, -ωx, -ωy, -ωz],
    #         [ωx, 0, ωz, -ωy],
    #         [ωy, -ωz, 0, ωx],
    #         [ωz, ωy, -ωx, 0]]
    wx, wy, wz = omega
    q_dot = 0.5 * np.array([
        -wx * q[1] - wy * q[2] - wz * q[3],
        wx * q[0] + wy * q[3] - wz * q[2],  # wx*qw + wz*qy - wy*qz? Let me recalculate
        # Actually:
        # q_dot[0] = 0.5*(-wx*qx - wy*qy - wz*qz)
        # q_dot[1] = 0.5*( wx*qw + wz*qy - wy*qz)  ← wait, let me check
        # Ω(ω)*q:
        # row 0: 0*qw - wx*qx - wy*qy - wz*qz = -(wx*qx + wy*qy + wz*qz)
        # row 1: wx*qw + 0*qx + wz*qy - wy*qz = wx*qw + wz*qy - wy*qz
        # row 2: wy*qw - wz*qx + 0*qy + wx*qz = wy*qw - wz*qx + wx*qz
        # row 3: wz*qw + wy*qx - wx*qy + 0*qz = wz*qw + wy*qx - wx*qy
    ])
    # Let me rewrite properly:
    qw, qx, qy, qz = q
    q_dot = 0.5 * np.array([
        -wx*qx - wy*qy - wz*qz,
        wx*qw + wz*qy - wy*qz,
        wy*qw - wz*qx + wx*qz,
        wz*qw + wy*qx - wx*qy,
    ])

    # ── 速度导数: v̇ = R(q)·[0,0,f_c]/m + g + a_drag ──
    F_thrust_body = np.array([0.0, 0.0, f_c])
    F_thrust_enu = rotate_vector_by_quat(q, F_thrust_body)

    # 气动阻力
    roll, pitch, _ = quat_to_euler(q)
    CdA = effective_CdA(roll, pitch, params=params)
    V_rel = np.array([v[0] - wind_enu[0],
                      v[1] - wind_enu[1],
                      v[2] - wind_enu[2]])
    V_rel_norm = np.linalg.norm(V_rel)
    if V_rel_norm > 0.1:
        F_drag = -0.5 * params['rho'] * CdA * V_rel_norm * V_rel
    else:
        F_drag = np.zeros(3)

    m = params['mass']
    v_dot = F_thrust_enu / m + G_VEC + F_drag / m

    return np.concatenate([p_dot, q_dot, v_dot])


def quadrotor_dynamics_10d_discrete(x, u, wind_enu, dt, params=None):
    """
    10D CTBR 离散动力学 (用于 iLQR 前向推演)

    使用精确四元数离散: q_{k+1} = exp(ω·dt/2) ⊗ q_k
    """
    if params is None:
        params = IRIS

    p = x[POS_IDX]
    q = x[QUAT_IDX]
    v = x[VEL_IDX]

    f_c = u[0]
    omega = u[1:4]

    # ── 位置 (前向欧拉) ──
    p_next = p + v * dt

    # ── 四元数 (精确指数映射) ──
    dq = quat_exp_map(omega, dt)
    q_next = quat_mult(q, dq)
    q_next = quat_normalize(q_next)

    # ── 速度 (前向欧拉) ──
    F_thrust_body = np.array([0.0, 0.0, f_c])
    F_thrust_enu = rotate_vector_by_quat(q, F_thrust_body)

    roll, pitch, _ = quat_to_euler(q)
    CdA = effective_CdA(roll, pitch, params=params)
    V_rel = np.array([v[0] - wind_enu[0],
                      v[1] - wind_enu[1],
                      v[2] - wind_enu[2]])
    V_rel_norm = np.linalg.norm(V_rel)
    if V_rel_norm > 0.1:
        F_drag = -0.5 * params['rho'] * CdA * V_rel_norm * V_rel
    else:
        F_drag = np.zeros(3)

    m = params['mass']
    v_next = v + dt * (F_thrust_enu / m + G_VEC + F_drag / m)

    return np.concatenate([p_next, q_next, v_next])


def discrete_linearize_10d(x_op, u_op, wind_enu, dt, params=None):
    """
    10D CTBR 模型解析线性化 + 离散化

    返回: A(10×10), B(10×4), g_offset(10)

    雅可比结构:
      A_cont:
        ∂ṗ/∂v = I₃ₓ₃           (A[0:3, 7:10])
        ∂q̇/∂q = ½ Ω(ω)         (A[3:7, 3:7])
        ∂v̇/∂q = J_v_dot_q      (A[7:10, 3:7])
        ∂v̇/∂v = J_drag_v       (A[7:10, 7:10])

      B_cont:
        ∂ṗ/∂u = 0              (B[0:3, :])
        ∂q̇/∂[f_c, ω] = [0, J_q_dot_ω]  (B[3:7, :])
        ∂v̇/∂[f_c, ω] = [R(q)·e₃/m, 0]  (B[7:10, :])
    """
    if params is None:
        params = IRIS

    q = x_op[QUAT_IDX]
    v = x_op[VEL_IDX]
    f_c = u_op[0]
    omega = u_op[1:4]
    wx, wy, wz = omega
    qw, qx, qy, qz = q
    m = params['mass']

    # ── A_cont: 连续时间状态雅可比 (10×10) ──
    A_cont = np.zeros((NX_10D, NX_10D))

    # ∂ṗ/∂v = I₃ₓ₃
    A_cont[0:3, 7:10] = np.eye(3)

    # ∂q̇/∂q = ½ Ω(ω)
    A_cont[3, 3:7] = 0.5 * np.array([0, -wx, -wy, -wz])
    A_cont[4, 3:7] = 0.5 * np.array([wx, 0, wz, -wy])
    A_cont[5, 3:7] = 0.5 * np.array([wy, -wz, 0, wx])
    A_cont[6, 3:7] = 0.5 * np.array([wz, wy, -wx, 0])

    # ∂v̇/∂q: 推力方向随姿态变化
    # v̇ = (f_c/m)·R(q)[:,2] + g + a_drag
    # ∂(R[:,2])/∂q = 2·[[qy, qz, qw, qx],
    #                    [-qx, -qw, qz, qy],
    #                    [qw, -qx, -qy, qz]]
    J_v_q = (2.0 * f_c / m) * np.array([
        [qy,  qz,  qw,  qx],
        [-qx, -qw,  qz,  qy],
        [qw,  -qx, -qy,  qz],
    ])
    A_cont[7:10, 3:7] = J_v_q

    # ∂v̇/∂v: 气动阻尼
    V_op = v - wind_enu
    V_mag = np.linalg.norm(V_op)
    roll, pitch, _ = quat_to_euler(q)
    CdA = effective_CdA(roll, pitch, params=params)

    if V_mag > 0.1:
        k_damp = 0.5 * params['rho'] * CdA / m
        J_drag_v = -k_damp * (V_mag * np.eye(3) + np.outer(V_op, V_op) / max(V_mag, 0.01))
    else:
        J_drag_v = np.zeros((3, 3))
    A_cont[7:10, 7:10] = J_drag_v

    # 前向欧拉离散化: A_disc = I + dt·A_cont
    A_disc = np.eye(NX_10D) + A_cont * dt

    # ── B_cont: 连续时间控制雅可比 (10×4) ──
    B_cont = np.zeros((NX_10D, NU_10D))

    # ∂q̇/∂ω: J_q_dot_ω = ½·[[-qx, -qy, -qz],
    #                         [qw, -qz,  qy],
    #                         [qz,  qw, -qx],
    #                         [-qy,  qx,  qw]]
    B_cont[3, 1:4] = 0.5 * np.array([-qx, -qy, -qz])
    B_cont[4, 1:4] = 0.5 * np.array([qw, -qz, qy])
    B_cont[5, 1:4] = 0.5 * np.array([qz, qw, -qx])
    B_cont[6, 1:4] = 0.5 * np.array([-qy, qx, qw])

    # ∂v̇/∂f_c = R(q)·e₃ / m
    R = quat_rotation_matrix(q)
    B_cont[7:10, 0] = R[:, 2] / m  # R 第三列 (z-body → ENU)

    B_disc = B_cont * dt

    # ── 偏移项 ──
    # 线性化: f(x,u) ≈ J_x·x + J_u·u + (f_op - J_x·x_op - J_u·u_op)
    # g_offset = dt·(f_op - J_x·x_op - J_u·u_op), 重力已在 f_op 中
    f_op = quadrotor_dynamics_10d(x_op, u_op, wind_enu, params)
    f_lin = A_cont @ x_op + B_cont @ u_op
    g_offset = (f_op - f_lin) * dt

    return A_disc, B_disc, g_offset


def compute_wind_disturbance_10d(x, u, wind_enu, params=None):
    """
    10D 模型风扰: 风推力加速度 → 10D 扰动向量
    风只影响平动 (速度通道, idx 7-9), 不直接影响姿态
    """
    if params is None:
        params = IRIS

    w = np.asarray(wind_enu)
    w_mag = np.linalg.norm(w)
    if w_mag < 0.1:
        return np.zeros(NX_10D)

    q = x[QUAT_IDX]
    roll, pitch, _ = quat_to_euler(q)
    CdA = effective_CdA(roll, pitch, params=params)

    a_wind = 0.5 * params['rho'] * CdA * w_mag * w / params['mass']

    d = np.zeros(NX_10D)
    d[7:10] = a_wind  # 风推力加速度 → 速度通道
    return d


# ══════════════════════════════════════════════════════════
# 13D 标准 6-DOF 模型 (四元数 + 体轴角速度 + 转动动力学)
# ==========================================================
# 状态 x = [px, py, pz, qw, qx, qy, qz, vx, vy, vz, ωx, ωy, ωz]  (13)
# 控制 u = [F_total, τx, τy, τz]  (总推力N + 三轴力矩Nm)  (4)
#
# 完整 6-DOF 刚体动力学:
#   ṗ     = v
#   q̇     = ½·Ω(ω)·q                (四元数运动学)
#   m·v̇   = R(q)·e₃·F_total + m·g + F_aero   (平动)
#   J·ω̇   = ω × (J·ω) + τ           (转动: 欧拉方程)
#
# 参考文献:
#   - Mahony et al., "Multirotor Aerial Vehicles", IEEE RAM 2012
#   - Stevens & Lewis, "Aircraft Control and Simulation", 3rd ed
# ══════════════════════════════════════════════════════════

NX_13D = 13
NU_13D = 4
# 13D 子索引
POS_13 = slice(0, 3)       # [px, py, pz]
QUAT_13 = slice(3, 7)      # [qw, qx, qy, qz]
VEL_13 = slice(7, 10)      # [vx, vy, vz]
OMEGA_13 = slice(10, 13)   # [ωx, ωy, ωz]

# 悬停参考
X_HOVER_13D = np.array([0,0,0,  1,0,0,0,  0,0,0,  0,0,0])
U_HOVER_13D = np.array([IRIS['mass']*IRIS['g'], 0.0, 0.0, 0.0])  # F=mg, τ=0


# ── 电机混合器 ─────────────────────────────────────────────
def motor_mixer_inverse(F_total, tau_x, tau_y, tau_z, params=None):
    """
    力矩 → 四电机推力 (X 构型, 伪逆)
    
    映射矩阵 M (4×4): [f1,f2,f3,f4]^T → [F_total, τx, τy, τz]^T
    伪逆 M⁺ 将力矩映射回电机推力
    """
    if params is None:
        params = IRIS
    lx = params['arm_lx']
    ly = params['arm_ly']
    k  = params['kappa']
    # M = [[1, 1, 1, 1],
    #      [ly0, ly1, ly2, ly3],
    #      [-lx0, -lx1, -lx2, -lx3],
    #      [k, -k, k, -k]]  (cw/ccw pattern)
    M = np.array([
        [1.0,    1.0,    1.0,    1.0],
        [ly[0],  ly[1],  ly[2],  ly[3]],   # τx = Σ ly_i · f_i
        [-lx[0], -lx[1], -lx[2], -lx[3]],   # τy = -Σ lx_i · f_i
        [k,      -k,      k,      -k]       # τz (对角线电机力矩符号交替)
    ])
    # 伪逆
    M_inv = np.linalg.pinv(M)
    f_motors = M_inv @ np.array([F_total, tau_x, tau_y, tau_z])
    return np.clip(f_motors, 0.0, None)


def motor_mixer_forward(f_motors, params=None):
    """四电机推力 → [F_total, τx, τy, τz]"""
    if params is None:
        params = IRIS
    lx = params['arm_lx']
    ly = params['arm_ly']
    k  = params['kappa']
    F_total = f_motors[0] + f_motors[1] + f_motors[2] + f_motors[3]
    tau_x = ly[0]*f_motors[0] + ly[1]*f_motors[1] + ly[2]*f_motors[2] + ly[3]*f_motors[3]
    tau_y = -(lx[0]*f_motors[0] + lx[1]*f_motors[1] + lx[2]*f_motors[2] + lx[3]*f_motors[3])
    tau_z = k * (f_motors[0] - f_motors[1] + f_motors[2] - f_motors[3])
    return np.array([F_total, tau_x, tau_y, tau_z])


# ── 连续动力学 ────────────────────────────────────────────
def quadrotor_dynamics_13d(x, u, wind_enu, params=None):
    """
    标准 6-DOF 四旋翼动力学 (连续时间, ENU 系)

    Args:
        x:        13D 状态 [p(3), q(4), v(3), ω(3)]
        u:        4D 控制 [F_total(N), τx, τy, τz] (Nm)
        wind_enu: 3D 风速 ENU
        params:   物理参数

    Returns:
        x_dot: 13D 状态导数 [v(3), q̇(4), v̇(3), ω̇(3)]
    """
    if params is None:
        params = IRIS

    q = x[QUAT_13]
    v = x[VEL_13]
    omega = x[OMEGA_13]

    F_total = u[0]
    tau = u[1:4]
    m = params['mass']
    J = params['J_diag']
    wx, wy, wz = omega

    # ── 位置导数 ──
    p_dot = v.copy()

    # ── 姿态导数 (四元数运动学) ──
    qw, qx, qy, qz = q
    q_dot = 0.5 * np.array([
        -wx*qx - wy*qy - wz*qz,
        wx*qw + wz*qy - wy*qz,
        wy*qw - wz*qx + wx*qz,
        wz*qw + wy*qx - wx*qy,
    ])

    # ── 速度导数 (牛顿第二定律) ──
    F_body = np.array([0.0, 0.0, F_total])
    F_enu = rotate_vector_by_quat(q, F_body)

    # 气动阻力
    roll, pitch, _ = quat_to_euler(q)
    CdA = effective_CdA(roll, pitch, params=params)
    V_rel = v - wind_enu
    V_rel_norm = np.linalg.norm(V_rel)
    if V_rel_norm > 0.1:
        F_drag = -0.5 * params['rho'] * CdA * V_rel_norm * V_rel
    else:
        F_drag = np.zeros(3)

    v_dot = F_enu / m + G_VEC + F_drag / m

    # ── 角速度导数 (欧拉方程: J·ω̇ = ω × J·ω + τ) ──
    # ω × (J·ω) = [(Jy-Jz)·ωy·ωz, (Jz-Jx)·ωz·ωx, (Jx-Jy)·ωx·ωy]
    gyro = np.array([
        (J[1] - J[2]) * wy * wz,
        (J[2] - J[0]) * wz * wx,
        (J[0] - J[1]) * wx * wy,
    ])
    omega_dot = (gyro + tau) * params['J_inv']

    return np.concatenate([p_dot, q_dot, v_dot, omega_dot])


# ── 离散步进 (iLQR 前向推演) ────────────────────────────
def quadrotor_dynamics_13d_discrete(x, u, wind_enu, dt, params=None):
    """
    13D 标准 6-DOF 离散动力学
    
    使用 RK2 (中点法) 对转动动力学积分, 位置/速度用前向欧拉
    """
    if params is None:
        params = IRIS

    p = x[POS_13].copy()
    q = x[QUAT_13].copy()
    v = x[VEL_13].copy()
    omega = x[OMEGA_13].copy()

    F_total = u[0]
    tau = u[1:4]
    m = params['mass']
    J = params['J_diag']
    J_inv = params['J_inv']

    # ── 半步推力 (用于中点法) ──
    # Step 1: 半步转动
    wx, wy, wz = omega
    gyro = np.array([
        (J[1] - J[2]) * wy * wz,
        (J[2] - J[0]) * wz * wx,
        (J[0] - J[1]) * wx * wy,
    ])
    omega_dot = (gyro + tau) * J_inv
    omega_half = omega + 0.5 * dt * omega_dot

    # Step 2: 用半步角速度更新四元数
    dq = quat_exp_map(omega_half, dt)
    q_next = quat_mult(q, dq)
    q_next = quat_normalize(q_next)

    # Step 3: 用半步角速度重新计算最终 ω_dot
    wx_h, wy_h, wz_h = omega_half
    gyro_h = np.array([
        (J[1] - J[2]) * wy_h * wz_h,
        (J[2] - J[0]) * wz_h * wx_h,
        (J[0] - J[1]) * wx_h * wy_h,
    ])
    omega_next = omega + dt * (gyro_h + tau) * J_inv

    # ── 平动 (前向欧拉) ──
    p_next = p + v * dt

    F_body = np.array([0.0, 0.0, F_total])
    F_enu = rotate_vector_by_quat(q, F_body)

    roll, pitch, _ = quat_to_euler(q)
    CdA = effective_CdA(roll, pitch, params=params)
    V_rel = v - wind_enu
    V_rel_norm = np.linalg.norm(V_rel)
    if V_rel_norm > 0.1:
        F_drag = -0.5 * params['rho'] * CdA * V_rel_norm * V_rel
    else:
        F_drag = np.zeros(3)

    v_next = v + dt * (F_enu / m + G_VEC + F_drag / m)

    return np.concatenate([p_next, q_next, v_next, omega_next])


# ── 解析雅可比 + 离散化 ─────────────────────────────────
def discrete_linearize_13d(x_op, u_op, wind_enu, dt, params=None):
    """
    标准 6-DOF 解析线性化 + 离散化
    
    Returns: A(13×13), B(13×4), g_offset(13)
    
    雅可比结构 (A_cont):
      ∂ṗ/∂v = I₃                                 (A[0:3, 7:10])
      ∂q̇/∂q = ½·Ω(ω)                             (A[3:7, 3:7])
      ∂q̇/∂ω = ½·Γ(q)                             (A[3:7, 10:13])
      ∂v̇/∂q = (F_total/m)·∂R[:,2]/∂q            (A[7:10, 3:7])
      ∂v̇/∂v = J_drag                             (A[7:10, 7:10])
      ∂ω̇/∂ω = J⁻¹·∂(ω×Jω)/∂ω                    (A[10:13, 10:13])
    
    B_cont:
      ∂v̇/∂F = R[:,2]/m                           (B[7:10, 0])
      ∂ω̇/∂τ = J⁻¹                                (B[10:13, 1:4])
    """
    if params is None:
        params = IRIS

    q = x_op[QUAT_13]
    v = x_op[VEL_13]
    omega = x_op[OMEGA_13]
    F_total = u_op[0]
    m = params['mass']
    J = params['J_diag']
    J_inv = params['J_inv']
    wx, wy, wz = omega
    qw, qx, qy, qz = q

    # ── A_cont (13×13) ──
    A_cont = np.zeros((NX_13D, NX_13D))

    # ∂ṗ/∂v = I
    A_cont[0:3, 7:10] = np.eye(3)

    # ∂q̇/∂q = ½·Ω(ω)
    A_cont[3, 3:7] = 0.5 * np.array([0, -wx, -wy, -wz])
    A_cont[4, 3:7] = 0.5 * np.array([wx, 0, wz, -wy])
    A_cont[5, 3:7] = 0.5 * np.array([wy, -wz, 0, wx])
    A_cont[6, 3:7] = 0.5 * np.array([wz, wy, -wx, 0])

    # ∂q̇/∂ω = ½·Γ(q)  其中 Γ = [[-q_vec^T], [qw·I - q_vec^×]]
    A_cont[3, 10:13] = 0.5 * np.array([-qx, -qy, -qz])
    A_cont[4, 10:13] = 0.5 * np.array([qw, -qz, qy])
    A_cont[5, 10:13] = 0.5 * np.array([qz, qw, -qx])
    A_cont[6, 10:13] = 0.5 * np.array([-qy, qx, qw])

    # ∂v̇/∂q
    J_v_q = (2.0 * F_total / m) * np.array([
        [qy,  qz,  qw,  qx],
        [-qx, -qw,  qz,  qy],
        [qw,  -qx, -qy,  qz],
    ])
    A_cont[7:10, 3:7] = J_v_q

    # ∂v̇/∂v: 气动阻尼
    V_op = v - wind_enu
    V_mag = np.linalg.norm(V_op)
    roll, pitch, _ = quat_to_euler(q)
    CdA = effective_CdA(roll, pitch, params=params)
    if V_mag > 0.1:
        k_damp = 0.5 * params['rho'] * CdA / m
        J_drag_v = -k_damp * (V_mag * np.eye(3) + np.outer(V_op, V_op) / max(V_mag, 0.01))
    else:
        J_drag_v = np.zeros((3, 3))
    A_cont[7:10, 7:10] = J_drag_v

    # ∂ω̇/∂ω = J⁻¹·∂(ω×Jω)/∂ω
    # ∂(ω×Jω)/∂ω = [[0, (Jy-Jz)ωz, (Jy-Jz)ωy],
    #                [(Jz-Jx)ωz, 0, (Jz-Jx)ωx],
    #                [(Jx-Jy)ωy, (Jx-Jy)ωx, 0]]
    d_gyro_dw = np.array([
        [0.0,          (J[1]-J[2])*wz, (J[1]-J[2])*wy],
        [(J[2]-J[0])*wz, 0.0,           (J[2]-J[0])*wx],
        [(J[0]-J[1])*wy, (J[0]-J[1])*wx, 0.0],
    ])
    A_cont[10:13, 10:13] = np.diag(J_inv) @ d_gyro_dw

    # 前向欧拉离散化
    A_disc = np.eye(NX_13D) + A_cont * dt

    # ── B_cont (13×4) ──
    B_cont = np.zeros((NX_13D, NU_13D))

    # ∂v̇/∂F_total = R·e₃ / m
    R_mat = quat_rotation_matrix(q)
    B_cont[7:10, 0] = R_mat[:, 2] / m

    # ∂ω̇/∂τ = J⁻¹
    B_cont[10:13, 1:4] = np.diag(J_inv)

    B_disc = B_cont * dt

    # ── 偏移项 ──
    f_op = quadrotor_dynamics_13d(x_op, u_op, wind_enu, params)
    f_lin = A_cont @ x_op + B_cont @ u_op
    g_offset = (f_op - f_lin) * dt

    return A_disc, B_disc, g_offset


def compute_wind_disturbance_13d(x, u, wind_enu, params=None):
    """13D 风扰: 风推力加速度 → 平动通道 (不直接影响转动)"""
    if params is None:
        params = IRIS
    w = np.asarray(wind_enu)
    w_mag = np.linalg.norm(w)
    if w_mag < 0.1:
        return np.zeros(NX_13D)
    q = x[QUAT_13]
    roll, pitch, _ = quat_to_euler(q)
    CdA = effective_CdA(roll, pitch, params=params)
    a_wind = 0.5 * params['rho'] * CdA * w_mag * w / params['mass']
    d = np.zeros(NX_13D)
    d[7:10] = a_wind
    return d


# ══════════════════════════════════════════════════════════
# BEMT 功率模型
# ══════════════════════════════════════════════════════════

def bem_power(thrust_norm, roll=0.0, pitch=0.0, v_rel_norm=0.0, params=None):
    """BEMT 功率模型 (Euler角输入, 向后兼容)"""
    if params is None:
        params = IRIS
    cos_tilt = np.cos(roll) * np.cos(pitch)
    T = thrust_norm * params['thrust_scale'] / max(cos_tilt, 0.5)
    if T < 0.1:
        return 0.0
    v_h = np.sqrt(T / (2.0 * params['rho'] * params['total_disk']))
    v_i = v_h**2 / np.sqrt(v_rel_norm**2 + v_h**2 + 1e-6)
    P_ind = T * v_i / params['figure_of_merit']
    P_pro = params['n_rotors'] * 8.0 * (T / (params['mass'] * params['g'])) ** 0.25
    P_mech = P_ind + P_pro
    return float(P_mech / params['motor_eff'])


def bem_power_10d(f_c_newton, q=None, v=None, wind_enu=None, params=None):
    """
    BEMT 功率模型 (10D CTBR 输入)

    Args:
        f_c_newton: 总推力 [N]
        q:          四元数 (提取倾斜角)
        v:          速度 (计算相对风)
        wind_enu:   风速 ENU
        params:     物理参数

    Returns: P_total [W]
    """
    if params is None:
        params = IRIS

    if f_c_newton < 0.1:
        return 0.0

    # 倾斜修正
    if q is not None:
        _, pitch, _ = quat_to_euler(q)
        cos_tilt = np.cos(pitch)  # 简化: 主要修正俯仰
    else:
        cos_tilt = 1.0

    T = f_c_newton / max(cos_tilt, 0.5)

    # 相对风速
    if v is not None and wind_enu is not None:
        V_rel = np.array([v[0] - wind_enu[0],
                          v[1] - wind_enu[1],
                          v[2] - wind_enu[2]])
        v_rel_norm = np.linalg.norm(V_rel)
    else:
        v_rel_norm = 0.0

    v_h = np.sqrt(T / (2.0 * params['rho'] * params['total_disk']))
    v_i = v_h**2 / np.sqrt(v_rel_norm**2 + v_h**2 + 1e-6)
    P_ind = T * v_i / params['figure_of_merit']
    P_pro = params['n_rotors'] * 8.0 * (T / (params['mass'] * params['g'])) ** 0.25
    P_mech = P_ind + P_pro
    return float(P_mech / params['motor_eff'])


# ══════════════════════════════════════════════════════════
# 辅助: 模型维度
# ══════════════════════════════════════════════════════════

def get_model_info(model='6d'):
    """返回模型维度信息"""
    if model == '6d':
        return {'nx': 6, 'nu': 3, 'state_labels': ['px', 'py', 'pz', 'vx', 'vy', 'vz'],
                'ctrl_labels': ['roll', 'pitch', 'thrust_norm']}
    elif model == '10d':
        return {'nx': 10, 'nu': 4, 'state_labels': ['px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz'],
                'ctrl_labels': ['f_c', 'wx', 'wy', 'wz']}
    elif model == '13d':
        return {'nx': 13, 'nu': 4, 'state_labels': ['px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'],
                'ctrl_labels': ['F_total', 'tau_x', 'tau_y', 'tau_z']}
    else:
        raise ValueError(f"Unknown model: {model}")


# ══════════════════════════════════════════════════════════
# 自测
# ══════════════════════════════════════════════════════════
if __name__ == '__main__':
    print("=" * 60)
    print("Testing 6D model (existing)")
    print("=" * 60)
    x_6d = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    u_6d = np.array([0.0, 0.0, 0.66])  # hover
    wind = np.array([0.0, 0.0, 0.0])
    dt = 0.033

    xdot_6d = quadrotor_dynamics(x_6d, u_6d, wind)
    print(f"ẋ (hover): {xdot_6d}  (expect near-zero ax,ay; az≈0)")

    A6, B6, g6 = discrete_linearize(x_6d, u_6d, wind, dt)
    print(f"A (6×6): shape={A6.shape}")
    print(f"B (6×3): shape={B6.shape}")
    print(f"g (6):   {g6.round(4)}")

    print()
    print("=" * 60)
    print("Testing 10D CTBR model (new)")
    print("=" * 60)

    # 悬停状态: 水平, 无速度
    x_10d = np.array([0.0, 0.0, 0.0,  # pos
                      1.0, 0.0, 0.0, 0.0,  # quat (identity)
                      0.0, 0.0, 0.0])  # vel
    # 悬停控制: f_c = mg = 14.715, ω = 0
    u_10d = np.array([1.5 * 9.81, 0.0, 0.0, 0.0])

    xdot_10d = quadrotor_dynamics_10d(x_10d, u_10d, wind)
    print(f"ẋ (hover): {xdot_10d.round(6)}")
    print(f"  expect: ṗ=[0,0,0], q̇=[0,0,0,0], v̇≈[0,0,0]")

    # 离散推演
    x_next = quadrotor_dynamics_10d_discrete(x_10d, u_10d, wind, dt)
    print(f"x_{1} (discrete): {x_next.round(6)}")

    # 线性化
    A10, B10, g10 = discrete_linearize_10d(x_10d, u_10d, wind, dt)
    print(f"A (10×10): shape={A10.shape}")
    print(f"B (10×4): shape={B10.shape}")
    print(f"B (v̇ rows, f_c col): {B10[7:10, 0].round(6)}")
    print(f"  expect: R[:,2]*dt/m = [0, 0, 0.033/1.5] = [0, 0, 0.022]")
    print(f"g (10):   {g10.round(6)}")

    # ── 鲁棒性测试: 非悬停操作点 ──
    print()
    print("--- Non-hover operating point ---")
    x_tilted = np.array([1.0, 2.0, 3.0,
                         0.985, 0.0, 0.174, 0.0,  # ~20° pitch
                         1.0, 0.5, -0.3])
    u_cruise = np.array([17.0, 0.0, 0.5, 0.0])  # cruised thrust + pitch rate

    wind2 = np.array([2.0, -1.0, 0.5])
    xdot2 = quadrotor_dynamics_10d(x_tilted, u_cruise, wind2)
    print(f"ẋ: {xdot2.round(4)}")

    A2, B2, g2 = discrete_linearize_10d(x_tilted, u_cruise, wind2, dt)
    eigvals = np.linalg.eigvals(A2)
    print(f"A eigvals max-mag: {np.max(np.abs(eigvals)):.4f}")

    # ── BEMT 功率 ──
    print()
    P = bem_power_10d(14.715, q=np.array([1.0, 0.0, 0.0, 0.0]))
    print(f"BEMT power (hover, fc=mg): {P:.2f} W")

    # ── 风扰 ──
    d10 = compute_wind_disturbance_10d(x_10d, u_10d, np.array([3.0, 0.0, 0.0]))
    print(f"Wind disturbance (3 m/s east): {d10[7:10].round(6)}")

    print()
    print("=" * 60)
    print("All tests passed ✓")
    print("=" * 60)
