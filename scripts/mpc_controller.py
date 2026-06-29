#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mpc_controller.py — 解耦的 TCW-MPC 控制核心 (无 ROS 依赖)
=========================================================
从 mpc_node.py 抽离控制逻辑, 满足 DESIGN.md §3 MPC 接口规范:

  R1  set_cost_weights(theta)   — RL 动作接口, 运行时热注入代价权重,
                                   不重建求解器 (只改 Q/R 对角), 带 clip。
  R2  get_power_estimate()      — 当前 BEMT 功耗 [W] (power_model_v3, 单一权威)
      get_predicted_energy()    — 预测时域累积能量 [J]
  R3  免梯度 (iLQR/OSQP 黑盒 + PPO) — 控制器对 RL 是纯函数式黑盒
  R4  与 plant/PX4 接口不变      — 输出仍是 CTBR [f_c, ωx, ωy, ωz]

设计要点:
  - 无 rospy / ROS 消息依赖 → 可离线单测 + RL 训练时直接调用 step()
  - 权重热更新线程安全 (与 50Hz 控制循环并发), 原地修改 Q/R 对角 →
    iLQR 持有同一数组引用, 每步重新线性化时自动生效, 无需重建求解器
  - 越界权重一律 clip 到 [θ_min, θ_max] → RL 给出任意值也不会让 MPC 失稳
  - get_power_estimate 用 power_model_v3 (BEMT 三项), 与 rotor_power_node /
    实机 ESC 共用同一权威模型 → sim-to-real 功耗一致

用法 (ROS 节点 / RL 环境共用):
    ctrl = MPCController(model='10d')
    ctrl.set_target([0, 0, 2.5])
    # 每个控制步:
    u, info = ctrl.step(x_current, wind_meas, dt_elapsed)
    # RL 外层 (低频):
    ctrl.set_cost_weights({'q_xy': 8.0, 'lambda_energy': 12.0})
    P = ctrl.get_power_estimate()      # W
    E = ctrl.get_predicted_energy()    # J

作者: 小龙 & 虾哥
"""
import os
import sys
import threading
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from mpc_solver import iLQRSolver
from mpc_components import (Integrator, LowPassFilter, TCWPPredictor,
                            CMAManager, WindUtilizationAdvisor,
                            PredictiveEnergyManager)
from quadrotor_dynamics import (quadrotor_dynamics_10d_discrete,
                                quadrotor_dynamics_10d,
                                discrete_linearize_10d,
                                compute_wind_disturbance_10d,
                                bem_power_10d, IRIS, NX_10D, NU_10D,
                                quat_to_euler, euler_to_quat)

try:
    from power_model_v3 import RotorPowerModel
    _HAS_POWER_V3 = True
except Exception:  # pragma: no cover - 退化路径
    _HAS_POWER_V3 = False


# ── 代价权重安全区间 (RL clip 边界) ───────────────────────────
# 键对应 DESIGN §3.2 R1 的 theta. q_xy/q_z 是位置跟踪权重,
# lambda_energy 加到推力控制代价 (能耗罚), r_omega 是角速率控制代价。
WEIGHT_BOUNDS = {
    'q_xy':          (0.5, 60.0),
    'q_z':           (0.5, 80.0),
    'lambda_energy': (0.0, 80.0),
    'r_omega':       (5.0, 200.0),
}

# Q 对角索引 (10D: pos(3)+quat(4)+vel(3))
_IDX_QX, _IDX_QY, _IDX_QZ = 0, 1, 2
# R 对角索引 (10D: fc, wx, wy, wz)
_IDX_R_FC = 0
_IDX_R_WX, _IDX_R_WY, _IDX_R_WZ = 1, 2, 3


class MPCController:
    """TCW-MPC 控制核心 (10D CTBR 直驱, 解耦 ROS)"""

    def __init__(self, model='10d', dt=0.03, N_pred=15,
                 lambda_energy=5.0, q_xy_scale=1.0, energy_first=False,
                 trajectory_mode=False):
        if model != '10d':
            raise NotImplementedError(
                "mpc_controller 当前仅支持 10D CTBR; 6D 请用旧 mpc_node 路径")
        self.model = '10d'
        self.dt = float(dt)
        # ── offset-free MPC: 集总扰动加速度估计 (ENU, m/s²) ──
        # 龙伯格/低通观测器在线估 "实测加速度 - 模型预测加速度" 残差,
        # 作为加性 d_acc 喂进 MPC 预测模型 → MPC 规划配平抵消 → 稳态无偏且无相位滞后。
        # 不同于外置 PI(加在角速率输出后, HIL 会正反馈发散): 这是在预测模型内补偿。
        # 文献: offset-free MPC (Pannocchia & Rawlings 2003), 外力估计 NMPC (Hanover RAL2021)。
        self._d_acc_est = np.zeros(3)
        # offset-free 观测器开关 (诊断用): MPC_OFFSET_FREE=0 关闭, 强制 d_acc=0。
        # 怀疑阵风下观测器 wind-up 削推力致发散 (2026-06-29 决定性对照), 用此隔离验证。
        self._offset_free_on = os.environ.get('MPC_OFFSET_FREE', '1') != '0'
        self._d_obs_tau = 2.0      # 观测器时间常数 s (加长: 平均掉零均值 EKF 噪声)
        self._d_acc_max = 3.0      # 估计幅值上限 m/s² (防发散)
        self._prev_vel = None      # 上步速度, 算实测加速度
        self._prev_u = None        # 上步控制, 算模型预测加速度
        self._prev_x = None
        self.N_pred = int(N_pred)
        self.g = 9.81
        self.drone_mass = 1.5
        self.hover_thrust = 0.66
        self.rho = 1.225

        # ── 节能 / 任务参数 ──
        self.lambda_energy = float(lambda_energy)
        self._q_xy_scale = float(q_xy_scale)
        self._energy_first = bool(energy_first)
        self._is_trajectory_task = bool(trajectory_mode)
        self._u_hover = np.array([self.drone_mass * self.g, 0.0, 0.0, 0.0])
        self._fc_max = IRIS['max_thrust']

        # ── 控制量约束 (nominal) ──
        # 角速率上限: 0.8 配 R_ω=300 过度平滑→强风下位置修正不足(离线漂9.6m)。
        # 暂回 1.5 (介于原2.0与官方0.5), 隔离验证: 先确认终端代价+权重重平衡的净效果。
        self._u_min_nom = np.array([2.0, -1.5, -1.5, -1.5])
        self._u_max_nom = np.array([27.0, 1.5, 1.5, 1.5])
        # ↑ 推力上限 20→27N (对标 NMPC-QUA max_thrust=27): 大倾角抗风需
        # mg/cos(倾角) 补偿垂直分量, 倾45°需20.8N, 倾60°鞀29.4N, 20N不够。
        # 角速率上限 ±2.0(roll/pitch)/±1.0(yaw) rad/s。
        # 原 ±0.1 rad/s(5.7°/s) 太小: 7m/s侧风下 ω顶死0.1仍被吹飞。

        # ── MPC 权重 (Q/R 为 iLQR 持有的同一引用; 热更新原地改对角) ──
        base_q_xy = 40.0
        # q_xy 15→40: 向参考工程"位置主导"平衡靠拢(NMPC-QUA 位置200:姿态1)。
        # 原 q_xy=15 vs 姿态=12 几乎1:1 → 姿态权重太高压制了为修正位置做的快速倾斜
        # (实测风中 ω 只用到0.07/上限2.0, 位置漂移1.5m)。q_z=40 高度跟踪不变。
        self.Q = np.diag([base_q_xy, base_q_xy, 40.0,   # pos
                          0.0, 4.0, 4.0, 3.0,            # quat [qw,qx,qy,qz]
                          8.0, 8.0, 25.0])               # vel
        # ↑ 四元数项恢复 v2 已验证值 4/4/3 (2026-06-29 02:40 HIL 拿到有界 0.9m)。
        # 回退原因: 03:47 commit 曾按 px4-mpc 官方降到 0.5, 但官方"姿态软(0.1)"是与
        # "角速率重惩罚 R_ω=500"配套的; 而该 commit 把 R_ω 增大推迟到 P2, R_ω 仍 50/30。
        # 结果 = 姿态软(0.5)+角速率放任(50) 最坏组合 → 姿态裸奔→侧翻发散(09:06 HIL 157m)。
        # px4-mpc 风格(quat0.1 + R_ω500 + 长时域)留到 P2 作为一次原子改动整体上。
        self.R = np.diag([3.0, 5.0, 5.0, 3.0])        # fc/ω
        # ★ R_ω 50→5 (2026-06-29 离线 sweep 决定性诊断): R_ω 是姿态恢复的主节流门。
        # roll=-25° 误差下: R_ω=50 仅命令 0.18rad/s(10°/s) → 斗不过风力力矩建倾
        #   → roll 单调 wind-up → 侧翻发散(2026-06-29 四次 HIL 157/156/109/122m 不变量)。
        # R_ω=5 → 0.60rad/s(34°/s) = 3.3×权限, 足以压住 tilt(实测风驱倾建速~5°/s)。
        # 注: Q_quat/时域 sweep 几乎不动 wx_cmd → 证明昨夜调 quat 是错旋钮, R_ω 才是。
        if self._energy_first:
            self.Q[_IDX_QX, _IDX_QX] *= self._q_xy_scale
            self.Q[_IDX_QY, _IDX_QY] *= self._q_xy_scale
            self.Q[7, 7] *= self._q_xy_scale
            self.Q[8, 8] *= self._q_xy_scale
        # lambda_energy 叠加到推力控制代价。基础项 3→1: 原 3+λ=8 惩罚偏离悬停推力太重
        # → MPC 舍不得用推力余量(爬升只用15N/上饨20N) → 响应慢+极端湍流下沉气流补不出推力。
        # 降到 1: 响应变快, 抹扶余量留给快速爬升/抗扰; 能耗惩罚仍由 lambda_energy(RL旋钮)控。
        self._r_fc_base = 1.0
        self.R[_IDX_R_FC, _IDX_R_FC] = self._r_fc_base + self.lambda_energy

        self._weights_lock = threading.Lock()
        self._theta_applied = self._read_theta()   # 最近生效的 θ (供日志/RL 观测)

        # ── 求解器: iLQR (持有 self.Q / self.R 引用) ──
        self.mpc = iLQRSolver(
            nx=NX_10D, nu=NU_10D, N=self.N_pred, dt=self.dt,
            Q=self.Q, R=self.R,
            u_min=self._u_min_nom, u_max=self._u_max_nom,
            dynamics_fn=lambda x, u, w: quadrotor_dynamics_10d_discrete(x, u, w, self.dt, d_acc=self._d_acc_est),
            linearize_fn=lambda x, u, w, d: discrete_linearize_10d(x, u, w, d),
            lqr_iter=2, linesearch_decay=0.5, max_linesearch_iter=2, eps=5e-3)

        # ── 终端代价 (P0, 对齐 px4-mpc 官方: 终端位置300=过程10的30倍, 终端速度100=10倍) ──
        # iLQR 已内建 P_terminal 接口, 原 = Q.copy()(1:1)。有限时域 MPC 靠终端代价逼近
        # 无限时域最优+稳定性保证。无终端代价 → MPC 短视(只顾 0.45s) → 稳态抗扰刚度不足。
        # 保守取值: 位置×15、速度×6 (低于官方30/10, 首轮验证稳健后再加)。
        P_term = self.Q.copy()
        P_term[0, 0] *= 15.0; P_term[1, 1] *= 15.0; P_term[2, 2] *= 15.0  # pos
        P_term[7, 7] *= 6.0;  P_term[8, 8] *= 6.0;  P_term[9, 9] *= 6.0   # vel
        self.mpc.P_terminal = P_term

        # ── 控制组件 ──
        self.tcwp = TCWPPredictor(N_pred=self.N_pred, M_history=25,
                                  L_coherence=15.0, dt=self.dt)
        self.cma = CMAManager(self._u_max_nom, self._u_min_nom,
                              gamma_t=0.002, gamma_a=0.0001, rho=self.rho,
                              model=self.model)
        self.wind_advisor = WindUtilizationAdvisor(
            alignment_threshold=0.3, coast_factor_max=0.5)
        self.wind_advisor.set_task_type(self._is_trajectory_task)
        self.energy_mgr = PredictiveEnergyManager(
            N_pred=self.N_pred, z_margin=0.5, ramp_steps=5, dz_threshold=2.0)
        self.integrator = Integrator(
            # Ki[2]=高度积分增益, max_int[0]=推力积分上限(N)。
            # 限速参考轨迹已防 windup。注: HIL 稳态 ~0.18m 正偏差加大积分无法消除
            # (试过 Ki[2]=0.004/上限0.3 反而冲过头再回落, 平衡点不变) → 指向非控制律根因
            # (疑: odom/EKF 高度估计稳态偏差)。保守值。
            Ki=[0.01, 0.01, 0.003], max_int=[0.20, 0.2, 0.2, 0.1],
            dt=self.dt, model='10d')
        self.lpf = LowPassFilter(alpha=0.4)

        # ── 功耗 (power_model_v3, 单一权威) ──
        self._power_model = RotorPowerModel() if _HAS_POWER_V3 else None
        self._power_est = 0.0
        self._predicted_energy = 0.0

        # ── 状态 ──
        self.nu = NU_10D
        self.target_pos = np.array([0.0, 0.0, 2.5])
        self._ref_pos = np.array([0.0, 0.0, 0.0])
        self._tau_mpc = 3.0
        # 限速梯形参考轨迹参数 (替代一阶低通)
        self._v_ref_max = 1.2      # 参考最大逼近速度 [m/s]
        self._decel_band = 1.0     # 减速带 [m]: 剩余距离<此时线性降速 → 到点刹停
        self._q_current = np.array([1.0, 0.0, 0.0, 0.0])
        self._u_last_safe = self._u_hover.copy()
        self._u_min_eff = self._u_min_nom.copy()
        self._u_max_eff = self._u_max_nom.copy()
        self._pos_traj_prev = [np.zeros(3)] * self.N_pred
        self._vel_traj_prev = [np.zeros(3)] * self.N_pred
        self._wind_seq = None
        self._d_seq = None
        self._q_dyn = 0.0
        self._coast_factor = 0.0
        self._cma_mode_str = 'normal'
        self._energy_mode = 'normal'
        self._thrust_energy_bias = 0.0
        self._last_ok = 0
        self._last_iter = 0

    # ══════════════════════════════════════════════════════
    # R1: 代价权重运行时热注入 (RL 动作接口)
    # ══════════════════════════════════════════════════════
    def set_cost_weights(self, theta: dict):
        """RL 动作接口。线程安全, 50Hz 控制循环中途可热更新。
        theta 键 (全部可选, 缺省保持当前):
            'q_xy'          水平位置跟踪权重
            'q_z'           高度跟踪权重
            'lambda_energy' 能耗权重 (加到推力控制代价 R[fc])
            'r_omega'       角速率控制代价 (R 的 wx/wy/wz 对角)
        所有值 clip 到 WEIGHT_BOUNDS; 不重建求解器 (原地改 Q/R 对角)。
        返回实际生效的 θ (clip 后)。
        """
        if not isinstance(theta, dict):
            raise TypeError("theta 必须是 dict")
        with self._weights_lock:
            for key, val in theta.items():
                if key not in WEIGHT_BOUNDS:
                    continue  # 未知键忽略 (鲁棒)
                try:
                    v = float(val)
                except (TypeError, ValueError):
                    continue
                if not np.isfinite(v):
                    continue  # nan/inf 丢弃, 保持当前值
                lo, hi = WEIGHT_BOUNDS[key]
                v = float(np.clip(v, lo, hi))
                if key == 'q_xy':
                    self.Q[_IDX_QX, _IDX_QX] = v
                    self.Q[_IDX_QY, _IDX_QY] = v
                elif key == 'q_z':
                    self.Q[_IDX_QZ, _IDX_QZ] = v
                elif key == 'lambda_energy':
                    self.lambda_energy = v
                    self.R[_IDX_R_FC, _IDX_R_FC] = self._r_fc_base + v
                elif key == 'r_omega':
                    self.R[_IDX_R_WX, _IDX_R_WX] = v
                    self.R[_IDX_R_WY, _IDX_R_WY] = v
                    self.R[_IDX_R_WZ, _IDX_R_WZ] = v
            self._theta_applied = self._read_theta()
        return dict(self._theta_applied)

    def _read_theta(self):
        return {
            'q_xy':          float(self.Q[_IDX_QX, _IDX_QX]),
            'q_z':           float(self.Q[_IDX_QZ, _IDX_QZ]),
            'lambda_energy': float(self.lambda_energy),
            'r_omega':       float(self.R[_IDX_R_WX, _IDX_R_WX]),
        }

    def get_cost_weights(self) -> dict:
        """返回当前生效的 θ (供 RL 观测 / 日志)。"""
        with self._weights_lock:
            return dict(self._theta_applied)

    # ══════════════════════════════════════════════════════
    # R2: 能耗可观测输出
    # ══════════════════════════════════════════════════════
    def get_power_estimate(self) -> float:
        """当前 BEMT 估算电功率 [W] (power_model_v3, 单一权威)。"""
        return float(self._power_est)

    def get_predicted_energy(self) -> float:
        """预测时域内累积能量 [J] = ∫P dt ≈ Σ P_k · dt。"""
        return float(self._predicted_energy)

    def _power_from_thrust(self, f_c_total, v_rel_world, tilt_normal):
        """总推力 [N] → 每转子 ω → power_model_v3 电功率 [W]。
        与 rotor_power_node (仿真电机指令) / 实机 ESC RPM 共用同一 BEMT。"""
        if self._power_model is None:
            return float(bem_power_10d(f_c_total, q=self._q_current,
                                       v=None, wind_enu=None))
        f_c_total = max(float(f_c_total), 0.0)
        T_per = f_c_total / 4.0
        C_T = self._power_model.p['C_T']
        omega = np.sqrt(max(T_per, 0.0) / C_T)
        omegas = np.full(4, omega)
        return self._power_model.power_from_omegas(
            omegas, v_world=v_rel_world, tilt_normal=tilt_normal)

    # ══════════════════════════════════════════════════════
    # 目标 / 状态设置
    # ══════════════════════════════════════════════════════
    def set_target(self, pos):
        self.target_pos = np.asarray(pos, dtype=float)[:3].copy()

    def reset(self):
        """RL 回合重置: 清积分器/参考/历史 (不动权重)。"""
        self._ref_pos = np.array([0.0, 0.0, 0.0])
        self._u_last_safe = self._u_hover.copy()
        self._pos_traj_prev = [np.zeros(3)] * self.N_pred
        self._vel_traj_prev = [np.zeros(3)] * self.N_pred
        self._predicted_energy = 0.0
        self._d_acc_est = np.zeros(3)   # offset-free 观测器重置
        self._prev_vel = None; self._prev_u = None
        try:
            self.integrator.reset()
        except AttributeError:
            pass

    # ══════════════════════════════════════════════════════
    # 主控制步 (从 mpc_node.run() 抽离; 无 ROS)
    # ══════════════════════════════════════════════════════
    def step(self, x_current, wind_meas=None, yaw_cur=0.0):
        """单步控制。
        Args:
            x_current: 10D 状态 [p(3), q(4), v(3)] (ENU)
            wind_meas: 3D 风速 (ENU) 或 None
            yaw_cur:   当前偏航 [rad] (积分器解耦用)
        Returns:
            u_smooth: CTBR 控制 [f_c(N), ωx, ωy, ωz]
            info:     dict (诊断 + RL 观测辅助)
        """
        x_current = np.asarray(x_current, dtype=float)
        self._q_current = x_current[3:7].copy()
        got_wind = wind_meas is not None
        if got_wind:
            wind_meas = np.asarray(wind_meas, dtype=float)
        else:
            wind_meas = np.zeros(3)

        # Step 0: offset-free 扰动观测器 — 速度残差积分型 (不做数值微分, 不放大噪声)
        # 比较 "实测速度" 与 "上步模型预测的当前速度"(无扰动) 的残差。
        # 残差 ∝ d_acc·dt, 对其做慢速积分估 d_acc。速度级残差不除 dt → 不放大 EKF 噪声。
        v_now = x_current[7:10].copy()
        if self._prev_x is not None and self._prev_u is not None:
            xd_next = quadrotor_dynamics_10d_discrete(
                self._prev_x, self._prev_u,
                wind_meas if got_wind else np.zeros(3), self.dt, d_acc=None)
            v_pred = xd_next[7:10]              # 模型(无扰动)预测的当前速度
            v_resid = v_now - v_pred           # ≈0 无失配; 非0 = 未建模外力·dt
            # 慢速积分: d ← d + Kobs·(v_resid/dt - d), Kobs 小 → 平滑不发散
            d_inst = v_resid / self.dt         # 等效瞬时加速度偏差
            Kobs = self.dt / max(self._d_obs_tau, self.dt)
            self._d_acc_est = self._d_acc_est + Kobs * (d_inst - self._d_acc_est)
            # 放开 z 轴观测: 水平抗扰倾斜时推力垂直分量亏空(fc·cos(tilt))
            # 作为 z 外力被观测到, MPC 预测内自动补推力 → 治高度回涨。
            # 与高度积分器分工: 观测器治可预测的推力亏空, 积分器治慢变残差。
            self._d_acc_est = np.clip(self._d_acc_est, -self._d_acc_max, self._d_acc_max)
        if not self._offset_free_on:
            self._d_acc_est = np.zeros(3)   # 诊断开关: 关闭 offset-free 观测器
        self._prev_x = x_current.copy()

        # Step 1: TCWP 风预测
        if got_wind:
            self.tcwp.add_sample(x_current[:3].copy(), wind_meas.copy())
            self.tcwp.set_previous_trajectory(self._pos_traj_prev)
            self._wind_seq = self.tcwp.predict()
            self._d_seq = self._compute_disturbance_seq(self._wind_seq, x_current)
        else:
            self._wind_seq = None
            self._d_seq = None

        # Step 2: CMA 约束 (读权重锁外, 不影响)
        self._u_min_eff, self._u_max_eff = self.cma.update(
            wind_meas, mpc_u_pred=self._u_last_safe)
        self._q_dyn = self.cma.get_q()
        self._cma_mode_str = self.cma.get_mode()
        self.mpc.u_min = self._u_min_eff
        self.mpc.u_max = self._u_max_eff

        # Step 3: 参考轨迹 (限速梯形 + 速度前馈)
        # 旧版纯一阶低通 + x_ref 速度=0 → MPC 看不到爬升速度目标,
        # 既给不出爬升推力前馈也无法到点主动刹停 → 爬升慢(42s)+静差(17cm)。
        # 新版: 参考位置以受限速度逼近目标, 接近时线性减速; 参考速度填入 x_ref[7:10],
        # MPC 看到协调的位置+速度目标 → 给足推力爬升, 到点刹停, 零静差。
        err_vec = self.target_pos - self._ref_pos
        dist = np.linalg.norm(err_vec)
        if dist > 1e-6:
            direction = err_vec / dist
            # 减速带内线性降速 (sqrt 型 → 匀减速, 这里用线性近似够稳)
            v_cmd = self._v_ref_max * min(1.0, dist / max(self._decel_band, 1e-3))
            step_len = min(v_cmd * self.dt, dist)   # 不越过目标
            self._ref_pos = self._ref_pos + direction * step_len
            ref_vel = direction * v_cmd
        else:
            ref_vel = np.zeros(3)
        ref_pos = self._ref_pos.copy()
        x_ref = np.zeros(10)
        x_ref[:3] = ref_pos
        # 姿态参考: 借鉴 NMPC-QUA — 期望姿态=水平(roll=pitch=0), 保留当前 yaw,
        # 而非 _q_current(跟随当前)。配合 Q 四元数项加权(见 __init__),
        # MPC 主动把飞机拉回水平 → 风吹斜后会主动回正, 不再放任倾角发散(74°侧翻根因)。
        _, _, yaw_cur = quat_to_euler(self._q_current)
        x_ref[3:7] = euler_to_quat(0.0, 0.0, yaw_cur)
        x_ref[7:10] = ref_vel

        # Step 3.5: 风能利用 + 能量管理
        pos_err = ref_pos - x_current[:3]
        coast, wind_mode, Q_scale = self.wind_advisor.evaluate(
            wind_meas, pos_err, wind_seq=self._wind_seq)
        self._coast_factor = coast
        if self._wind_seq is not None and got_wind:
            self._energy_mode, self._thrust_energy_bias = self.energy_mgr.plan(
                self._wind_seq)
        else:
            self._energy_mode = 'normal'
            self._thrust_energy_bias = 0.0

        # Step 4: MPC 求解 (持有 self.Q/self.R 引用, 热更新自动生效)
        u_ref = self._u_hover.copy()
        # 注: 试过倾斜推力补偿前馈 u_ref[0]=mg/cos(tilt), 会造成正反馈发散
        # (倾斜加推→推力水平分量增→推得更远更斜) → 已回退。保持恒 mg 最稳。
        u_ref[0] += self._thrust_energy_bias
        with self._weights_lock:
            u_mpc = self.mpc.solve(x_current, x_ref, u_ref=u_ref,
                                   d_seq=self._d_seq)
        self._last_ok = int(self.mpc.last_ok)
        self._last_iter = self.mpc.last_iter

        # Step 5: 提取轨迹
        if u_mpc is not None and self.mpc.last_ok:
            u_safe = u_mpc
            pos_traj, vel_traj = self.mpc.get_trajectory()
            if pos_traj is not None:
                self._pos_traj_prev = pos_traj
                self._vel_traj_prev = vel_traj
        else:
            u_safe = self._u_last_safe

        # (移除旧 MPC-driven τ 逻辑: 参考轨迹改限速梯形+速度前馈, 不再用一阶低通 tau)

        # Step 6: 积分器
        int_corr = self.integrator.update(pos_err, yaw=yaw_cur)

        # Step 7: 合成 + clip + 平滑
        u_raw = u_safe.copy()
        for i in range(self.nu):
            u_raw[i] += int_corr[i]
            u_raw[i] = np.clip(u_raw[i], self._u_min_eff[i], self._u_max_eff[i])
        u_smooth = self.lpf.apply(u_raw)
        self._u_last_safe = u_smooth.copy()
        self._prev_u = u_smooth.copy()   # offset-free 观测器: 记实际施加控制

        # 功耗估算 (power_model_v3, BEMT)
        v_rel_world = (x_current[7:10] - wind_meas) if got_wind else x_current[7:10]
        zb = _quat_to_zaxis(self._q_current)
        self._power_est = self._power_from_thrust(u_smooth[0], v_rel_world, zb)
        self._predicted_energy += self._power_est * self.dt

        info = {
            'ref_pos': ref_pos,
            'pos_err': pos_err,
            'int_corr': int_corr,
            'u_min_eff': self._u_min_eff.copy(),
            'u_max_eff': self._u_max_eff.copy(),
            'q_dyn': self._q_dyn,
            'cma_mode': self._cma_mode_str,
            'energy_mode': self._energy_mode,
            'coast_factor': self._coast_factor,
            'thrust_bias': self._thrust_energy_bias,
            'tau_mpc': self._tau_mpc,
            'wind_seq0': (self._wind_seq[0] if self._wind_seq else np.zeros(3)),
            'power_est': self._power_est,
            'mpc_ok': self._last_ok,
            'mpc_iter': self._last_iter,
            'theta': self.get_cost_weights(),
        }
        return u_smooth, info

    # ══════════════════════════════════════════════════════
    # 风扰序列 (从 mpc_node 抽离)
    # ══════════════════════════════════════════════════════
    def _compute_disturbance_seq(self, wind_seq, x_current):
        if not wind_seq:
            return [np.zeros(10)] * self.N_pred
        d_seq = []
        for k in range(self.N_pred):
            w_k = wind_seq[k] if k < len(wind_seq) else wind_seq[-1]
            x_k = np.zeros(10)
            if self._pos_traj_prev and k < len(self._pos_traj_prev):
                x_k[0:3] = self._pos_traj_prev[k]
                x_k[3:7] = self._q_current
                x_k[7:10] = (self._vel_traj_prev[k]
                             if k < len(self._vel_traj_prev) else np.zeros(3))
            else:
                x_k = x_current.copy()
            d = compute_wind_disturbance_10d(x_k, self._u_last_safe, w_k)
            d[7:10] *= self.dt
            d_seq.append(d.copy())
        return d_seq


def _quat_to_zaxis(q):
    """四元数 [w,x,y,z] → 机体 z 轴(桨盘法线)在世界系方向。"""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return np.array([
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    ])
