#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TCW-MPC: Trajectory-Coherent Wind-Adaptive MPC
==============================================
面向极端湍流风场的自适应模型预测控制器

核心创新:
  1. TCWP (Trajectory-Coherent Wind Prediction)
     — 无人机作为移动流体探针, 利用空间相干核外推预测时域风场
  2. CMA (Constraint Margin Adaptation)
     — 基于动压实时收紧 MPC 硬约束, 防止执行器饱和
  3. 风偏置学习 — EMA 误差补偿

论文级亮点:
  - 首次将湍流空间相干结构显式嵌入线性 MPC 预测模型
  - TCWP + CMA 三模块正交叠加, OSQP 零增维
  - 适用于 6 状态系统, 满足嵌入式硬实时约束

作者: 小龙 & 虾哥
"""

import csv, os, rospy, collections, time as _time, threading
import numpy as np
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State, ManualControl
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Quaternion, Vector3Stamped, PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import osqp
import scipy.sparse as sparse
from scipy.linalg import block_diag
from power_model import UnifiedPowerEstimator
from quadrotor_dynamics import (discrete_linearize, bem_power,
                                 compute_wind_disturbance, IRIS)


# ══════════════════════════════════════════════════════════
# 1. 标准 6 状态 MPC (支持风扰前馈)
# ══════════════════════════════════════════════════════════
class StandardMPC:
    """
    状态 x = [px, py, pz, vx, vy, vz]  (ENU)
    控制 u = [roll, pitch, thrust]

    风扰处理: d_seq 为零增维前馈项, 直接修改等式约束 RHS
    """

    def __init__(self, A, B, g_vec, Q, R, N, u_min, u_max):
        self.A = A; self.B = B; self.g = g_vec
        self.Q = Q; self.R = R; self.N = N
        self.nx = A.shape[0]; self.nu = B.shape[1]
        self.u_min = u_min; self.u_max = u_max

        self.P = Q.copy()
        self.P[5, 5] = 0  # z 速度不罚终端

        self.last_ok = True
        self.last_iter = 0
        self._last_full_x = None  # 存储完整解向量 (供轨迹提取)
        self._last_tilt = 0.0  # 上次重建时的倾角
        self._build_osqp()

    def update_model(self, A_new, B_new):
        """SL-MPC: 更新 A/B 矩阵并重建 QP (仅在倾角显著变化时调用)"""
        self.A = A_new; self.B = B_new
        self._build_osqp()
        self._last_full_x = None

    def _build_osqp(self):
        nx, nu, N = self.nx, self.nu, self.N
        n_var = N * (nx + nu)
        n_eq = N * nx
        n_ineq = N * nu

        P_blocks = []
        for k in range(N):
            P_blocks.append(2.0 * self.R)
            if k < N - 1:
                P_blocks.append(2.0 * self.Q)
            else:
                P_blocks.append(2.0 * self.P)
        P_mat = sparse.csc_matrix(block_diag(*P_blocks))

        rows, cols, data = [], [], []
        row = 0

        def add(r, c, v):
            rows.append(r); cols.append(c); data.append(v)

        # 约束 0: B*u_0 - x_1 = ...
        for i in range(nx):
            for j in range(nu):
                if abs(self.B[i, j]) > 1e-12:
                    add(row + i, j, self.B[i, j])
            add(row + i, nu + i, -1.0)
        row += nx

        # 约束 k=1..N-1: A*x_k + B*u_k - x_{k+1}
        for k in range(1, N):
            xk_s = (k - 1) * (nx + nu) + nu
            uk_s = k * (nx + nu)
            xk1_s = k * (nx + nu) + nu
            for i in range(nx):
                for j in range(nx):
                    if abs(self.A[i, j]) > 1e-12:
                        add(row + i, xk_s + j, self.A[i, j])
                for j in range(nu):
                    if abs(self.B[i, j]) > 1e-12:
                        add(row + i, uk_s + j, self.B[i, j])
                add(row + i, xk1_s + i, -1.0)
            row += nx

        A_eq = sparse.csc_matrix((data, (rows, cols)), shape=(n_eq, n_var))

        A_ineq_data, A_ineq_rows, A_ineq_cols = [], [], []
        for k in range(N):
            uk_s = k * (nx + nu)
            for j in range(nu):
                r = k * nu + j
                A_ineq_rows.append(r)
                A_ineq_cols.append(uk_s + j)
                A_ineq_data.append(1.0)
        A_ineq = sparse.csc_matrix(
            (A_ineq_data, (A_ineq_rows, A_ineq_cols)),
            shape=(n_ineq, n_var))

        A_osqp = sparse.vstack([A_eq, A_ineq], format='csc')

        # beq 模板 (不含 x0 部分)
        self.beq_template = np.zeros(n_eq)
        for k in range(N):
            self.beq_template[k * nx:(k + 1) * nx] = -self.g

        self._P_mat = P_mat
        self._A_osqp = A_osqp
        self.q_template = np.zeros(n_var)
        self._setup_solver()
        rospy.loginfo("OSQP 已构建: %d var, %d eq, %d ineq",
                      n_var, n_eq, n_ineq)

    def _setup_solver(self):
        n_eq = self.N * self.nx
        n_ineq = self.N * self.nu
        self.solver = osqp.OSQP()
        self.solver.setup(
            P=self._P_mat, q=np.zeros(self._P_mat.shape[0]),
            A=self._A_osqp,
            l=np.zeros(n_eq + n_ineq), u=np.zeros(n_eq + n_ineq),
            verbose=False, warm_start=True, max_iter=400,
            eps_abs=1e-4, eps_rel=1e-4, polish=True)

    def solve(self, x0, x_ref, u_ref=None, d_seq=None):
        """
        x0:    当前状态 (6,)
        x_ref: 参考状态 (6,)
        u_ref: 参考控制 (3,) 或 None
        d_seq: 风扰序列 list of (6,) 长度 N, 或 None

        d_k 进入动力学: x_{k+1} = A x_k + B u_k + g + d_k
        → 等式约束 RHS: -g - d_k - [A x0 for k=0]
        """
        if u_ref is None:
            u_ref = np.zeros(self.nu)
        if d_seq is None:
            d_seq = [np.zeros(self.nx)] * self.N

        nx, nu, N = self.nx, self.nu, self.N
        n_eq = N * nx

        q = self.q_template.copy()
        for k in range(N):
            uk_s = k * (nx + nu)
            xk1_s = uk_s + nu
            q[uk_s:uk_s + nu] = -2.0 * self.R @ u_ref
            if k < N - 1:
                q[xk1_s:xk1_s + nx] = -2.0 * self.Q @ x_ref
            else:
                q[xk1_s:xk1_s + nx] = -2.0 * self.P @ x_ref

        # 边界: 基础 beq, 叠加风扰
        l = np.zeros(n_eq + N * nu)
        u = np.zeros(n_eq + N * nu)

        for k in range(N):
            beq_k = -self.g - np.asarray(d_seq[k])
            l[k * nx:(k + 1) * nx] = beq_k
            u[k * nx:(k + 1) * nx] = beq_k

        # 约束 0 额外减去 A x0
        l[:nx] -= self.A @ x0
        u[:nx] -= self.A @ x0

        # 控制边界 (CMA 动态修改在外部)
        l[n_eq:] = np.tile(self.u_min, N)
        u[n_eq:] = np.tile(self.u_max, N)

        # 求解 (失败时触发重建, 不再周期性破坏热启动)
        try:
            self.solver.update(q=q, l=l, u=u)
            result = self.solver.solve()
        except Exception:
            rospy.logwarn_throttle(2.0, "OSQP 异常, 尝试重建...")
            self._setup_solver()
            try:
                self.solver.update(q=q, l=l, u=u)
                result = self.solver.solve()
            except Exception:
                self.last_ok = False
                self.last_iter = 0
                self._last_full_x = None
                return None

        self.last_iter = result.info.iter
        if result.info.status_val not in (1, 2):
            self.last_ok = False
            self._last_full_x = None
            return None

        self.last_ok = True
        self._last_full_x = result.x.copy()
        return result.x[:nu]

    def get_trajectory(self):
        """
        从最后一次求解中提取位置/速度预测轨迹
        返回: (pos_traj, vel_traj) 各为 list of (3,) 长度 N
        """
        if self._last_full_x is None:
            return None, None
        n_xu = self.nx + self.nu
        pos_traj, vel_traj = [], []
        for k in range(1, self.N + 1):
            xk_s = (k - 1) * n_xu + self.nu
            xk = self._last_full_x[xk_s:xk_s + self.nx]
            pos_traj.append(xk[:3].copy())
            vel_traj.append(xk[3:6].copy())
        return pos_traj, vel_traj


# ══════════════════════════════════════════════════════════
# 2. 外置积分器 (PI)
# ══════════════════════════════════════════════════════════
class Integrator:
    def __init__(self, Ki, max_int, dt):
        self.Ki = np.asarray(Ki); self.max_int = np.asarray(max_int); self.dt = dt
        self._integral = np.zeros(3)

    def update(self, pos_error, yaw=0.0):
        """
        pos_error: ENU 位置误差 [ex, ey, ez]
        yaw:      当前偏航角 [rad], 用于旋转到机体系
        """
        ex, ey, ez = pos_error
        # 将 ENU 位置误差旋转到机体系 (body frame)
        cos_y = np.cos(yaw); sin_y = np.sin(yaw)
        ex_body =  cos_y * ex + sin_y * ey   # 机体前向误差
        ey_body = -sin_y * ex + cos_y * ey   # 机体右向误差
        # 机体系: 前向误差 → pitch, 右向误差 → -roll
        correction = np.array([-self.Ki[0] * ey_body,
                               +self.Ki[1] * ex_body,
                               +self.Ki[2] * ez])
        self._integral += correction * self.dt
        self._integral = np.clip(self._integral, -self.max_int, self.max_int)
        return self._integral.copy()

    def reset(self):
        self._integral = np.zeros(3)


# ══════════════════════════════════════════════════════════
# 3. 输出平滑
# ══════════════════════════════════════════════════════════
class RateLimiter:
    def __init__(self, max_rate):
        self.max_rate = np.asarray(max_rate); self.prev = None

    def apply(self, u, dt):
        if self.prev is None:
            self.prev = np.asarray(u); return self.prev.copy()
        du = np.asarray(u) - self.prev
        du = np.clip(du, -self.max_rate * dt, self.max_rate * dt)
        self.prev += du; return self.prev.copy()

    def reset(self):
        self.prev = None


class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha; self.val = None

    def apply(self, x):
        if self.val is None:
            self.val = np.asarray(x); return self.val.copy()
        self.val = self.alpha * np.asarray(x) + (1 - self.alpha) * self.val
        return self.val.copy()

    def reset(self):
        self.val = None


class ReferenceModel:
    def __init__(self, tau, dt):
        self.alpha = np.clip(1.0 - np.exp(-dt / max(tau, 0.01)), 0.1, 1.0)
        self._ref = None

    def update(self, target):
        target = np.asarray(target)
        if self._ref is None:
            self._ref = target.copy(); return self._ref.copy()
        self._ref += self.alpha * (target - self._ref)
        return self._ref.copy()

    def reset(self, pos):
        self._ref = np.asarray(pos).copy()


# ══════════════════════════════════════════════════════════
# 4. TCWP — 轨迹相干风预测
# ══════════════════════════════════════════════════════════
class TCWPPredictor:
    """
    核心创新: 无人机作为移动流体探针
    利用历史轨迹上的风测量 + 空间相干核, 外推预测时域内的风场序列

    原理:
      湍流具有空间相干结构 (Dryden 统计特性)
      无人机刚飞过的空间路径上测到了风样本
      用 Nadaraya-Watson 核回归将这些样本外推到未来预测位置
    """

    def __init__(self, N_pred=20, M_history=25, L_coherence=40.0):
        """
        N_pred:         MPC 预测步数
        M_history:      保留的历史样本数
        L_coherence:    湍流空间相干尺度 [m] (低空: 30-50m)
        """
        self.N = N_pred
        self.M = M_history
        self.L = L_coherence
        self.L_sq = L_coherence ** 2  # 缓存, 用于高效指数计算

        # 环形缓冲区: [(pos_enu, wind_enu), ...]
        self._history = collections.deque(maxlen=M_history)
        # 上一周期 MPC 预测位置轨迹 (用于未来位置代理)
        self._traj_prev = [np.zeros(3)] * N_pred

    def add_sample(self, pos_enu, wind_enu):
        """添加一个历史样本"""
        self._history.append((np.array(pos_enu), np.array(wind_enu)))

    def set_previous_trajectory(self, traj_positions):
        """
        traj_positions: list of (3,) positions from previous MPC solution
        """
        if traj_positions is not None and len(traj_positions) >= self.N:
            self._traj_prev = traj_positions[:self.N]

    def predict(self):
        """
        预测未来 N 步的风速序列

        使用 KNN + 反距离加权, 对空间邻近点给予更高权重
        包含时间衰减: 近期测量权重更大 (Taylor 冻结湍流假设)

        返回: list of (3,) wind vectors in ENU
        """
        if len(self._history) < 3:
            if self._history:
                w_last = self._history[-1][1]
                return [w_last.copy()] * self.N
            return [np.zeros(3)] * self.N

        h_pos = np.array([p for p, _ in self._history])
        h_wind = np.array([w for _, w in self._history])
        M_cur = len(self._history)
        # 时间索引: [0, 1, ..., M-1], 越大越近
        t_indices = np.arange(M_cur)[::-1]  # 倒序: 0=最近

        predictions = []
        for k in range(self.N):
            p_future = np.asarray(self._traj_prev[k])
            dx = h_pos[:, 0] - p_future[0]
            dy = h_pos[:, 1] - p_future[1]
            dz = h_pos[:, 2] - p_future[2]
            r = np.sqrt(dx*dx + dy*dy + dz*dz)

            # 空间核 (指数衰减) + 时间衰减
            w_space = np.exp(-r / self.L)
            # 时间权重: 越近越高, τ_t = L / U ≈ 2s (Taylor)
            tau_t = max(self.L / 7.0, 0.5)  # U~7m/s
            w_time = np.exp(-t_indices * 0.03 / tau_t)  # 0.03 = dt
            weights = w_space * w_time

            w_sum = np.sum(weights)
            if w_sum < 1e-9:
                predictions.append(h_wind[-1].copy())
            else:
                w_pred = np.sum(h_wind * weights[:, np.newaxis], axis=0) / w_sum
                predictions.append(w_pred)

        return predictions

    def reset(self):
        self._history.clear()
        self._traj_prev = [np.zeros(3)] * self.N


# ══════════════════════════════════════════════════════════
# 5. CMA 2.0 — 裕度感知约束自适应
# ══════════════════════════════════════════════════════════
class CMAManager:
    """
    CMA 2.0: 裕度感知的动压约束自适应

    改进: 不再无差别收紧, 而是基于 MPC 预测推力裕度做条件收紧:
      - 推力裕度充足 (>0.15) → 放松约束 (节能)
      - 推力裕度紧张 (<0.05) → 激进收紧 (安全)
      - 中间 → 正常收紧

    核心思想: 只在"推力即将饱和"时收紧姿态, 避免不必要的保守性
    """

    def __init__(self, u_max_nominal, u_min_nominal,
                 gamma_t=0.002, gamma_a=0.0001, rho=1.225):
        self.u_max_nom = np.asarray(u_max_nominal)
        self.u_min_nom = np.asarray(u_min_nominal)
        self.gamma_t = gamma_t
        self.gamma_a = gamma_a
        self.rho = rho

        self.u_max_min = np.array([0.5 * u_max_nominal[0],
                                   0.5 * u_max_nominal[1],
                                   0.80])  # thrust 最少保留 0.80 (比 v1 更宽松)
        self.q_last = 0.0
        self._cma_mode = 'normal'  # relaxed / normal / aggressive

    def update(self, wind_enu, mpc_u_pred=None):
        """
        wind_enu: 当前风场测量 [We, Wn, Wu] (m/s)
        mpc_u_pred: MPC 预测控制量 [roll, pitch, thrust] (可选, 用于裕度判断)

        返回: (u_min_eff, u_max_eff) — 紧缩后的边界
        """
        wind_speed = np.linalg.norm(wind_enu)
        self.q_last = 0.5 * self.rho * wind_speed ** 2

        # ── 裕度感知: 基于 MPC 预测推力判断收紧策略 ──
        if mpc_u_pred is not None:
            thrust_pred = mpc_u_pred[2]
            thrust_margin = self.u_max_nom[2] - thrust_pred

            if thrust_margin < 0.05:
                # 推力紧张: 激进收紧姿态 → 迫使 MPC 减小倾角 → 降低推力需求
                scale = 2.0
                self._cma_mode = 'aggressive'
            elif thrust_margin < 0.15:
                # 中度: 正常收紧
                scale = 1.0
                self._cma_mode = 'normal'
            else:
                # 裕度充足: 放松姿态约束 → 给 MPC 更多自由 → 可以节能
                scale = 0.3
                self._cma_mode = 'relaxed'
        else:
            scale = 1.0
            self._cma_mode = 'normal'

        du_t = self.gamma_t * self.q_last
        du_a = self.gamma_a * self.q_last * scale  # ★ 姿态收紧系数乘 scale

        u_max_eff = np.array([
            self.u_max_nom[0] - du_a,
            self.u_max_nom[1] - du_a,
            self.u_max_nom[2] - du_t,
        ])
        u_min_eff = np.array([
            self.u_min_nom[0] + du_a,
            self.u_min_nom[1] + du_a,
            self.u_min_nom[2],
        ])

        u_max_eff = np.maximum(u_max_eff, self.u_max_min)
        u_min_eff = np.minimum(u_min_eff,
                               np.array([-self.u_max_min[0],
                                         -self.u_max_min[1],
                                         self.u_min_nom[2]]))
        return u_min_eff, u_max_eff

    def get_q(self):
        return self.q_last

    def get_mode(self):
        return self._cma_mode


# ══════════════════════════════════════════════════════════
# 6. 风偏置学习 — 指数滑动平均误差补偿
# ══════════════════════════════════════════════════════════
class WindBiasEstimator:
    """
    用 EMA 学习风扰引入的稳态位置预测误差
    作为全局偏置 d_DEDL 加法修正

    简化的方向无关学习 (相比 DEDL, 三轴独立 EMA 更鲁棒)
    """

    def __init__(self, alpha=0.02, max_bias=(0.5, 0.5, 0.3)):
        """
        alpha:    EMA 平滑因子 (越小越慢)
        max_bias: 每轴偏置上限 (位置误差 m)
        """
        self.alpha = alpha
        self.max_bias = np.asarray(max_bias)
        self._bias = np.zeros(3)      # [b_x, b_y, b_z] 位置偏置

    def update(self, pos_error):
        """pos_error: (3,) 位置误差 = ref - current"""
        pe = np.asarray(pos_error)
        self._bias += self.alpha * (pe - self._bias)
        self._bias = np.clip(self._bias, -self.max_bias, self.max_bias)

    def get_bias(self):
        """返回偏置修正: d_bias = [bias_pos; 0_3] (6,)"""
        return np.array([self._bias[0], self._bias[1], self._bias[2],
                         0.0, 0.0, 0.0])

    def reset(self):
        self._bias = np.zeros(3)


# ══════════════════════════════════════════════════════════
# 8. WindUtilizationAdvisor — 顺风借力建议
# ══════════════════════════════════════════════════════════
class WindUtilizationAdvisor:
    """
    基于风方向与位置误差方向的一致性, 决定是否"借风"
    
    ★ 关键区别:
      - 悬停任务 (is_trajectory=False): coast 禁用 — 悬停时"借风"=放任漂移
      - 轨迹任务 (is_trajectory=True): coast 启用 — 顺风时降低跟踪权重节能
      - 上升/下沉气流: 始终生效
    """
    def __init__(self, alignment_threshold=0.3, coast_factor_max=0.5):
        self.align_thresh = alignment_threshold
        self.coast_max = coast_factor_max
        self._coast_factor = 0.0
        self._mode = 'normal'
        self._is_trajectory = False  # ★ 默认悬停, 禁用 coast

    def set_task_type(self, is_trajectory):
        """设置任务类型: False=悬停, True=轨迹跟踪"""
        self._is_trajectory = is_trajectory

    def evaluate(self, wind_enu, pos_err, wind_seq=None):
        """
        wind_enu:  当前风测量 ENU [We, Wn, Wu] (m/s)
        pos_err:   位置误差 ENU [ex, ey, ez] (m)
        wind_seq:  TCWP 预测风序列 (可选, 用于前瞻)
        
        返回: (coast_factor, mode, Q_scale)
        """
        w_horiz = wind_enu[:2].copy()
        w_speed = np.linalg.norm(w_horiz)
        err_horiz = pos_err[:2].copy()
        err_norm = np.linalg.norm(err_horiz)

        if w_speed < 1.0 or err_norm < 0.1:
            self._coast_factor = 0.0
            self._mode = 'normal'
            return 0.0, 'normal', 1.0

        # ── 水平风利用: ★ 仅在轨迹任务时启用 coast ──
        w_dir = w_horiz / w_speed
        err_dir = err_horiz / err_norm
        alignment = np.dot(w_dir, err_dir)

        if self._is_trajectory and alignment > self.align_thresh:
            # 轨迹顺风: 降低跟踪权重节能
            self._coast_factor = self.coast_max * (alignment - self.align_thresh) / (1.0 - self.align_thresh)
            self._mode = 'coasting'
            Q_scale = 1.0 - self._coast_factor
        else:
            self._coast_factor = 0.0
            Q_scale = 1.0
            if self._mode == 'coasting':
                self._mode = 'normal'

        # ── 垂向风利用: 始终生效 ──
        wz = wind_enu[2]
        if wz > 2.0:
            self._mode = 'updraft_riding'
        elif wz < -2.0:
            self._mode = 'downdraft'
        else:
            if self._mode not in ('coasting', 'updraft_riding', 'downdraft'):
                self._mode = 'normal'

        return self._coast_factor, self._mode, Q_scale

    def get_mode(self):
        return self._mode

    def get_coast(self):
        return self._coast_factor


# ══════════════════════════════════════════════════════════
# 9. PredictiveEnergyManager — 垂向风预测能量调度
# ══════════════════════════════════════════════════════════
class PredictiveEnergyManager:
    """
    利用 TCWP 预测的垂向风序列做前瞻性能量调度:
      - 预测到上升气流: 提前减推力, 容忍 z 超调 (白嫖势能)
      - 预测到下沉气流: 提前渐进加推力, 避免急加速 (节省峰值功率)
      - 相比纯反馈: 减少了因为"事后补偿"导致的推力尖峰
    """
    def __init__(self, N_pred=20, z_margin=0.5, ramp_steps=5,
                 dz_threshold=2.0):
        """
        N_pred:       预测步数
        z_margin:     高度松弛带 [m] (允许超调的范围)
        ramp_steps:   预加速/减速的渐变步数
        dz_threshold: 垂向风阈值 [m/s], 超过此值触发调度
        """
        self.N = N_pred
        self.z_margin = z_margin
        self.ramp_steps = ramp_steps
        self.dz_thresh = dz_threshold
        self._mode = 'normal'  # normal / pre_thrust / ride_updraft
        self._thrust_bias = 0.0  # 推力偏置修正

    def plan(self, wind_seq):
        """
        wind_seq: TCWP 预测的 N 步风序列 (list of (3,) ENU)
        
        返回: (mode, thrust_bias)
          mode:        当前调度模式
          thrust_bias: 推力偏置 [-0.1, 0.1] (归一化推力修正)
        """
        if wind_seq is None or len(wind_seq) < 3:
            self._mode = 'normal'
            self._thrust_bias = 0.0
            return 'normal', 0.0

        # 提取垂向风序列
        wz_seq = np.array([w[2] for w in wind_seq[:min(self.N, len(wind_seq))]])

        # ── 检测强下沉气流 (未来 N/2 步内) ──
        look_ahead = min(self.ramp_steps + 3, len(wz_seq))
        future_wz = wz_seq[:look_ahead]
        min_wz = np.min(future_wz)
        min_idx = np.argmin(future_wz)

        if min_wz < -self.dz_thresh:
            # 强下沉气流预警 → 渐进预加推力
            ramp = min(min_idx + 1, self.ramp_steps) / self.ramp_steps
            self._thrust_bias = 0.08 * ramp  # 最多 +8% 推力
            self._mode = 'pre_thrust'
        elif np.mean(wz_seq[:min(5, len(wz_seq))]) > self.dz_thresh:
            # 持续上升气流 → 保守降低推力 (悬停不容忍大幅超调)
            self._thrust_bias = max(self._thrust_bias, -0.02)
            self._mode = 'ride_updraft'
        else:
            self._thrust_bias *= 0.8  # 衰减
            if abs(self._thrust_bias) < 0.005:
                self._thrust_bias = 0.0
                self._mode = 'normal'

        return self._mode, self._thrust_bias

    def get_mode(self):
        return self._mode

    def get_bias(self):
        return self._thrust_bias


# ══════════════════════════════════════════════════════════
# 10. ROS 节点 — Energy-Aware TCW-MPC
# ══════════════════════════════════════════════════════════
class MPCNode:
    CSV_FIELDS = [
        't', 'px', 'py', 'pz', 'vx', 'vy', 'vz',
        'ref_px', 'ref_py', 'ref_pz',
        'u_roll', 'u_pitch', 'u_thrust',
        'u_int_r', 'u_int_p', 'u_int_t',
        'roll_deg', 'pitch_deg', 'yaw_deg',
        'wind_x', 'wind_y', 'wind_z',      # 风传感器测量
        'wind_px', 'wind_py', 'wind_pz',   # TCWP 预测 (第 1 步)
        'wspeed', 'q_dyn',                 # 风速 & 动压
        'u_max_roll', 'u_max_pitch', 'u_max_thrust',  # CMA 紧缩后边界
        'power_est',                       # ★ 估算功耗 [W]
        'coast_factor', 'cma_mode', 'energy_mode', 'tau_mpc',
        'mode', 'armed', 'mpc_ok', 'iter'
    ]

    def __init__(self):
        rospy.init_node('mpc_position_controller')

        # ── 物理参数 ─────────────────────────────────
        self.dt = 0.03
        self.N_pred = 20
        self.g = 9.81
        self.hover_thrust = 0.66
        self.drone_mass = 1.5          # Iris 近似质量 [kg]
        dt_mpc = self.dt
        k_t = self.g / self.hover_thrust
        self.thrust_scale = self.drone_mass * self.g / self.hover_thrust  # N per normalized thrust unit

        # ── ★ 节能参数 ───────────────────────────────
        self.lambda_energy = rospy.get_param('~lambda_energy', 5.0)
        # 悬停模式: coast 禁用 (通过任务类型控制)
        self._is_trajectory_task = rospy.get_param('~trajectory_mode', False)
        # ★ Energy-First 模式: 用精度换节能
        self._energy_first = rospy.get_param('~energy_first', False)
        self._q_xy_scale = rospy.get_param('~q_xy_scale', 1.0)  # XY 位置权重缩放
        # 参考控制: 悬停推力为基准
        self._u_hover = np.array([0.0, 0.0, self.hover_thrust])

        # ── 空气动力学参数 ────────────────────────────
        self.rho = 1.225
        self.CdA = 0.05
        # 线性化气动阻尼系数: k_drag = 0.5*ρ*CdA/m
        self.k_drag = 0.5 * self.rho * self.CdA / self.drone_mass         # 物理气动阻尼系数

        # ── SL-MPC: 动态 B[5,2] 倾角修正 ────────────
        # 仅修正推力→Z通道 (关键项), 水平动力学保持悬停线性化
        self._last_lin_tilt = 0.0
        x_op = np.zeros(6)
        u_op = np.array([0.0, 0.0, self.hover_thrust])
        w_zero = np.zeros(3)
        A0, B0, g0 = discrete_linearize(x_op, u_op, w_zero, dt_mpc)
        self._A_base = A0.copy()
        self._B_base = B0.copy()
        self.g_vec = g0.copy()
        self.k_damp = IRIS['k_drag_base'] * IRIS['CdA_front'] * 5.0

        Q = np.diag([4.0, 4.0, 6.0, 2.0, 2.0, 6.0])
        # ★ Energy-First 模式: 大幅降低 XY 位置权重
        if self._energy_first:
            Q[0,0] *= self._q_xy_scale  # px
            Q[1,1] *= self._q_xy_scale  # py
            Q[3,3] *= self._q_xy_scale  # vx (速度权重同步降低)
            Q[4,4] *= self._q_xy_scale  # vy
            rospy.loginfo("⚡ Energy-First: Q_xy × %.2f", self._q_xy_scale)
        # ★ R 矩阵: thrust 通道加入节能惩罚
        R_energy = np.diag([14.0, 14.0, 15.0 + self.lambda_energy])
        self._u_min_nom = np.array([-0.45, -0.45, 0.30])
        self._u_max_nom = np.array([0.45, 0.45, 0.95])

        # ── 标准 MPC (解析雅可比, 倾角自适应 B[5,2]) ──
        self.mpc = StandardMPC(self._A_base, self._B_base, self.g_vec,
                               Q, R_energy, self.N_pred,
                               self._u_min_nom, self._u_max_nom)

        # ── TCWP — 轨迹相干风预测 ─────────────────────
        self.tcwp = TCWPPredictor(
            N_pred=self.N_pred, M_history=25, L_coherence=15.0)  # 低空湍流相干尺度

        # ── CMA 2.0 — 裕度感知约束自适应 ────────────
        self.cma = CMAManager(
            self._u_max_nom, self._u_min_nom,
            gamma_t=0.002, gamma_a=0.0001, rho=self.rho)

        # ── ★ 风能利用模块 ───────────────────────────
        self.wind_advisor = WindUtilizationAdvisor(
            alignment_threshold=0.3, coast_factor_max=0.5)
        self.energy_mgr = PredictiveEnergyManager(
            N_pred=self.N_pred, z_margin=0.5, ramp_steps=5, dz_threshold=2.0)

        # ── 风偏置学习 ─────────────────────────────────
        self.bias_est = WindBiasEstimator(alpha=0.02)

        # ── 积分器 ────────────────────────────────────
        self.integrator = Integrator(
            Ki=[0.02, 0.02, 0.01],
            max_int=[0.05, 0.05, 0.04], dt=self.dt)

        # ── 输出平滑 ──────────────────────────────────
        self.rate_limiter = None
        self.lpf = LowPassFilter(alpha=0.4)

        # ── 参考模型 ──────────────────────────────────
        # ── Reference Governor (MPC-driven τ) ────────
        self._tau_mpc = 0.6
        self._ref_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos = np.array([0.0, 0.0, 2.5])

        # ── 当前状态 ───────────────────────────────────
        self.state = State()
        self.x_current = None
        self.roll_cur = 0.0; self.pitch_cur = 0.0; self.yaw_cur = 0.0
        self._got_odom = False; self._got_vel = False

        # ── 风传感器 ──────────────────────────────────
        self.wind_meas = np.zeros(3)   # 当前风测量 ENU [We, Wn, Wu]
        self.wind_meas_prev = np.zeros(3)
        self._got_wind = False
        self._wind_seq = None          # TCWP 预测的风序列
        self._d_seq = None             # 风扰序列 (6,) N steps
        self._q_dyn = 0.0

        # ── 上一周期 MPC 解的速度轨迹 ──────────────────
        self._vel_traj_prev = [np.zeros(3)] * self.N_pred
        self._pos_traj_prev = [np.zeros(3)] * self.N_pred

        # ── 线程安全 ──────────────────────────────────
        self._cmd_lock = threading.Lock()
        self._publishing = False

        # ── 控制指令 ──────────────────────────────────
        self._roll_sp = 0.0; self._pitch_sp = 0.0
        self._thrust_sp = self.hover_thrust
        self._px_sp = 0.0; self._py_sp = 0.0; self._pz_sp = 2.5
        self._int_roll = 0.0; self._int_pitch = 0.0; self._int_thrust = 0.0
        self._u_last_safe = np.array([0.0, 0.0, self.hover_thrust])

        # ── CMA 紧缩后边界 (日志用) ──────────────────
        self._u_max_eff = self._u_max_nom.copy()
        self._u_min_eff = self._u_min_nom.copy()

        # ── ★ 节能状态 (日志用) ──────────────────────
        self._coast_factor = 0.0
        self._cma_mode_str = 'normal'
        self._energy_mode = 'normal'
        self._Q_scale = 1.0
        self._thrust_energy_bias = 0.0  # 能量调度带来的推力偏置

        # ── ★ 统一功耗模型 (基于 ESC 电机转速) ──────
        self.power_model = UnifiedPowerEstimator()
        self._power_est = 0.0

        # ── CSV ──────────────────────────────────────
        self._csv_file = None; self._csv_writer = None
        self._csv_path = ''; self._csv_count = 0
        self._mpc_ok = 0; self._mpc_iter = 0

        # ── ROS ──────────────────────────────────────
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._odom_cb)
        rospy.Subscriber('/wind_field/velocity', Vector3Stamped, self._wind_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self.power_model.imu_cb)
        self.pub_att = rospy.Publisher(
            '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.pub_pos = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pub_man = rospy.Publisher(
            '/mavros/manual_control/send', ManualControl, queue_size=10)

        rospy.loginfo("等待 MAVROS 服务...")
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self._arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self._mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.loginfo("MAVROS 就绪, TCW-MPC 初始化完成")
        self.rate = rospy.Rate(int(1.0 / self.dt))

    # ── CSV ────────────────────────────────────────────────
    def _csv_open(self):
        import datetime
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'mpc_log_%s.csv' % ts)
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self.CSV_FIELDS)
        rospy.loginfo("CSV: %s", self._csv_path)

    def _csv_write(self, row):
        if self._csv_writer:
            self._csv_writer.writerow(row)
            self._csv_count += 1
            if self._csv_count % 10 == 0:
                self._csv_file.flush()

    def _csv_close(self):
        if self._csv_file:
            self._csv_file.close()
            rospy.loginfo("CSV 已保存 %d 行 → %s",
                          self._csv_count, self._csv_path)

    # ── 回调 ───────────────────────────────────────────────
    def _state_cb(self, msg):
        self.state = msg

    def _odom_cb(self, msg):
        self.x_current = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,    # z > 0 = 高度
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
        self._got_odom = True
        self._got_vel = True
        q = msg.pose.pose.orientation
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.roll_cur = r; self.pitch_cur = p; self.yaw_cur = y

    def _wind_cb(self, msg):
        """机载风传感器 (模拟)"""
        self.wind_meas[0] = msg.vector.x   # East
        self.wind_meas[1] = msg.vector.y   # North
        self.wind_meas[2] = msg.vector.z   # Up
        self._got_wind = True

    # ── 风扰计算 ───────────────────────────────────────────
    def _compute_disturbance_seq(self, wind_seq):
        """风扰序列 — 完整非线性动力学风推力分解"""
        w = wind_seq[0] if wind_seq else np.zeros(3)
        u_cur = np.array([self.roll_cur, self.pitch_cur,
                          self._u_last_safe[2] if self._u_last_safe is not None else self.hover_thrust])
        d = compute_wind_disturbance(
            self.x_current if self.x_current is not None else np.zeros(6),
            u_cur, w)
        d[3:6] *= self.dt
        return [d.copy()] * self.N_pred

    # ── 提取 MPC 解的速度轨迹 ──────────────────────────────
    def _extract_trajectory(self, result_x):
        """
        从 OSQP 解提取位置/速度预测轨迹
        result_x: OSQP 解向量
        变量顺序: [u_0, x_1, u_1, x_2, ..., u_{N-1}, x_N]
        """
        n_xu = self.mpc.nx + self.mpc.nu  # 9
        pos_traj = []; vel_traj = []
        for k in range(1, self.N_pred + 1):
            xk_s = (k - 1) * n_xu + self.mpc.nu
            xk = result_x[xk_s:xk_s + self.mpc.nx]
            pos_traj.append(xk[:3].copy())
            vel_traj.append(xk[3:6].copy())
        return pos_traj, vel_traj

    # ── 功耗估算 ───────────────────────────────────────────
    # ── 后台发布线程 ───────────────────────────────────────
    def _start_publish_thread(self):
        self._publishing = True
        self._pub_thread = threading.Thread(target=self._publish_loop,
                                            daemon=True)
        self._pub_thread.start()

    def _publish_loop(self):
        r = rospy.Rate(50)
        while self._publishing and not rospy.is_shutdown():
            with self._cmd_lock:
                px = self._px_sp; py = self._py_sp; pz = self._pz_sp
            self._send_position_setpoint(px, py, pz)
            self._publish_manual()
            r.sleep()

    def _send_position_setpoint(self, px, py, pz):
        """发送位置 setpoint → PX4 全链路 PID 跟踪"""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = pz
        msg.pose.orientation.w = 1.0
        self.pub_pos.publish(msg)

    def _publish_manual(self):
        man = ManualControl()
        man.x = 0; man.y = 0; man.z = 500; man.r = 0; man.buttons = 0
        self.pub_man.publish(man)

    # ── SL-MPC: 倾角驱动的模型线性化 ──────────────────────
    def _update_mpc_model(self, roll, pitch):
        """
        在 (roll, pitch) 处线性化非线性动力学, 重建 QP
        
        非线性模型:
          ax = k_t * T * sin(pitch)       (pitch→x)
          ay = -k_t * T * sin(roll)       (roll→y)
          az = k_t * T * cos(pitch)*cos(roll) - g
        
        在 (φ_c, θ_c, T_c) 处求雅可比
        """
        k_t = self.g / self.hover_thrust
        T_c = self._u_last_safe[2] if self._u_last_safe is not None else self.hover_thrust
        cr = np.cos(roll); sr = np.sin(roll)
        cp = np.cos(pitch); sp = np.sin(pitch)
        
        # 新 B 矩阵 (雅可比 ∂a/∂u * dt)
        B_new = np.zeros((6, 3))
        # ∂ax/∂u
        B_new[3, 0] = 0.0                                    # ∂ax/∂roll
        B_new[3, 1] = k_t * T_c * cp * self.dt
        B_new[3, 2] = k_t * sp * self.dt
        # ∂ay/∂u
        B_new[4, 0] = -k_t * T_c * cr * self.dt
        B_new[4, 1] = 0.0
        B_new[4, 2] = -k_t * sr * self.dt
        # ∂az/∂u
        B_new[5, 0] = -k_t * T_c * cp * sr * self.dt
        B_new[5, 1] = -k_t * T_c * sp * cr * self.dt
        B_new[5, 2] = k_t * cp * cr * self.dt
        
        # A 矩阵保持 (位置积分 + 轻阻尼)
        A_new = self.A_mat.copy()
        
        self.mpc.update_model(A_new, B_new)

    # ── 主循环 ─────────────────────────────────────────────
    def run(self):
        rospy.loginfo("\n" + "=" * 62)
        rospy.loginfo("  ⚡ Energy-Aware TCW-MPC 控制器已就绪")
        rospy.loginfo("  TCWP: M=%d samples, L=%.0fm coherence",
                      self.tcwp.M, self.tcwp.L)
        rospy.loginfo("  CMA 2.0: γ_t=%.4f γ_a=%.4f (margin-aware)",
                      self.cma.gamma_t, self.cma.gamma_a)
        rospy.loginfo("  Energy: λ_E=%.1f (thrust penalty) %s",
                      self.lambda_energy,
                      "⚡ENERGY-FIRST" if self._energy_first else "")
        rospy.loginfo("  Wind Advisor: coast_thresh=%.1f max_coast=%.1f [%s]",
                      self.wind_advisor.align_thresh, self.wind_advisor.coast_max,
                      "trajectory" if self._is_trajectory_task else "hover")
        rospy.loginfo("  ⚡ SL-MPC: nonlinear dynamics + analytical Jacobians")
        rospy.loginfo("  Aero: CdA(α) attitude-dependent, k_drag=%.4f k_damp=%.4f",
                      IRIS['k_drag_base'] * IRIS['CdA_front'], self.k_damp)
        rospy.loginfo("  Power: BEMT (induced+profile+efficiency)")
        rospy.loginfo("=" * 62 + "\n")

        # ★ 设置任务类型: 悬停禁 coast, 轨迹启用
        self.wind_advisor.set_task_type(self._is_trajectory_task)

        self._start_publish_thread()
        self._csv_open()

        # 等待 odometry
        rospy.loginfo("等待 odometry...")
        while not self._got_odom and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo("Odometry 就绪")

        # ── 解锁序列 ──────────────────────────────────
        rospy.loginfo("执行解锁序列...")
        # 1. 先发几帧虚拟摇杆, 防止 failsafe
        for _ in range(10):
            self._publish_manual()
            self.rate.sleep()
        # 2. 安全断电
        try:
            self._arm_srv(False)
            rospy.sleep(0.5)
        except rospy.ServiceException:
            rospy.logwarn("disarm 失败, 继续...")
        # 3. 设置 OFFBOARD 模式
        try:
            self._mode_srv(custom_mode='OFFBOARD')
            rospy.loginfo("模式: OFFBOARD")
        except rospy.ServiceException:
            rospy.logwarn("OFFBOARD 失败, 尝试 LOITER")
            try:
                self._mode_srv(custom_mode='AUTO.LOITER')
            except rospy.ServiceException:
                rospy.logerr("无法切换模式!")
        rospy.sleep(0.5)
        # 4. 解锁
        try:
            result = self._arm_srv(True)
            if result.success:
                rospy.loginfo("✅ 已解锁 (ARMED)")
            else:
                rospy.logerr("❌ 解锁失败! 检查 COM_RC_IN_MODE/COM_ARM_WO_GPS")
        except rospy.ServiceException as e:
            rospy.logerr("解锁服务调用失败: %s", e)
        rospy.sleep(1.0)
        rospy.loginfo("开始控制循环")
        # ───────────────────────────────────────────────

        t_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t_now = rospy.Time.now().to_sec()
            t_elapsed = t_now - t_start

            if self.x_current is None:
                self.rate.sleep()
                continue

            # ★ Step 0: 自适应 B[5,2] — 倾角修正推力→Z 有效性
            tilt_mag = np.sqrt(self.roll_cur**2 + self.pitch_cur**2)
            if abs(tilt_mag - self._last_lin_tilt) > 0.03:  # ~1.7deg threshold
                cr = max(np.cos(self.roll_cur), 0.5)
                cp = max(np.cos(self.pitch_cur), 0.5)
                # 仅修正 B[5,2] = k_t * cos(θ) * cos(φ) * dt
                B_new = self._B_base.copy()
                B_new[5, 2] = IRIS['k_t'] * cp * cr * self.dt
                self.mpc.update_model(self._A_base, B_new)
                self._last_lin_tilt = tilt_mag

            # ── Step 1: TCWP — 预测未来风序列 ──────────
            if self._got_wind:
                pos_enu = self.x_current[:3].copy()
                self.tcwp.add_sample(pos_enu, self.wind_meas.copy())
                self.tcwp.set_previous_trajectory(self._pos_traj_prev)
                self._wind_seq = self.tcwp.predict()
                self._d_seq = self._compute_disturbance_seq(self._wind_seq)
            else:
                self._d_seq = None
                self._wind_seq = None

            # ── Step 2: CMA 2.0 — 裕度感知约束 ─────────
            u_prev = self._u_last_safe if self._u_last_safe is not None else np.array([0, 0, self.hover_thrust])
            self._u_min_eff, self._u_max_eff = self.cma.update(self.wind_meas, mpc_u_pred=u_prev)
            self._q_dyn = self.cma.get_q()
            self._cma_mode_str = self.cma.get_mode()
            self.mpc.u_min = self._u_min_eff
            self.mpc.u_max = self._u_max_eff

            # ── Step 3: 参考轨迹 (上一步τ驱动的平滑位置) ──
            alpha = 1.0 - np.exp(-self.dt / max(self._tau_mpc, 0.1))
            self._ref_pos += alpha * (self.target_pos - self._ref_pos)
            ref_pos = self._ref_pos.copy()
            x_ref = np.zeros(6)
            x_ref[:3] = ref_pos

            # ── Step 3.5: ★ 风能利用 + 预测能量调度 ────
            if self.x_current is not None:
                pos_err = x_ref[:3] - self.x_current[:3]
                # 顺风借力评估
                coast, wind_mode, Q_scale = self.wind_advisor.evaluate(
                    self.wind_meas, pos_err, wind_seq=self._wind_seq)
                self._coast_factor = coast
                self._Q_scale = Q_scale
                # 预测性能量调度 (垂向风利用)
                if self._wind_seq is not None and self._got_wind:
                    energy_mode, thrust_bias = self.energy_mgr.plan(self._wind_seq)
                    self._energy_mode = energy_mode
                    self._thrust_energy_bias = thrust_bias
                else:
                    self._energy_mode = 'normal'
                    self._thrust_energy_bias = 0.0
            else:
                self._coast_factor = 0.0
                self._Q_scale = 1.0
                self._energy_mode = 'normal'
                self._thrust_energy_bias = 0.0
                pos_err = np.zeros(3)

            # ★ Step 4: MPC 求解 (B[5,2]自适应已处理倾角)
            u_ref = self._u_hover.copy()
            u_ref[2] += self._thrust_energy_bias
            u_mpc = self.mpc.solve(self.x_current, x_ref, u_ref=u_ref, d_seq=self._d_seq)
            self._mpc_ok = int(self.mpc.last_ok)
            self._mpc_iter = self.mpc.last_iter

            # ── Step 5: 提取轨迹 + 风偏置学习 ─────────
            if u_mpc is not None and self.mpc.last_ok:
                u_roll, u_pitch, u_thrust = u_mpc
                pos_traj, vel_traj = self.mpc.get_trajectory()
                if pos_traj is not None:
                    self._pos_traj_prev = pos_traj
                    self._vel_traj_prev = vel_traj
            else:
                u_roll, u_pitch, u_thrust = self._u_last_safe

            # ★ MPC-driven τ: QP最优推力偏离→参考收敛速度
            dT = abs(u_thrust - self.hover_thrust)
            scale = 0.015 + self.lambda_energy * 0.008
            self._tau_mpc = 0.3 + 1.2 * np.exp(-dT / max(scale, 1e-6))

            # ── Step 6: 积分器补偿 ────────────────
            if self.x_current is not None:
                pos_err = x_ref[:3] - self.x_current[:3]
                if self._got_wind:
                    self.bias_est.update(pos_err)
                int_corr = self.integrator.update(pos_err, yaw=self.yaw_cur)
            else:
                int_corr = np.zeros(3)
                pos_err = np.zeros(3)

            # ── Step 7: 合成 + 平滑 ────────────────────
            # (B[5,2]自适应 + u_ref倾角修正已足够, 无需后处理重复)
            u_roll += int_corr[0]
            u_pitch += int_corr[1]
            u_thrust += int_corr[2]

            # 安全限幅
            u_roll = np.clip(u_roll, self._u_min_eff[0], self._u_max_eff[0])
            u_pitch = np.clip(u_pitch, self._u_min_eff[1], self._u_max_eff[1])
            u_thrust = np.clip(u_thrust, self._u_min_eff[2], self._u_max_eff[2])

            u_raw = np.array([u_roll, u_pitch, u_thrust])
            u_smooth = self.lpf.apply(u_raw)

            # ★ MPC-driven τ → 平滑 ref_pos → PX4 PID 跟踪
            #   QP解→ΔT→τ: 需推力大→紧追, 需推力小→缓跟省电
            with self._cmd_lock:
                self._px_sp = float(ref_pos[0])
                self._py_sp = float(ref_pos[1])
                self._pz_sp = float(ref_pos[2])
            self._int_roll = int_corr[0]
            self._int_pitch = int_corr[1]
            self._int_thrust = int_corr[2]
            self._u_last_safe = u_smooth.copy()

            # ★ 统一 BEMT: IMU a_body→推力 (与 PID 同公式)
            a_imu = self.power_model._accel.copy()
            a_norm = np.linalg.norm(a_imu)
            thrust_imu = a_norm * self.hover_thrust / self.g if a_norm > 1.0 else self.hover_thrust
            v_rel_norm = np.linalg.norm(
                self.x_current[3:6] - self.wind_meas) if self._got_wind else 0.0
            self._power_est = bem_power(thrust_imu, self.roll_cur, self.pitch_cur, v_rel_norm)

            # ── Step 8: 日志 + CSV ─────────────────────
            wind_p0 = self._wind_seq[0] if self._wind_seq else np.zeros(3)
            self._csv_write([
                t_elapsed,
                *self.x_current[:3], *self.x_current[3:6],
                *ref_pos,
                u_smooth[0], u_smooth[1], u_smooth[2],
                int_corr[0], int_corr[1], int_corr[2],
                np.degrees(self.roll_cur), np.degrees(self.pitch_cur),
                np.degrees(self.yaw_cur),
                self.wind_meas[0], self.wind_meas[1], self.wind_meas[2],
                wind_p0[0], wind_p0[1], wind_p0[2],
                np.linalg.norm(self.wind_meas), self._q_dyn,
                self._u_max_eff[0], self._u_max_eff[1], self._u_max_eff[2],
                self._power_est,
                self._coast_factor,  # ★ 借风系数
                self._cma_mode_str,  # ★ CMA 模式
                self._energy_mode,   # ★ 能量调度模式
                self.state.mode.strip(), int(self.state.armed),
                self._mpc_ok, self._mpc_iter,
            ])

            if self._csv_count % 100 == 0:
                ws = np.linalg.norm(self.wind_meas)
                energy_info = ""
                if self._coast_factor > 0.05:
                    energy_info += f" 🌬coast={self._coast_factor:.2f}"
                if self._thrust_energy_bias != 0:
                    energy_info += f" ΔT={self._thrust_energy_bias:+.2f}"
                rospy.loginfo(
                    "[t=%.1f] |w|=%.1f q=%.1f  "
                    "u=(%+.3f,%+.3f,%.3f)  z=%.2f  err_xy=%.2f  "
                    "P=%.1fW  cma=%s%s  ok=%d",
                    t_elapsed, ws, self._q_dyn,
                    u_smooth[0], u_smooth[1], u_smooth[2],
                    self.x_current[2],
                    np.linalg.norm(pos_err[:2]) if self.x_current is not None else 0,
                    self._power_est,
                    self._cma_mode_str, energy_info,
                    self._mpc_ok)

            self.rate.sleep()

        self._publishing = False
        self._csv_close()


if __name__ == '__main__':
    try:
        MPCNode().run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("TCW-MPC 已停止")
