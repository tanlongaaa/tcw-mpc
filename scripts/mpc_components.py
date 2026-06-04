#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mpc_components.py — TCW-MPC 控制组件
=====================================
与求解器无关的控制辅助模块, 负责:
  - 积分器 (PI 位置误差修正)
  - 输出平滑 (速率限幅 + 低通滤波)
  - 参考轨迹平滑 (一阶低通参考模型)
  - TCWP 风预测 (轨迹相干风预测)
  - CMA 约束自适应 (裕度感知)
  - 风能利用建议 + 预测能量调度
"""

import collections
import numpy as np


# ══════════════════════════════════════════════════════════
# 1. 积分器 — PI 位置误差修正 (机体系映射)
# ══════════════════════════════════════════════════════════
class Integrator:
    """外置 PI 积分器: 位置误差 → Yaw旋转 → 控制修正

    6D 模式: 输出 [d_roll, d_pitch, d_thrust] (Euler角+推力)
    10D 模式: 输出 [d_fc, d_wx, d_wy, d_wz] (推力+体轴角速度)
    """

    def __init__(self, Ki, max_int, dt, model='6d'):
        self.Ki = np.asarray(Ki)
        self.max_int = np.asarray(max_int)
        self.dt = dt
        self.model = model
        self._integral = np.zeros_like(self.max_int)

    def update(self, pos_error, yaw=0.0):
        """
        pos_error: ENU 位置误差 [ex, ey, ez]
        yaw:      当前偏航角 [rad] (仅 6D 模式需要)
        返回:     控制修正向量 (nu,)
        """
        ex, ey, ez = pos_error
        # ENU → Body 旋转
        cos_y = np.cos(yaw); sin_y = np.sin(yaw)
        ex_body =  cos_y * ex + sin_y * ey
        ey_body = -sin_y * ex + cos_y * ey

        if self.model == '6d':
            # 机体系: 前向误差 → pitch, 右向误差 → -roll
            correction = np.array([-self.Ki[0] * ey_body,
                                   +self.Ki[1] * ex_body,
                                   +self.Ki[2] * ez])
        else:
            # 10D CTBR: 前向误差 → pitch rate (ωy)
            #            右向误差 → roll rate (-ωx)
            #            高度误差 → thrust (f_c)
            correction = np.array([+self.Ki[2] * ez,
                                   -self.Ki[0] * ey_body,
                                   +self.Ki[1] * ex_body,
                                   0.0])

        self._integral += correction * self.dt
        self._integral = np.clip(self._integral, -self.max_int, self.max_int)
        return self._integral.copy()

    def reset(self):
        self._integral = np.zeros_like(self.max_int)


# ══════════════════════════════════════════════════════════
# 2. 输出平滑
# ══════════════════════════════════════════════════════════
class RateLimiter:
    """输出变化率限幅"""

    def __init__(self, max_rate):
        self.max_rate = np.asarray(max_rate)
        self._prev = None

    def apply(self, u):
        if self._prev is None:
            self._prev = np.asarray(u)
            return self._prev.copy()
        du = np.asarray(u) - self._prev
        du = np.clip(du, -self.max_rate, self.max_rate)
        self._prev = self._prev + du
        return self._prev.copy()

    def reset(self):
        self._prev = None


class LowPassFilter:
    """EMA 一阶低通滤波器"""

    def __init__(self, alpha=0.4):
        self.alpha = alpha
        self._val = None

    def apply(self, x):
        x = np.asarray(x)
        if self._val is None:
            self._val = x.copy()
        else:
            self._val += self.alpha * (x - self._val)
        return self._val.copy()

    def reset(self):
        self._val = None


# ══════════════════════════════════════════════════════════
# 3. 参考轨迹平滑
# ══════════════════════════════════════════════════════════
class ReferenceModel:
    """一阶低通参考轨迹生成器"""

    def __init__(self, tau=0.6, dt=0.03):
        self.tau = tau
        self.dt = dt
        self._ref = None

    def step(self, target):
        target = np.asarray(target)
        alpha = 1.0 - np.exp(-self.dt / max(self.tau, 0.05))
        if self._ref is None:
            self._ref = target.copy()
        else:
            self._ref += alpha * (target - self._ref)
        return self._ref.copy()

    def reset(self):
        self._ref = None


# ══════════════════════════════════════════════════════════
# 4. TCWP — 轨迹相干风预测
# ══════════════════════════════════════════════════════════
class TCWPPredictor:
    """
    无人机作为移动流体探针, 利用历史轨迹风测量 + 空间相干核
    外推预测时域风场序列 (Nadaraya-Watson 核回归)
    """

    def __init__(self, N_pred=20, M_history=25, L_coherence=40.0, dt=0.03):
        self.N = N_pred
        self.M = M_history
        self.L = L_coherence
        self.L_sq = L_coherence ** 2
        self.dt = dt
        self._history = collections.deque(maxlen=M_history)
        self._traj_prev = [np.zeros(3)] * N_pred

    def add_sample(self, pos_enu, wind_enu):
        self._history.append((np.array(pos_enu, dtype=np.float64),
                              np.array(wind_enu, dtype=np.float64)))

    def set_previous_trajectory(self, traj_positions):
        if traj_positions is not None and len(traj_positions) >= self.N:
            self._traj_prev = traj_positions[:self.N]

    def predict(self):
        """预测未来 N 步风速序列 [list of (3,) ENU vectors]"""
        if len(self._history) < 3:
            if self._history:
                return [self._history[-1][1].copy()] * self.N
            return [np.zeros(3)] * self.N

        h_pos = np.array([p for p, _ in self._history])
        h_wind = np.array([w for _, w in self._history])
        M_cur = len(self._history)
        t_indices = np.arange(M_cur)[::-1]  # 0=最近

        predictions = []
        for k in range(self.N):
            p_future = np.asarray(self._traj_prev[k])
            dx = h_pos[:, 0] - p_future[0]
            dy = h_pos[:, 1] - p_future[1]
            dz = h_pos[:, 2] - p_future[2]
            r = np.sqrt(dx * dx + dy * dy + dz * dz)

            w_space = np.exp(-r / self.L)
            tau_t = max(self.L / 7.0, 0.5)
            w_time = np.exp(-t_indices * self.dt / tau_t)
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
# 5. CMA — 裕度感知约束自适应
# ══════════════════════════════════════════════════════════
class CMAManager:
    """
    基于动压实时收紧 MPC 硬约束, 防止执行器饱和
    支持 6D 和 10D 控制格式
    """

    def __init__(self, u_max_nom, u_min_nom, gamma_t=0.002, gamma_a=0.0001, rho=1.225,
                 model='6d'):
        self.u_max_nom = np.asarray(u_max_nom); self.u_min_nom = np.asarray(u_min_nom)
        self.gamma_t = gamma_t; self.gamma_a = gamma_a
        self.rho = rho
        self.model = model
        self.thrust_idx = 2 if model == '6d' else 0  # 推力在控制向量中的位置
        self._q_dyn = 0.0; self._mode = 'normal'

    def update(self, wind_meas, mpc_u_pred=None):
        w_mag = np.linalg.norm(wind_meas)
        self._q_dyn = 0.5 * self.rho * w_mag**2

        if self._q_dyn < 20.0:
            self._mode = 'normal'
            u_min = self.u_min_nom.copy(); u_max = self.u_max_nom.copy()
        elif self._q_dyn < 60.0:
            self._mode = 'cautious'
            shrink = 1.0 - self.gamma_t * (self._q_dyn - 20.0)
            shrink = max(shrink, 0.5)
            u_max = self.u_max_nom * shrink
            u_min = self.u_min_nom * shrink
            u_min[self.thrust_idx] = max(u_min[self.thrust_idx],
                                         0.25 if self.model == '6d' else 5.0)
        else:
            self._mode = 'aggressive'
            u_max = self.u_max_nom * 0.5
            u_min = self.u_min_nom * 0.5
            u_min[self.thrust_idx] = max(u_min[self.thrust_idx],
                                         0.25 if self.model == '6d' else 5.0)

        return u_min, u_max

    def get_q(self):
        return self._q_dyn

    def get_mode(self):
        return self._mode


# ══════════════════════════════════════════════════════════
# 6. 风能利用建议
# ══════════════════════════════════════════════════════════
class WindUtilizationAdvisor:
    """基于风方向与位置误差方向是否一致, 决定借风策略"""

    def __init__(self, alignment_threshold=0.3, coast_factor_max=0.5):
        self.align_thresh = alignment_threshold
        self.coast_max = coast_factor_max
        self._is_trajectory = False

    def set_task_type(self, is_trajectory):
        self._is_trajectory = is_trajectory

    def evaluate(self, wind_meas, pos_err, wind_seq=None):
        w_mag = np.linalg.norm(wind_meas)
        pe_mag = np.linalg.norm(pos_err)

        if w_mag < 0.5 or pe_mag < 0.1 or not self._is_trajectory:
            return 0.0, 'normal', 1.0

        w_dir = wind_meas / max(w_mag, 0.01)
        pe_dir = pos_err / max(pe_mag, 0.01)
        alignment = np.dot(w_dir, pe_dir)  # >0: 顺风, <0: 逆风

        Q_scale = 1.0
        if alignment > self.align_thresh:
            coast = min(self.coast_max * alignment, self.coast_max)
            Q_scale = 1.0 - 0.3 * alignment
            mode = 'coasting'
        elif alignment < -0.3:
            coast = 0.0
            Q_scale = 1.0 + 0.5 * abs(alignment)
            mode = 'fighting'
        else:
            coast = 0.0
            mode = 'normal'

        return coast, mode, Q_scale


# ══════════════════════════════════════════════════════════
# 7. 预测性能量调度
# ══════════════════════════════════════════════════════════
class PredictiveEnergyManager:
    """利用垂向风预测调整推力偏置以实现升力节能"""

    def __init__(self, N_pred=20, z_margin=0.5, ramp_steps=5, dz_threshold=2.0):
        self.N = N_pred
        self.z_margin = z_margin
        self.ramp_steps = ramp_steps
        self.dz_thresh = dz_threshold
        self._ramp = 0.0

    def plan(self, wind_seq):
        if not wind_seq:
            return 'normal', 0.0

        w_z_avg = np.mean([w[2] for w in wind_seq[:5]])

        if abs(w_z_avg) < 0.3:
            target = 0.0
            mode = 'normal'
        elif w_z_avg > 1.0:
            target = -min(w_z_avg * 0.03, 0.12)
            mode = 'lift_assist'
        elif w_z_avg < -1.0:
            target = abs(w_z_avg) * 0.04
            mode = 'sink_compensate'
        else:
            target = 0.0
            mode = 'normal'

        self._ramp += (target - self._ramp) / max(self.ramp_steps, 1)
        thrust_bias = float(np.clip(self._ramp, -0.15, 0.15))
        return mode, thrust_bias
