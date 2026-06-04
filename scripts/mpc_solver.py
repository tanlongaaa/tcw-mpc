#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mpc_solver.py — TCW-MPC 统一求解器接口
=======================================
提供 BaseMPCSolver 抽象基类 + 两种后端:
  - OSQPSolver:    OSQP QP 求解器 (快速, 线性MPC)
  - iLQRSolver:    迭代 LQR (非线性, 连续线性化)

用法:
    solver = OSQPSolver(A, B, g, Q, R, N, u_min, u_max)
    # 或
    solver = iLQRSolver(dynamics_func, Q, R, N, u_min, u_max, dt)
    
    u_opt = solver.solve(x0, x_ref, u_ref, d_seq)
    pos_traj, vel_traj = solver.get_trajectory()
"""

import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Tuple, List
from scipy.linalg import block_diag
import osqp
import scipy.sparse as sparse


# ══════════════════════════════════════════════════════════
# 抽象基类
# ══════════════════════════════════════════════════════════
class BaseMPCSolver(ABC):
    """MPC 求解器统一接口"""

    @abstractmethod
    def solve(self, x0: np.ndarray, x_ref: np.ndarray,
              u_ref: Optional[np.ndarray] = None,
              d_seq: Optional[List[np.ndarray]] = None) -> Optional[np.ndarray]:
        """
        求解 MPC 问题, 返回最优控制序列首项 u_0.

        Args:
            x0:     当前状态 (nx,)
            x_ref:  参考状态 (nx,)
            u_ref:  参考控制 (nu,) 或 None
            d_seq:  风扰序列 list of (nx,) 长度 N, 或 None

        Returns:
            u_opt:  最优控制 (nu,) 或 None (求解失败)
        """
        ...

    @abstractmethod
    def get_trajectory(self) -> Tuple[Optional[List], Optional[List]]:
        """返回上一帧 MPC 解的预测轨迹 (pos_traj, vel_traj)"""
        ...

    @abstractmethod
    def update_model(self, A_new: np.ndarray, B_new: np.ndarray):
        """更新线性化模型 (倾角自适应修正)"""
        ...

    @property
    @abstractmethod
    def nx(self) -> int:
        ...

    @property
    @abstractmethod
    def nu(self) -> int:
        ...

    @property
    @abstractmethod
    def N(self) -> int:
        ...


# ══════════════════════════════════════════════════════════
# OSQP 线性 MPC 求解器 (保持现有实现)
# ══════════════════════════════════════════════════════════
class OSQPSolver(BaseMPCSolver):
    """
    标准线性 MPC: QP 形式, OSQP 求解.

    状态 x = [px, py, pz, vx, vy, vz]  (6)
    控制 u = [roll, pitch, thrust]      (3)
    """

    def __init__(self, A: np.ndarray, B: np.ndarray, g_vec: np.ndarray,
                 Q: np.ndarray, R: np.ndarray, N: int,
                 u_min: np.ndarray, u_max: np.ndarray):
        self._nx = A.shape[0]; self._nu = B.shape[1]; self._N = N
        self.A = A; self.B = B; self.g = g_vec
        self.Q = Q; self.R = R
        self.u_min = u_min; self.u_max = u_max

        self.P = Q.copy()  # 终端权重, 调用方可设置 P[-1,-1]=0 抑制 z 速度

        self.last_ok = True
        self.last_iter = 0
        self._last_full_x = None
        self._last_tilt = 0.0
        self._build_osqp()

    # ── 属性 ──
    @property
    def nx(self): return self._nx

    @property
    def nu(self): return self._nu

    @property
    def N(self): return self._N

    # ── 模型更新 ──
    def update_model(self, A_new: np.ndarray, B_new: np.ndarray):
        self.A = A_new; self.B = B_new
        self._build_osqp()
        self._last_full_x = None

    # ── QP 构建 ──
    def _build_osqp(self):
        nx, nu, N = self._nx, self._nu, self._N
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

        # 约束0: B*u0 - x1 = ...
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

        self._P_mat = P_mat
        self._A_osqp = A_osqp
        self.q_template = np.zeros(n_var)
        self._setup_solver()

    def _setup_solver(self):
        self.solver = osqp.OSQP()
        self.solver.setup(
            P=self._P_mat, q=np.zeros(self._P_mat.shape[0]),
            A=self._A_osqp,
            l=np.zeros(self._N * self._nx + self._N * self._nu),
            u=np.zeros(self._N * self._nx + self._N * self._nu),
            verbose=False, warm_start=True, max_iter=400,
            eps_abs=1e-4, eps_rel=1e-4, polish=True)

    # ── 求解 ──
    def solve(self, x0, x_ref, u_ref=None, d_seq=None):
        if u_ref is None:
            u_ref = np.zeros(self._nu)
        if d_seq is None:
            d_seq = [np.zeros(self._nx)] * self._N

        nx, nu, N = self._nx, self._nu, self._N
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

        l = np.zeros(n_eq + N * nu)
        u = np.zeros(n_eq + N * nu)

        for k in range(N):
            beq_k = -self.g - np.asarray(d_seq[k])
            l[k * nx:(k + 1) * nx] = beq_k
            u[k * nx:(k + 1) * nx] = beq_k

        l[:nx] -= self.A @ x0
        u[:nx] -= self.A @ x0

        l[n_eq:] = np.tile(self.u_min, N)
        u[n_eq:] = np.tile(self.u_max, N)

        try:
            self.solver.update(q=q, l=l, u=u)
            result = self.solver.solve()
        except Exception:
            self._setup_solver()
            try:
                self.solver.update(q=q, l=l, u=u)
                result = self.solver.solve()
            except Exception:
                self.last_ok = False
                self.last_iter = 0
                self._last_full_x = None
                return None

        if result.info.status_val not in (1, 2):
            self.last_ok = False
            self.last_iter = 0
            self._last_full_x = None
            return None

        self.last_ok = True
        self.last_iter = result.info.iter
        self._last_full_x = result.x.copy()
        # u_0 在变量向量最前面: [u0(4), x1(10), u1(4), ...]
        return result.x[0:nu].copy()  # u_0

    def get_trajectory(self):
        if self._last_full_x is None:
            return None, None
        n_xu = self._nx + self._nu
        pos_traj = []; vel_traj = []
        for k in range(1, self._N + 1):
            xk_s = (k - 1) * n_xu + self._nu
            xk = self._last_full_x[xk_s:xk_s + self._nx]
            pos_traj.append(xk[:3].copy())
            # 速度通道: 6D 模型在 [3:6], 10D 模型在 [7:10]
            vel_s = 3 if self._nx <= 6 else 7
            vel_traj.append(xk[vel_s:vel_s+3].copy())
        return pos_traj, vel_traj


# ══════════════════════════════════════════════════════════
# iLQR 非线性 MPC 求解器 (新增)
# ══════════════════════════════════════════════════════════
class iLQRSolver(BaseMPCSolver):
    """
    迭代线性二次型调节器 (Iterative LQR)
    
    每步内部:
      1. 沿当前控制序列前向推演非线性轨迹
      2. 在每个 (x_t, u_t) 处线性化动力学 (解析雅可比)
      3. LQR 反向传播 → 前馈 k + 反馈 K
      4. Line search 保证代价下降
      5. 控制投影到 box 约束内
    
    利用已有 discrete_linearize() 的解析雅可比
    """

    def __init__(self, nx: int, nu: int, N: int, dt: float,
                 Q: np.ndarray, R: np.ndarray,
                 u_min: np.ndarray, u_max: np.ndarray,
                 dynamics_fn,      # f(x, u, wind) → x_next  (离散化后)
                 linearize_fn,     # discrete_linearize(x, u, wind, dt) → A, B, g
                 lqr_iter: int = 5,
                 linesearch_decay: float = 0.5,
                 max_linesearch_iter: int = 5,
                 eps: float = 1e-3):
        self._nx = nx; self._nu = nu; self._N = N; self.dt = dt
        self.Q = Q; self.R = R
        self.u_min = u_min; self.u_max = u_max
        self._dynamics = dynamics_fn
        self._linearize = linearize_fn
        self.lqr_iter = lqr_iter
        self.linesearch_decay = linesearch_decay
        self.max_linesearch_iter = max_linesearch_iter
        self.eps = eps

        self.P_terminal = Q.copy()  # 终端权重, 调用方可配置

        self.last_ok = True
        self.last_iter = 0
        self._last_full_x = None   # 最优解 x_{1:N} 状态轨迹
        self._last_full_u = None   # 最优解 u_{0:N-1} 控制序列
        self._last_cost = float('inf')

    @property
    def nx(self): return self._nx
    @property
    def nu(self): return self._nu
    @property
    def N(self): return self._N

    def update_model(self, A_new=None, B_new=None):
        # iLQR 无需手动更新模型; 每步自动线性化
        pass

    def _cost(self, x_traj, u_seq, x_ref, u_ref):
        """计算轨迹总代价"""
        J = 0.0
        for k in range(self._N):
            dx = x_traj[k] - x_ref
            du = u_seq[k] - u_ref
            J += dx @ self.Q @ dx + du @ self.R @ du
        dxN = x_traj[self._N] - x_ref
        J += dxN @ self.P_terminal @ dxN
        return J

    def _lqr_backward(self, x_traj, u_seq, wind_seq, x_ref, u_ref):
        """
        LQR 反向传播: 沿轨迹在每个点线性化, 计算前馈 k 和反馈 K
        
        返回: k_seq (N, nu), K_seq (N, nu, nx)
        """
        nx, nu, N = self._nx, self._nu, self._N
        k_seq = np.zeros((N, nu))
        K_seq = np.zeros((N, nu, nx))

        V_xx = self.P_terminal.copy()
        V_x = self.P_terminal @ (x_traj[N] - x_ref)

        for t in reversed(range(N)):
            xt = x_traj[t]; ut = u_seq[t]
            A_t, B_t, g_t = self._linearize(xt, ut, wind_seq[t], self.dt)

            # 代价导数
            dx = xt - x_ref; du = ut - u_ref
            Q_x = 2.0 * self.Q @ dx
            Q_u = 2.0 * self.R @ du
            Q_xx = 2.0 * self.Q
            Q_uu = 2.0 * self.R
            Q_xu = np.zeros((nx, nu))  # 二次型可分离

            # 总 Q函数 导数
            Qx = Q_x + A_t.T @ V_x
            Qu = Q_u + B_t.T @ V_x
            Qxx = Q_xx + A_t.T @ V_xx @ A_t
            Quu = Q_uu + B_t.T @ V_xx @ B_t
            Qux = B_t.T @ V_xx @ A_t

            # Cholesky 求解 (数值稳定)
            try:
                L = np.linalg.cholesky(Quu)
                k = -np.linalg.solve(L.T, np.linalg.solve(L, Qu))
                K = -np.linalg.solve(L.T, np.linalg.solve(L, Qux))
            except np.linalg.LinAlgError:
                # 正则化
                Quu_reg = Quu + 1e-4 * np.eye(nu)
                L = np.linalg.cholesky(Quu_reg)
                k = -np.linalg.solve(L.T, np.linalg.solve(L, Qu))
                K = -np.linalg.solve(L.T, np.linalg.solve(L, Qux))

            k_seq[t] = k; K_seq[t] = K

            # 更新 Value 函数
            V_x = Qx + K.T @ Quu @ k + K.T @ Qu + Qux.T @ k
            V_xx = Qxx + K.T @ Quu @ K + K.T @ Qux + Qux.T @ K

        return k_seq, K_seq

    def _forward_rollout(self, x0, u_seq, wind_seq):
        """从 x0 出发, 沿 u_seq 推演非线性轨迹"""
        nx = self._nx
        x_traj = np.zeros((self._N + 1, nx))
        x_traj[0] = x0
        for t in range(self._N):
            x_traj[t + 1] = self._dynamics(x_traj[t], u_seq[t], wind_seq[t])
        return x_traj

    def solve(self, x0, x_ref, u_ref=None, d_seq=None):
        x0 = np.asarray(x0, dtype=np.float64)
        x_ref = np.asarray(x_ref, dtype=np.float64)
        if u_ref is None:
            u_ref = np.zeros(self._nu)
        if d_seq is None:
            d_seq = [np.zeros(self._nx)] * self._N

        # 提取风扰中的速度通道 → 回推 3D 风速
        # 6D 模型: d[3:6]; 10D 模型: d[7:10]
        vel_s = 3 if self._nx <= 6 else 7
        wind_seq = []
        for d in d_seq:
            w = np.zeros(3)
            if np.any(np.abs(d[vel_s:vel_s+3]) > 1e-12):
                w = d[vel_s:vel_s+3] / max(self.dt, 0.001)
            wind_seq.append(w)

        nx, nu, N = self._nx, self._nu, self._N

        # Warm start: 使用上一帧解
        if self._last_full_u is not None:
            u_seq = self._last_full_u.copy()
        else:
            u_seq = np.tile(u_ref, (N, 1))

        best_u = u_seq.copy()
        best_cost = float('inf')

        for iteration in range(self.lqr_iter):
            # 1. 前向推演
            x_traj = self._forward_rollout(x0, u_seq, wind_seq)

            # 2. 计算当前代价
            cost = self._cost(x_traj, u_seq, x_ref, u_ref)
            if cost < best_cost:
                best_cost = cost
                best_u = u_seq.copy()
                self._last_full_x = x_traj[1:].copy()
                self._last_full_u = u_seq.copy()

            # 3. LQR 反向传播
            k_seq, K_seq = self._lqr_backward(x_traj, u_seq, wind_seq, x_ref, u_ref)

            # 4. Line search 前向
            alpha = 1.0
            accepted = False
            for _ in range(self.max_linesearch_iter):
                u_new = np.zeros_like(u_seq)
                x_new_traj = np.zeros((N + 1, nx))
                x_new_traj[0] = x0

                for t in range(N):
                    dx = x_new_traj[t] - x_traj[t]
                    du = k_seq[t] + K_seq[t] @ dx
                    u_new[t] = u_seq[t] + alpha * du
                    u_new[t] = np.clip(u_new[t], self.u_min, self.u_max)
                    x_new_traj[t + 1] = self._dynamics(x_new_traj[t], u_new[t], wind_seq[t])

                new_cost = self._cost(x_new_traj, u_new, x_ref, u_ref)
                if new_cost < cost:
                    u_seq = u_new
                    accepted = True
                    break
                alpha *= self.linesearch_decay

            if not accepted:
                u_seq = best_u.copy()

            # 收敛检查
            if iteration > 0 and accepted:
                du_norm = np.max(np.abs(u_seq - u_seq_prev))
                if du_norm < self.eps:
                    break
            u_seq_prev = u_seq.copy()

        self.last_ok = True
        self.last_iter = iteration + 1
        self._last_full_u = best_u.copy()
        self._last_cost = best_cost
        return best_u[0].copy()

    def get_trajectory(self):
        if self._last_full_x is None:
            return None, None
        pos_traj = []; vel_traj = []
        vel_s = 3 if self._nx <= 6 else 7
        for k in range(self._N):
            xk = self._last_full_x[k]
            pos_traj.append(xk[:3].copy())
            vel_traj.append(xk[vel_s:vel_s+3].copy())
        return pos_traj, vel_traj
