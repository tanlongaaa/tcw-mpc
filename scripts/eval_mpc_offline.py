#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
eval_mpc_offline.py — MPC 控制律离线闭环评估 (隔离 PX4/OFFBOARD)
================================================================
目的: 把"MPC 算法是否正确"与"HIL 的 OFFBOARD 握手问题"彻底隔离。
做法: 用 mpc_controller 对着 10D CTBR 动力学(当 nominal plant)滚动闭环,
      从地面 z=0 起飞 → 目标 2.5m → 看能否拉起并稳住。

注意: 这是 nominal 闭环 (控制器内模 = plant), 无 PX4 内环、无 model mismatch。
      它只回答一个问题: MPC 控制律本身能不能工作?
      若这里都飞不起来 → MPC 算法有问题。
      若这里能飞 → 问题在 HIL 集成层 (OFFBOARD/PX4), 不在 MPC。
"""
import os, sys
import numpy as np
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from mpc_controller import MPCController
from quadrotor_dynamics import quadrotor_dynamics_10d_discrete


def run(target_z=2.5, T=12.0, dt=0.03, wind=None, label=""):
    ctrl = MPCController(model='10d', dt=dt, N_pred=15)
    ctrl.set_target([0.0, 0.0, target_z])
    # 初始: 地面静止, identity 四元数
    x = np.zeros(10); x[3] = 1.0
    n = int(T / dt)
    traj = []
    for k in range(n):
        w = None
        if wind is not None:
            w = np.array(wind)
        u, info = ctrl.step(x, wind_meas=w, yaw_cur=0.0)
        # 用同一 10D 动力学当 plant 前向 (nominal 闭环)
        x = quadrotor_dynamics_10d_discrete(x, u, w if w is not None else np.zeros(3), dt)
        traj.append((k * dt, x[0], x[1], x[2], u[0], u[1], u[2], u[3],
                     info['power_est'], info['mpc_ok']))
    traj = np.array(traj)
    t, px, py, pz = traj[:, 0], traj[:, 1], traj[:, 2], traj[:, 3]
    fc = traj[:, 4]
    # 稳态: 最后 3s
    tail = traj[t >= (T - 3.0)]
    z_tail = tail[:, 3]
    xy_tail = np.sqrt(tail[:, 1]**2 + tail[:, 2]**2)
    z_err = abs(np.mean(z_tail) - target_z)
    print(f"\n── {label or 'eval'} (target z={target_z}, wind={wind}) ──")
    print(f"{'t':>5} {'px':>8} {'py':>8} {'pz':>8} {'fc':>7} {'ok':>3}")
    for i in range(0, len(traj), max(1, len(traj) // 12)):
        print(f"{t[i]:5.1f} {px[i]:8.3f} {py[i]:8.3f} {pz[i]:8.3f} {fc[i]:7.2f} {int(traj[i,9]):3d}")
    print(f"  稳态(末3s): z_mean={np.mean(z_tail):.3f} z_err={z_err:.3f}m "
          f"xy_drift={np.mean(xy_tail):.3f}m fc_mean={np.mean(tail[:,4]):.2f}N "
          f"P_mean={np.mean(tail[:,8]):.0f}W")
    ok_solve = np.all(traj[:, 9] == 1)
    finite = np.all(np.isfinite(traj[:, 1:9]))
    verdict = "PASS" if (z_err < 0.2 and np.mean(xy_tail) < 0.3 and ok_solve and finite) else "FAIL"
    print(f"  → {verdict}  (z_err<0.2:{z_err<0.2} xy<0.3:{np.mean(xy_tail)<0.3} "
          f"solve_ok:{ok_solve} finite:{finite})")
    return verdict, traj


if __name__ == '__main__':
    print("=" * 64)
    print("MPC 控制律离线闭环评估 (nominal, 隔离 PX4/OFFBOARD)")
    print("=" * 64)
    r1, _ = run(target_z=2.5, T=12.0, label="无风起飞悬停")
    r2, _ = run(target_z=2.5, T=14.0, wind=[3.0, 0.0, 0.0], label="3m/s侧风")
    r3, _ = run(target_z=2.5, T=14.0, wind=[6.0, 2.0, 0.0], label="6m/s斜风")
    print("\n" + "=" * 64)
    print(f"汇总: 无风={r1}  3m/s={r2}  6m/s={r3}")
    print("=" * 64)
