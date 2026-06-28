#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_mpc_controller.py — mpc_controller R1-R2 接口离线单测
==========================================================
验证 DESIGN §3.3 交付物的单元测试要求:
  - 权重热更新不崩
  - 越界 clip
  - 能耗输出正确
  - 闭环 step 稳定 (悬停收敛)

无 ROS 依赖, 纯离线。运行: python3 test_mpc_controller.py
"""
import os
import sys
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from mpc_controller import MPCController, WEIGHT_BOUNDS

_PASS = 0
_FAIL = 0


def check(name, cond, detail=''):
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"  ✅ {name}")
    else:
        _FAIL += 1
        print(f"  ❌ {name}  {detail}")


def hover_state(z=2.5):
    x = np.zeros(10)
    x[2] = z
    x[3] = 1.0  # qw
    return x


# ── T1: 初始化 + 默认权重读出 ──
def test_init():
    print("\n[T1] 初始化 + 默认权重")
    ctrl = MPCController(model='10d')
    th = ctrl.get_cost_weights()
    check("get_cost_weights 返回四键", set(th) == {'q_xy', 'q_z', 'lambda_energy', 'r_omega'}, th)
    check("默认 q_xy=40", abs(th['q_xy'] - 40.0) < 1e-9, th['q_xy'])
    check("默认 lambda_energy=5", abs(th['lambda_energy'] - 5.0) < 1e-9, th['lambda_energy'])
    # R[fc] = 1 + lambda (base 3→1 提升响应)
    check("R[fc]=1+λ=6", abs(ctrl.R[0, 0] - 6.0) < 1e-9, ctrl.R[0, 0])
    return ctrl


# ── T2: R1 权重热更新 (不重建求解器) ──
def test_hot_update(ctrl):
    print("\n[T2] R1 权重热更新")
    solver_id_before = id(ctrl.mpc)
    Q_id_before = id(ctrl.Q)
    R_id_before = id(ctrl.R)

    applied = ctrl.set_cost_weights({'q_xy': 8.0, 'lambda_energy': 20.0, 'r_omega': 100.0})
    check("set 返回生效 θ", abs(applied['q_xy'] - 8.0) < 1e-9, applied)
    check("q_xy 同步改 Q[0,0]/Q[1,1]",
          abs(ctrl.Q[0, 0] - 8.0) < 1e-9 and abs(ctrl.Q[1, 1] - 8.0) < 1e-9,
          (ctrl.Q[0, 0], ctrl.Q[1, 1]))
    check("lambda_energy → R[fc]=1+20=21", abs(ctrl.R[0, 0] - 21.0) < 1e-9, ctrl.R[0, 0])
    check("r_omega → R[wx/wy/wz]=100",
          all(abs(ctrl.R[i, i] - 100.0) < 1e-9 for i in (1, 2, 3)),
          [ctrl.R[i, i] for i in (1, 2, 3)])
    # 不重建: solver / Q / R 对象身份不变 (原地修改)
    check("求解器对象未重建", id(ctrl.mpc) == solver_id_before)
    check("Q 数组原地修改 (身份不变)", id(ctrl.Q) == Q_id_before)
    check("R 数组原地修改 (身份不变)", id(ctrl.R) == R_id_before)
    # iLQR 持有同一引用
    check("iLQR.Q 与 ctrl.Q 同一引用", ctrl.mpc.Q is ctrl.Q)
    check("iLQR.R 与 ctrl.R 同一引用", ctrl.mpc.R is ctrl.R)


# ── T3: 越界 clip ──
def test_clip(ctrl):
    print("\n[T3] 越界 clip")
    lo_qxy, hi_qxy = WEIGHT_BOUNDS['q_xy']
    applied = ctrl.set_cost_weights({'q_xy': 1e6})
    check("超大 q_xy clip 到上界", abs(applied['q_xy'] - hi_qxy) < 1e-9, applied['q_xy'])
    applied = ctrl.set_cost_weights({'q_xy': -100.0})
    check("负 q_xy clip 到下界", abs(applied['q_xy'] - lo_qxy) < 1e-9, applied['q_xy'])

    # nan/inf 不破坏状态
    before = ctrl.get_cost_weights()['q_xy']
    ctrl.set_cost_weights({'q_xy': float('nan')})
    check("nan 被丢弃, 保持原值", abs(ctrl.get_cost_weights()['q_xy'] - before) < 1e-9)
    ctrl.set_cost_weights({'q_xy': float('inf')})
    check("inf 被丢弃, 保持原值", abs(ctrl.get_cost_weights()['q_xy'] - before) < 1e-9)

    # 未知键忽略, 不崩
    try:
        ctrl.set_cost_weights({'garbage_key': 99, 'q_z': 30.0})
        check("未知键忽略且不崩", abs(ctrl.get_cost_weights()['q_z'] - 30.0) < 1e-9)
    except Exception as e:
        check("未知键忽略且不崩", False, str(e))

    # 非 dict 报错
    try:
        ctrl.set_cost_weights([1, 2, 3])
        check("非 dict 抛 TypeError", False)
    except TypeError:
        check("非 dict 抛 TypeError", True)


# ── T4: R2 能耗输出 ──
def test_power(ctrl):
    print("\n[T4] R2 能耗输出")
    x = hover_state()
    u, info = ctrl.step(x, wind_meas=np.zeros(3))
    P = ctrl.get_power_estimate()
    check("悬停功率 > 0", P > 0, P)
    check("悬停功率落真实区间 120-220W", 120 < P < 220, P)
    check("info.power_est == get_power_estimate", abs(info['power_est'] - P) < 1e-6)
    E0 = ctrl.get_predicted_energy()
    ctrl.step(x, wind_meas=np.zeros(3))
    E1 = ctrl.get_predicted_energy()
    check("累积能量单调增", E1 > E0, (E0, E1))
    check("一步能量 ≈ P·dt", abs((E1 - E0) - P * ctrl.dt) < P * ctrl.dt * 0.5)


# ── T5: 闭环 step 稳定性 (悬停 + CTBR 有界) ──
def test_closed_loop():
    print("\n[T5] 闭环 step 稳定性")
    ctrl = MPCController(model='10d')
    ctrl.set_target([0, 0, 2.5])
    x = hover_state(z=2.5)
    ok_all = True
    u_fc_range = []
    for k in range(200):  # 6s @ 33Hz
        u, info = ctrl.step(x, wind_meas=np.array([3.0, 0.0, 0.0]), yaw_cur=0.0)
        if not np.all(np.isfinite(u)):
            ok_all = False
            break
        u_fc_range.append(u[0])
        # CTBR 角速率必须在约束内
        if not (abs(u[1]) <= 0.1 + 1e-6 and abs(u[2]) <= 0.1 + 1e-6 and abs(u[3]) <= 0.05 + 1e-6):
            ok_all = False
            break
    check("200 步全程有限且角速率有界", ok_all)
    check("推力在 [2,20]N 约束内", all(2.0 - 1e-6 <= f <= 20.0 + 1e-6 for f in u_fc_range))
    check("mpc 求解成功", info['mpc_ok'] == 1, info['mpc_ok'])


# ── T6: 热更新中途不打断闭环 ──
def test_hot_update_inflight():
    print("\n[T6] 闭环中途热更新不崩")
    ctrl = MPCController(model='10d')
    ctrl.set_target([0, 0, 2.5])
    x = hover_state(z=2.5)
    ok = True
    try:
        for k in range(100):
            u, info = ctrl.step(x, wind_meas=np.array([5.0, 2.0, 0.0]))
            if k % 10 == 0:
                # 模拟 RL 低频调权重 (含极端值, 验证 clip 兜底)
                ctrl.set_cost_weights({
                    'q_xy': np.random.uniform(-50, 200),
                    'lambda_energy': np.random.uniform(0, 150),
                    'r_omega': np.random.uniform(-10, 500),
                })
            if not np.all(np.isfinite(u)):
                ok = False
                break
    except Exception as e:
        ok = False
        print(f"      异常: {e}")
    check("中途随机调权重 100 步不崩、输出有限", ok)


if __name__ == '__main__':
    np.random.seed(0)
    print("=" * 60)
    print("mpc_controller R1-R2 接口单测 (离线, 无 ROS)")
    print("=" * 60)
    c = test_init()
    test_hot_update(c)
    test_clip(c)
    test_power(c)
    test_closed_loop()
    test_hot_update_inflight()
    print("\n" + "=" * 60)
    print(f"结果: {_PASS} passed, {_FAIL} failed")
    print("=" * 60)
    sys.exit(1 if _FAIL else 0)
