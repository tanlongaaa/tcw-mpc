# TCW-MPC: Trajectory-Coherent Wind-Adaptive MPC 🚁🌪️

**面向极端湍流风场的小型四旋翼自适应模型预测控制器**

> ROS1 Noetic | PX4 v1.13.3 | Gazebo 11 | OSQP / iLQR | HIL Plant 仿真

---

## 项目概述

TCW-MPC 是一个面向 **极端湍流风场中四旋翼抗风与节能控制** 的 ROS 仿真项目，包含两条通路：

| 通路 | 说明 | 状态 |
|------|------|------|
| **MPC 直驱** (SITL) | MPC → AttitudeTarget → PX4 → Gazebo | 6D ✅ / 10D ⚠️ / 13D ✅ |
| **PID 基线** (HIL) | pid_baseline → MAVROS → PX4 → plant_6dof | ✅ 通过 |

### HIL Plant (v9 完整版, 2026-06-19 验证通过)

HIL 通路使用自建 6-DOF 物理引擎 `plant_6dof.py` 替代 Gazebo，经对标 `iris.sdf` 的参数标定，在无风悬停和阶跃风扰测试中均达到与 Gazebo 一致的精度。

**v9 物理效应（全开验证通过）：**

| 效应 | 来源 | 配置值 |
|------|------|--------|
| Blade Flapping | wind-vane 效应 | `flapping_coefficient: 5e-05` |
| Rotor Gyroscopic | 陀螺进动 | `I_rotor: 4e-05` |
| Dynamic Inflow | Pitt-Peters 一阶滤波 | `inflow_tau: 0.05` |
| 各向异性 body drag | iris 标定 | `body_drag_x/y/z: 0.10/0.30/0.1` |
| 二次方 body drag | 高速风阻 | `body_CdA: 0.12` |

**HIL 性能（PX4 出厂 PID + v9 plant）：**

| 指标 | 结果 |
|------|------|
| 无风悬停 err_xy | ±6 mm |
| 5 m/s 阶跃风最大漂移 | 0.86 m |
| 风停后恢复时间 | ~15 s |
| 恢复精度 | ±5 mm（与无风基线一致） |

> **关键教训 (2026-06-19):** plant mixer 的 `motor_roll_moment` 曾误设为 0.08，比正确值 1.30 小 16 倍，导致 PX4 出厂 PID 严重欠驱动。修复后所有 v9 效应均正常。

---

## 项目结构

```
offboard_test/                       ← tanlongaaa/tcw-mpc (本仓库)
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── scripts/
│   ├── mpc_node.py                  # TCW-MPC 控制器 (ROS 节点)
│   ├── mpc_solver.py                # MPC 求解器 (OSQP / iLQR)
│   ├── mpc_components.py            # 控制组件 (TCWP/CMA/Integrator/EnergyMgr)
│   ├── quadrotor_dynamics.py        # 非线性动力学 (6D/10D/13D + 解析雅可比)
│   ├── wind_field.py                # 极端湍流风场 (Dryden + 幂律 + 阵风)
│   ├── pid_baseline.py              # PID 基线控制器
│   ├── step_gust.py                 # 阶跃阵风测试脚本
│   ├── gazebo_step.py               # Gazebo 阶跃风测试
│   ├── start_hil_wind.sh            # HIL 一键风扰测试
│   ├── power_model.py               # IMU 功耗估计
│   ├── analyze_logs.py              # MPC vs PID 对比分析
│   ├── env.sh                       # 环境变量
│   └── start_sitl.sh                # SITL 一键启动
│
├── improvemd/
│   ├── HANDOVER_FULL.md             # 全面交接文档
│   ├── HANDOVER_TCW_MPC_V1.md       # TCW-MPC 阶段性总结
│   └── 改进.md                       # 技术评估与改进路线图
│
├── docs/                             # 新增文档
└── 改进建议.md
```

**HIL Plant 相关代码** 在独立 repo `tanlongaaa/px4-hil-plant`：

```
px4-ros-6dof_project-pid-eso-/       ← HIL 仿真框架 (独立 repo)
└── quad_sim/
    ├── config/sim_default.yaml      # ★ Plant + PID 完整参数
    ├── scripts/
    │   ├── plant_6dof.py            # ★★★ 6-DOF 可编程物理引擎 (v9)
    │   ├── backend_main.py          # HIL Backend (MAVLink ↔ Plant)
    │   ├── mavlink_backend.py       # MAVLink HIL 接口
    │   ├── sensor_models.py         # IMU/GPS/气压计模型
    │   └── sim_bridge_odom.py       # 备用通路 (Odom 直驱)
    ├── launch/hil_backend.launch    # 启动文件 (支持 RViz)
    ├── rviz/quad_sim.rviz           # RViz 配置
    └── urdf/quadrotor.urdf          # 四旋翼 URDF 模型
```

---

## 动力学模型

### 6D Euler 模型 (已实现 ✅)

```
状态: [px, py, pz, vx, vy, vz]  (6)
控制: [φ, θ, T_norm]  (roll, pitch, 归一化推力)
v̇ = R(φ,θ)·[0,0,T]/m + g + a_drag
```

### 10D CTBR 模型 (仿 ACMPC, 迭代中 ⚠️)

```
状态: [p(3), q(4), v(3)]  (10)
控制: [f_c, ωx, ωy, ωz]  (总推力N + 体轴角速度)
ṗ = v, q̇ = ½Ω(ω)·q, v̇ = R(q)·e₃·f_c/m + g + a_drag
参考: Kaufmann et al., TRO 2025
```

### 13D 标准 6-DOF 模型 (已实现 ✅)

```
状态: [p(3), q(4), v(3), ω(3)]  (13)
控制: [F_total, τx, τy, τz]
ω̇ = J⁻¹·[ω × J·ω + τ]  (欧拉刚体方程)
J = diag(0.029125, 0.029125, 0.055225)  kg·m²  (源自 iris.sdf)
```

---

## 环境要求

| 组件 | 版本 |
|------|------|
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| PX4 | v1.13.3 @ `~/Desktop/px4rl/PX4-Autopilot` |
| Gazebo | 11 |
| Python | 3.8 |
| MAVROS | 1.20.1 |

```bash
pip install numpy scipy osqp
```

---

## 快速启动

### HIL 通路 (pid_baseline.py, 推荐)

```bash
# 0. 环境
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 1. Backend + RViz
roslaunch quad_sim hil_backend.launch rviz:=true

# 2. PX4 SITL (无 Gazebo)
cd ~/Desktop/px4rl/PX4-Autopilot
NO_PXH=1 no_sim=1 make px4_sitl none_iris

# 3. MAVROS
rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14580

# 4. 设置参数 + 起飞
rosservice call /mavros/param/set '{param_id: "MPC_XY_VEL_P_ACC", value: {real: 1.8}}'
rosservice call /mavros/param/set '{param_id: "MPC_XY_VEL_I_ACC", value: {real: 0.4}}'
rosservice call /mavros/param/set '{param_id: "MPC_XY_P", value: {real: 0.95}}'
rosservice call /mavros/param/set '{param_id: "MPC_TILTMAX_AIR", value: {real: 45.0}}'
rosrun offboard_test pid_baseline.py

# 5. 阶跃风测试
python3 /tmp/step_wind.py
```

### SITL 通路 (mpc_node.py)

```bash
source scripts/env.sh
./start_sitl.sh                     # PX4 + Gazebo + MAVROS + 风场
rosrun offboard_test mpc_node.py    # 6D 直驱
```

---

## 性能

### HIL PID 基线 (v9 plant, PX4 出厂 PID, 2026-06-19)

| 指标 | 结果 |
|------|------|
| 无风悬停 XY 误差 | ±6 mm |
| 5 m/s 阶跃风最大漂移 | 0.86 m |
| 风停恢复时间 | ~15 s |
| 恢复后悬停精度 | ±5 mm |

### MPC 6D 直驱悬停 (无风, Gazebo SITL)

| 指标 | 结果 |
|------|------|
| z 悬停稳态 | 2.52 ± 0.02 m |
| XY 误差 | < 0.07 m |
| 悬停功耗 | 170 W |

---

## 踩坑记录

| 坑 | 现象 | 修复 |
|----|------|------|
| **motor_roll_moment 16x 偏小** | v9 全效应坠机/振荡 | 0.08 → 1.30 |
| **OSQP 取错变量** | 10D fc=2.0 不爬升 | `result.x[0:nu]` |
| **PX4 PID 被改乱** | err_xy 7.5m | 恢复出厂默认 |
| **风力三重计数** | 3-6x 超量风阻 | 只用 set_wind_vel_enu |
| **FRD sign** | ENU pitch rate 符号反 | `body_rate.y = -wy` |
| **二次方阻尼替代线性** | 悬停欠阻尼 | 保留线性 + 二次方附加 |
| **iris.sdf 未生成** | GAZEBO_MODEL_PATH 找不到 | `jinja_gen.py` |
| **解锁前未预发 setpoint** | PX4 拒切 OFFBOARD | 先发 ≥3s |

---

## 论文方向

1. **ACMPC-style Direct Drive MPC** — MPC 直驱 vs 分层架构对比
2. **Energy-Aware Wind-Adaptive Control** — BEMT 功率模型 + 风能利用
3. **Standard 6-DOF Quadrotor MPC** — 完整转动动力学 + 解析雅可比
4. **HIL Verification** — 自建 plant 对标 Gazebo 的验证方法论

---

## 许可证

MIT License

## 作者

- **小龙** — 项目主人
- **虾哥 🦐** — AI 助手
