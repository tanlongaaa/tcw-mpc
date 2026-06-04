# TCW-MPC: Trajectory-Coherent Wind-Adaptive MPC 🚁🌪️

**面向极端湍流风场的小型四旋翼自适应模型预测控制器**

> ROS1 Noetic | PX4 v1.13.3 | Gazebo 11 | OSQP / iLQR

---

## 项目概述

TCW-MPC 是一个面向 **极端湍流风场中四旋翼抗风与节能控制** 的 ROS 仿真项目。采用了 **MPC 直驱控制架构**（仿 ACMPC, TRO 2025），MPC 直接输出控制指令（姿态 + 推力或体轴角速度 + 推力），通过 MAVROS `AttitudeTarget` 发送给 PX4 执行，消除了原有的"MPC → 位置 setpoint → PX4 PID"分层中间层。

支持三种动力学模型：
- **6D Euler**：姿态角直驱（已验证悬停稳定）
- **10D CTBR**：体轴角速度直驱（仿 ACMPC，迭代优化中）
- **13D 标准 6-DOF**：完整刚体转动动力学（欧拉方程，解析雅可比）

### 核心创新

| 模块 | 说明 |
|------|------|
| **TCWP** — 轨迹相干风预测 | 利用无人机作为移动流体探针，基于空间相干核回归外推预测时域内的风场遭遇序列 |
| **CMA** — 动压约束裕度自适应 | 基于实时动压 $q = \frac12\rho\|W\|^2$ 收紧 MPC 硬约束，防止大风中执行器饱和 |
| **BEMT 功率模型** — 叶素动量理论 | 诱导功率 + 型阻功率 + 电机效率，物理级功耗估计 |
| **WindUtilizationAdvisor** | 风向与位置误差方向匹配，决定 Coast/Fight/Normal 策略 |
| **PredictiveEnergyManager** | 利用垂向风预测调整推力偏置，实现升力节能 |

### 技术演进

```
线性 MPC (6D Euler 直驱) ✅  →  10D CTBR 体轴角速度 (迭代中)
→  13D 标准 6-DOF (完整转动动力学 + 欧拉方程) ← 导师要求
→  ACMPC-style PPO + 可微 MPC (远期)
```

---

## 项目结构

```
offboard_test/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── scripts/
│   ├── env.sh                        # ★ 环境变量 (每终端必 source)
│   ├── start_sitl.sh                 # ★ 一键启动 SITL + Gazebo + MAVROS + 风场
│   │
│   ├── mpc_node.py                   # ★★ TCW-MPC 控制器 (ROS 节点, ~630 行)
│   ├── mpc_solver.py                 # ★★ MPC 求解器 (BaseMPCSolver → OSQP | iLQR)
│   ├── mpc_components.py             # ★★ 控制组件 (TCWP / CMA / Integrator / 滤波器)
│   ├── quadrotor_dynamics.py         # ★★★ 非线性动力学 (6D/10D/13D + 解析雅可比)
│   ├── wind_field.py                 # ★ 极端湍流风场 (Dryden + 幂律 + 阵风)
│   ├── pid_baseline.py               # PID 基线脚本
│   ├── power_model.py                # IMU 功耗估计
│   ├── analyze_logs.py               # MPC vs PID 对比分析
│   │
│   ├── diagnose_offboard.py          # 一键诊断
│   ├── offboard_waypoint_test.py     # 航点飞行测试
│   └── fixed_attitude_node_v2.py     # 姿态控制参考
│
├── improvemd/
│   ├── HANDOVER_FULL.md              # 全面交接文档
│   ├── HANDOVER_TCW_MPC_V1.md        # TCW-MPC 阶段性总结
│   └── 改进.md                       # 技术评估与改进路线图
│
├── 改进建议.md
└── 风场.md
```

### 模块依赖关系

```
quadrotor_dynamics.py  ←─┐   wind_field.py
   (6D/10D/13D 模型)     │        │
        │                │        │  (ROS topic)
        ▼                │        ▼
   mpc_solver.py ────────┘   /wind_field/velocity
   (OSQP / iLQR)
        │
        ▼
   mpc_components.py
   (TCWP / CMA / Integrator / LPF)
        │
        ▼
    mpc_node.py  ←── power_model.py
        │
        ├──→ PX4 via MAVROS: AttitudeTarget (直驱)
        │      • 6D: type_mask=0  (姿态四元数 + thrust)
        │      • 10D: type_mask=128 (body rate + thrust)
        └──→ CSV logging (mpc_log_*.csv)
```

---

## 动力学模型

### 6D Euler 模型 (已验证 ✅)

```
状态: x = [px, py, pz, vx, vy, vz]  (6)
控制: u = [φ, θ, T_norm]  (roll, pitch, 归一化推力)

v̇ = R(φ,θ)·[0,0,T]/m + g + a_drag
姿态作为控制输入 (无姿态动力学)
```

### 10D CTBR 模型 (仿 ACMPC, 迭代中 ⚠️)

```
状态: x = [p(3), q(4), v(3)]  (10)
控制: u = [f_c, ωx, ωy, ωz]  (总推力N + 体轴角速度)

ṗ = v,  q̇ = ½Ω(ω)·q,  v̇ = R(q)·e₃·f_c/m + g + a_drag
体轴角速度作为控制输入 (无转动动力学)
参考: Kaufmann et al., TRO 2025
```

### 13D 标准 6-DOF 模型 (导师要求, 已实现 ✅)

```
状态: x = [p(3), q(4), v(3), ω(3)]  (13)
控制: u = [F_total, τx, τy, τz]  (总推力 + 三轴力矩)

ṗ  = v
q̇  = ½·Ω(ω)·q                        (四元数运动学)
v̇  = R(q)·e₃·F_total/m + g + a_drag  (牛顿第二定律)
ω̇  = J⁻¹·[ω × J·ω + τ]               (★ 欧拉刚体方程)

惯性参数: Jx=Jy=0.029, Jz=0.055 kg·m²
陀螺耦合: ω̇x ∝ (Jy-Jz)·ωy·ωz  等
电机混合器: 力矩 → 四电机推力 (X 构型)
```

---

## 环境要求

| 组件 | 版本 |
|------|------|
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| PX4 | v1.13.3 |
| Gazebo | 11 |
| Python | 3.8 |
| MAVROS | 1.20.1 |

### Python 依赖

```bash
pip install numpy scipy osqp
```

---

## 快速启动

### 1. 环境变量

```bash
source /home/tan/catkin_ws/src/offboard_test/scripts/env.sh
```

### 2. 一键启动仿真

```bash
cd /home/tan/catkin_ws/src/offboard_test/scripts

# 6D Euler 直驱 (已验证稳定)
./start_sitl.sh --no-wind           # 无风悬停
./start_sitl.sh                     # 含风场抗风测试

# 选项
./start_sitl.sh --headless          # 无 GUI (headless 测试)
./start_sitl.sh --no-mavros         # 仅 PX4 + Gazebo
```

### 3. 运行控制器

```bash
# 在新终端
source scripts/env.sh

# 6D Euler 直驱 (默认)
rosrun offboard_test mpc_node.py

# 10D CTBR
rosrun offboard_test mpc_node.py _model:=10d

# PID 基线
rosrun offboard_test pid_baseline.py
```

---

## 控制器参数

### 6D Euler (默认)

```python
model = '6d'
N_pred = 30           # 预测时域 (1.0s @ 33Hz)
dt = 0.03             # 控制周期

Q = diag([4, 4, 6,    # 位置权重
          2, 2, 6])   # 速度权重
R = diag([14, 14, 15 + λ_energy])  # 控制惩罚

u_bound: roll/pitch ±0.45 rad, thrust [0.30, 0.95]
```

### 10D CTBR (实验性)

```python
model = '10d'
N_pred = 15           # 预测时域 (iLQR 加速)
lqr_iter = 2          # iLQR 迭代次数

Q = diag([15, 15, 20, 0,0,0,0, 8,8,15])
R = diag([3, 50, 50, 30])

u_bound: fc [2, 20]N, ωx/ωy ±0.1rad/s, ωz ±0.05rad/s
```

---

## 性能

### 6D Euler 直驱悬停 (无风, SITL 验证 2026-06-04)

| 指标 | 结果 |
|------|------|
| **z 悬停稳态** | 2.52 ± 0.02 m (目标 2.5) |
| **XY 误差** | < 0.07 m |
| **姿态稳定度** | roll/pitch ±0.02 rad (±1°) |
| **推力稳态** | ~0.705 (hover=0.66 + 调节余量) |
| **悬停功耗** | 170 W |
| **MPC 收敛** | 100% @ 33Hz |

### 历史数据 (分层 MPC 架构)

| 指标 | 分层 MPC | PID 基线 |
|------|---------|----------|
| **RMSE_XY** | 0.27 m | 0.27 m |
| 6.6 m/s 均风 XY 误差 | ~0.5 m | — |
| 最大阵风 (11.8 m/s) | <1 m | — |

> **注**: 分层 MPC 架构已被直驱架构替代。上表为历史数据，用于论文对比。

---

## 架构变更日志

### 2026-06-04: ACMPC-style 直驱重构

**P0 Bug 修复：**
| Bug | 文件 | 修复 |
|-----|------|------|
| OSQP 取 u0 下标错误: `result.x[4:8]` = x1 前4元素 | `mpc_solver.py` | → `result.x[0:4]` |
| Integrator `_integral` 3D 硬编码, 10D 需 4D | `mpc_components.py` | → `np.zeros_like(max_int)` |

**P1 修复：**
| 问题 | 修复 |
|------|------|
| PX4 FRD pitch rate 符号反 (ENU nose-up ≠ FRD body_rate.y) | → `body_rate.y = -wy` |
| 四元数参考强制 q=[1,0,0,0] 导致 yaw 跳变 | → `x_ref[3:7] = _q_current` |

**架构变更：**
- 控制接口: `PoseStamped` (位置 setpoint) → `AttitudeTarget` (直驱)
- 6D: `type_mask=0`, 姿态四元数 + thrust
- 10D: `type_mask=128`, body rate + thrust
- 新增 `mpc_solver.py` (OSQP + iLQR 统一接口)
- 新增 `mpc_components.py` (TCWP/CMA/Integrator 模块化)
- 新增 13D 标准 6-DOF 动力学模型 (欧拉方程 + 解析雅可比)

---

## 论文方向

1. **ACMPC-style Direct Drive MPC** — MPC 直驱 vs 分层架构对比
2. **Energy-Aware Wind-Adaptive Control** — BEMT 功率模型 + 风能利用
3. **Standard 6-DOF Quadrotor MPC** — 完整转动动力学 + 解析雅可比
4. **PPO + Differentiable MPC** — 数据驱动学习 MPC 代价参数 (远期)

---

## 踩坑记录

| 坑 | 现象 | 修复 |
|----|------|------|
| **OSQP 取错变量** | 6D 碰巧能跑, 10D fc=2.0 不爬升 | `result.x[0:nu]` 而非 `[nu:2*nu]` |
| **Integrator 维度** | 10D 抛出 ValueError shape mismatch | 用 `max_int` 长度初始化 `_integral` |
| **FRD sign** | ENU pitch rate 发给 PX4 FRD 符号反 | `body_rate.y = -wy` |
| **SITL 状态残留** | 不重启 PX4 导致 EKF 位置残留 | `rm -rf /tmp/px4_sitl_*` + 重开 PX4 |
| PX4 进程被挂起 | MAVROS `connected: False` | 不能 Ctrl+Z, 需 `kill -9` |
| iris 模型不可见 | `.sdf.jinja` 未编译 | `jinja_gen.py` |
| 解锁前必须预发 setpoint | PX4 拒绝切 OFFBOARD | 先发 setpoint ≥3s 再切模式 |
| 纯线性 MPC 在 10+ m/s 风中坠机 | 25° 倾角突破小角度假设 | 改用直驱架构 |
| 悬停时 Coast 策略有害 | 风能利用导致 MPC 与 PX4 目标分裂 | 悬停时强制 Coast=0 |
| ASLR segfault | PX4 v1.13.3 + Gazebo 11 | `setarch x86_64 -R` 禁用 ASLR |

---

## 许可证

MIT License

## 作者

- **小龙** — 项目主人
- **虾哥 🦐** — AI 助手
