# TCW-MPC: Trajectory-Coherent Wind-Adaptive MPC 🚁🌪️

**面向极端湍流风场的小型四旋翼自适应模型预测控制器**

> ROS1 Noetic | PX4 v1.13.3 | Gazebo 11 | OSQP

---

## 项目概述

TCW-MPC 是一个面向 **极端湍流风场中四旋翼抗风与节能控制** 的 ROS 仿真项目。核心是一个正交叠加了三项创新的线性 MPC 控制器，在 6 状态线性模型骨架下同时处理了 **风扰前馈、约束自适应、误差学习**，全部在 OSQP 的 QP 框架内零增维完成。

### 核心创新

| 模块 | 说明 |
|------|------|
| **TCWP** — 轨迹相干风预测 | 利用无人机作为移动流体探针，基于空间相干核回归外推预测时域内的风场遭遇序列 |
| **CMA** — 动压约束裕度自适应 | 基于实时动压 $q = \frac12\rho|W|^2$ 收紧 MPC 硬约束，防止大风中执行器饱和 |
| **风偏置学习** — EMA 误差补偿 | 三轴独立指数滑动平均补偿稳态偏置 |

### 技术演进

```
线性 MPC (直驱姿态)   →  Energy-Aware MPC (推力惩罚)   →  全非线性动力学  
→  分层 MPC (MPC规划位置 → PX4 PID 执行) ✅ (当前主线)
```

---

## 项目结构

```
offboard_test/
├── CMakeLists.txt                    # ROS catkin 包构建
├── package.xml                       # 包元数据
├── README.md                         # 本文档
│
├── scripts/
│   ├── env.sh                        # ★ 环境变量脚本 (每个终端必 source)
│   ├── start_sitl.sh                 # ★ 一键启动 PX4 SITL + Gazebo + MAVROS + 风场
│   │
│   ├── mpc_node.py                   # ★★ TCW-MPC 控制器 (主线, ~1250 行)
│   ├── quadrotor_dynamics.py         # ★★ 非线性动力学 + BEMT 功耗模型
│   ├── wind_field.py                 # ★ 极端湍流风场生成器 (Dryden + 幂律 + 阵风)
│   ├── pid_baseline.py               # ★ PID 基线对比脚本
│   ├── power_model.py                # 统一 IMU 功耗模型
│   ├── analyze_logs.py               # MPC vs PID 日志对比分析
│   │
│   ├── diagnose_offboard.py          # 一键诊断 (MAVROS/EKF/解锁/Offboard)
│   ├── offboard_waypoint_test.py     # 简单航点飞行测试
│   └── fixed_attitude_node_v2.py     # 姿态控制参考实现
│
├── improvemd/
│   ├── HANDOVER_FULL.md              # 全面交接文档
│   ├── HANDOVER_TCW_MPC_V1.md        # TCW-MPC 阶段性总结
│   └── 改进.md                       # 技术评估与改进路线图
│
├── 改进建议.md                        # 算法设计稿
└── 风场.md                            # 风场建模理论参考
```

### 模块依赖关系

```
quadrotor_dynamics.py  ←─┐   wind_field.py
        │                │        │
        │   (import)     │        │  (ROS topic)
        ▼                │        ▼
    mpc_node.py ─────────┘   /wind_field/velocity
        │
        │  (import)
        ▼
   power_model.py
        │
        ├──→ PX4 via MAVROS (setpoint_position/local)   [分层架构]
        └──→ CSV logging (mpc_log_*.csv)
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

每个新终端都需执行：

```bash
source /home/tan/catkin_ws/src/offboard_test/scripts/env.sh
```

### 2. 一键启动仿真 (推荐)

```bash
cd /home/tan/catkin_ws/src/offboard_test/scripts
./start_sitl.sh                    # 默认（含风场）
./start_sitl.sh --no-wind          # 无风场
./start_sitl.sh --headless         # 无 GUI
./start_sitl.sh --no-mavros        # 不启 MAVROS
```

### 3. 或在独立终端中手动启动

**终端 1 — PX4 SITL + Gazebo：**

```bash
source scripts/env.sh
roslaunch px4 mavros_posix_sitl.launch gui:=true vehicle:=iris
```

**终端 2 — 启动风场 + 运行控制脚本：**

```bash
source scripts/env.sh

# 启动风场
python3 -u scripts/wind_field.py &

# 设置 PX4 参数（首次）
rosrun mavros mavparam set COM_RC_IN_MODE 1
rosrun mavros mavparam set COM_ARM_WO_GPS 1
rosrun mavros mavparam set NAV_RCL_ACT 0

# 运行 MPC
rosrun offboard_test mpc_node.py
```

---

## 风场参数

极端湍流风场（`wind_field.py`）：

| 组件 | 模型 | 参数 |
|------|------|------|
| 平均风 | 幂律剖面 $u(z) = u_{\text{ref}}(z/z_{\text{ref}})^\alpha$ | $u_{\text{ref}} = 12$ m/s, $\alpha = 0.35$ |
| 湍流 | Dryden 成形滤波 (3 通道) | $\sigma_{u,v} = 4.0$, $\sigma_w = 2.5$ m/s |
| 阵风 | 1-cos 垂向 | $\pm 8$ m/s, 间隔 $\sim 20$ s |
| 气动阻力 | $F = \frac12 \rho C_d A \|V_{\text{rel}}\| V_{\text{rel}}$ | $\rho = 1.225$, $C_dA = 0.05$ m² |

---

## 控制器参数

MPC 核心参数 (`mpc_node.py`):

```python
N_pred = 20               # 预测时域 (0.6s @ 30Hz)
dt     = 0.033             # 控制周期 (30Hz)

Q = diag([4, 4, 5,         # 位置权重 (x, y, z)
           2, 2, 5])        # 速度权重
R = diag([14, 14, 15])     # 控制惩罚

# 分层架构
# MPC 输出位置 setpoint → PX4 PID 跟踪执行
```

---

## 性能 (SITL 悬停实验)

| 指标 | 分层 MPC | PID 基线 |
|------|---------|----------|
| **RMSE_XY** | **0.27 m** | 0.27 m |
| 6.6 m/s 均风 XY 误差 | ~0.5 m | — |
| 最大阵风 (11.8 m/s) XY 误差 | <1 m | — |
| 求解成功率 | 100% @ 30Hz | — |

分层 MPC 的跟踪精度在悬停场景下打平 PX4 原生 PID，但 MPC 在航点跟踪中具备预测优势（预测时域内优化轨迹），是论文的主要实验方向。

---

## 论文方向

1. **Physical Limits of Linear MPC in Hover** — 小角度假设在抗风倾角下失效
2. **Hierarchical MPC Architecture** — MPC 位置规划 + PX4 姿态跟踪
3. **Energy-Aware Trajectory Planning** — 利用风能降低功耗（需航点跟踪验证）
4. **DNN / PPO for Parameter Adaptation** — 数据驱动调优 MPC 权重

---

## 踩坑记录

| 坑 | 现象 | 修复 |
|----|------|------|
| PX4 进程被挂起 | MAVROS `connected: False` | 不能 Ctrl+Z, 需 `kill -9` 重开 |
| iris 模型不可见 | `.sdf.jinja` 未编译 | `python3 Tools/sitl_gazebo/scripts/jinja_gen.py` |
| 解锁前必须预发 setpoint | PX4 拒绝切 OFFBOARD | 先发 setpoint ≥3s 再切模式 |
| 纯线性 MPC 在 10+ m/s 风中坠机 | 25° 倾角突破小角度假设 | 分层 MPC（MPC 规划 → PX4 执行） |
| 悬停时 Coaste 策略有害 | 风能利用导致 MPC 与 PX4 目标分裂 | 悬停时强制 Coast=0 |
| ASLR segfault | PX4 v1.13.3 + Gazebo 11 | `setarch x86_64 -R` 禁用 ASLR |

---

## 许可证

MIT License

## 作者

- **小龙** — 项目主人
- **虾哥 🦐** — AI 助手
