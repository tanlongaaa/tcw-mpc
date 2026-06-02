# 🦐 虾哥工作全面交接文档

> **生成时间**: 2026-05-31 17:47 CST  
> **生成者**: 虾哥 🦐  
> **目的**: 完整记录全部工作进展、路径、环境、参数和待办，确保下一个 Agent 可以直接接手  

---

## 一、工作全景

### 1.1 项目时间线

| 日期 | 阶段 | 核心成果 |
|------|------|----------|
| 2026-05-25 | 工作区初始化 + PX4 Offboard 起步 | 身份配置、航点飞行脚本、诊断脚本、学术检索 Skills |
| 2026-05-27 | 首次收尾总结 | `HANDOVER_PX4_OFFBOARD.md` + `WORK_SUMMARY.md` |
| 2026-05-28 | TCW-MPC 第一阶段 | 极端湍流风场 + TCW-MPC 控制器 + SITL 调通 |
| 2026-05-30 | TCW-MPC 大迭代 (Round 1-5) | 非线性动力学 + BEMT 功耗 + 分层 MPC 架构 |

### 1.2 两条独立工作线

| 工作线 | 状态 | 进度 |
|--------|------|------|
| **PX4 Offboard 无人机控制** | 🔥 主力 | 分层 MPC 悬停验证通过，航点跟踪待做 |
| **学术检索 Skills** | 💤 待机 | google-scholar-search 可用，Asta key 403 待修 |

---

## 二、环境全貌

### 2.1 系统环境

| 项目 | 值 |
|------|-----|
| OS | Ubuntu 20.04 (x64) |
| 用户 | tan |
| 主机名 | tan-Legion-R7000P-ARH7 |
| 内核 | Linux 5.15.0-139-generic |
| Python | 3.8 (system) |
| Shell | bash |

### 2.2 核心软件栈

| 组件 | 版本 | 安装路径 |
|------|------|----------|
| ROS | Noetic | `/opt/ros/noetic/` |
| PX4 | v1.13.3 | `/home/tan/Desktop/px4rl/PX4-Autopilot/` |
| Gazebo | 11.15.1 | system |
| MAVROS | 1.20.1 | `/opt/ros/noetic/` (ros-noetic-mavros) |
| OSQP | 0.6.x | pip |
| OpenClaw | latest | `/home/tan/.npm-global/lib/node_modules/openclaw/` |

### 2.3 关键路径速查

```
# === PX4 / 仿真 ===
/home/tan/Desktop/px4rl/PX4-Autopilot/                     # PX4 v1.13.3 源码
/home/tan/Desktop/px4rl/PX4-Autopilot/build/px4_sitl_default/ # SITL 编译产物

# === ROS 工作区 ===
/home/tan/catkin_ws/                                       # catkin workspace
/home/tan/catkin_ws/src/offboard_test/                     # offboard 功能包
/home/tan/catkin_ws/src/offboard_test/scripts/             # ★ 项目主目录（全部脚本）

# === OpenClaw 工作区 ===
/home/tan/.openclaw/workspace/                             # Agent workspace root
/home/tan/.openclaw/agents/main/sessions/                  # 历史会话 transcripts
/home/tan/.openclaw/openclaw.json                          # Gateway 配置 (含 API keys)

# === OpenClaw Skills ===
~/.openclaw/workspace/skills/asta-skill/                   # Asta MCP (403 不可用)
~/.openclaw/workspace/skills/google-scholar-search/        # Semantic Scholar (可用)
~/.openclaw/workspace/skills/songge-academic-search/       # 多源学术检索
```

---

## 三、项目目录完整文件清单

### 3.1 `/home/tan/catkin_ws/src/offboard_test/scripts/`

```
scripts/
├── env.sh                          # ★ 环境变量脚本 (每个终端必 source)
│
├── mpc_node.py                     # ★★ TCW-MPC 控制器 (主线, ~1250 行)
│   # 包含: SL-MPC / B[5,2]自适应 / 分层架构 / BEMT 功耗 / CMA 2.0
│
├── mpc_node_v5_backup.py           # 旧版 MPC 备份 (v5, 不含分层/BEMT)
├── mpc_node_v4_broken.py           # 更早版本 (已废弃)
│
├── quadrotor_dynamics.py           # ★★ 完整非线性动力学 (新建, ~280 行)
│   # 包含: 全旋转矩阵 / CdA(α)气动 / BEMT功率 / 解析雅可比
│
├── wind_field.py                   # ★ 极端湍流风场生成器 (~517 行)
│   # Dryden 湍流 + 幂律平均风 + 1-cos 阵风 + Gazebo 加力
│
├── pid_baseline.py                 # ★ PID 基线对比脚本 (~200 行)
│   # v2: 修复起飞 + 50Hz 线程 + ManualControl
│
├── power_model.py                  # 统一 IMU 功耗模型 (~50 行)
├── analyze_logs.py                 # 日志对比分析脚本 (~180 行)
│
├── offboard_waypoint_test.py       # 简单航点飞行测试 (已验证可用)
├── fixed_attitude_node.py          # 原始姿态控制参考
├── fixed_attitude_node_v2.py       # 姿态控制 v2 (修复版)
├── diagnose_offboard.py            # 一键诊断脚本
│
├── mpc_log_*.csv                   # MPC 运行日志 (自动生成, ~50个)
├── pid_log_*.csv                   # PID 运行日志 (自动生成)
│
└── HANDOVER_TCW_MPC_V1.md          # TCW-MPC 阶段性总结 (5/28)
```

### 3.2 脚本/模块依赖关系

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
        ├──→ PX4 via MAVROS (setpoint_position/local)
        ├──→ PX4 via MAVROS (setpoint_raw/attitude)  [旧版直驱]
        └──→ CSV logging (mpc_log_*.csv)
```

---

## 四、核心技术演进 (5/28 → 5/30)

### 4.1 5/28: TCW-MPC v1 — 线性 MPC + 风模块

**架构**: 6 状态线性 MPC [px,py,pz,vx,vy,vz] 直接输出 AttitudeTarget  
**风模块**:
- 风扰前馈: k_drag × |W|×W 嵌入 MPC 等式约束 RHS
- CMA (约束裕度自适应): 动压驱动收紧控制边界
- EMA 风偏置学习: 三轴独立指数滑动平均
- TCWP (轨迹相干风预测): 空间核回归（长时域留论文用）
**结果**: 6.6m/s 均风中 XY 误差 ~0.5m ✅  
**局限**: 10+m/s 风中小角度假设失效 → 倾角 25° 时累积误差 ×20 步导致 crash

### 4.2 5/30: 大迭代 — 从线性到非线性分层

#### Round 1 — Bug 修复
- `_pos_traj_prev` 零值 → 从 MPC 解提取完整轨迹
- 相对风速 `V_rel = V_drone - V_wind`
- Integrator Yaw 旋转修复
- OSQP 线程安全 + 去周期重建

#### Round 2 — Energy-Aware MPC
- λ_E 推力惩罚 + u_ref = hover_thrust
- CMA 2.0 裕度感知（放松版）
- WindUtilizationAdvisor (Coast) → **悬停 Coast 有害** → 加 task_type 修复
- PredictiveEnergyManager

#### Round 3 — 实测 + 消融
- Coast=0 修复验证
- 风速归一化对比：MPC λ=5 归一化节能 16% vs PID（但绝对追踪差 4x）
- **结论**: 悬停下线性 MPC 无法在功耗上击败 PID（物理下限）

#### Round 4 — 全非线性动力学 (`quadrotor_dynamics.py`)
- 完整旋转矩阵（无小角度假设）
- CdA(α) 姿态相关气动阻力
- BEMT 叶素动量功率模型（诱导功率 + 型阻功率 + 效率）
- 解析雅可比
- SL-MPC 尝试 → 耦合不稳定 → 退化为 B[5,2] 自适应

#### Round 5 — 分层 MPC (最终方案) 🎯
- **MPC 输出位置 setpoint** → PX4 全链路 PID 跟踪
- MPC = 大脑（规划轨迹）、PX4 = 小脑（执行控制）
- **结果: 追踪精度打平 PID**（RMSE_XY = 0.27m vs 0.27m）

### 4.3 论文叙事方向

1. **Physical Limits of Linear MPC Hover** — 小角度假设在抗风所需倾角下失效
2. **Hierarchical MPC** — 分层架构弥补 MPC 频率不足
3. **Energy-Aware Trajectory Planning** — 需要在航点跟踪中验证（悬停无自由度）
4. **Motivates NMPC/PPO** — 线性模型边界已明确

---

## 五、当前参数配置

### 5.1 MPC 核心参数 (mpc_node.py)

```python
# 预测与控制
N_pred = 20                      # 预测步 (0.6s @ 30Hz)
dt     = 0.033                   # 时间步长 (30Hz)

# 分层架构
# MPC 输出: ref_pos (位置setpoint) → PX4 PID 执行

# 自适应
B[5,2] adaptive                  # cos(θ)cos(φ) 倾角修正

# Energy
lambda_energy = 3.0              # 推力惩罚（适中）

# CMA 2.0
relaxed                          # 裕度充足，不激进收紧

# BEMT
enabled                          # 诱导+型阻+效率功率计算

# 控制边界（直接输出时用）
u_max_nom = [+0.45, +0.45, 0.95]  # rad / normalized
u_min_nom = [-0.45, -0.45, 0.30]

# MPC 权重
Q = diag([4, 4, 5,  2, 2, 5])    # 位置+速度
R = diag([14, 14, 15])            # 控制惩罚
```

### 5.2 PX4 参数 (需设置)

```bash
rosrun mavros mavparam set COM_RC_IN_MODE 1   # No RC checks
rosrun mavros mavparam set COM_ARM_WO_GPS 1   # Arm without GPS
rosrun mavros mavparam set NAV_RCL_ACT 0      # No RC loss action
```

### 5.3 风场参数 (wind_field.py)

```python
# 极端湍流风场
u_ref   = 12.0   # m/s 参考风速 @10m
alpha   = 0.35   # 幂律指数 (城市)
σ_u,v   = 4.0    # m/s 水平湍流强度
σ_w     = 2.5    # m/s 垂直湍流强度
Lu,v    = 150    # m 水平湍流尺度
Lw      = 50     # m 垂直湍流尺度
```

---

## 六、启动流程

### 6.1 环境变量（每个终端第一步）

```bash
source /home/tan/catkin_ws/src/offboard_test/scripts/env.sh
```

`env.sh` 包含: ROS Noetic → catkin workspace → PX4/Gazebo 路径 → GAZEBO 插件/模型路径 → ROS_PACKAGE_PATH

### 6.2 终端 1: 启动 PX4 SITL

```bash
source /home/tan/catkin_ws/src/offboard_test/scripts/env.sh
roslaunch px4 mavros_posix_sitl.launch gui:=true vehicle:=iris
```

⚠️ 长期运行，**不能 Ctrl+Z 挂起**（会导致 MAVROS connected=false）

### 6.3 终端 2: 运行控制脚本

```bash
source /home/tan/catkin_ws/src/offboard_test/scripts/env.sh

# 启动风场（如需抗风测试）
python3 -u /home/tan/catkin_ws/src/offboard_test/scripts/wind_field.py &

# 设置 PX4 参数（首次/参数被重置后执行）
rosrun mavros mavparam set COM_RC_IN_MODE 1
rosrun mavros mavparam set NAV_RCL_ACT 0
rosrun mavros mavparam set COM_ARM_WO_GPS 1

# 运行 MPC 控制器（当前主线）
rosrun offboard_test mpc_node.py

# 或者运行 PID 基线（对比测试）
rosrun offboard_test pid_baseline.py

# 或者运行诊断（环境正常性检查）
rosrun offboard_test diagnose_offboard.py

# 或者运行简单航点测试
rosrun offboard_test offboard_waypoint_test.py
```

### 6.4 停止

```bash
Ctrl+C                         # 停止控制脚本
pkill -f wind_field            # 停止风场
# 终端 1 按 Ctrl+C 停止 SITL
# 如有残留:
kill -9 $(pgrep -f gzserver) $(pgrep -f gzclient) 2>/dev/null
```

---

## 七、踩过的坑 ⚠️

### 坑 1 — PX4 SITL 进程状态
**现象**: MAVROS `connected: False`  
**原因**: PX4 进程被 Ctrl+Z 挂起（T 状态）  
**检查**: `ps aux | grep px4 | grep -v grep`  
**修复**: `kill -9` 残留 → 重新 `make px4_sitl_default gazebo`

### 坑 2 — iris.sdf 缺失
**现象**: 无人机在 Gazebo 中不可见  
**原因**: 只有 `.sdf.jinja` 模板，缺少编译生成的 `.sdf`  
**修复**: `cd /home/tan/Desktop/px4rl/PX4-Autopilot && python3 Tools/sitl_gazebo/scripts/jinja_gen.py`

### 坑 3 — ManualControl 冲突
**现象**: 脚本同时发 offboard setpoint 和手动控制 → 阻止 offboard 切换  
**根因**: 原始 `fixed_attitude_node.py` 同时发布 `/mavros/manual_control/send`  
**修复**: v2 版本移除 man_pub，只保留 offboard setpoint

### 坑 4 — 模式切换无验证
**现象**: `set_mode()` 调用后不检查返回值，切换失败后脚本盲目继续  
**修复**: 加入 `resp.mode_sent` 检查和 `wait_for_mode()` 验证

### 坑 5 — Offboard 前置要求
**关键**: PX4 要求 setpoint 流已存在 ≥500ms 才能切 OFFBOARD  
**做法**: 切模式前先持续发 setpoint 2-3s

### 坑 6 — 10+m/s 风中纯线性 MPC 必坠
**物理原因**: 25° 倾角突破小角度假设，累积误差 ×20 步 `cos(25°) = 0.906` → `0.906^20 ≈ 0.14`  
**修复**: 分层 MPC（MPC 输出位置 → PX4 PID 执行）

### 坑 7 — 悬停 Coast 有害
**现象**: WindUtilizationAdvisor 在悬停中消极用风反而加功耗  
**根因**: 悬停无自由度，Coast 导致 MPC 与 PX4 目标分裂  
**修复**: 加 task_type 判断，悬停时 Coast=0

### 坑 8 — Gazebo 构建目标名
PX4 v1.13.3 命令是 `make px4_sitl_default gazebo`，不是 `px4_sitl`

---

## 八、日志文件说明

日志为自描述的 CSV 格式，自动生成于脚本同目录。命名 `mpc_log_YYYYMMDD_HHMMSS.csv` 和 `pid_log_YYYYMMDD_HHMMSS.csv`。

**分析工具**: `analyze_logs.py` 可对比 MPC vs PID 日志

**注意**: 日志文件非常庞大（每个 ~300-600KB），总存储 ~30MB+。如需清理旧日志，参考 5/30 完成版是当天最后一批。

**5/30 最终日志 (分层 MPC)**:
- `mpc_log_20260530_184818.csv` ~ `184901.csv`
- `pid_log_20260530_184425.csv`, `184945.csv`

---

## 九、当前状态 & 待办

### ✅ 已完成

- [x] PX4 SITL 环境（编译、启动、MAVROS 连接）
- [x] 极端湍流风场建模（Dryden + 幂律 + 阵风）
- [x] TCW-MPC 控制器（风扰前馈 + CMA + EMA 偏置 + TCWP）
- [x] 全非线性动力学 `quadrotor_dynamics.py`（旋转矩阵 + CdA(α) + BEMT + 解析雅可比）
- [x] BEMT 功耗模型 `power_model.py`
- [x] 分层 MPC 架构（MPC 输出位置 setpoint → PX4 PID）
- [x] PID 基线 `pid_baseline.py`（含 BEMT 功耗）
- [x] 悬停对比验证（分层 MPC RMSE_XY=0.27m 打平 PID）
- [x] google-scholar-search skill 可用
- [x] 日志分析工具 `analyze_logs.py`

### ⏳ 待办（按优先级）

#### 🔴 高优先级
- [ ] **统一 BEMT 功耗到 PID 脚本**（确保功率对比公平）
- [ ] **λ=0/3/5 三轮悬停对比**（同 BEMT 功耗模型，隔离节能效果）
- [ ] **航点跟踪实验**（MPC 预测优势的主场——悬停无自由度验证不了节能规划）

#### 🟡 中优先级
- [ ] SL-MPC 正则化/信任域改进（解决耦合不稳定问题）
- [ ] 延长预测时域至 1.5s（N=50+），让 TCWP 空间记忆体现价值
- [ ] PPO 接口预留
- [ ] Asta MCP API key 重新申请（当前 403）

#### 🟢 低优先级
- [ ] MPC 部署到硬件（`px4_fmu-v5_default` 构建目标）
- [ ] 风传感器噪声模型（验证鲁棒性）
- [ ] 学术检索论文深挖（RL + 无人机 + 湍流 + 节能方向）

---

## 十、已有交接文档索引

| 文档 | 路径 | 内容 |
|------|------|------|
| `HANDOVER_FULL.md` | `/home/tan/.openclaw/workspace/HANDOVER_FULL.md` | **本文档** — 最全面最新的交接文档 |
| `HANDOVER_TCW_MPC_V1.md` | `/home/tan/catkin_ws/src/offboard_test/scripts/HANDOVER_TCW_MPC_V1.md` | TCW-MPC 5/28 阶段性总结（含完整启动流程和参数） |
| `HANDOVER_PX4_OFFBOARD.md` | `/home/tan/.openclaw/workspace/HANDOVER_PX4_OFFBOARD.md` | 原始 PX4 Offboard 交接（环境搭建 + 坑记录） |
| `WORK_SUMMARY.md` | `/home/tan/.openclaw/workspace/WORK_SUMMARY.md` | 首次收尾（OpenClaw 中文支持调研） |
| `memory/2026-05-27.md` | `/home/tan/.openclaw/workspace/memory/2026-05-27.md` | 日志：工作区初始化 |
| `memory/2026-05-30.md` | `/home/tan/.openclaw/workspace/memory/2026-05-30.md` | 日志：TCW-MPC 大迭代详细记录 |

---

## 十一、给接手 Agent 的快速启动指南

### 如果你想：

**验证环境正常**:
```bash
source /home/tan/catkin_ws/src/offboard_test/scripts/env.sh
rosrun offboard_test diagnose_offboard.py
```

**跑一次 MPC 悬停测试**:
```bash
# T1: roslaunch px4 mavros_posix_sitl.launch gui:=true vehicle:=iris
# T2:
source /home/tan/catkin_ws/src/offboard_test/scripts/env.sh
python3 -u /home/tan/catkin_ws/src/offboard_test/scripts/wind_field.py &
sleep 2
rosrun mavros mavparam set COM_RC_IN_MODE 1 && rosrun mavros mavparam set COM_ARM_WO_GPS 1
rosrun offboard_test mpc_node.py
```

**看最新代码**:
- MPC 主线: `/home/tan/catkin_ws/src/offboard_test/scripts/mpc_node.py`
- 非线性动力学: `/home/tan/catkin_ws/src/offboard_test/scripts/quadrotor_dynamics.py`
- PID 基线: `/home/tan/catkin_ws/src/offboard_test/scripts/pid_baseline.py`

**理解论文架构**:
1. 读 5/30 日志 → 2. 读 `quadrotor_dynamics.py`（物理建模） → 3. 读 `mpc_node.py`（分层 MPC） → 4. 看 5/30 最终日志验证分层架构效果

---

*文档结束 — 2026-05-31 虾哥 🦐*
