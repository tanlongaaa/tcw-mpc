# Plant 6-DOF 模型改进文档

> 记录从最简 6-DOF 刚体模型到当前 v9 最优配置的全部改进
>
> 日期: 2026-06-11 | 迭代: 11 轮 | err_xy: 20.9m → 7.5m (↓64%)

---

## 1. 总体架构

```
                    ┌─────────────────────────────────────────────┐
                    │              PX4 SITL (真飞控)               │
                    │   EKF2 → Position Ctrl → Velocity Ctrl →     │
                    │   Attitude Ctrl → Rate Ctrl → Mixer          │
                    └──────────────┬────────────────┬─────────────┘
                                   │ MAVLink        │ MAVLink
                              HIL_SENSOR       HIL_ACTUATOR
                                   │                │
                    ┌──────────────▼────────────────▼─────────────┐
                    │           backend_main.py                    │
                    │  ┌─────────────┐  ┌──────────────────────┐  │
                    │  │ SensorModels │  │ get_wind_drag_force() │  │
                    │  └──────┬──────┘  └──────────┬───────────┘  │
                    │         │                     │              │
                    │  ┌──────▼─────────────────────▼───────────┐  │
                    │  │         Quad6DOFPlant                  │  │
                    │  │  ┌──────────────────────────────────┐  │  │
                    │  │  │  RotorModel × 4                  │  │  │
                    │  │  │  · 二次方推力 + 动态入流           │  │  │
                    │  │  │  · 旋翼拖拽(运动阻尼,去风偏置)     │  │  │
                    │  │  │  · Blade Flapping 恢复力矩        │  │  │
                    │  │  │  · 旋翼滚转力矩 + 反扭矩          │  │  │
                    │  │  │  · 旋翼陀螺力矩                   │  │  │
                    │  │  │  · 一阶电机滞后 (τ_up ≠ τ_down)  │  │  │
                    │  │  └──────────────────────────────────┘  │  │
                    │  │  + 6-DOF 刚体 (均质, ENU/FLU 坐标系)  │  │
                    │  │  + 各向异性机身气动阻力                 │  │
                    │  └────────────────────────────────────────┘  │
                    └──────────────────────────────────────────────┘
                                      ▲
                                      │ /wind_field/velocity
                    ┌─────────────────┴──────────────────────────┐
                    │            wind_field.py (--no-wrench)      │
                    │  Dryden Turbulence + Power Law + Gust       │
                    └────────────────────────────────────────────┘
```

**坐标系:**
- 世界系 ENU: x=东, y=北, z=上
- 机体 FLU: x=前, y=左, z=上 (与 PX4 / MAVROS 一致)

---

## 2. 6-DOF 刚体动力学

### 2.1 平动方程 (ENU)

$$\dot{\mathbf{p}} = \mathbf{v}$$

$$\dot{\mathbf{v}} = \frac{1}{m}\left( \mathbf{R}_{FLU}^{ENU} \cdot \mathbf{F}_{body} + \mathbf{F}_{gravity} + \mathbf{F}_{disturb} \right)$$

其中:

$$\mathbf{F}_{gravity} = \begin{bmatrix} 0 \\ 0 \\ -mg \end{bmatrix}$$

$$\mathbf{F}_{disturb} = \mathbf{F}_{ext\_enu} + \mathbf{F}_{wind\_sine}$$

$\mathbf{R}_{FLU}^{ENU}$ 为 body FLU → world ENU 旋转矩阵:

$$\mathbf{R} = \mathbf{R}_z(\psi) \cdot \mathbf{R}_y(\theta) \cdot \mathbf{R}_x(\phi)$$

### 2.2 转动方程 (body FLU)

$$\mathbf{J}\dot{\boldsymbol{\omega}} = \boldsymbol{\tau}_{motor} + \boldsymbol{\tau}_{ext} - \boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega}) - k_\omega \boldsymbol{\omega}$$

其中惯量矩阵:

$$\mathbf{J} = \begin{bmatrix}
J_x & 0 & 0 \\
0 & J_y & 0 \\
0 & 0 & J_z
\end{bmatrix} = \begin{bmatrix}
0.029125 & 0 & 0 \\
0 & 0.029125 & 0 \\
0 & 0 & 0.055225
\end{bmatrix} \text{ kg·m}^2 \quad \text{(Iris SDF 真值)}$$

### 2.3 欧拉角运动学

$$\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$

### 改进前后对比

| 项目 | 改进前 | 改进后 |
|---|---|---|
| 惯量矩阵 | 估计值 | Iris SDF 真值 (Jx=Jy=0.029125, Jz=0.055225) |
| 角速度阻尼 | 无 | $k_\omega=0.3$ |
| 外部扰动力矩 | 无 | $\boldsymbol{\tau}_{ext\_flu}$ 接口 |

---

## 3. 旋翼模型 (RotorModel)

每个旋翼独立建模，包含完整的气动和机电动力学。

### 3.1 电机动力学

**一阶滞后滤波器** (复刻 Gazebo `FirstOrderFilter`):

$$\omega_{t+dt} = \alpha \cdot \omega_t + (1-\alpha) \cdot \omega_{cmd}$$

$$\alpha = \begin{cases}
e^{-dt/\tau_{up}} & \text{if } \omega_{cmd} > \omega_t \\
e^{-dt/\tau_{down}} & \text{if } \omega_{cmd} \leq \omega_t
\end{cases}$$

$$\tau_{up}=0.0125\text{s}, \quad \tau_{down}=0.025\text{s}$$

$$\omega_{cmd} = \omega_{idle} + \omega_{range} \cdot u \in [100, 1100] \text{ rad/s}$$

### 3.2 推力模型

**改进前:** 线性推力 (legacy 模式)

$$T = 4 \cdot k_{thrust} \cdot u_{throttle}$$

**改进后:** 二次方推力 + 动态入流

**原始推力 (Gazebo line 192):**

$$T_{raw} = C_T \cdot |\omega| \cdot \omega = 5.84\times10^{-6} \cdot \omega^2$$

**Pitt-Peters 动态入流滞后** (一阶滤波器):

$$\tau_{inflow} \frac{dT_{lagged}}{dt} = T_{raw} - T_{lagged}$$

$$T_{lagged}^{t+dt} = e^{-dt/\tau_{inflow}} \cdot T_{lagged}^t + (1 - e^{-dt/\tau_{inflow}}) \cdot T_{raw}^t$$

$$\tau_{inflow} = 0.05\text{s (最优值)}$$

**轴向速度推力衰减** (Gazebo line 203-206):

$$T = T_{lagged} \cdot \max\left(0, \min\left(1, 1 - \frac{|v_{axial}|}{25}\right)\right)$$

$v_{axial} = v_{rel\_body}[2]$ 为相对来流在旋翼轴向的分量。

> **物理含义:** 旋翼尾迹具有惯性，推力不能瞬时跟随转速变化。$\tau_{inflow} \approx \frac{0.849R}{\Omega R} \cdot \frac{1}{C_T/\sigma}$，对于 10 英寸桨在悬停转速下约 0.05-0.15s。

### 3.3 旋翼拖拽力 (Martin & Salaün 2010, IROS)

**改进前:** 无独立旋翼拖拽模型，使用集总体阻尼 (linear_damping=3.5)。

**改进后:** 每旋翼独立拖拽，但分离运动阻尼与风致偏置:

$$\mathbf{F}_{aero}^{(i)} = -|\omega_i| \cdot k_D \cdot \mathbf{v}_{plane}^{(motion)}$$

$$\mathbf{v}_{plane}^{(motion)} = \begin{bmatrix} v_{rotor\_body,x} \\ v_{rotor\_body,y} \\ 0 \end{bmatrix} \quad \text{(仅无人机自身运动，不含风)}$$

$$\mathbf{v}_{rotor\_body} = \mathbf{v}_{FLU} + \boldsymbol{\omega} \times \mathbf{r}_{rotor}$$

$$k_D = 0.000175 \quad \text{(Iris SDF)},\quad \mathbf{r}_{rotor} \text{ 为旋翼在机体坐标系的位置}$$

> **关键设计决策:** 旋翼拖拽仅使用无人机自身运动速度，**不包含风场速度**。风致旋翼偏置力由机身 CdA 统一吸收（见 §4），匹配 Gazebo 行为。

### 3.4 旋翼滚转力矩

$$\boldsymbol{\tau}_{roll}^{(i)} = -|\omega_i| \cdot dir_i \cdot k_\mu \cdot \mathbf{v}_{plane}^{(full)}$$

$$\mathbf{v}_{plane}^{(full)} = \begin{bmatrix} v_{rel\_body,x} \\ v_{rel\_body,y} \\ 0 \end{bmatrix} \quad \text{(含风场，用于气动力矩)}$$

$$k_\mu = 1 \times 10^{-6} \quad \text{(Iris SDF)}$$

$dir_i = +1$ (CCW) 或 $-1$ (CW)

### 3.5 反扭矩 (Yaw)

$$\tau_{yaw}^{(i)} = -dir_i \cdot T_i \cdot k_M$$

$$k_M = 0.06 \quad \text{(Iris SDF)}$$

### 3.6 旋翼拖拽力矩

$$\boldsymbol{\tau}_{drag}^{(i)} = \mathbf{r}_{rotor}^{(i)} \times \mathbf{F}_{aero}^{(i)}$$

注意: $\mathbf{F}_{aero}^{(i)}$ 使用 motion-only 速度 (§3.3)，因此拖拽力矩也不含风致偏置。

### 3.7 Blade Flapping 恢复力矩 ⭐ 新增

**物理原理:** 水平来流导致前行/后行桨叶升力不对称 → 旋翼盘倾斜 → 产生恢复力矩（风标效应/dihedral effect）。

$$\boldsymbol{\tau}_{flap}^{(i)} = -k_{flap} \cdot |\omega_i| \cdot \mathbf{v}_{horizontal}^{(full)}$$

$$\mathbf{v}_{horizontal}^{(full)} = \begin{bmatrix} v_{rel\_body,x} \\ v_{rel\_body,y} \\ 0 \end{bmatrix}$$

$$k_{flap} = 5 \times 10^{-5} \text{ N·m·s/rad·m (最优值)}$$

$$\mathbf{v}_{rel\_body} = \mathbf{v}_{rotor\_body} - \mathbf{v}_{wind\_FLU}$$

> **注意:** flapping 恢复力矩使用含风场的全相对速度，与旋翼拖拽力使用 motion-only 速度不同。这是因为 flapping 是气动力矩效应，应感知完整来流；而拖拽力的风偏置分量已由机身 CdA 处理。

> **调参记录:** $k_{flap}=1\times10^{-4}$ 导致 attitude 振荡失控 (roll max 64°)。$5\times10^{-5}$ 为稳定工作值。

### 3.8 旋翼陀螺力矩 ⭐ 新增

**物理原理:** 转子高速旋转时，机体转动会产生陀螺耦合。

$$\boldsymbol{\tau}_{gyro}^{(i)} = dir_i \cdot I_{rotor} \cdot \omega_i \cdot (\mathbf{e}_z \times \boldsymbol{\omega}_{body})$$

展开:

$$\boldsymbol{\tau}_{gyro}^{(i)} = dir_i \cdot I_{rotor} \cdot \omega_i \cdot \begin{bmatrix} q \\ -p \\ 0 \end{bmatrix}$$

$$I_{rotor} = 4 \times 10^{-5} \text{ kg·m}^2 \quad \text{(10寸桨转子惯量)}$$

### 3.9 单旋翼合力矩汇总

$$\boldsymbol{\tau}_{total}^{(i)} = \boldsymbol{\tau}_{roll}^{(i)} + \boldsymbol{\tau}_{flap}^{(i)} + \boldsymbol{\tau}_{reaction}^{(i)} + \boldsymbol{\tau}_{drag}^{(i)}$$

陀螺力矩在整车层面累加:

$$\boldsymbol{\tau}_{gyro}^{total} = \sum_{i=1}^{4} \boldsymbol{\tau}_{gyro}^{(i)}$$

---

## 4. 机身气动模型

### 4.1 各向异性线性阻尼 (body FLU)

$$\mathbf{F}_{body\_drag} = -\begin{bmatrix}
k_{bx} \cdot v_{rel\_flu,x} \\
k_{by} \cdot v_{rel\_flu,y} \\
k_{bz} \cdot v_{rel\_flu,z}
\end{bmatrix}$$

$$\mathbf{v}_{rel\_flu} = \mathbf{v}_{FLU} - \mathbf{v}_{wind\_FLU}$$

$$k_{bx}=0.10, \quad k_{by}=0.30, \quad k_{bz}=0.20 \text{ N/(m/s)}$$

> **物理含义:** $k_{by} > k_{bx}$ 反映四旋翼臂杆侧方投影面积大于前方截面。

### 4.2 二次方气动阻力 (ENU) ⭐ 新增

**改进前:** 无。

**改进后:** 风场通过机身 CdA 产生 quadratic drag，在 backend_main.py 中注入:

$$\mathbf{F}_{quad\_drag} = \frac{1}{2} \rho C_d A \cdot |\mathbf{V}_{rel}| \cdot \mathbf{V}_{rel}$$

$$\mathbf{V}_{rel} = \mathbf{V}_{wind\_ENU} - \mathbf{V}_{drone\_ENU} \quad \text{(风相对于无人机)}$$

$$\rho = 1.225 \text{ kg/m}^3, \quad C_d A = 0.12 \text{ m}^2 \quad \text{(v9最优)}$$

该力通过 `plant.set_ext_force_enu()` 注入平动方程。

> **关键修正 (P0):** 原始代码中 $\mathbf{V}_{rel} = \mathbf{V}_{drone} - \mathbf{V}_{wind}$ (符号反了)，导致静止无人机在东风中受到向西的力。修正后 $\mathbf{V}_{rel} = \mathbf{V}_{wind} - \mathbf{V}_{drone}$，力与风向一致。

### 4.3 风场两路径架构

```
wind_field.py ──┬── /wind_field/velocity ── set_wind_vel_enu() ── 旋翼层面
                │    · 推力轴向修正 (v_axial)
                │    · Blade Flapping 恢复力矩
                │    · 旋翼滚转力矩
                │
                └── get_wind_drag_force() ── set_ext_force_enu() ── 机身层面
                     · 二次方气动阻力 (0.5·ρ·CdA·|Vrel|·Vrel)
```

**旋翼拖拽力使用 motion-only 速度 (不含风)** — 避免旋翼与机身双重计算风致偏置力。

---

## 5. 合力/合力矩汇总

### 5.1 Body 合力 (FLU)

$$\mathbf{F}_{body} = \underbrace{\sum_{i=1}^{4} \begin{bmatrix} 0 \\ 0 \\ T_i \end{bmatrix}}_{\text{旋翼推力}} + \underbrace{\sum_{i=1}^{4} \mathbf{F}_{aero}^{(i)}}_{\text{旋翼拖拽 (motion-only)}} + \underbrace{\mathbf{F}_{body\_drag}}_{\text{机身线性阻尼}}$$

### 5.2 Body 合力矩 (FLU)

$$\boldsymbol{\tau}_{body} = \sum_{i=1}^{4} \left( \boldsymbol{\tau}_{total}^{(i)} + \mathbf{r}_{rotor}^{(i)} \times \begin{bmatrix} 0 \\ 0 \\ T_i \end{bmatrix} \right) + \boldsymbol{\tau}_{gyro}^{total}$$

### 5.3 ENU 合力

$$\mathbf{F}_{ENU} = \mathbf{R} \cdot \mathbf{F}_{body} + \mathbf{F}_{gravity} + \mathbf{F}_{quad\_drag} + \mathbf{F}_{ext}$$

---

## 6. 旋翼布局 (Iris)

| 旋翼 | 位置 (FLU, m) | 方向 | 电机编号 |
|---|---|---|---|
| rotor_0 | (+0.13, -0.22, +0.023) | CCW (+1) | 前右 |
| rotor_1 | (-0.13, +0.20, +0.023) | CCW (+1) | 后左 |
| rotor_2 | (+0.13, +0.22, +0.023) | CW (-1) | 前左 |
| rotor_3 | (-0.13, -0.20, +0.023) | CW (-1) | 后右 |

---

## 7. 传感器模型

```
特定力 (body FLU):         s_f = R^T · (a_ENU - g_ENU)
GPS 位置:                   p_GPS = p_ENU + noise
气压高度:                    h = -pz + baro_offset + noise
磁力计:                     m = R^T · m_NED + noise
```

---

## 8. PX4 控制器集成

### HIL 接口

```
backend_main.py (250Hz)
  ↓ MAVLink HIL_SENSOR (position, velocity, attitude, IMU, GPS, baro)
PX4 SITL (EKF2 + 级联PID + 混控器)
  ↓ MAVLink HIL_ACTUATOR_CONTROLS (4× motor_outputs [0,1])
plant_6dof.set_motor_outputs() → RotorModel
```

### PX4 PID 参数 (v9 最优, 11轮迭代)

| 参数 | 默认值 | v9 最优 | 含义 |
|---|---|---|---|
| `MPC_TILTMAX_AIR` | 15° | **25°** | 最大倾斜角 |
| `MPC_XY_P` | 0.95 | **0.8** | 位置 P 增益 |
| `MPC_XY_VEL_P_ACC` | 0.04 | **0.08** | 速度→加速度 P 增益 |
| `MPC_XY_VEL_I_ACC` | 0.03 | **0.08** | 速度→加速度 I 增益 |
| `MPC_XY_VEL_D_ACC` | 0.08 | **0.04** | 速度→加速度 D 增益 |

> **关键发现:** 默认 `MPC_TILTMAX_AIR=15°` 严重限制抗风能力 (最大水平推力仅 $14.7 \times \tan(15°) = 3.9N$)。从 15°→25° 的单参数改动贡献了约 30% 的误差降低。

---

## 9. 风场模型 (wind_field.py)

```
风场速度 (ENU):
  V_wind(pz, t) = V_mean(z) + V_turb(t) + V_gust(t)

幂律平均风:     V_mean(z) = u_ref × (z / z_ref)^α
Dryden 湍流:    V_turb ~ 成型滤波器, σ_u,v=4.0, σ_w=2.5 m/s
  Lu,v=150m, Lw=50m
阵风:            V_gust ~ 矩形/正弦调制
```

HIL 模式使用 `--no-wrench` (仅发布风速，不施加 Gazebo body wrench)。

---

## 10. 性能对比

### 11 轮迭代历程

| 版本 | err_xy均值 | err_max | 末30s | 关键改动 |
|---|---|---|---|---|
| **v0** | **20.9m** | 61.8m | 16.8m | 基线 (原始 6-DOF 刚体) |
| v1 | 17.3m | 32.4m | 17.9m | +P0(quadratic body drag 方向修正) +P1(blade flapping, k_flap=5e-05) |
| v2 | 62.5m | 102.9m | 85.8m | ❌ k_flap=1e-04 (振荡失控) |
| v3 | 21.3m | 33.1m | 22.0m | 各向异性 drag + gyro (无 flapping 调参) |
| v4 | 19.8m | 51.6m | 27.3m | +动态入流 τ=0.10 (推力响应过慢) |
| v5 | 14.1m | 34.0m | 20.4m | ⭐ drag分离(去风偏置) + CdA=0.10 + τ=0.05 |
| v6 | 28.7m | 65.9m | 54.7m | ❌ 统一drag (x=y=0.20) + CdA=0.12 → 失去侧向优势 |
| v7 | 11.5m | 24.0m | 13.6m | ⭐ 恢复anisotropic drag + CdA=0.12 |
| v8 | 7.4m | 19.2m | 4.7m | ⭐ +PID粗调 (TILTMAX=30, P=0.8, I=0.10) |
| **v9** | **7.5m** | 28.1m | **3.6m** | ⭐⭐ +PID精调 (TILTMAX=25, I=0.08, D=0.04) |
| v10 | 7.8m | 32m | 6.2m | 激进PID (P=1.0, I=0.12) → 边际递减 |
| v11 | 13.0m | 69m | 31.8m | ❌ CdA=0.15 + TILTMAX=30 → z不稳定 |

### 最终成果 (v9 vs v0)

| 指标 | v0 | v9 | 改善 |
|---|---|---|---|
| err_xy 均值 | 20.9m | 7.5m | ↓64% |
| err_xy 峰值 | 61.8m | 28.1m | ↓55% |
| 末30s err | 16.8m | 3.6m | ↓79% |
| x/y 末30s | -15.6/-5.0 | +1.0/-1.1 | 原点附近 |
| 漂移 | 18.2m | 4.7m | ↓74% |
| z 稳定性 | 2.47±0.08 | 2.50±0.05 | ✅ |

---

## 11. 与 Gazebo 的差异分析

| 物理效应 | Gazebo Iris | plant_6dof v9 | 备注 |
|---|---|---|---|
| 推力模型 | $C_T \cdot \omega^2$ | $C_T \cdot \omega^2$ + 动态入流 | HIL 更真实 (有入流滞后) |
| 旋翼拖拽 | 运动阻尼 | 运动阻尼 | HIL 匹配 Gazebo (drag分离) |
| Blade Flapping | ✅ (隐式, 多体物理) | ✅ (显式, k_flap=5e-05) | 恢复力矩 |
| 旋翼陀螺力矩 | ✅ (多体解算) | ✅ (解析公式) | |
| 电机滞后 | τ_up/τ_down | τ_up/τ_down | 一致 |
| 机身气动 | 碰撞几何 | 各向异性线性 + quadratic CdA | |
| **风场耦合** | **body wrench 集中力** | **速度场 → 旋翼 + 机身** | **根本差异** 🎯 |
| err_xy (5m/s风) | ~0.6m | ~7.5m | 差距 12x |

### 根本差异

**Gazebo:** `wind_field.py` 通过 `apply_body_wrench` 将风效应打包为一个集中力 (CdA=0.05 拟合总效果)，作用在机体上。Gazebo 旋翼模型**不感知**外部风场速度，仅做运动阻尼。

**HIL:** 风场作为速度场传入旋翼和机身两个层面，物理上更完整但产生更大的净扰动力。此差异无法通过 PID 调参消除——需要 MPC 带风场前馈或进一步提高 plant 物理保真度 (如旋翼-旋翼气动干扰)。

---

## 12. 参数汇总

### plant_6dof (`sim_default.yaml`)

```yaml
plant:
  mass: 1.5
  gravity: 9.81
  Jx: 0.029125; Jy: 0.029125; Jz: 0.055225

  body_drag_x: 0.10    # 前方阻尼 N/(m/s)
  body_drag_y: 0.30    # 侧方阻尼 N/(m/s)
  body_drag_z: 0.20
  angular_damping: 0.3

  rotor:
    motor_constant: 5.84e-06       # C_T
    moment_constant: 0.06           # C_M
    rotor_drag_coefficient: 0.000175 # k_D (Martin & Salaün)
    rolling_moment_coefficient: 1e-06 # k_μ
    flapping_coefficient: 5e-05      # k_flap ★
    rotor_inertia: 4e-05             # 转子惯量 (gyro) ★
    rotor_radius: 0.127              # 10寸桨 ★
    inflow_tau: 0.05                 # 入流时间常数 ★
    time_constant_up: 0.0125
    time_constant_down: 0.025
    max_rot_velocity: 1100.0
    idle_speed: 100.0
    speed_range: 1400.0
```

★ 标记为本次新增参数。

### backend_main.py

```python
CdA = 0.12    # 二次方气动阻力等效面积 (m²)
rho = 1.225   # 空气密度 (kg/m³)
```

### PX4 PID

```bash
MPC_TILTMAX_AIR=25
MPC_XY_P=0.8
MPC_XY_VEL_P_ACC=0.08
MPC_XY_VEL_I_ACC=0.08
MPC_XY_VEL_D_ACC=0.04
```

---

## 13. 文件列表

| 文件 | 改动 |
|---|---|
| `quad_sim/scripts/plant_6dof.py` | RotorModel 完整重写 + 6-DOF 扩展 |
| `quad_sim/scripts/backend_main.py` | 风场两路径注入 + CdA 调优 |
| `quad_sim/config/sim_default.yaml` | 所有新增参数 |
| `scripts/start_hil_wind.sh` | PX4 PID 自动设置 (v9 值) |
| `scripts/pid_baseline.py` | 无改动 (仅采集 CSV) |
| `scripts/wind_field.py` | 无改动 (--no-wrench 模式) |
