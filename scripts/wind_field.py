#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wind_field.py — 极端湍流风场生成器

基于 Dryden 湍流模型 + 幂律平均风 + 1-cos 离散阵风，
通过 Gazebo apply_body_wrench 将气动阻力实时施加到 Iris 无人机。

风场 = W_mean(幂律) + W_turb(Dryden) + W_gust(1-cos)

启动方式:
  python3 wind_field.py                          # 默认极端参数
  python3 wind_field.py --rate 20 --debug        # 20Hz + 打印详细日志
  python3 wind_field.py --no-wrench              # 只生成风场不施加力 (测试用)
"""

import argparse
import sys
import numpy as np
from scipy import signal
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Vector3, Vector3Stamped, Point
from tf.transformations import euler_from_quaternion


# ══════════════════════════════════════════════════════════
# 1. Dryden 连续湍流模型
#    白噪声 → 成形滤波器 → 有色湍流
#    离散化: 前向欧拉 (保方差)
# ══════════════════════════════════════════════════════════
class DrydenTurbulence:
    """
    Dryden 湍流模型 — 3 通道独立滤波

    纵向 (u): H_u(s) = K_u / (τ_u·s + 1)             — 1 阶
    横向 (v): H_v(s) = K_v·(√3τ_v·s+1)/(τ_v·s+1)²   — 2 阶
    垂向 (w): H_w(s) = K_w·(√3τ_w·s+1)/(τ_w·s+1)²   — 2 阶

    前向欧拉离散化保证输出方差接近连续理论值。
    """

    def __init__(self, dt, U0=10.0,
                 Lu=150.0, Lv=150.0, Lw=50.0,
                 sigma_u=4.0, sigma_v=4.0, sigma_w=2.5):
        self.dt = dt
        self.U0 = max(U0, 1.0)          # 参考空速 (不能为零)
        # 保存参数 (供打印使用)
        self.sigma_u = sigma_u
        self.sigma_v = sigma_v
        self.sigma_w = sigma_w
        self.Lu = Lu
        self.Lv = Lv
        self.Lw = Lw

        # ── 纵向: 1 阶滤波器 ─────────────────────────
        self.tau_u = Lu / self.U0
        self.K_u = sigma_u * np.sqrt(2.0 * Lu / (np.pi * self.U0))
        self._u = 0.0                    # 滤波器状态

        # ── 横向: 2 阶状态空间 ───────────────────────
        self.tau_v = Lv / self.U0
        self.K_v = sigma_v * np.sqrt(Lv / (np.pi * self.U0))
        # 可控标准型: ẋ = A·x + B·η, y = C·x
        tv = self.tau_v
        self.A_v = np.array([[0.0, 1.0],
                             [-1.0 / tv**2, -2.0 / tv]])
        self.B_v = np.array([[0.0], [1.0]])
        self.C_v = np.array([[self.K_v / tv**2,
                              self.K_v * np.sqrt(3.0) / tv]])
        self._v = np.zeros(2)

        # ── 垂向: 2 阶状态空间 ───────────────────────
        self.tau_w = Lw / self.U0
        self.K_w = sigma_w * np.sqrt(Lw / (np.pi * self.U0))
        tw = self.tau_w
        self.A_w = np.array([[0.0, 1.0],
                             [-1.0 / tw**2, -2.0 / tw]])
        self.B_w = np.array([[0.0], [1.0]])
        self.C_w = np.array([[self.K_w / tw**2,
                              self.K_w * np.sqrt(3.0) / tw]])
        self._w = np.zeros(2)

        # 统计 (调试用)
        self._n_steps = 0
        self._sum_u = 0.0
        self._sum_v = 0.0
        self._sum_w = 0.0
        self._sum_uu = 0.0
        self._sum_vv = 0.0
        self._sum_ww = 0.0

    def step(self):
        """生成一步湍流, 返回 [u_t, v_t, w_t] (m/s)"""
        # 连续白噪声 (PSD=1) → 离散白噪声方差 = 1/dt
        eta = np.random.randn(3) / np.sqrt(self.dt)

        # 纵向 — 前向欧拉
        self._u += (self.dt / self.tau_u) * (self.K_u * eta[0] - self._u)

        # 横向
        self._v += self.dt * (self.A_v @ self._v + self.B_v.flatten() * eta[1])
        v_out = float(self.C_v @ self._v)

        # 垂向
        self._w += self.dt * (self.A_w @ self._w + self.B_w.flatten() * eta[2])
        w_out = float(self.C_w @ self._w)

        # 统计
        self._n_steps += 1
        self._sum_u += self._u
        self._sum_v += v_out
        self._sum_w += w_out
        self._sum_uu += self._u ** 2
        self._sum_vv += v_out ** 2
        self._sum_ww += w_out ** 2

        return np.array([self._u, v_out, w_out])

    def stats(self):
        """返回各通道的均值 & 标准差 (调试用)"""
        if self._n_steps < 2:
            return None
        n = self._n_steps
        return {
            'u': (self._sum_u / n, np.sqrt(self._sum_uu / n - (self._sum_u / n) ** 2)),
            'v': (self._sum_v / n, np.sqrt(self._sum_vv / n - (self._sum_v / n) ** 2)),
            'w': (self._sum_w / n, np.sqrt(self._sum_ww / n - (self._sum_w / n) ** 2)),
        }

    def reset(self):
        self._u = 0.0
        self._v = np.zeros(2)
        self._w = np.zeros(2)
        self._n_steps = 0
        self._sum_u = self._sum_v = self._sum_w = 0.0
        self._sum_uu = self._sum_vv = self._sum_ww = 0.0


# ══════════════════════════════════════════════════════════
# 2. 完整风场 = 平均风(幂律) + 湍流(Dryden) + 阵风(1-cos)
# ══════════════════════════════════════════════════════════
class WindField:
    """极端湍流风场, ENU 坐标系输出"""

    def __init__(self, dt=0.05):
        self.dt = dt

        # ── 平均风 (幂律剖面) ─────────────────────────
        self.u_ref = 12.0          # 10m 参考风速 [m/s]
        self.z_ref = 10.0          # 参考高度 [m]
        self.alpha = 0.35          # 风切变指数 (城市/森林: 极端)
        self.wind_dir = np.deg2rad(45.0)  # 风向 45° (东北风, 气象惯例: 来向)

        # ── 湍流 ─────────────────────────────────────
        self.turb = DrydenTurbulence(
            dt=dt,
            U0=self.u_ref,
            Lu=150.0, Lv=150.0, Lw=50.0,
            sigma_u=4.0, sigma_v=4.0, sigma_w=2.5,
        )

        # ── 阵风 (1-cos 垂向阵风) ────────────────────
        self.gust_w_max = 8.0       # 最大垂向阵风 [m/s]
        self.gust_H = 15.0          # 半波长 [m]
        self.gust_interval = 20.0   # 阵风间隔 [s]
        self.gust_prop_speed = self.u_ref  # 阵风传播速度 ≈ 平均风速

        self._gust_active = False
        self._gust_t0 = 0.0
        self._next_gust = np.random.uniform(5.0, 15.0)

        # ── 气动参数 (Iris 近似) ──────────────────────
        self.rho = 1.225            # 空气密度 [kg/m³]
        self.CdA = 0.05             # 等效 Cd*A [m²]

    def _mean_wind_enu(self, altitude):
        """
        幂律平均风 → ENU 速度 [We, Wn, Wu]

        气象风向惯例: dir = 风来向 (0°=北风来自北, 吹向南)
        ENU 下: 北风 → 向南吹 → (0, -u, 0)
                东风 → 向西吹 → (-u, 0, 0)
                东北风(45°) → 向西南吹 → (-u·sin45, -u·cos45, 0)
        """
        z = max(altitude, 0.5)
        u_mag = self.u_ref * (z / self.z_ref) ** self.alpha
        We = -u_mag * np.sin(self.wind_dir)   # 东分量
        Wn = -u_mag * np.cos(self.wind_dir)   # 北分量
        return np.array([We, Wn, 0.0])

    def _turb_enu(self):
        """
        Dryden 湍流 → ENU 速度

        湍流分量定义在"风轴系":
          纵向 u_t = 沿平均风向 (从风来向到去向)
          横向 v_t = 垂直平均风向 (水平面内)
          垂向 w_t = 垂直

        转换到 ENU:
          Wn_t = -u_t·cos(dir) - v_t·sin(dir)
          We_t = -u_t·sin(dir) + v_t·cos(dir)
          Wu_t =  w_t
        """
        turb_body = self.turb.step()  # [u_t, v_t, w_t] in wind-axis
        cd = np.cos(self.wind_dir)
        sd = np.sin(self.wind_dir)
        We_t = -turb_body[0] * sd + turb_body[1] * cd
        Wn_t = -turb_body[0] * cd - turb_body[1] * sd
        Wu_t = turb_body[2]
        return np.array([We_t, Wn_t, Wu_t])

    def _gust_enu(self, sim_time):
        """1-cos 垂向阵风 → ENU 垂直分量"""
        Wu_g = 0.0
        # 触发
        if not self._gust_active and sim_time >= self._next_gust:
            self._gust_active = True
            self._gust_t0 = sim_time
            rospy.loginfo("💨 阵风触发! t=%.1fs", sim_time)

        if self._gust_active:
            dt_g = sim_time - self._gust_t0
            duration = 2.0 * self.gust_H / self.gust_prop_speed
            if dt_g <= duration:
                # x: 相对阵风中心的位移 [-H, +H]
                x = self.gust_prop_speed * (dt_g - duration / 2.0)
                Wu_g = self.gust_w_max * 0.5 * (1.0 - np.cos(np.pi * x / self.gust_H))
            else:
                self._gust_active = False
                self._next_gust = sim_time + np.random.uniform(
                    self.gust_interval * 0.7, self.gust_interval * 1.3)
                rospy.loginfo("💨 阵风结束 t=%.1fs, 下次约 t=%.1fs",
                              sim_time, self._next_gust)
        return Wu_g

    def step(self, altitude, sim_time):
        """
        生成一步风场

        altitude: 离地高度 [m] (正=上)
        sim_time: 仿真时间 [s]
        返回:  ENU 风速 [We, Wn, Wu] (m/s)
        """
        W_mean = self._mean_wind_enu(altitude)
        W_turb = self._turb_enu()
        Wu_gust = self._gust_enu(sim_time)

        W_enu = W_mean + W_turb
        W_enu[2] += Wu_gust              # 阵风只加在垂直分量
        return W_enu

    def drag_force(self, wind_enu, drone_vel_enu):
        """
        计算气动阻力 (ENU)

        F = 0.5·ρ·CdA·|Vrel|·Vrel
        其中 Vrel = V_drone - V_wind (无人机相对空气的速度)
        阻力方向与 Vrel 相同 (风对无人机的拖拽力)
        """
        Vrel = np.asarray(drone_vel_enu) - np.asarray(wind_enu)
        speed = np.linalg.norm(Vrel)
        if speed < 1e-6:
            return np.zeros(3)
        return 0.5 * self.rho * self.CdA * speed * Vrel


# ══════════════════════════════════════════════════════════
# 3. ROS 风场节点
# ══════════════════════════════════════════════════════════
class WindFieldNode:
    """ROS 节点: 订阅 odometry, 计算风场, 通过 Gazebo 施加气动阻力"""

    def __init__(self, rate_hz=20, debug=False, no_wrench=False):
        rospy.init_node('wind_field_node', log_level=rospy.INFO)

        self.dt = 1.0 / rate_hz
        self.debug = debug
        self.wind = WindField(dt=self.dt)

        # 无人机状态
        self._pos = np.zeros(3)       # ENU 位置 [m]
        self._vel = np.zeros(3)       # ENU 速度 [m/s]
        self._euler = np.zeros(3)     # [roll, pitch, yaw] [rad]
        self._got_odom = False
        self._t0 = None

        # Odometry
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._odom_cb)

        # Gazebo 服务
        self._use_wrench = not no_wrench
        if self._use_wrench:
            svc = '/gazebo/apply_body_wrench'
            if svc in [s[0] for s in rospy.get_published_topics()]:
                pass  # will wait below
            try:
                rospy.wait_for_service(svc, timeout=10.0)
                from gazebo_msgs.srv import ApplyBodyWrench
                self._wrench_srv = rospy.ServiceProxy(svc, ApplyBodyWrench,
                                                      persistent=True)
                rospy.loginfo("✅ Gazebo apply_body_wrench 就绪")
            except (rospy.ROSException, rospy.ROSInterruptException):
                rospy.logwarn("⚠️ 未找到 %s, 仅记录风场数据", svc)
                self._use_wrench = False

        # 风传感器数据发布 (模拟机载风速计)
        self._pub_wind = rospy.Publisher(
            '/wind_field/velocity', Vector3Stamped, queue_size=5)

        self.rate = rospy.Rate(rate_hz)
        self._step_count = 0
        self._print_div = max(1, rate_hz)       # 每秒打印一次

    def _odom_cb(self, msg):
        self._pos[0] = msg.pose.pose.position.x
        self._pos[1] = msg.pose.pose.position.y
        self._pos[2] = msg.pose.pose.position.z   # 正=上 (ENU)
        self._vel[0] = msg.twist.twist.linear.x
        self._vel[1] = msg.twist.twist.linear.y
        self._vel[2] = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        self._euler = np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))
        self._got_odom = True

    def _apply_wrench(self, force_enu):
        """通过 Gazebo 向 iris 施加世界坐标系下的力"""
        if not self._use_wrench:
            return
        try:
            w = Wrench()
            w.force = Vector3(*force_enu)
            w.torque = Vector3(0.0, 0.0, 0.0)
            self._wrench_srv(
                body_name='iris::base_link',
                reference_frame='',                     # world frame
                reference_point=Point(0.0, 0.0, 0.0),
                wrench=w,
                start_time=rospy.Time(0),
                duration=rospy.Duration(self.dt * 1.5),
            )
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, "apply_body_wrench 失败: %s", e)

    def _print_banner(self):
        w = self.wind
        t = w.turb
        rospy.loginfo("\n" + "=" * 62)
        rospy.loginfo("  🌪️  极端湍流风场")
        rospy.loginfo("  平均风: %.1f m/s @%dm  幂律 α=%.2f  方向 %d°(来向)",
                      w.u_ref, int(w.z_ref), w.alpha,
                      int(np.degrees(w.wind_dir)))
        rospy.loginfo("  湍流:  σ_u,v=%.1f  σ_w=%.1f m/s  "
                      "Lu,v=%d  Lw=%dm",
                      t.sigma_u, t.sigma_w, int(t.Lu), int(t.Lw))
        rospy.loginfo("  阵风:  ±%.0f m/s 垂向  半波长 %dm  间隔 ~%ds",
                      w.gust_w_max, w.gust_H, w.gust_interval)
        rospy.loginfo("  气动:  ρ=%.3f  CdA=%.3f m²  施加: %s",
                      w.rho, w.CdA,
                      "Gazebo Wrench" if self._use_wrench else "仅记录")
        rospy.loginfo("=" * 62 + "\n")

    def run(self):
        self._print_banner()

        while not rospy.is_shutdown():
            t_now = rospy.Time.now().to_sec()
            if self._t0 is None:
                self._t0 = t_now
            sim_t = t_now - self._t0

            # 计算风场
            wind_enu = self.wind.step(self._pos[2], sim_t)

            # 发布风传感器数据 (供 MPC 订阅)
            ws_msg = Vector3Stamped()
            ws_msg.header.stamp = rospy.Time.now()
            ws_msg.header.frame_id = 'map'
            ws_msg.vector.x = wind_enu[0]  # ENU: East
            ws_msg.vector.y = wind_enu[1]  # ENU: North
            ws_msg.vector.z = wind_enu[2]  # ENU: Up
            self._pub_wind.publish(ws_msg)

            # 计算并施加气动阻力
            force = self.wind.drag_force(wind_enu, self._vel)
            self._apply_wrench(force)

            # 日志
            if self.debug or self._step_count % self._print_div == 0:
                ws = np.linalg.norm(wind_enu)
                fs = np.linalg.norm(force)
                gust_flag = "💨" if self.wind._gust_active else "  "
                rospy.loginfo(
                    "[t=%5.1f] %s wind=(%+5.1f,%+5.1f,%+5.1f) |w|=%.1f  "
                    "drag=%.3fN  pos_z=%.1fm",
                    sim_t, gust_flag,
                    wind_enu[0], wind_enu[1], wind_enu[2],
                    ws, fs, self._pos[2])

            self._step_count += 1
            self.rate.sleep()


# ══════════════════════════════════════════════════════════
# 4. 独立测试 (无需 ROS)
# ══════════════════════════════════════════════════════════
def run_standalone_test():
    """离线测试: 验证 Dryden 滤波器方差 & 风场输出"""
    import time

    dt = 0.05
    duration = 300.0          # 仿真 5 分钟 (足够慢速滤波器收敛)
    warmup = 60.0             # 丢弃前 60s (滤波器暂态)
    n_steps = int(duration / dt)
    n_warmup = int(warmup / dt)
    n_steady = n_steps - n_warmup

    print("=" * 60)
    print("  Dryden 湍流滤波器离线测试")
    print("  dt=%.2fs  duration=%ds  warmup=%ds  steady=%ds"
          % (dt, duration, warmup, duration - warmup))
    print("=" * 60)

    # ── 测试 1: 湍流滤波器方差 ─────────────────────
    print("\n[1/3] 测试湍流滤波器...")
    turb = DrydenTurbulence(dt=dt, U0=12.0, Lu=150, Lv=150, Lw=50,
                            sigma_u=4.0, sigma_v=4.0, sigma_w=2.5)
    samples = np.zeros((n_steps, 3))
    t0 = time.time()
    for i in range(n_steps):
        w = turb.step()
        samples[i] = w
    elapsed = time.time() - t0

    # 只用稳态段计算统计
    ss = samples[n_warmup:]
    print(f"  计算耗时: {elapsed:.2f}s ({n_steps/elapsed:.0f} steps/s)")
    # 理论值 (连续时间): σ_th = σ_u/√π ≈ 0.564*σ_target
    print(f"  理论 std (连续):  u=2.26  v=2.26  w=1.41  (= σ_target/√π)")
    for idx, ch in enumerate(['u', 'v', 'w']):
        target_cont = {'u': 2.26, 'v': 2.26, 'w': 1.41}[ch]
        mean = np.mean(ss[:, idx])
        std = np.std(ss[:, idx])
        err = abs(std - target_cont) / target_cont
        print(f"  {ch}: mean={mean:+.3f}  std={std:.3f}  (target_cont={target_cont})  "
              f"{'✅' if err < 0.20 else '⚠️ 偏差%.0f%%' % (err*100)}")

    # ── 测试 2: 完整风场 ─────────────────────────
    print("\n[2/3] 测试完整风场 (平均风 + 湍流 + 阵风)...")
    wf = WindField(dt=dt)
    wind_samples = np.zeros((n_steps, 3))
    gust_count = 0
    t0 = time.time()
    for i in range(n_steps):
        t = i * dt
        alt = 2.5  # 固定高度 2.5m
        w = wf.step(alt, t)
        wind_samples[i] = w
        if wf._gust_active:
            gust_count += 1
    elapsed = time.time() - t0

    ws_mag = np.linalg.norm(wind_samples, axis=1)
    print(f"  计算耗时: {elapsed:.2f}s ({n_steps/elapsed:.0f} steps/s)")
    print(f"  风场 |W|: mean={np.mean(ws_mag):.2f}  "
          f"std={np.std(ws_mag):.2f}  "
          f"min={np.min(ws_mag):.2f}  max={np.max(ws_mag):.2f} m/s")
    print(f"  各轴: We mean/std={np.mean(wind_samples[:,0]):.2f}/{np.std(wind_samples[:,0]):.2f}")
    print(f"        Wn mean/std={np.mean(wind_samples[:,1]):.2f}/{np.std(wind_samples[:,1]):.2f}")
    print(f"        Wu mean/std={np.mean(wind_samples[:,2]):.2f}/{np.std(wind_samples[:,2]):.2f}")
    print(f"  阵风激活步数: {gust_count}/{n_steps}")
    print(f"  阵风触发次数: (应为 {duration/wf.gust_interval:.0f} 次左右)")

    # ── 测试 3: 阻力计算 ────────────────────────
    print("\n[3/3] 测试气动阻力计算...")
    wind_test = np.array([-8.5, -8.5, -2.0])  # 典型极端值 ENU
    drone_vel = np.array([0.0, 0.0, 0.0])     # 悬停
    F = wf.drag_force(wind_test, drone_vel)
    print(f"  wind={wind_test}  vel={drone_vel}  →  F={np.round(F,3)} N  "
          f"|F|={np.linalg.norm(F):.3f} N")

    drone_vel2 = np.array([2.0, 1.0, 0.0])    # 前飞
    F2 = wf.drag_force(wind_test, drone_vel2)
    print(f"  wind={wind_test}  vel={drone_vel2}  →  F={np.round(F2,3)} N  "
          f"|F|={np.linalg.norm(F2):.3f} N")

    # Iris 约 1.5kg, 悬停推力 ~14.7N
    print(f"\n  Iris 悬停推力约 14.7N (1.5kg)")
    print(f"  极端风力 ≈ {np.linalg.norm(F):.2f}N → 推力扰动约 "
          f"{np.linalg.norm(F)/14.7*100:.0f}%")

    print("\n✅ 离线测试完成")


# ══════════════════════════════════════════════════════════
# main
# ══════════════════════════════════════════════════════════
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='极端湍流风场生成器')
    parser.add_argument('--rate', type=int, default=20, help='运行频率 Hz (默认 20)')
    parser.add_argument('--debug', action='store_true', help='每步打印日志')
    parser.add_argument('--no-wrench', action='store_true', help='不施加力,仅记录')
    parser.add_argument('--seed', type=int, default=None, help='随机种子 (可复现风场)')
    parser.add_argument('--test', action='store_true', help='离线测试 (无需 ROS/Gazebo)')
    args = parser.parse_args()
    # 固定随机种子
    if args.seed is not None:
        np.random.seed(args.seed)
        print(f"[wind_field] 随机种子: {args.seed}")

    if args.test:
        run_standalone_test()
    else:
        try:
            WindFieldNode(rate_hz=args.rate,
                          debug=args.debug,
                          no_wrench=args.no_wrench).run()
        except rospy.ROSInterruptException:
            pass
        except KeyboardInterrupt:
            rospy.loginfo("风场节点已停止")
