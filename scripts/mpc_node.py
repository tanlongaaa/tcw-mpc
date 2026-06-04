#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TCW-MPC: Trajectory-Coherent Wind-Adaptive MPC
==============================================
面向极端湍流风场的自适应模型预测控制器

核心模块:
  - mpc_solver.py:     统一求解器 (OSQP / iLQR)
  - mpc_components.py: 控制组件 (TCWP / CMA / 积分器 / 风能利用)
  - quadrotor_dynamics.py: 非线性动力学 + BEMT 功耗
  - wind_field.py:     极端湍流风场生成器

架构 (ACMPC-style CTBR Direct Drive):
  ├── mpc_node.py (本文件) — ROS 控制节点, MPC 直接输出 body rate + thrust
  ├── mpc_solver.py        — BaseMPCSolver → OSQPSolver | iLQRSolver
  ├── mpc_components.py    — TCWP / CMA / Integrator / WindAdvisor / EnergyMgr
  └── quadrotor_dynamics.py — 10D CTBR 动力学 + 线性化 + 风扰计算

控制流: MPC → [f_c, ωx, ωy, ωz] → AttitudeTarget (body rate mode) → PX4

作者: 小龙 & 虾哥
"""

import csv, os, rospy, threading, datetime
import numpy as np
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State, ManualControl
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion

# ── 自研模块 ──
from mpc_solver import OSQPSolver as StandardMPC  # 向后兼容, 可换 iLQRSolver
from mpc_components import (Integrator, LowPassFilter,
                            TCWPPredictor, CMAManager,
                            WindUtilizationAdvisor, PredictiveEnergyManager)
from power_model import UnifiedPowerEstimator
from quadrotor_dynamics import (discrete_linearize, bem_power,
                                 compute_wind_disturbance, IRIS,
                                 get_model_info)


class MPCNode:
    """TCW-MPC ROS 控制节点 (6D / 10D CTBR)"""

    CSV_FIELDS_6D = [
        't', 'px', 'py', 'pz', 'vx', 'vy', 'vz',
        'ref_px', 'ref_py', 'ref_pz',
        'u_roll', 'u_pitch', 'u_thrust',
        'u_int_r', 'u_int_p', 'u_int_t',
        'roll_deg', 'pitch_deg', 'yaw_deg',
        'wind_x', 'wind_y', 'wind_z',
        'wind_px', 'wind_py', 'wind_pz',
        'wspeed', 'q_dyn',
        'u_max_roll', 'u_max_pitch', 'u_max_thrust',
        'power_est',
        'coast_factor', 'cma_mode', 'energy_mode', 'tau_mpc',
        'mode', 'armed', 'mpc_ok', 'iter'
    ]

    CSV_FIELDS_10D = [
        't', 'px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz',
        'ref_px', 'ref_py', 'ref_pz',
        'u_fc', 'u_wx', 'u_wy', 'u_wz',
        'u_int_fc', 'u_int_wx', 'u_int_wy', 'u_int_wz',
        'roll_deg', 'pitch_deg', 'yaw_deg',
        'wind_x', 'wind_y', 'wind_z',
        'wind_px', 'wind_py', 'wind_pz',
        'wspeed', 'q_dyn',
        'u_max_fc', 'u_max_wx', 'u_max_wy', 'u_max_wz',
        'power_est',
        'coast_factor', 'cma_mode', 'energy_mode', 'tau_mpc',
        'mode', 'armed', 'mpc_ok', 'iter'
    ]

    def _csv_fields(self):
        return self.CSV_FIELDS_6D if self.model == '6d' else self.CSV_FIELDS_10D

    def __init__(self):
        rospy.init_node('mpc_position_controller')

        # ── 模型选择 ─────────────────────────────────
        self.model = rospy.get_param('~model', '10d')  # '6d' (Euler) | '10d' (CTBR body rate, 仿 ACMPC)
        nx = 6 if self.model == '6d' else 10
        nu = 3 if self.model == '6d' else 4

        # ── 物理参数 ─────────────────────────────────
        self.dt = 0.03
        self.N_pred = 15  # 0.5s 预测时域 (iLQR 加速)
        self.g = 9.81
        self.hover_thrust = 0.66
        self.drone_mass = 1.5
        dt_mpc = self.dt

        # ── 节能参数 ─────────────────────────────────
        self.lambda_energy = rospy.get_param('~lambda_energy', 5.0)
        self._is_trajectory_task = rospy.get_param('~trajectory_mode', False)
        self._energy_first = rospy.get_param('~energy_first', False)
        self._q_xy_scale = rospy.get_param('~q_xy_scale', 1.0)
        if self.model == '6d':
            self._u_hover = np.array([0.0, 0.0, self.hover_thrust])
        else:
            self._u_hover = np.array([self.drone_mass * self.g, 0.0, 0.0, 0.0])

        # ── 空气动力学参数 ────────────────────────────
        self.rho = 1.225

        # ── SL-MPC: 动态 B 矩阵修正 (仅 6D OSQP) ──────
        self._last_lin_tilt = 0.0
        if self.model == '6d':
            x_op = np.zeros(6)
            u_op = np.array([0.0, 0.0, self.hover_thrust])
            w_zero = np.zeros(3)
            from quadrotor_dynamics import discrete_linearize
            A0, B0, g0 = discrete_linearize(x_op, u_op, w_zero, dt_mpc)
            self._A_base = A0.copy()
            self._B_base = B0.copy()
            self.g_vec = g0.copy()
        else:
            from quadrotor_dynamics import discrete_linearize_10d, NX_10D, NU_10D
            x_op = np.zeros(NX_10D)
            x_op[3] = 1.0  # qw = 1 (identity quaternion)
            u_op = np.array([self.drone_mass * self.g, 0.0, 0.0, 0.0])
            w_zero = np.zeros(3)
            A0, B0, g0 = discrete_linearize_10d(x_op, u_op, w_zero, dt_mpc)
            self._A_base = A0.copy()
            self._B_base = B0.copy()
            self.g_vec = g0.copy()
        self.k_damp = IRIS['k_drag_base'] * IRIS['CdA_front'] * 5.0

        # ── MPC 权重 ──────────────────────────────────
        if self.model == '6d':
            Q = np.diag([4.0, 4.0, 6.0, 2.0, 2.0, 6.0])
            R_energy = np.diag([14.0, 14.0, 15.0 + self.lambda_energy])
            self._u_min_nom = np.array([-0.45, -0.45, 0.30])
            self._u_max_nom = np.array([0.45, 0.45, 0.95])
        else:
            # 10D: pos(3) + quat(4) + vel(3) — CTBR 直驱权重
            Q = np.diag([15.0, 15.0, 20.0,  # pos (强跟踪)
                         0.0, 0.0, 0.0, 0.0,  # quat (不惩罚, 防 yaw 跳变)
                         8.0, 8.0, 15.0])  # vel (阻尼防超调)
            R_energy = np.diag([3.0, 50.0, 50.0, 30.0])  # fc/ω (重罚体轴角速度)
            # 10D: fc [2, 20N], 极小体轴角速度
            self._u_min_nom = np.array([2.0, -0.1, -0.1, -0.05])
            self._u_max_nom = np.array([20.0, 0.1, 0.1, 0.05])

        if self._energy_first:
            Q[0, 0] *= self._q_xy_scale; Q[1, 1] *= self._q_xy_scale
            if self.model == '6d':
                Q[3, 3] *= self._q_xy_scale; Q[4, 4] *= self._q_xy_scale
            else:
                Q[7, 7] *= self._q_xy_scale; Q[8, 8] *= self._q_xy_scale
            rospy.loginfo("⚡ Energy-First: Q_xy × %.2f", self._q_xy_scale)

        # ── ★ 求解器: iLQR 加速版 (1 次迭代, 仿 ACMPC 训练模式) ──
        from mpc_solver import iLQRSolver
        from quadrotor_dynamics import (quadrotor_dynamics_10d_discrete,
                                         discrete_linearize_10d, NX_10D, NU_10D)
        self.mpc = iLQRSolver(
            nx=NX_10D, nu=NU_10D, N=self.N_pred, dt=self.dt,
            Q=Q, R=R_energy,
            u_min=self._u_min_nom, u_max=self._u_max_nom,
            dynamics_fn=lambda x, u, w: quadrotor_dynamics_10d_discrete(x, u, w, self.dt),
            linearize_fn=lambda x, u, w, d: discrete_linearize_10d(x, u, w, d),
            lqr_iter=2, linesearch_decay=0.5, max_linesearch_iter=2, eps=5e-3)

        # ── 控制组件 ──────────────────────────────────
        self.tcwp = TCWPPredictor(
            N_pred=self.N_pred, M_history=25, L_coherence=15.0, dt=self.dt)
        self.cma = CMAManager(
            self._u_max_nom, self._u_min_nom,
            gamma_t=0.002, gamma_a=0.0001, rho=self.rho,
            model=self.model)
        self.wind_advisor = WindUtilizationAdvisor(
            alignment_threshold=0.3, coast_factor_max=0.5)
        self.energy_mgr = PredictiveEnergyManager(
            N_pred=self.N_pred, z_margin=0.5, ramp_steps=5, dz_threshold=2.0)
        # 6D: Ki=[d_roll, d_pitch, d_thrust]; 10D: Ki=[d_fc, d_wx, d_wy]
        if self.model == '6d':
            self.integrator = Integrator(
                Ki=[0.02, 0.02, 0.01], max_int=[0.05, 0.05, 0.04], dt=self.dt,
                model='6d')
        else:
            self.integrator = Integrator(
                Ki=[0.01, 0.01, 0.005], max_int=[0.5, 0.2, 0.2, 0.1], dt=self.dt,
                model='10d')
        self.lpf = LowPassFilter(alpha=0.4)

        # ── 状态变量 ─────────────────────────────────
        self._tau_mpc = 3.0  # 慢参考轨迹, 防 CTBR 超调
        self._ref_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos = np.array([0.0, 0.0, 2.5])
        self.state = State()
        self.x_current = None  # 6D or 10D
        self.roll_cur = self.pitch_cur = self.yaw_cur = 0.0
        self._q_current = np.array([1.0, 0.0, 0.0, 0.0])  # 10D用
        self._got_odom = False
        self.wind_meas = np.zeros(3)
        self._got_wind = False
        self._wind_seq = None; self._d_seq = None; self._q_dyn = 0.0
        self._vel_traj_prev = [np.zeros(3)] * self.N_pred
        self._pos_traj_prev = [np.zeros(3)] * self.N_pred

        # ── 控制指令 ──────────────────────────────────
        self.nu = 3 if self.model == '6d' else 4
        self._cmd_lock = threading.Lock()
        self._publishing = False
        # CTBR 直驱控制 (仿 ACMPC): 体轴角速度 + 总推力
        self._fc = self._u_hover[0] if self.model == '10d' else 0.0
        self._wx = 0.0; self._wy = 0.0; self._wz = 0.0
        self._fc_max = IRIS['max_thrust']  # 22.3N, PX4 thrust 归一化 [0,1]
        # 6D Euler 姿态直驱
        self._roll_sp = 0.0; self._pitch_sp = 0.0; self._thrust_sp = self.hover_thrust
        self._u_last_safe = self._u_hover.copy()
        self._u_max_eff = self._u_max_nom.copy()
        self._u_min_eff = self._u_min_nom.copy()
        self._coast_factor = 0.0
        self._cma_mode_str = 'normal'
        self._energy_mode = 'normal'
        self._Q_scale = 1.0
        self._thrust_energy_bias = 0.0

        # ── 功耗 ─────────────────────────────────────
        self.power_model = UnifiedPowerEstimator()
        self._power_est = 0.0

        # ── CSV ──────────────────────────────────────
        self._csv_file = None; self._csv_writer = None
        self._csv_count = 0
        self._mpc_ok = 0; self._mpc_iter = 0

        # ── ROS 接口 ─────────────────────────────────
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._odom_cb)
        rospy.Subscriber('/wind_field/velocity', Vector3Stamped, self._wind_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self.power_model.imu_cb)
        # 仿 ACMPC: setpoint_raw/attitude (body rate + thrust CTBR)
        self.pub_att = rospy.Publisher(
            '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.pub_man = rospy.Publisher(
            '/mavros/manual_control/send', ManualControl, queue_size=10)

        rospy.loginfo("等待 MAVROS 服务...")
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self._arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self._mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.loginfo("MAVROS 就绪, TCW-MPC 初始化完成")
        self.rate = rospy.Rate(int(1.0 / self.dt))

    # ══════════════════════════════════════════════════════
    # CSV 日志
    # ══════════════════════════════════════════════════════
    def _csv_open(self):
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'mpc_log_%s.csv' % ts)
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self._csv_fields())
        rospy.loginfo("CSV: %s", self._csv_path)

    def _csv_write(self, row):
        if self._csv_writer:
            self._csv_writer.writerow(row)
            self._csv_count += 1
            if self._csv_count % 10 == 0:
                self._csv_file.flush()

    def _csv_close(self):
        if self._csv_file:
            self._csv_file.close()
            rospy.loginfo("CSV 已保存 %d 行 → %s", self._csv_count, self._csv_path)

    # ══════════════════════════════════════════════════════
    # ROS 回调
    # ══════════════════════════════════════════════════════
    def _state_cb(self, msg):
        self.state = msg

    def _odom_cb(self, msg):
        if self.model == '6d':
            self.x_current = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ])
        else:
            q = msg.pose.pose.orientation
            q_vec = np.array([q.w, q.x, q.y, q.z])
            self._q_current = q_vec
            self.x_current = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                q.w, q.x, q.y, q.z,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ])
        self._got_odom = True
        q = msg.pose.pose.orientation
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.roll_cur = r; self.pitch_cur = p; self.yaw_cur = y

    def _wind_cb(self, msg):
        self.wind_meas = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        self._got_wind = True

    # ══════════════════════════════════════════════════════
    # 风扰计算 — 利用 TCWP 时变预测
    # ══════════════════════════════════════════════════════
    def _compute_disturbance_seq(self, wind_seq):
        nx = 6 if self.model == '6d' else 10
        if not wind_seq:
            return [np.zeros(nx)] * self.N_pred

        d_seq = []
        for k in range(self.N_pred):
            w_k = wind_seq[k] if k < len(wind_seq) else wind_seq[-1]
            if self._pos_traj_prev and k < len(self._pos_traj_prev):
                if self.model == '6d':
                    x_k = np.zeros(6)
                    x_k[:3] = self._pos_traj_prev[k]
                    x_k[3:6] = (self._vel_traj_prev[k] if k < len(self._vel_traj_prev)
                                else np.zeros(3))
                    u_cur = np.array([self.roll_cur, self.pitch_cur,
                                      self._u_last_safe[2]])
                    d = compute_wind_disturbance(x_k, u_cur, w_k)
                    d[3:6] *= self.dt
                else:
                    import quadrotor_dynamics as qd
                    x_k = np.zeros(10)
                    x_k[0:3] = self._pos_traj_prev[k]
                    x_k[3:7] = self._q_current
                    x_k[7:10] = (self._vel_traj_prev[k] if k < len(self._vel_traj_prev)
                                 else np.zeros(3))
                    d = qd.compute_wind_disturbance_10d(x_k, self._u_last_safe, w_k)
                    d[7:10] *= self.dt
            else:
                if self.model == '6d':
                    x_k = (self.x_current if self.x_current is not None else np.zeros(6))
                    u_cur = np.array([self.roll_cur, self.pitch_cur,
                                      self._u_last_safe[2]])
                    d = compute_wind_disturbance(x_k, u_cur, w_k)
                    d[3:6] *= self.dt
                else:
                    import quadrotor_dynamics as qd
                    x_k = (self.x_current if self.x_current is not None else np.zeros(10))
                    d = qd.compute_wind_disturbance_10d(x_k, self._u_last_safe, w_k)
                    d[7:10] *= self.dt
            d_seq.append(d.copy())
        return d_seq

    # ══════════════════════════════════════════════════════
    # 后台发布
    # ══════════════════════════════════════════════════════
    def _start_publish_thread(self):
        self._publishing = True
        t = threading.Thread(target=self._publish_loop, daemon=True)
        t.start()

    def _publish_loop(self):
        """50Hz: AttitudeTarget — 6D=Euler姿态直驱 / 10D=体轴角速度"""
        r = rospy.Rate(50)
        while self._publishing and not rospy.is_shutdown():
            with self._cmd_lock:
                if self.model == '6d':
                    roll_cmd, pitch_cmd, thrust_cmd = self._roll_sp, self._pitch_sp, self._thrust_sp
                else:
                    fc, wx, wy, wz = self._fc, self._wx, self._wy, self._wz
            msg = AttitudeTarget()
            msg.header.stamp = rospy.Time.now()
            if self.model == '6d':
                # 姿态四元数直驱: Euler → quat, yaw=当前值
                from tf.transformations import quaternion_from_euler
                q_list = quaternion_from_euler(roll_cmd, pitch_cmd, self.yaw_cur)
                msg.orientation.w = q_list[3]
                msg.orientation.x = q_list[0]
                msg.orientation.y = q_list[1]
                msg.orientation.z = q_list[2]
                msg.type_mask = 0  # 使用姿态四元数
                msg.thrust = float(np.clip(thrust_cmd, 0.05, 1.0))
            else:
                msg.type_mask = 128  # IGNORE_ATTITUDE → body rate control
                msg.body_rate.x = wx
                msg.body_rate.y = -wy  # ENU→FRD pitch 反号
                msg.body_rate.z = wz
                msg.thrust = float(np.clip(fc / self._fc_max, 0.05, 1.0))
            self.pub_att.publish(msg)
            self._publish_manual()
            r.sleep()

    def _publish_manual(self):
        """ManualControl: 防止 RC loss failsafe"""
        man = ManualControl()
        man.x = 0; man.y = 0; man.z = 500; man.r = 0; man.buttons = 0
        self.pub_man.publish(man)



    # ══════════════════════════════════════════════════════
    # 主循环
    # ══════════════════════════════════════════════════════
    def run(self):
        info = get_model_info(self.model)
        rospy.loginfo("\n" + "=" * 55)
        rospy.loginfo("  ⚡ TCW-MPC 控制器 v2  |  Model: %s (%dD→%dD)",
                      self.model.upper(), info['nx'], info['nu'])
        rospy.loginfo("  TCWP: M=%d L=%.0fm  |  CMA: 3-mode margin-aware",
                      self.tcwp.M, self.tcwp.L)
        rospy.loginfo("  Energy: λ=%.1f %s  |  Solver: OSQP  |  Mode: CTBR Direct (ACMPC-style)",
                      self.lambda_energy,
                      "⚡ENERGY-FIRST" if self._energy_first else "")
        rospy.loginfo("  Aero: CdA(α) + BEMT power model")
        rospy.loginfo("=" * 55 + "\n")

        self.wind_advisor.set_task_type(self._is_trajectory_task)
        self._start_publish_thread()
        self._csv_open()

        # ── 等待 odometry ──
        while not self._got_odom and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo("Odometry 就绪")

        # ── 解锁序列 ──
        for _ in range(10):
            self._publish_manual(); self.rate.sleep()
        try:
            self._arm_srv(False); rospy.sleep(0.5)
        except rospy.ServiceException:
            pass
        try:
            self._mode_srv(custom_mode='OFFBOARD')
        except rospy.ServiceException:
            self._mode_srv(custom_mode='AUTO.LOITER')
        rospy.sleep(0.5)
        try:
            result = self._arm_srv(True)
            rospy.loginfo("✅ 已解锁" if result.success else "❌ 解锁失败")
        except rospy.ServiceException as e:
            rospy.logerr("解锁失败: %s", e)
        rospy.sleep(1.0)
        rospy.loginfo("开始控制循环")

        t_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t_now = rospy.Time.now().to_sec()
            t_elapsed = t_now - t_start

            if self.x_current is None:
                self.rate.sleep(); continue

            # ★ Step 1: TCWP 风预测
            if self._got_wind:
                self.tcwp.add_sample(self.x_current[:3].copy(),
                                     self.wind_meas.copy())
                self.tcwp.set_previous_trajectory(self._pos_traj_prev)
                self._wind_seq = self.tcwp.predict()
                self._d_seq = self._compute_disturbance_seq(self._wind_seq)
            else:
                self._d_seq = None; self._wind_seq = None

            # ★ Step 2: CMA 约束
            u_prev = self._u_last_safe
            self._u_min_eff, self._u_max_eff = self.cma.update(
                self.wind_meas, mpc_u_pred=u_prev)
            self._q_dyn = self.cma.get_q()
            self._cma_mode_str = self.cma.get_mode()
            self.mpc.u_min = self._u_min_eff
            self.mpc.u_max = self._u_max_eff

            # ★ Step 3: 参考轨迹
            alpha = 1.0 - np.exp(-self.dt / max(self._tau_mpc, 0.1))
            self._ref_pos += alpha * (self.target_pos - self._ref_pos)
            ref_pos = self._ref_pos.copy()
            if self.model == '6d':
                x_ref = np.zeros(6); x_ref[:3] = ref_pos
            else:
                x_ref = np.zeros(10); x_ref[:3] = ref_pos
                # 不强制姿态: 让 MPC 自主选择四元数 (避免 yaw 跳变)
                x_ref[3:7] = self._q_current.copy() if self._q_current is not None else np.array([1.0, 0.0, 0.0, 0.0])

            # ★ Step 3.5: 风能利用
            if self.x_current is not None:
                pos_err = ref_pos - self.x_current[:3]
                coast, wind_mode, Q_scale = self.wind_advisor.evaluate(
                    self.wind_meas, pos_err, wind_seq=self._wind_seq)
                self._coast_factor = coast; self._Q_scale = Q_scale
                if self._wind_seq is not None and self._got_wind:
                    energy_mode, thrust_bias = self.energy_mgr.plan(
                        self._wind_seq)
                    self._energy_mode = energy_mode
                    self._thrust_energy_bias = thrust_bias
                else:
                    self._energy_mode = 'normal'
                    self._thrust_energy_bias = 0.0
            else:
                self._coast_factor = 0.0; self._Q_scale = 1.0
                self._energy_mode = 'normal'
                self._thrust_energy_bias = 0.0
                pos_err = np.zeros(3)

            # ★ Step 4: MPC 求解
            u_ref = self._u_hover.copy()
            thrust_idx = 2 if self.model == '6d' else 0
            u_ref[thrust_idx] += self._thrust_energy_bias
            u_mpc = self.mpc.solve(
                self.x_current, x_ref, u_ref=u_ref, d_seq=self._d_seq)
            self._mpc_ok = int(self.mpc.last_ok)
            self._mpc_iter = self.mpc.last_iter

            # ★ Step 5: 提取轨迹
            if u_mpc is not None and self.mpc.last_ok:
                u_safe = u_mpc
                pos_traj, vel_traj = self.mpc.get_trajectory()
                if pos_traj is not None:
                    self._pos_traj_prev = pos_traj
                    self._vel_traj_prev = vel_traj
            else:
                u_safe = self._u_last_safe

            # ★ MPC-driven τ (基于推力偏差)
            u_thrust_key = u_safe[thrust_idx]
            hover_key = self.hover_thrust if self.model == '6d' else (self.drone_mass * self.g)
            dT = abs(u_thrust_key - hover_key)
            scale = 0.015 + self.lambda_energy * 0.008
            self._tau_mpc = 0.3 + 1.2 * np.exp(-dT / max(scale, 1e-6))

            # ★ Step 6: 积分器
            if self.x_current is not None:
                pos_err = ref_pos - self.x_current[:3]
                int_corr = self.integrator.update(pos_err, yaw=self.yaw_cur)
            else:
                int_corr = np.zeros(self.nu)

            # ★ Step 7: 合成 + 平滑
            u_raw = u_safe.copy()
            for i in range(self.nu):
                u_raw[i] += int_corr[i]
                u_raw[i] = np.clip(u_raw[i], self._u_min_eff[i], self._u_max_eff[i])
            u_smooth = self.lpf.apply(u_raw)

            # ★ 直接输出控制 (仿 ACMPC)
            with self._cmd_lock:
                if self.model == '6d':
                    self._roll_sp = float(u_smooth[0])
                    self._pitch_sp = float(u_smooth[1])
                    self._thrust_sp = float(u_smooth[2])
                else:
                    self._fc = float(u_smooth[0])
                    self._wx = float(u_smooth[1])
                    self._wy = float(u_smooth[2])
                    self._wz = float(u_smooth[3])
            self._u_last_safe = u_smooth.copy()

            # ★ 功耗估算
            a_imu = self.power_model._accel.copy()
            a_norm = np.linalg.norm(a_imu)
            thrust_imu = (a_norm * self.hover_thrust / self.g
                          if a_norm > 1.0 else self.hover_thrust)
            vel_idx = 3 if self.model == '6d' else 7
            v_rel_norm = (np.linalg.norm(self.x_current[vel_idx:vel_idx+3] - self.wind_meas)
                          if self._got_wind else 0.0)
            if self.model == '6d':
                self._power_est = bem_power(
                    thrust_imu, self.roll_cur, self.pitch_cur, v_rel_norm)
            else:
                import quadrotor_dynamics as qd
                self._power_est = qd.bem_power_10d(
                    u_smooth[0], q=self._q_current,
                    v=self.x_current[7:10], wind_enu=self.wind_meas)

            # ★ Step 8: 日志
            wind_p0 = self._wind_seq[0] if self._wind_seq else np.zeros(3)
            if self.model == '6d':
                self._csv_write([
                    t_elapsed,
                    *self.x_current[:3], *self.x_current[3:6],
                    *ref_pos,
                    u_smooth[0], u_smooth[1], u_smooth[2],
                    int_corr[0], int_corr[1], int_corr[2],
                    np.degrees(self.roll_cur), np.degrees(self.pitch_cur),
                    np.degrees(self.yaw_cur),
                    self.wind_meas[0], self.wind_meas[1], self.wind_meas[2],
                    wind_p0[0], wind_p0[1], wind_p0[2],
                    np.linalg.norm(self.wind_meas), self._q_dyn,
                    self._u_max_eff[0], self._u_max_eff[1], self._u_max_eff[2],
                    self._power_est,
                    self._coast_factor, self._cma_mode_str, self._energy_mode,
                    self.state.mode.strip(), int(self.state.armed),
                    self._mpc_ok, self._mpc_iter,
                ])
            else:
                self._csv_write([
                    t_elapsed,
                    *self.x_current[:3], *self.x_current[3:7], *self.x_current[7:10],
                    *ref_pos,
                    u_smooth[0], u_smooth[1], u_smooth[2], u_smooth[3],
                    int_corr[0], int_corr[1], int_corr[2], int_corr[3],
                    np.degrees(self.roll_cur), np.degrees(self.pitch_cur),
                    np.degrees(self.yaw_cur),
                    self.wind_meas[0], self.wind_meas[1], self.wind_meas[2],
                    wind_p0[0], wind_p0[1], wind_p0[2],
                    np.linalg.norm(self.wind_meas), self._q_dyn,
                    self._u_max_eff[0], self._u_max_eff[1],
                    self._u_max_eff[2], self._u_max_eff[3],
                    self._power_est,
                    self._coast_factor, self._cma_mode_str, self._energy_mode,
                    self.state.mode.strip(), int(self.state.armed),
                    self._mpc_ok, self._mpc_iter,
                ])

            if self._csv_count % 100 == 0:
                ws = np.linalg.norm(self.wind_meas)
                ee = ""
                if self._coast_factor > 0.05:
                    ee += f" 🌬coast={self._coast_factor:.2f}"
                if self._thrust_energy_bias != 0:
                    ee += f" ΔT={self._thrust_energy_bias:+.2f}"
                if self.model == '6d':
                    rospy.loginfo(
                        "[t=%.1f] |w|=%.1f q=%.1f  "
                        "u=(%+.3f,%+.3f,%.3f)  z=%.2f  err=%.2f  "
                        "P=%.0fW  cma=%s%s",
                        t_elapsed, ws, self._q_dyn,
                        u_smooth[0], u_smooth[1], u_smooth[2],
                        self.x_current[2],
                        np.linalg.norm(pos_err[:2]) if self.x_current is not None else 0,
                        self._power_est, self._cma_mode_str, ee)
                else:
                    rospy.loginfo(
                        "[t=%.1f] |w|=%.1f q=%.1f  "
                        "u=(fc=%.1f,ωx=%+.2f,ωy=%+.2f,ωz=%.1f)  "
                        "z=%.2f  err=%.2f  P=%.0fW  cma=%s%s",
                        t_elapsed, ws, self._q_dyn,
                        u_smooth[0], u_smooth[1], u_smooth[2], u_smooth[3],
                        self.x_current[2],
                        np.linalg.norm(pos_err[:2]) if self.x_current is not None else 0,
                        self._power_est, self._cma_mode_str, ee)

            self.rate.sleep()

        self._publishing = False
        self._csv_close()


if __name__ == '__main__':
    try:
        MPCNode().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("TCW-MPC 已停止")
