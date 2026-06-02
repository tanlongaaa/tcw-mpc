#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_baseline.py — PX4 原生 PID 位置控制对比基准 v2
===================================================
与 TCW-MPC 公平对比:
  - 相同风场 (wind_field.py)
  - 相同解锁序列 (参考 MPC)
  - 相同启动时序
  - 50Hz 独立发布线程 + ManualControl 防 failsafe
"""

import csv, os, rospy, time as _time, threading
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from tf.transformations import euler_from_quaternion
from power_model import UnifiedPowerEstimator
from quadrotor_dynamics import bem_power, IRIS


class PIDBaseline:
    """PX4 原生 PID — 位置 setpoint 对比 (v2: 修复起飞)"""

    CSV_FIELDS = [
        't', 'px', 'py', 'pz', 'vx', 'vy', 'vz',
        'ref_px', 'ref_py', 'ref_pz',
        'roll_deg', 'pitch_deg', 'yaw_deg',
        'wind_x', 'wind_y', 'wind_z',
        'wspeed',
        'power_est',
        'mode', 'armed',
    ]

    def __init__(self):
        rospy.init_node('pid_baseline')

        self.dt = 0.03; self.g = 9.81; self.hover_thrust = 0.66
        self.drone_mass = 1.5; self.rho = 1.225
        self._disk_area = 0.25
        self.target_pos = np.array([0.0, 0.0, 2.5])

        self.state = State()
        self.x_current = None
        self.roll_cur = 0.0; self.pitch_cur = 0.0; self.yaw_cur = 0.0
        self._got_odom = False
        self.wind_meas = np.zeros(3); self._got_wind = False
        self._power_est = 0.0

        self._csv_file = None; self._csv_writer = None
        self._csv_path = ''; self._csv_count = 0

        # ── 统一功耗模型 (ESC 电机转速) ────────────
        self.power_mgr = UnifiedPowerEstimator()

        # ── ROS ──────────────────────────────────────
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._odom_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self.power_mgr.imu_cb)
        rospy.Subscriber('/wind_field/velocity', Vector3Stamped, self._wind_cb)
        self.pub_sp = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pub_man = rospy.Publisher(
            '/mavros/manual_control/send', ManualControl, queue_size=10)

        rospy.loginfo("等待 MAVROS 服务...")
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self._arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self._mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.loginfo("PID v2 就绪")
        self.rate = rospy.Rate(int(1.0 / self.dt))

    # ── CSV ────────────────────────────────────────────────
    def _csv_open(self):
        import datetime
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'pid_log_%s.csv' % ts)
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self.CSV_FIELDS)
        rospy.loginfo("PID CSV: %s", self._csv_path)

    def _csv_write(self, row):
        if self._csv_writer:
            self._csv_writer.writerow(row)
            self._csv_count += 1
            if self._csv_count % 10 == 0:
                self._csv_file.flush()

    def _csv_close(self):
        if self._csv_file:
            self._csv_file.close()
            rospy.loginfo("PID CSV 已保存 %d 行 → %s",
                          self._csv_count, self._csv_path)

    # ── 回调 ───────────────────────────────────────────────
    def _state_cb(self, msg):
        self.state = msg
    def _odom_cb(self, msg):
        self.x_current = np.array([msg.pose.pose.position.x,
            msg.pose.pose.position.y, msg.pose.pose.position.z,
            msg.twist.twist.linear.x, msg.twist.twist.linear.y,
            msg.twist.twist.linear.z])
        self._got_odom = True
        q = msg.pose.pose.orientation
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.roll_cur = r; self.pitch_cur = p; self.yaw_cur = y
    def _wind_cb(self, msg):
        self.wind_meas = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        self._got_wind = True

    # ── BEMT 统一功耗 (与 MPC 相同模型) ──────────────
    def _compute_bem_power(self):
        """从 IMU 加速度估算推力 → BEMT 功率"""
        a_body = np.array([
            float(self.power_mgr._accel[0]),
            float(self.power_mgr._accel[1]),
            float(self.power_mgr._accel[2])
        ]) if self.power_mgr._got_data else np.array([0,0,0])
        a_norm = np.linalg.norm(a_body)
        if a_norm < 1.0:
            return 0.0
        thrust_norm = a_norm * self.hover_thrust / self.g
        v_rel = np.linalg.norm(
            self.x_current[3:6] - self.wind_meas) if self.x_current is not None else 0.0
        return bem_power(thrust_norm, self.roll_cur, self.pitch_cur, v_rel)

    # ── 后台发布线程 (★ 修复: 50Hz 独立线程, 同 MPC 设计) ──
    def _start_publish_thread(self):
        self._publishing = True
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

    def _publish_loop(self):
        r = rospy.Rate(50)
        while self._publishing and not rospy.is_shutdown():
            self._send_setpoint()
            self._publish_manual()
            r.sleep()

    def _send_setpoint(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.target_pos[0]
        msg.pose.position.y = self.target_pos[1]
        msg.pose.position.z = self.target_pos[2]
        msg.pose.orientation.w = 1.0
        self.pub_sp.publish(msg)

    def _publish_manual(self):
        man = ManualControl()
        man.x = 0; man.y = 0; man.z = 500; man.r = 0; man.buttons = 0
        self.pub_man.publish(man)

    # ── 主循环 ─────────────────────────────────────────────
    def run(self):
        rospy.loginfo("\n" + "=" * 62)
        rospy.loginfo("  📐 PX4 PID 对比 v2 (位置 setpoint)")
        rospy.loginfo("  目标: (%.1f, %.1f, %.1f)", *self.target_pos)
        rospy.loginfo("=" * 62 + "\n")

        # ★ 先启动 50Hz 发布线程 (确保 setpoint 流在 OFFBOARD 前就持续)
        self._start_publish_thread()
        self._csv_open()

        rospy.loginfo("等待 odometry...")
        while not self._got_odom and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo("Odometry 就绪")

        # ── 解锁序列 (★ 完全对齐 MPC) ─────────────────
        rospy.loginfo("执行解锁序列...")
        # 1. 先发虚拟摇杆 (防 failsafe)
        for _ in range(10):
            self._publish_manual()
            self.rate.sleep()
        # 2. 安全断电
        try:
            self._arm_srv(False)
            rospy.sleep(0.5)
        except rospy.ServiceException:
            rospy.logwarn("disarm 失败, 继续...")
        # 3. OFFBOARD
        try:
            self._mode_srv(custom_mode='OFFBOARD')
            rospy.loginfo("模式: OFFBOARD")
        except rospy.ServiceException:
            rospy.logwarn("OFFBOARD 失败, 尝试 LOITER")
            try:
                self._mode_srv(custom_mode='AUTO.LOITER')
            except rospy.ServiceException:
                rospy.logerr("无法切换模式!")
        rospy.sleep(0.5)
        # 4. 解锁
        try:
            result = self._arm_srv(True)
            if result.success:
                rospy.loginfo("✅ 已解锁 (ARMED)")
            else:
                rospy.logerr("❌ 解锁失败!")
        except rospy.ServiceException as e:
            rospy.logerr("解锁失败: %s", e)
        rospy.sleep(1.0)
        rospy.loginfo("开始 PID 控制记录")
        # ───────────────────────────────────────────────

        t_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t_now = rospy.Time.now().to_sec()
            t_elapsed = t_now - t_start

            if self.x_current is None:
                self.rate.sleep()
                continue

            self._power_est = self._compute_bem_power()

            self._csv_write([
                t_elapsed,
                *self.x_current[:3], *self.x_current[3:6],
                *self.target_pos,
                np.degrees(self.roll_cur), np.degrees(self.pitch_cur),
                np.degrees(self.yaw_cur),
                self.wind_meas[0], self.wind_meas[1], self.wind_meas[2],
                np.linalg.norm(self.wind_meas),
                self._power_est,
                self.state.mode.strip(), int(self.state.armed),
            ])

            if self._csv_count % 100 == 0:
                ws = np.linalg.norm(self.wind_meas)
                pos_err = np.linalg.norm(self.x_current[:3] - self.target_pos)
                rospy.loginfo(
                    "[t=%.1f] |w|=%.1f  z=%.2f  err=%.2f  "
                    "P=%.1fW  (PID)",
                    t_elapsed, ws, self.x_current[2], pos_err,
                    self._power_est)

            self.rate.sleep()

        self._publishing = False
        self._csv_close()


if __name__ == '__main__':
    try:
        PIDBaseline().run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("PID 对比已停止")
