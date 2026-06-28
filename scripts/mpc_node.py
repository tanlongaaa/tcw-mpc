#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mpc_node.py — TCW-MPC ROS 包装层 (瘦)
=====================================
控制逻辑已抽离到 mpc_controller.MPCController (解耦 ROS, 满足 DESIGN §3 R1-R4)。
本文件只负责:
  - ROS I/O: 订阅 odom/state/wind, 发布 AttitudeTarget + manual_control
  - PX4 解锁 / OFFBOARD 序列
  - CSV 日志
  - 把 MPCController.step() 的输出转成 mavros 消息

RL 接入点 (后续 M4): RL 节点订阅 /sim/rotor_power + 状态, 调用
  controller.set_cost_weights(theta) 低频热注入 (本节点暴露 ROS service/topic 即可)。

控制流: MPCController.step → [f_c, ωx, ωy, ωz] → AttitudeTarget(body rate) → PX4

作者: 小龙 & 虾哥
"""
import csv, os, datetime, threading
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State, ManualControl
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from mpc_controller import MPCController, WEIGHT_BOUNDS
from quadrotor_dynamics import get_model_info


class MPCNode:
    """TCW-MPC ROS 包装层 (10D CTBR)"""

    CSV_FIELDS = [
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
        'q_xy', 'q_z', 'lambda_energy', 'r_omega',
        'coast_factor', 'cma_mode', 'energy_mode', 'tau_mpc',
        'mode', 'armed', 'mpc_ok', 'iter'
    ]

    def __init__(self):
        rospy.init_node('mpc_position_controller')

        # ── 控制核心 (解耦) ──
        self.dt = 0.03
        self.controller = MPCController(
            model='10d', dt=self.dt, N_pred=15,
            lambda_energy=rospy.get_param('~lambda_energy', 5.0),
            q_xy_scale=rospy.get_param('~q_xy_scale', 1.0),
            energy_first=rospy.get_param('~energy_first', False),
            trajectory_mode=rospy.get_param('~trajectory_mode', False))
        self.controller.set_target([0.0, 0.0, 2.5])
        self._fc_max = self.controller._fc_max
        # ── 推力归一化标定 (HIL 关键!) ──
        # PX4 body_rate 模式下 thrust 直喂混控器(无高度环修正), 必须让
        # fc=mg 精确映射到 plant 悬停油门, 否则猛冲/猛掉。
        # plant_6dof: k_thrust=7.36, hover_throttle=0.50 → mg=14.72N @ throttle 0.50。
        # ⚠ IRIS['max_thrust']=22.3 基于 hover=0.66, 与 plant(0.50) 不符 → 不能用 fc/fc_max。
        # 线性标定: thrust_norm = hover_throttle * fc / mg → fc=mg 时恰为 hover_throttle。
        self._hover_throttle = float(rospy.get_param('~hover_throttle', 0.50))
        self._mg = float(self.controller.drone_mass * self.controller.g)

        # ── 状态 ──
        self.state = State()
        self.x_current = None
        self.wind_meas = np.zeros(3)
        self._got_odom = False
        self._got_wind = False
        self.roll_cur = self.pitch_cur = self.yaw_cur = 0.0

        # ── 发布指令缓存 (50Hz 线程读) ──
        self._cmd_lock = threading.Lock()
        self._publishing = False
        # 启动预热阶段: 发"姿态 hold + hover 推力"建立 OFFBOARD 流 (issue #6873:
        # 纯 body_rate 流在部分 PX4 版本进不去 OFFBOARD, 先用带姿态的 setpoint 握手)。
        self._warmup = True
        self._pub_cnt = 0
        self._pub_t0 = 0.0
        self._hover_thrust_norm = self._hover_throttle
        u_hover = self.controller._u_hover
        self._fc = u_hover[0]; self._wx = 0.0; self._wy = 0.0; self._wz = 0.0

        # ── CSV ──
        self._csv_file = None; self._csv_writer = None; self._csv_count = 0

        # ── ROS 接口 ──
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._odom_cb)
        rospy.Subscriber('/wind_field/velocity', Vector3Stamped, self._wind_cb)
        # RL 动作接口 (M4): 低频订阅 θ → 热注入 (无需改本文件)
        rospy.Subscriber('/mpc/set_cost_weights', Float32MultiArray, self._weights_cb)
        self.pub_att = rospy.Publisher(
            '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.pub_pos = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
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
    # ROS 回调
    # ══════════════════════════════════════════════════════
    def _state_cb(self, msg):
        self.state = msg

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.x_current = np.array([
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            q.w, q.x, q.y, q.z,
            msg.twist.twist.linear.x, msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.roll_cur, self.pitch_cur, self.yaw_cur = r, p, y
        self._got_odom = True

    def _wind_cb(self, msg):
        self.wind_meas = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        self._got_wind = True

    def _weights_cb(self, msg):
        """RL 动作接口: Float32MultiArray = [q_xy, q_z, lambda_energy, r_omega]
        (任意前缀长度; 缺省键不更新)。线程安全热注入。"""
        keys = ['q_xy', 'q_z', 'lambda_energy', 'r_omega']
        theta = {keys[i]: float(v) for i, v in enumerate(msg.data) if i < len(keys)}
        if theta:
            applied = self.controller.set_cost_weights(theta)
            rospy.loginfo_throttle(2.0, "θ 热更新: %s", applied)

    # ══════════════════════════════════════════════════════
    # CSV
    # ══════════════════════════════════════════════════════
    def _csv_open(self):
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'mpc_log_%s.csv' % ts)
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self.CSV_FIELDS)
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
    # 50Hz 发布线程 (CTBR body rate + thrust)
    # ══════════════════════════════════════════════════════
    def _start_publish_thread(self):
        self._publishing = True
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _publish_loop(self):
        r = rospy.Rate(50)
        self._pub_t0 = rospy.Time.now().to_sec()
        while self._publishing and not rospy.is_shutdown():
            msg = AttitudeTarget()
            msg.header.stamp = rospy.Time.now()
            msg.type_mask = 128  # IGNORE_ATTITUDE → body rate control (全程 CTBR)
            if self._warmup:
                # 预热: body_rate=0 + 悬停推力, 建立纯CTBR offboard流
                msg.body_rate.x = 0.0; msg.body_rate.y = 0.0; msg.body_rate.z = 0.0
                msg.thrust = self._hover_thrust_norm
            else:
                with self._cmd_lock:
                    fc, wx, wy, wz = self._fc, self._wx, self._wy, self._wz
                msg.body_rate.x = wx
                # 注: MPC 模型体轴(ENU右手系)与 PX4 FRD 在 pitch 上同号。
                # 数值验证: MPC模型里 +ωy→+pitch→+vx, PX4 FRD +pitch_rate 亦抬头→+vx。
                # 原 body_rate.y=-wy 是错的 → 造成 pitch 正反馈雪崩(2026-06-29定位)。
                msg.body_rate.y = wy
                msg.body_rate.z = wz
                # 悬停标定线性映射: fc=mg → hover_throttle (匹配 plant)
                msg.thrust = float(np.clip(self._hover_throttle * fc / self._mg, 0.05, 1.0))
            self.pub_att.publish(msg)
            # 频率自检: 统计实际发布速率 (小龙要确认指令发送+频率)
            self._pub_cnt += 1
            now = rospy.Time.now().to_sec()
            if now - self._pub_t0 >= 5.0:
                hz = self._pub_cnt / (now - self._pub_t0)
                rospy.loginfo("[pub] attitude 实发 %.1f Hz (warmup=%s)", hz, self._warmup)
                self._pub_cnt = 0; self._pub_t0 = now
            r.sleep()

    def _publish_manual(self):
        man = ManualControl()
        man.x = 0; man.y = 0; man.z = 500; man.r = 0; man.buttons = 0
        self.pub_man.publish(man)

    # ══════════════════════════════════════════════════════
    # 主循环
    # ══════════════════════════════════════════════════════
    def run(self):
        info = get_model_info('10d')
        rospy.loginfo("\n" + "=" * 55)
        rospy.loginfo("  ⚡ TCW-MPC v3 (解耦)  |  Model: 10D (%dD→%dD)",
                      info['nx'], info['nu'])
        rospy.loginfo("  控制核心: mpc_controller.MPCController (R1-R4 接口)")
        rospy.loginfo("  功耗: power_model_v3 BEMT  |  RL 接口: /mpc/set_cost_weights")
        rospy.loginfo("=" * 55 + "\n")

        self._start_publish_thread()
        self._csv_open()

        while not self._got_odom and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo("Odometry 就绪")

        # ── OFFBOARD 握手序列 (标准: 先建立 setpoint 流, 再切模式) ──
        # 发布线程已在发 body_rate attitude 流; 预流 ~2s 确保 PX4 收到健康流。
        # ⚠ 不发 manual_control: 同时发 manual 会与 offboard 竞争, 阻止切换(坊2, HANDOVER文档)。
        # RC 失联已由 COM_RCL_EXCEPT=4 豁免。
        rospy.loginfo("预流 attitude setpoint 建立 OFFBOARD 流 (~2s)...")
        rospy.sleep(2.0)
        try:
            self._arm_srv(False); rospy.sleep(0.5)
        except rospy.ServiceException:
            pass

        # 循环重试 OFFBOARD, 每次确认 state.mode + 记录 service 返回值(坊3)
        offboard_ok = False
        for attempt in range(20):
            try:
                resp = self._mode_srv(custom_mode='OFFBOARD')
                if attempt < 3 or attempt % 5 == 0:
                    rospy.loginfo("set_mode OFFBOARD 尝试%d: mode_sent=%s, 当前mode=%s, armed=%s",
                                  attempt + 1, getattr(resp, 'mode_sent', '?'),
                                  self.state.mode.strip(), self.state.armed)
            except rospy.ServiceException as e:
                rospy.logwarn("set_mode service 异常: %s", e)
            rospy.sleep(0.3)
            if self.state.mode.strip() == 'OFFBOARD':
                offboard_ok = True
                rospy.loginfo("OFFBOARD 已进入 (尝试 %d)", attempt + 1)
                break
        if not offboard_ok:
            rospy.logwarn("OFFBOARD 未确认 (mode=%s), 退回 AUTO.LOITER",
                          self.state.mode.strip())
            try:
                self._mode_srv(custom_mode='AUTO.LOITER')
            except rospy.ServiceException:
                pass

        # 解锁, 循环确认 armed
        armed_ok = False
        for attempt in range(10):
            try:
                self._arm_srv(True)
            except rospy.ServiceException as e:
                rospy.logerr("解锁调用失败: %s", e)
            rospy.sleep(0.3)
            if self.state.armed:
                armed_ok = True
                break
        if armed_ok:
            rospy.loginfo("已解锁 (mode=%s)", self.state.mode.strip())
        else:
            rospy.logerr("解锁失败 (mode=%s)", self.state.mode.strip())
        rospy.sleep(1.0)
        # 纯 CTBR: 预热 body_rate 流稳定 2s 后, MPC 从地面接管起飞
        if armed_ok and offboard_ok:
            rospy.loginfo("预热 body_rate 流 2s 后 MPC 接管起飞...")
            rospy.sleep(2.0)
        self._warmup = False
        rospy.loginfo("开始控制循环 (MPC body_rate 接管)")

        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t_elapsed = rospy.Time.now().to_sec() - t_start
            if self.x_current is None:
                self.rate.sleep(); continue

            wind = self.wind_meas if self._got_wind else None
            x = self.x_current.copy()
            u, cinfo = self.controller.step(x, wind_meas=wind, yaw_cur=self.yaw_cur)

            # 写发布缓存
            with self._cmd_lock:
                self._fc = float(u[0]); self._wx = float(u[1])
                self._wy = float(u[2]); self._wz = float(u[3])

            self._log_row(t_elapsed, x, u, cinfo)
            self.rate.sleep()

        self._publishing = False
        self._csv_close()

    def _log_row(self, t, x, u, ci):
        wp0 = ci['wind_seq0']
        ic = ci['int_corr']
        umx = ci['u_max_eff']
        th = ci['theta']
        self._csv_write([
            t, *x[:3], *x[3:7], *x[7:10],
            *ci['ref_pos'],
            u[0], u[1], u[2], u[3],
            ic[0], ic[1], ic[2], ic[3],
            np.degrees(self.roll_cur), np.degrees(self.pitch_cur),
            np.degrees(self.yaw_cur),
            self.wind_meas[0], self.wind_meas[1], self.wind_meas[2],
            wp0[0], wp0[1], wp0[2],
            np.linalg.norm(self.wind_meas), ci['q_dyn'],
            umx[0], umx[1], umx[2], umx[3],
            ci['power_est'],
            th['q_xy'], th['q_z'], th['lambda_energy'], th['r_omega'],
            ci['coast_factor'], ci['cma_mode'], ci['energy_mode'], ci['tau_mpc'],
            self.state.mode.strip(), int(self.state.armed),
            ci['mpc_ok'], ci['mpc_iter'],
        ])
        if self._csv_count % 100 == 0:
            ee = ""
            if ci['coast_factor'] > 0.05:
                ee += f" 🌬coast={ci['coast_factor']:.2f}"
            if ci['thrust_bias'] != 0:
                ee += f" ΔT={ci['thrust_bias']:+.2f}"
            rospy.loginfo(
                "[t=%.1f] |w|=%.1f q=%.1f  u=(fc=%.1f,ωx=%+.2f,ωy=%+.2f,ωz=%.1f)  "
                "z=%.2f err=%.2f P=%.0fW λ=%.0f cma=%s%s",
                t, np.linalg.norm(self.wind_meas), ci['q_dyn'],
                u[0], u[1], u[2], u[3], x[2],
                np.linalg.norm(ci['pos_err'][:2]), ci['power_est'],
                th['lambda_energy'], ci['cma_mode'], ee)


if __name__ == '__main__':
    try:
        MPCNode().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("TCW-MPC 已停止")
