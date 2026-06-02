#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PX4 Offboard 模式航点飞行测试脚本 (v3)
- 独立线程持续发布 setpoint_position
- 同时发布 ManualControl 虚拟摇杆 (防止 RC 丢失触发 failsafe)
- 切换到 Offboard → 解锁 → 飞航点 → 降落

用法: rosrun offboard_test offboard_waypoint_test.py
"""

import rospy
import threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import SetMode, CommandBool

# ─── 配置 ────────────────────────────────────────────────
TAKEOFF_HEIGHT = 2.5          # 起飞高度 (m)
HOVER_TIME = 4.0              # 每个航点悬停时间 (s)
SETPOINT_RATE = 20            # 发布频率 Hz (>2Hz)
LAND_HEIGHT = 0.3             # 降落前低空高度

# 航点序列 (x,y,z) ENU 坐标系
WAYPOINTS = [
    (0.0, 0.0, TAKEOFF_HEIGHT),   # 起飞
    (3.0, 0.0, TAKEOFF_HEIGHT),   # 前飞
    (3.0, 3.0, TAKEOFF_HEIGHT),   # 右飞
    (0.0, 3.0, TAKEOFF_HEIGHT),   # 后飞
    (0.0, 0.0, TAKEOFF_HEIGHT),   # 回原点
]


class OffboardWaypointTest:
    def __init__(self):
        rospy.init_node('offboard_waypoint_test', anonymous=True)

        # ── 共享状态 ────────────────────────────────────
        self.state = State()
        self.lock = threading.Lock()

        # 当前目标 setpoint
        self._sp = PoseStamped()
        self._sp.header.frame_id = "map"
        self._sp.pose.position.x = 0.0
        self._sp.pose.position.y = 0.0
        self._sp.pose.position.z = TAKEOFF_HEIGHT
        self._sp.pose.orientation.w = 1.0

        # ── 订阅 ────────────────────────────────────────
        rospy.Subscriber('/mavros/state', State, self._state_cb)

        # ── 发布 ────────────────────────────────────────
        self.pos_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10
        )
        # 虚拟摇杆 - 防止 RC 丢失触发 failsafe
        self.man_pub = rospy.Publisher(
            '/mavros/manual_control/send', ManualControl, queue_size=10
        )

        # ── 服务 ────────────────────────────────────────
        rospy.loginfo("等待 MAVROS 服务...")
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        self._set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self._arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        rospy.loginfo("MAVROS 服务就绪")

        # ── 持续发布线程 ────────────────────────────────
        self._publishing = False
        self._pub_thread = None

    def _state_cb(self, msg):
        with self.lock:
            self.state = msg

    def get_mode(self):
        with self.lock:
            return self.state.mode

    def get_armed(self):
        with self.lock:
            return self.state.armed

    def get_connected(self):
        with self.lock:
            return self.state.connected

    def _publish_loop(self):
        """持续发布 setpoint + 虚拟摇杆 (满足 PX4 Offboard 所有前置条件)"""
        rate = rospy.Rate(SETPOINT_RATE)
        while self._publishing and not rospy.is_shutdown():
            # 1. 位置 setpoint
            with self.lock:
                sp = PoseStamped()
                sp.header.stamp = rospy.Time.now()
                sp.header.frame_id = self._sp.header.frame_id
                sp.pose = self._sp.pose
            self.pos_pub.publish(sp)

            # 2. 虚拟摇杆 (中位) - 防止 RC 丢失 failsafe
            man = ManualControl()
            man.header.stamp = rospy.Time.now()
            man.x = 0    # pitch
            man.y = 0    # roll
            man.z = 500  # throttle 中位 (500 = 悬停, 范围 0~1000)
            man.r = 0    # yaw
            man.buttons = 0
            self.man_pub.publish(man)

            rate.sleep()

    def start_publishing(self):
        """启动后台发布线程"""
        if self._publishing:
            return
        self._publishing = True
        self._pub_thread = threading.Thread(target=self._publish_loop)
        self._pub_thread.daemon = True
        self._pub_thread.start()
        rospy.loginfo("📡 后台发布已启动 (setpoint + 虚拟摇杆, %.0f Hz)", SETPOINT_RATE)

    def stop_publishing(self):
        """停止后台发布"""
        self._publishing = False
        if self._pub_thread:
            self._pub_thread.join(timeout=2.0)
        rospy.loginfo("📡 后台发布已停止")

    def set_target(self, x, y, z):
        """更新目标 setpoint"""
        with self.lock:
            self._sp.pose.position.x = x
            self._sp.pose.position.y = y
            self._sp.pose.position.z = z

    def wait_for_connection(self, timeout=30):
        rospy.loginfo("等待 PX4 连接...")
        t0 = rospy.Time.now()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.get_connected():
                rospy.loginfo("✅ PX4 已连接")
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr("❌ 连接超时")
                return False
            rate.sleep()

    def wait_for_ekf(self, timeout=30):
        rospy.loginfo("等待 EKF...")
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                from nav_msgs.msg import Odometry
                odom = rospy.wait_for_message(
                    '/mavros/local_position/odom', Odometry, timeout=2
                )
                rospy.loginfo("✅ EKF: (%.2f, %.2f, %.2f)",
                              odom.pose.pose.position.x,
                              odom.pose.pose.position.y,
                              odom.pose.pose.position.z)
                return True
            except rospy.ROSException:
                if (rospy.Time.now() - t0).to_sec() > timeout:
                    rospy.logerr("❌ EKF 超时")
                    return False
                rospy.logwarn_throttle(5, "等待 EKF...")
            rate.sleep()

    def switch_mode(self, mode, timeout=10):
        """切换飞行模式并等待确认"""
        current = self.get_mode()
        rospy.loginfo("请求切换模式: %s (当前: %s)", mode, current)
        try:
            resp = self._set_mode(base_mode=0, custom_mode=mode)
            if not resp.mode_sent:
                rospy.logerr("❌ 模式请求被拒绝")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("❌ set_mode 失败: %s", e)
            return False

        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.get_mode() == mode:
                rospy.loginfo("✅ 已进入 %s 模式", mode)
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr("❌ 超时: 未能进入 %s (当前: %s)", mode, self.get_mode())
                return False
            rate.sleep()

    def arm(self, timeout=10):
        rospy.loginfo("🔓 解锁...")
        for attempt in range(5):
            try:
                resp = self._arm(value=True)
                if resp.success:
                    break
                rospy.logwarn("解锁被拒 (attempt %d/5), 重试...", attempt + 1)
                rospy.sleep(1.0)
            except rospy.ServiceException as e:
                rospy.logerr("❌ arm 失败: %s", e)
                rospy.sleep(1.0)
        else:
            rospy.logerr("❌ 解锁失败 (5次尝试)")
            return False

        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.get_armed():
                rospy.loginfo("✅ 已解锁")
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr("❌ 解锁超时")
                return False
            rate.sleep()

    def disarm(self):
        rospy.loginfo("上锁...")
        try:
            self._arm(value=False)
            rospy.loginfo("✅ 已上锁")
        except rospy.ServiceException:
            rospy.logwarn("⚠️ disarm 失败")

    def fly_waypoints(self):
        """按顺序飞航点"""
        for i, (x, y, z) in enumerate(WAYPOINTS):
            rospy.loginfo("\n━━━ 航点 %d/%d: (%.1f, %.1f, %.1f) ━━━",
                          i + 1, len(WAYPOINTS), x, y, z)
            self.set_target(x, y, z)
            rospy.sleep(HOVER_TIME)

    def land(self):
        """降落流程"""
        rospy.loginfo("\n🛬 开始降落...")

        # 降到低空
        rospy.loginfo("降低到 %.1fm...", LAND_HEIGHT)
        self.set_target(0.0, 0.0, LAND_HEIGHT)
        rospy.sleep(4.0)

        # 切 Land
        rospy.loginfo("切换到 AUTO.LAND...")
        try:
            self._set_mode(base_mode=0, custom_mode="AUTO.LAND")
        except rospy.ServiceException:
            pass

        rospy.sleep(6.0)
        self.stop_publishing()
        self.disarm()

    def run(self):
        rospy.loginfo("\n╔══════════════════════════════════════════════╗")
        rospy.loginfo("║   PX4 Offboard 航点飞行测试 v3               ║")
        rospy.loginfo("╚══════════════════════════════════════════════╝\n")

        # 1. 连接 & EKF
        if not self.wait_for_connection():
            return
        if not self.wait_for_ekf():
            return

        # 2. 启动后台发布 (setpoint + 虚拟摇杆)
        rospy.loginfo("\n📡 启动 setpoint + 虚拟摇杆流...")
        self.start_publishing()
        rospy.sleep(3.0)  # 预热

        # 3. 切 Offboard
        rospy.loginfo("\n🎮 切换到 Offboard 模式...")
        if not self.switch_mode("OFFBOARD", timeout=15):
            self.stop_publishing()
            return

        # 4. 解锁
        rospy.loginfo("\n🔓 解锁...")
        if not self.arm():
            self.stop_publishing()
            return

        rospy.loginfo("\n🚁 开始航点飞行！\n")
        rospy.sleep(1.0)

        # 5. 飞航点
        self.fly_waypoints()

        # 6. 降落
        self.land()

        rospy.loginfo("\n✅ 测试完成！")


if __name__ == '__main__':
    try:
        tester = OffboardWaypointTest()
        tester.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("用户中断")
