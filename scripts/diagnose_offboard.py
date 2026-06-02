#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键诊断脚本：检查 PX4 Offboard 模式的所有前置条件
用法: python3 diagnose_offboard.py
"""

import rospy
import sys
from mavros_msgs.msg import State, ExtendedState
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import SetMode, CommandBool


class OffboardDiagnoser:
    def __init__(self):
        rospy.init_node('offboard_diagnoser', anonymous=True)

        self.state = State()
        self.ext_state = ExtendedState()
        self.gps_fix = None

        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/extended_state', ExtendedState, self._ext_state_cb)

    def _state_cb(self, msg):
        self.state = msg

    def _ext_state_cb(self, msg):
        self.ext_state = msg

    def check_connection(self):
        """检查 MAVROS 是否连接 PX4"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("  1. MAVROS 连接状态")
        rospy.loginfo("=" * 60)

        timeout = rospy.Time.now() + rospy.Duration(10)
        while not self.state.connected and rospy.Time.now() < timeout:
            rospy.sleep(0.5)

        if self.state.connected:
            rospy.loginfo("  ✅ PX4 已连接 (system_status=%d)", self.state.system_status)
            return True
        else:
            rospy.logerr("  ❌ PX4 未连接！")
            rospy.logerr("     请检查:")
            rospy.logerr("     1. PX4 SITL 是否启动? (make px4_sitl gazebo)")
            rospy.logerr("     2. MAVROS fcu_url 是否正确?")
            rospy.logerr("     3. 防火墙是否阻止了 UDP 14540/14557 端口?")
            return False

    def check_ekf(self):
        """检查 EKF2 是否收敛"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("  2. EKF2 状态 (local_position)")
        rospy.loginfo("=" * 60)

        try:
            from nav_msgs.msg import Odometry
            odom = rospy.wait_for_message('/mavros/local_position/odom', Odometry, timeout=5)
            x, y, z = odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z
            if x == 0 and y == 0 and z == 0:
                rospy.logwarn("  ⚠️  EKF 位置全零，可能尚未收敛")
                rospy.logwarn("     等待姿态估计稳定后再尝试 Offboard")
                return False
            else:
                rospy.loginfo("  ✅ EKF 位置有效: (%.2f, %.2f, %.2f)", x, y, z)
                return True
        except rospy.ROSException:
            rospy.logerr("  ❌ 无法获取 /mavros/local_position/odom")
            rospy.logerr("     EKF2 可能未融合，检查 GPS/Mocap/Vision 是否正常")
            return False

    def check_arming_flags(self):
        """检查解锁前置条件"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("  3. 当前状态")
        rospy.loginfo("=" * 60)

        rospy.loginfo("  Mode:      %s", self.state.mode or "N/A")
        rospy.loginfo("  Armed:     %s", "YES" if self.state.armed else "NO")
        rospy.loginfo("  Guided:    %s", "YES" if self.state.guided else "NO")
        rospy.loginfo("  Manual in: %s", "YES" if self.state.manual_input else "NO")

        rospy.loginfo("  Landed:    %s",
                       "YES" if self.ext_state.landed_state == 1 else
                       "In Air" if self.ext_state.landed_state == 2 else
                       "UNKNOWN")

        # 检查是否有 RC 输入
        try:
            from mavros_msgs.msg import RCIn
            rc_in = rospy.wait_for_message('/mavros/rc/in', RCIn, timeout=3)
            if len(rc_in.channels) > 0 and any(c > 0 for c in rc_in.channels):
                rospy.loginfo("  RC Input:  DETECTED (channels=%s)", rc_in.channels[:4])
            else:
                rospy.logwarn("  ⚠️  RC Input: NOT DETECTED (全零)")
                rospy.logwarn("     如果 COM_RC_IN_MODE 需要 RC，会被拒绝解锁")
        except rospy.ROSException:
            rospy.logwarn("  ⚠️  无法检查 RC 输入")

        return True

    def check_sys_status(self):
        """检查系统状态码"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("  4. 系统状态 (system_status)")
        rospy.loginfo("=" * 60)

        status_map = {
            0: "MAV_STATE_UNINIT",
            1: "MAV_STATE_BOOT",
            2: "MAV_STATE_CALIBRATING",
            3: "MAV_STATE_STANDBY (可解锁)",
            4: "MAV_STATE_ACTIVE (已解锁)",
            5: "MAV_STATE_CRITICAL",
            6: "MAV_STATE_EMERGENCY",
            7: "MAV_STATE_POWEROFF",
        }
        status_text = status_map.get(self.state.system_status, "UNKNOWN")
        rospy.loginfo("  Status: %d → %s", self.state.system_status, status_text)

        if self.state.system_status < 3:
            rospy.logwarn("  ⚠️  系统尚未就绪，无法解锁")

        return True

    def test_offboard_switch(self):
        """尝试切换到 Offboard 模式（不解锁）"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("  5. 测试 Offboard 切换")
        rospy.loginfo("=" * 60)

        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=5)
            set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

            resp = set_mode(base_mode=0, custom_mode='OFFBOARD')
            rospy.loginfo("  Mode sent: %s", resp.mode_sent)

            if resp.mode_sent:
                rospy.loginfo("  ✅ 模式切换请求已发送，等待确认...")
                rospy.sleep(1.5)

                if self.state.mode == "OFFBOARD":
                    rospy.loginfo("  ✅ Offboard 模式切换成功！")
                else:
                    rospy.logerr("  ❌ 请求发送了但未切换到 Offboard")
                    rospy.logerr("     当前模式: %s", self.state.mode)
                    rospy.logerr("     可能原因:")
                    rospy.logerr("     1. 未持续发布 setpoint (PX4 要求在切模式前就开始发)")
                    rospy.logerr("     2. COM_RC_IN_MODE 参数设置问题")
                    rospy.logerr("     3. EKF 未收敛")
            else:
                rospy.logerr("  ❌ PX4 拒绝切换模式请求")
                rospy.logerr("     检查 QGC / mavlink console 中的拒绝原因")
        except rospy.ServiceException as e:
            rospy.logerr("  ❌ set_mode 服务调用失败: %s", e)

    def run(self):
        rospy.loginfo("")
        rospy.loginfo("╔══════════════════════════════════════════════════════════╗")
        rospy.loginfo("║        PX4 Offboard 模式诊断工具                         ║")
        rospy.loginfo("╚══════════════════════════════════════════════════════════╝")
        rospy.loginfo("")

        # 等待话题数据
        rospy.sleep(1.0)

        if not self.check_connection():
            return

        ok = self.check_ekf()
        self.check_arming_flags()
        self.check_sys_status()

        if ok:
            self.test_offboard_switch()

        rospy.loginfo("")
        rospy.loginfo("══════════════════════════════════════════════════════════")
        rospy.loginfo("  诊断完成。如需设置 PX4 参数，使用:")
        rospy.loginfo("  rosrun mavros mavparam set COM_RC_IN_MODE 1")
        rospy.loginfo("  rosrun mavros mavparam set COM_ARM_WO_GPS 1")
        rospy.loginfo("══════════════════════════════════════════════════════════")


if __name__ == '__main__':
    try:
        diagnoser = OffboardDiagnoser()
        diagnoser.run()
    except rospy.ROSInterruptException:
        pass
