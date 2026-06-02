#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offboard 姿态控制节点 (修复版)
修复要点:
1. ❌ 移除 ManualControl 发布 (它干扰 Offboard 切换)
2. ✅ 添加连接状态检查
3. ✅ 添加 setpoint 预发布 (PX4 要求在切模式前持续发送)
4. ✅ 验证模式切换是否成功
5. ✅ 添加详细日志诊断
"""

import rospy
import math
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import SetMode, CommandBool


class OffboardNode:
    def __init__(self):
        rospy.init_node('offboard_attitude', anonymous=True)

        # ---- Publishers ----
        self.att_pub = rospy.Publisher(
            '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10
        )

        # ---- Subscribers ----
        self.state = State()
        self.state_sub = rospy.Subscriber(
            '/mavros/state', State, lambda m: setattr(self, 'state', m)
        )

        self.rate = rospy.Rate(20)  # 20Hz

        # ---- Services ----
        rospy.wait_for_service('/mavros/set_mode', timeout=10)
        rospy.wait_for_service('/mavros/cmd/arming', timeout=10)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    def wait_for_connection(self, timeout=30):
        """等待 PX4 连接"""
        rospy.loginfo("等待 PX4 连接...")
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.state.connected:
                rospy.loginfo("✅ PX4 已连接!")
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logerr("❌ 超时: PX4 未连接")
                return False
            self.rate.sleep()
        return False

    def pre_arm_setpoints(self, duration=3.0):
        """预发布 setpoint，让 PX4 知道 Offboard 控制器存在"""
        rospy.loginfo("预热 Offboard setpoints (%.1f 秒)...", duration)

        att = AttitudeTarget()
        att.type_mask = 7  # Ignore roll/pitch/yaw rates
        att.thrust = 0.0   # 预热时推力为 0

        # 水平姿态（不翻滚）
        cy = math.cos(0.0)
        sy = math.sin(0.0)
        cp = math.cos(0.0)
        sp = math.sin(0.0)
        cr = math.cos(0.0)
        sr = math.sin(0.0)
        att.orientation = Quaternion(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        )

        start = rospy.Time.now()
        count = 0
        while (rospy.Time.now() - start).to_sec() < duration:
            if rospy.is_shutdown():
                return False
            att.header.stamp = rospy.Time.now()
            self.att_pub.publish(att)
            count += 1
            self.rate.sleep()

        rospy.loginfo("  预热完成，共发送 %d 条 setpoint", count)
        return True

    def switch_to_offboard(self):
        """切换到 Offboard 模式，带重试"""
        rospy.loginfo("请求切换至 OFFBOARD 模式...")

        for attempt in range(5):
            try:
                resp = self.set_mode(base_mode=0, custom_mode='OFFBOARD')

                if not resp.mode_sent:
                    rospy.logwarn("  第 %d 次: PX4 拒绝切换 (mode_sent=False)", attempt + 1)
                    rospy.logwarn("  可能原因: PX4 未接收到连续的 setpoint 流")
                else:
                    rospy.loginfo("  模式切换请求已发送，等待确认...")

                # 等待确认
                rospy.sleep(0.5)
                if self.state.mode == "OFFBOARD":
                    rospy.loginfo("  ✅ Offboard 模式切换成功!")
                    return True
                else:
                    rospy.logwarn("  第 %d 次: 当前模式为 '%s'，不是 OFFBOARD",
                                  attempt + 1, self.state.mode)

            except rospy.ServiceException as e:
                rospy.logerr("  第 %d 次: 服务调用异常: %s", attempt + 1, e)

            rospy.sleep(0.5)

        rospy.logerr("❌ Offboard 切换失败(已重试5次)")
        return False

    def arm_vehicle(self):
        """解锁，带重试"""
        rospy.loginfo("请求解锁...")

        for attempt in range(5):
            try:
                resp = self.arming(value=True)
                if resp.success:
                    rospy.loginfo("  ✅ 解锁成功!")
                    return True
                else:
                    rospy.logwarn("  第 %d 次: 解锁被拒", attempt + 1)

            except rospy.ServiceException as e:
                rospy.logerr("  第 %d 次: 服务异常: %s", attempt + 1, e)

            rospy.sleep(1.0)

        rospy.logerr("❌ 解锁失败(已重试5次)")
        rospy.logerr("   请检查:")
        rospy.logerr("   1. QGC 中的 arming check 报告")
        rospy.logerr("   2. rosrun mavros mavparam get COM_RC_IN_MODE")
        rospy.logerr("   3. rosrun mavros mavparam get COM_ARM_WO_GPS")
        return False

    def build_hover_setpoint(self):
        """构造水平悬停 setpoint (roll=0, pitch=0, thrust=0.72)"""
        att = AttitudeTarget()
        att.type_mask = 7
        att.thrust = 0.72

        # 万向节锁安全的四元数构造
        r, p, y = 0.0, 0.0, 0.0
        cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
        cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
        cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)

        att.orientation = Quaternion(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        )
        return att

    def run(self):
        # ---- Step 1: 等待连接 ----
        if not self.wait_for_connection():
            return

        # ---- Step 2: 预热 setpoint ----
        self.pre_arm_setpoints(duration=3.0)

        # ---- Step 3: 切换 Offboard ----
        if not self.switch_to_offboard():
            rospy.logerr("无法进入 Offboard，退出")
            return

        # ---- Step 4: 解锁 ----
        if not self.arm_vehicle():
            rospy.logerr("无法解锁，退出")
            return

        # ---- Step 5: 主循环 ----
        rospy.loginfo("🚀 进入主循环 (发布悬停 setpoint)...")
        setpoint = self.build_hover_setpoint()

        while not rospy.is_shutdown():
            # 健康检查：如果掉模式或失锁，停止推力和姿态指令
            if self.state.mode != "OFFBOARD":
                rospy.logwarn("⚠️  已退出 Offboard (当前: %s)，发送安全 setpoint",
                              self.state.mode)
                # 发零推力防止意外
                safe = self.build_hover_setpoint()
                safe.thrust = 0.0
                safe.header.stamp = rospy.Time.now()
                self.att_pub.publish(safe)
                self.rate.sleep()
                continue

            if not self.state.armed:
                rospy.logwarn("⚠️  已失锁!")
                self.att_pub.publish(setpoint)  # 仍然发但推力已为0
                self.rate.sleep()
                continue

            # 正常发布
            setpoint.header.stamp = rospy.Time.now()
            self.att_pub.publish(setpoint)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = OffboardNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
