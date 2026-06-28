#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_omega_calib.py — Gazebo SITL 转速标定采集
=================================================
目的: 拿 Gazebo 真值标定 power_model_v2 的输入适配层 (u→ω) 与悬停转速。
- Gazebo 不仿真电功率, 只算力 (T=C_T·ω²)。故标定目标 = 转速链路, 非功率本身。
- 采集: PX4 发给 Gazebo 的真实参考转速 (command/motor_speed, rad/s)
        + MAVROS 油门/姿态/位置, 在稳态悬停 + 几个推力工况下。

飞行: OFFBOARD 位置控制, 起飞 → 悬停 z=3 → 记录稳态 → 降。
输出: /tmp/gazebo_calib.csv  (t, motor_speed[4], thrust_sp, z, vz)
"""
import rospy, csv, time
import numpy as np
import subprocess, threading
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped


class Calib:
    def __init__(self):
        rospy.init_node('gazebo_omega_calib')
        self.state = State()
        self.pose = PoseStamped()
        self.vel = TwistStamped()
        self.motor_speed = [0.0]*4          # 来自 gztopic, 后台线程更新
        self.thr_sp = 0.0
        rospy.Subscriber('/mavros/state', State, lambda m: setattr(self, 'state', m))
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, lambda m: setattr(self, 'pose', m))
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, lambda m: setattr(self, 'vel', m))
        self.sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        # gztopic 真转速后台读取
        self._gz_run = True
        self.t_gz = threading.Thread(target=self._gz_motor_reader, daemon=True)
        self.t_gz.start()

    def _gz_motor_reader(self):
        """订阅 Gazebo command/motor_speed (PX4→Gazebo 参考转速, rad/s)."""
        topic = '/gazebo/default/iris/gazebo/command/motor_speed'
        try:
            proc = subprocess.Popen(['gz', 'topic', '-e', topic],
                                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                    universal_newlines=True)
        except Exception as e:
            rospy.logwarn('gz topic 启动失败: %s', e)
            return
        vals = []
        for line in proc.stdout:
            if not self._gz_run:
                proc.terminate(); break
            line = line.strip()
            # CommandMotorSpeed: 仅 repeated motor_speed; gz echo 每值一行, 消息间空行分隔
            if line.startswith('motor_speed:'):
                try:
                    vals.append(float(line.split(':')[1]))
                except ValueError:
                    pass
                if len(vals) >= 4:
                    self.motor_speed = vals[:4]; vals = []
            elif line == '' or line == '}':
                if len(vals) >= 4:
                    self.motor_speed = vals[:4]
                vals = []

    def mkpose(self, x, y, z):
        p = PoseStamped(); p.header.stamp = rospy.Time.now()
        p.pose.position.x = x; p.pose.position.y = y; p.pose.position.z = z
        p.pose.orientation.w = 1.0
        return p

    def run(self):
        r = rospy.Rate(20)
        # 预热设定点
        for _ in range(40):
            self.sp_pub.publish(self.mkpose(0, 0, 3)); r.sleep()
        # OFFBOARD + ARM
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and (self.state.mode != 'OFFBOARD' or not self.state.armed):
            if self.state.mode != 'OFFBOARD':
                self.set_mode(custom_mode='OFFBOARD')
            elif not self.state.armed:
                self.arm(True)
            self.sp_pub.publish(self.mkpose(0, 0, 3)); r.sleep()
            if (rospy.Time.now()-t0).to_sec() > 10:
                break
        rospy.loginfo('mode=%s armed=%s', self.state.mode, self.state.armed)

        f = open('/tmp/gazebo_calib.csv', 'w', newline='')
        w = csv.writer(f)
        w.writerow(['t', 'm0', 'm1', 'm2', 'm3', 'z', 'vz', 'phase'])
        t_start = rospy.Time.now()

        def fly_to(z, dur, phase):
            t1 = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now()-t1).to_sec() < dur:
                self.sp_pub.publish(self.mkpose(0, 0, z))
                ms = self.motor_speed
                w.writerow(['%.2f' % (rospy.Time.now()-t_start).to_sec(),
                            '%.2f' % ms[0], '%.2f' % ms[1], '%.2f' % ms[2], '%.2f' % ms[3],
                            '%.3f' % self.pose.pose.position.z,
                            '%.3f' % self.vel.twist.linear.z, phase])
                r.sleep()

        rospy.loginfo('上升到 3m ...'); fly_to(3, 12, 'climb_hover3')
        rospy.loginfo('悬停 5m ...');   fly_to(5, 10, 'hover5')
        rospy.loginfo('悬停 2m ...');   fly_to(2, 10, 'hover2')
        rospy.loginfo('降落 ...');       fly_to(0.3, 8, 'land')
        f.close()
        self._gz_run = False
        try: self.arm(False)
        except Exception: pass
        rospy.loginfo('采集完成 → /tmp/gazebo_calib.csv')


if __name__ == '__main__':
    try:
        Calib().run()
    except rospy.ROSInterruptException:
        pass
