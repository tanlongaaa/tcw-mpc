#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""持续 OFFBOARD 悬停节点: 永不停发设定点, 直到被 kill。用于标定数据采集期间维持飞行。"""
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

rospy.init_node('persistent_hover')
st = {'mode': '', 'armed': False}
rospy.Subscriber('/mavros/state', State, lambda m: st.update(mode=m.mode, armed=m.armed))
pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
rospy.wait_for_service('/mavros/cmd/arming'); rospy.wait_for_service('/mavros/set_mode')
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
sm = rospy.ServiceProxy('/mavros/set_mode', SetMode)
Z = float(rospy.get_param('~z', 3.0))

def mp(z):
    p = PoseStamped(); p.header.stamp = rospy.Time.now()
    p.pose.position.x = 0; p.pose.position.y = 0; p.pose.position.z = z
    p.pose.orientation.w = 1.0
    return p

r = rospy.Rate(20)
for _ in range(60):           # 预热 3s 设定点流
    pub.publish(mp(Z)); r.sleep()

last = rospy.Time.now()
while not rospy.is_shutdown():
    now = rospy.Time.now()
    if st['mode'] != 'OFFBOARD' and (now-last).to_sec() > 1.0:
        sm(custom_mode='OFFBOARD'); last = now
    elif not st['armed'] and (now-last).to_sec() > 1.0:
        arm(True); last = now
    pub.publish(mp(Z))        # 永不停
    r.sleep()
