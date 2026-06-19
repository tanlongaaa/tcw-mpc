#!/usr/bin/env python3
"""gazebo_step.py — 对 Gazebo SITL 施加阶跃 body_wrench 脉冲"""
import sys
import time
import math
import numpy as np
import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from geometry_msgs.msg import Wrench, Vector3, Point
from nav_msgs.msg import Odometry

def main():
    rospy.init_node('gazebo_step', anonymous=True)
    
    mag = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    direction = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=15)
    svc = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    
    # 等待 odom
    odom = rospy.wait_for_message('/mavros/local_position/odom', Odometry, timeout=20)
    vel = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z])
    
    # 风向 → ENU 速度 (与 wind_field.py 相同公式)
    dir_rad = math.radians(direction)
    wind_enu = np.array([-mag * math.sin(dir_rad), -mag * math.cos(dir_rad), 0.0])
    Vrel = vel - wind_enu
    speed = np.linalg.norm(Vrel)
    if speed < 1e-6:
        force = np.zeros(3)
    else:
        force = 0.5 * 1.225 * 0.05 * speed * Vrel  # CdA=0.05 匹配 wind_field.py
    
    rospy.loginfo(f"Gust: {mag}m/s @ {direction}deg -> F={np.round(force,3)}N × {dur}s")
    
    w = Wrench()
    w.force = Vector3(*force)
    w.torque = Vector3(0.0, 0.0, 0.0)
    
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(20)
    while rospy.Time.now().to_sec() - t0 < dur and not rospy.is_shutdown():
        req = ApplyBodyWrenchRequest()
        req.body_name = 'iris::base_link'
        req.reference_frame = ''
        req.reference_point = Point(0, 0, 0)
        req.wrench = w
        req.start_time = rospy.Time(0)
        req.duration = rospy.Duration(0.1)
        try:
            svc(req)
        except:
            pass
        rate.sleep()
    
    rospy.loginfo("Step done")

if __name__ == '__main__':
    main()
