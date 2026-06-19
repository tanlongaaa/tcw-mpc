#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
step_gust.py — 阶跃阵风注入器

用法:
  python3 step_gust.py --magnitude 12 --duration 2 --direction 0
    从正东方向 (0°) 吹 12m/s 持续 2 秒

发布 /wind_field/velocity 到 plant_6dof 的 wind_vel_enu 路径
与 wind_field.py 完全兼容，backend 已订阅此 topic
"""

import sys
import time
import math
import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped


def main():
    rospy.init_node('step_gust', anonymous=True)

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--magnitude', type=float, default=10.0)
    parser.add_argument('--duration', type=float, default=2.0)
    parser.add_argument('--direction', type=float, default=0.0)
    args, _ = parser.parse_known_args()
    mag = args.magnitude
    dur = args.duration
    direction = args.direction

    pub = rospy.Publisher('/wind_field/velocity', Vector3Stamped, queue_size=5)

    # 等待 subscriber (backend_main)
    rospy.loginfo("等待 /wind_field/velocity 订阅者 ...")
    while pub.get_num_connections() < 1 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    rospy.loginfo("✅ 已连接, 准备施放阶跃阵风")

    # 风向 → ENU 速度
    dir_rad = math.radians(direction)
    wind_enu = np.array([
        -mag * math.sin(dir_rad),   # East
        -mag * math.cos(dir_rad),   # North
        0.0,                         # Up (纯水平风)
    ])

    # 阶跃注入
    msg = Vector3Stamped()
    msg.header.frame_id = 'map'
    msg.vector.x = wind_enu[0]
    msg.vector.y = wind_enu[1]
    msg.vector.z = wind_enu[2]

    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(20)  # 20Hz 维持
    while rospy.Time.now().to_sec() - t0 < dur and not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    # 立即清除 (发布零风速)
    clear = Vector3Stamped()
    clear.header.frame_id = 'map'
    clear.vector.x = 0.0
    clear.vector.y = 0.0
    clear.vector.z = 0.0
    clear.header.stamp = rospy.Time.now()
    # 连发几次确保 backend 收到
    for _ in range(5):
        pub.publish(clear)
        rospy.sleep(0.1)

    rospy.loginfo("✅ 阶跃阵风结束 (%.1f m/s @ %.0f° × %.1fs)", mag, direction, dur)


if __name__ == '__main__':
    main()
