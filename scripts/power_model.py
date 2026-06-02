#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
power_model.py — 统一功耗估算 (基于 IMU 加速度)
=================================================
MPC 和 PID 共用：两者都订阅 /mavros/imu/data，同一公式算功耗。

原理:
  T ≈ m × |a_body|       (比力模长 = 推力/质量, 含重力)
  P ∝ T^(3/2)             (动量理论: 诱导功率)
  P_est = C_I × T^(3/2)

标定: C_I 使 hover |a_body|≈9.81 → T≈14.7N → P≈80W
"""

import numpy as np
from sensor_msgs.msg import Imu


class UnifiedPowerEstimator:
    """
    统一功耗估计器 (IMU 加速度 → 推力 → 功率)

    用法:
      est = UnifiedPowerEstimator(mass=1.5)
      rospy.Subscriber('/mavros/imu/data', Imu, est.imu_cb)
      power_w = est.get_power()
    """

    def __init__(self, mass=1.5):
        self.mass = mass
        self._accel = np.zeros(3)
        self._got_data = False
        self._thrust_n = 0.0

        # 标定: Hover T=14.7N → P≈80W
        # C_I = 80 / (14.7^(3/2)) = 80 / 56.35 ≈ 1.42
        self.C_I = 1.42

    def imu_cb(self, msg):
        self._accel[0] = msg.linear_acceleration.x
        self._accel[1] = msg.linear_acceleration.y
        self._accel[2] = msg.linear_acceleration.z
        self._got_data = True

    def get_power(self):
        """返回总功率 [W]"""
        if not self._got_data:
            return 0.0
        a_norm = np.linalg.norm(self._accel)
        if a_norm < 0.5:
            return 0.0
        self._thrust_n = self.mass * a_norm
        return float(self.C_I * (self._thrust_n ** 1.5))

    def get_thrust(self):
        """返回估算推力 [N]"""
        return self._thrust_n

    def has_data(self):
        return self._got_data
