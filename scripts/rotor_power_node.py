#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rotor_power_node.py — 实时功率估算 ROS 节点 (sim/real 统一)
============================================================
订阅电机指令 + 速度 + 姿态 + 风, 用 power_model_v3 (BEMT) 估算电功率,
发布 /sim/rotor_power。松耦合: 不改 backend/控制器, 任何节点可订阅。

输入 (自动适配 sim/real):
  仿真: /sim/hil_actuator_controls (ActuatorControl, 归一化电机指令 u∈[0,1])
  实机: /mavros/esc_status (ESCStatus, RPM) — 若可用优先 (更准)
  速度: /mavros/local_position/velocity_local (TwistStamped, ENU)
  姿态: /mavros/local_position/pose (PoseStamped, 四元数)
  风:   /wind_field/velocity (Vector3Stamped, 可选, 算相对气流)
  实测: /mavros/battery (BatteryState, V·I 真值, 可选, 用于对比)

输出:
  /sim/rotor_power (Float32, 估算电功率 W)
  /sim/rotor_power_detail (Float32MultiArray: [P_est, P_measured, V_rel, tilt_deg])
"""
import rospy
import numpy as np
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from power_model_v3 import RotorPowerModel

from std_msgs.msg import Float32, Float32MultiArray
from mavros_msgs.msg import ActuatorControl, ESCStatus
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
from sensor_msgs.msg import BatteryState


def quat_to_zaxis(qx, qy, qz, qw):
    """机体 z 轴(桨盘法线)在世界系的方向 = 旋转矩阵第三列。"""
    return np.array([
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    ])


def quat_to_tilt_deg(qx, qy, qz, qw):
    """倾角 = 机体 z 轴与世界 z 轴夹角 [deg]。"""
    zb = quat_to_zaxis(qx, qy, qz, qw)
    cos_t = np.clip(zb[2], -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_t)))


class RotorPowerNode:
    def __init__(self):
        rospy.init_node('rotor_power_node')
        self.model = RotorPowerModel()

        # 状态
        self.motor_cmd = np.zeros(4)       # 归一化指令 (仿真)
        self.esc_rpm = None                # ESC RPM (实机, 优先)
        self.vel = np.zeros(3)             # ENU 速度
        self.quat = np.array([0, 0, 0, 1.0])  # x,y,z,w
        self.wind = np.zeros(3)            # ENU 风速
        self.batt_v = 0.0
        self.batt_i = 0.0
        self._got_cmd = False

        # 订阅
        rospy.Subscriber('/sim/hil_actuator_controls', ActuatorControl, self._cmd_cb)
        rospy.Subscriber('/mavros/esc_status', ESCStatus, self._esc_cb)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self._vel_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/wind_field/velocity', Vector3Stamped, self._wind_cb)
        rospy.Subscriber('/mavros/battery', BatteryState, self._batt_cb)

        # 发布
        self.pub_power = rospy.Publisher('/sim/rotor_power', Float32, queue_size=10)
        self.pub_detail = rospy.Publisher('/sim/rotor_power_detail', Float32MultiArray, queue_size=10)

        self.rate_hz = float(rospy.get_param('~rate', 50.0))
        rospy.loginfo('rotor_power_node: BEMT 功率估算, 发布 /sim/rotor_power @ %.0fHz', self.rate_hz)

    # ── 回调 ──
    def _cmd_cb(self, m):
        if len(m.controls) >= 4:
            self.motor_cmd = np.array([m.controls[0], m.controls[1], m.controls[2], m.controls[3]])
            self._got_cmd = True

    def _esc_cb(self, m):
        if len(m.esc_status) >= 4:
            self.esc_rpm = np.array([abs(e.rpm) for e in m.esc_status[:4]], dtype=float)

    def _vel_cb(self, m):
        self.vel = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])

    def _pose_cb(self, m):
        q = m.pose.orientation
        self.quat = np.array([q.x, q.y, q.z, q.w])

    def _wind_cb(self, m):
        self.wind = np.array([m.vector.x, m.vector.y, m.vector.z])

    def _batt_cb(self, m):
        self.batt_v = float(m.voltage) if not np.isnan(m.voltage) else 0.0
        self.batt_i = float(m.current) if not np.isnan(m.current) else 0.0

    # ── 主循环 ──
    def run(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            qx, qy, qz, qw = self.quat
            # 相对气流 = 地速 - 风速 (ENU)
            v_rel_world = self.vel - self.wind
            zb = quat_to_zaxis(qx, qy, qz, qw)        # 桨盘法线(世界系)
            tilt_deg = quat_to_tilt_deg(qx, qy, qz, qw)

            # 转速来源: 实机 ESC RPM 优先, 否则仿真指令
            if self.esc_rpm is not None:
                omegas = self.model.omegas_from_rpm(self.esc_rpm)
            elif self._got_cmd:
                omegas = self.model.omegas_from_motor_cmd(self.motor_cmd)
            else:
                r.sleep(); continue

            P_est = self.model.power_from_omegas(
                omegas, v_world=v_rel_world, tilt_normal=zb)

            # 实测功率 (电流计 V·I, 若有)
            P_meas = abs(self.batt_v * self.batt_i)

            self.pub_power.publish(Float32(data=P_est))
            detail = Float32MultiArray()
            detail.data = [float(P_est), float(P_meas),
                           float(np.linalg.norm(v_rel_world)), float(tilt_deg)]
            self.pub_detail.publish(detail)
            r.sleep()


if __name__ == '__main__':
    try:
        RotorPowerNode().run()
    except rospy.ROSInterruptException:
        pass
