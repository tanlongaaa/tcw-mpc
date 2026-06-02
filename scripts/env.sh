#!/bin/bash
# ──────────────────────────────────────────────────────────
# PX4 SITL + MAVROS 环境变量设置脚本
# 每个新终端都需要 source 这个文件
#
# 用法: source /home/tan/catkin_ws/src/offboard_test/scripts/env.sh
# ──────────────────────────────────────────────────────────

# ── 1. ROS Noetic ───────────────────────────────────────
source /opt/ros/noetic/setup.bash

# ── 2. catkin workspace ──────────────────────────────────
# devel/setup.bash 会重写 ROS_PACKAGE_PATH，先记下旧的以便恢复
_OLD_PKG_PATH="$ROS_PACKAGE_PATH"
source /home/tan/catkin_ws/devel/setup.bash --extend 2>/dev/null || \
    source /home/tan/catkin_ws/devel/setup.bash

# ── 3. PX4 路径定义 ─────────────────────────────────────
export PX4_ROOT=/home/tan/Desktop/px4rl/PX4-Autopilot
export PX4_BUILD=$PX4_ROOT/build/px4_sitl_default
export SITL_GAZEBO=$PX4_ROOT/Tools/sitl_gazebo

# ── 4. Gazebo 仿真环境 ──────────────────────────────────
export GAZEBO_PLUGIN_PATH=$PX4_BUILD/build_gazebo:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$SITL_GAZEBO/models:$GAZEBO_MODEL_PATH
export LD_LIBRARY_PATH=$PX4_BUILD/build_gazebo:$LD_LIBRARY_PATH

# ── 5. ROS 包路径 ───────────────────────────────────────
# 确保 px4 和 mavlink_sitl_gazebo 可被 rospack 找到
# 放在最前面，确保优先于其他同名包
export ROS_PACKAGE_PATH=$PX4_ROOT:$SITL_GAZEBO:$ROS_PACKAGE_PATH

echo "✅ PX4 SITL 环境已加载"
echo "   PX4_ROOT       → $PX4_ROOT"
echo "   SITL_GAZEBO    → $SITL_GAZEBO"
echo "   GAZEBO_MODELS  → $SITL_GAZEBO/models"
echo "   GAZEBO_PLUGINS → $PX4_BUILD/build_gazebo"
