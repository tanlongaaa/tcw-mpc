#!/usr/bin/env bash
# ============================================================================
# start_hil_wind.sh — PX4 HIL 仿真 + RViz 可视化 + 极端湍流风场 (一键拉起)
#
# 用法:
#   ./start_hil_wind.sh              # 正常启动
#   ./start_hil_wind.sh stop         # 停止所有相关进程
#   ./start_hil_wind.sh status       # 查看运行状态
#   ./start_hil_wind.sh restart      # 重启
#
# 依赖: ROS1 Noetic, PX4 v1.13.3 @ ~/Desktop/px4rl/PX4-Autopilot
#       quad_sim @ ~/catkin_ws/src/px4-ros-6dof_project-pid-eso-
#       offboard_test @ ~/catkin_ws/src/offboard_test
#
# 日志文件: /tmp/hil_{roscore,backend,px4,mavros,pidbl,wind}.log
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJ_DIR="/home/tan/catkin_ws/src/px4-ros-6dof_project-pid-eso-"
ROS_WS="/home/tan/catkin_ws"
PX4_DIR="/home/tan/Desktop/px4rl/PX4-Autopilot"
WIND_SCRIPT="/home/tan/catkin_ws/src/offboard_test/scripts/wind_field.py"
PID_SCRIPT="/home/tan/catkin_ws/src/offboard_test/scripts/pid_baseline.py"

# ── 停止 ──────────────────────────────────────────────────────────────────────
do_stop() {
    echo "🛑 停止 HIL 全部进程..."
    # 按依赖逆序杀，避免误杀 exec 自身
    for name in "pid_baseline" "wind_field" "mavros_node" "px4" "rviz" "backend_main" "roslaunch" "rosmaster"; do
        pids=$(ps aux | grep "$name" | grep -v grep | awk '{print $2}')
        [ -n "$pids" ] && kill $pids 2>/dev/null || true
    done
    sleep 2
    for name in "pid_baseline" "wind_field" "mavros_node" "px4" "rviz" "backend_main" "roslaunch" "rosmaster"; do
        pids=$(ps aux | grep "$name" | grep -v grep | awk '{print $2}')
        [ -n "$pids" ] && kill -9 $pids 2>/dev/null || true
    done
    sleep 1
    echo "✅ 已停止"
}

# ── 状态 ──────────────────────────────────────────────────────────────────────
do_status() {
    echo "===== 📊 HIL 运行状态 ====="
    for comp in "rosmaster:roscore" "hil_backend:Backend" "backend_main:plant_6dof" "rviz:RViz" \
                "bin/px4:PX4-SITL" "mavros_node:MAVROS" "pid_baseline:PID" "wind_field:Wind🌪️"; do
        pattern="${comp%%:*}"
        label="${comp##*:}"
        pid=$(ps aux | grep "$pattern" | grep -v grep | awk '{print $2}' | head -1)
        if [ -n "$pid" ]; then
            echo "  ✅ $label  (PID $pid)"
        else
            echo "  ❌ $label  (未运行)"
        fi
    done

    # 如果 roscore 活着，查 MAVROS 状态
    if rostopic list 2>/dev/null | grep -q /mavros/state; then
        echo ""
        echo "--- MAVROS ---"
        rostopic echo /mavros/state -n 1 2>/dev/null | grep -E "connected|armed|mode"
        echo ""
        echo "--- 无人机位置 ---"
        rostopic echo /sim/odom -n 1 --noarr 2>/dev/null | grep "position:" -A 3
        echo ""
        echo "--- 风场 ---"
        rostopic echo /wind_field/velocity -n 1 --noarr 2>/dev/null | grep "vector:" -A 3
    fi
}

# ── 启动 ──────────────────────────────────────────────────────────────────────
do_start() {
    source /opt/ros/noetic/setup.bash
    source "$ROS_WS/devel/setup.bash"
    export DISPLAY=:0

    echo "========================================="
    echo "  🌪️  PX4 HIL + RViz + 风场 一键启动"
    echo "========================================="
    echo "  日志: /tmp/hil_*.log"
    echo ""

    # (1) roscore
    echo "===== (1/6) roscore ====="
    if rostopic list 2>/dev/null | head -1 | grep -q .; then
        echo "  ⚡ roscore 已运行，跳过"
    else
        roscore > /tmp/hil_roscore.log 2>&1 &
        sleep 3
        echo "  ✅ roscore 已启动"
    fi

    # (2) Backend + RViz
    echo "===== (2/6) HIL Backend + RViz ====="
    roslaunch quad_sim hil_backend.launch rviz:=true > /tmp/hil_backend.log 2>&1 &
    sleep 4
    echo "  ✅ Backend + RViz 已启动"

    # (3) PX4 SITL (HIL mode — 守护进程)
    echo "===== (3/6) PX4 SITL (HIL) ====="
    if ps aux | grep -q "[b]in/px4"; then
        echo "  ⚡ PX4 已运行，跳过"
    else
        cd "$PX4_DIR"
        PX4_SIM_HOST_ADDR=127.0.0.1 NO_PXH=1 no_sim=1 make px4_sitl none_iris > /tmp/hil_px4.log 2>&1 &
        # 等待 PX4 启动完成（等待 "Startup script returned successfully" 或超时）
        echo -n "  等待 PX4 启动 "
        for i in $(seq 1 30); do
            if tail -3 /tmp/hil_px4.log 2>/dev/null | grep -q "Startup script returned"; then
                echo " ✅ ($i 秒)"
                break
            fi
            echo -n "."
            sleep 1
        done
    fi

    # (4) MAVROS
    echo "===== (4/6) MAVROS ====="
    if ps aux | grep -q "[m]avros_node"; then
        echo "  ⚡ MAVROS 已运行，跳过"
    else
        rosrun mavros mavros_node _fcu_url:=udp://127.0.0.1:14540@127.0.0.1:14560 > /tmp/hil_mavros.log 2>&1 &
        echo -n "  等待 MAVROS 连接 PX4 "
        for i in $(seq 1 15); do
            if rostopic echo /mavros/state -n 1 2>/dev/null | grep -q "connected: True"; then
                echo " ✅ ($i 秒)"
                break
            fi
            echo -n "."
            sleep 1
        done
    fi

    # (5) 设置 PX4 参数
    echo "===== (5/6) PX4 参数 ====="
    # HIL 模式必须开
    rosservice call /mavros/param/set '{param_id: "SYS_HITL", value: {integer: 1}}' > /dev/null 2>&1
    echo "  SYS_HITL=1  ✅"

    # PID 参数 — 保持 PX4 v1.13.3 出厂默认 (不修改, 仅读取)
    echo "  PID (factory defaults, no override):"
    for param in MPC_XY_P MPC_XY_VEL_P_ACC MPC_XY_VEL_I_ACC MPC_XY_VEL_D_ACC MPC_TILTMAX_AIR MPC_Z_VEL_P_ACC MPC_Z_VEL_I_ACC MPC_THR_HOVER; do
        actual=$(rosservice call /mavros/param/get "{param_id: '$param'}" 2>/dev/null | grep -oP 'real: \K[0-9.]+' || echo "N/A")
        echo "    $param = $actual (factory)"
    done

    # (6) 控制节点 + 风场
    echo "===== (6/6) pid_baseline + wind_field ====="
    if ps aux | grep -q "[p]id_baseline"; then
        echo "  ⚡ pid_baseline 已运行"
    else
        python3 "$PID_SCRIPT" > /tmp/hil_pidbl.log 2>&1 &
        echo "  ✅ pid_baseline 已启动"
    fi
    sleep 2

    if ps aux | grep -q "[w]ind_field"; then
        echo "  ⚡ wind_field 已运行"
    else
        python3 "$WIND_SCRIPT" --no-wrench > /tmp/hil_wind.log 2>&1 &
        echo "  ✅ wind_field 已启动 (--no-wrench, 风阻经 backend 注入 plant)"
    fi
    sleep 2

    echo ""
    echo "========================================="
    echo "  ✅ HIL 全套已启动"
    echo "========================================="
    echo "  RViz  → 屏幕 :0"
    echo "  查看:  $0 status"
    echo "  停止:  $0 stop"
    echo "  日志:  tail -f /tmp/hil_{backend,mavros,pidbl,wind}.log"
    echo ""
    echo "  🌪️  风场: rostopic echo /wind_field/velocity"
    echo "  🚁  位置: rostopic echo /sim/odom"
}

# ── 入口 ──────────────────────────────────────────────────────────────────────
case "${1:-start}" in
    stop|kill)   do_stop ;;
    status|stat) do_status ;;
    start)       do_start ;;
    restart)
        do_stop
        sleep 2
        do_start
        ;;
    *)
        echo "用法: $0 {start|stop|status|restart}"
        exit 1
        ;;
esac
