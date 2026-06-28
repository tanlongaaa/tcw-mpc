#!/bin/bash
# ==============================================================================
# smoke_mpc_hil.sh — 重构后 mpc_node.py 的 HIL 通路B 冒烟测试 (无风, 自动判定)
# ==============================================================================
# 目的: 验证 mpc_controller 重构后 mpc_node 实跑行为正常 (解锁→悬停收敛)
# 通路B: Backend(plant_6dof) → PX4 SITL(none_iris) → MAVROS → mpc_node
# 无 Gazebo。无风。悬停 HOLD 秒后自动判定 z 误差并退出。
set -eo pipefail
ROS_DISTRO="${ROS_DISTRO:-noetic}"

PX4_DIR="/home/tan/Desktop/px4rl/PX4-Autopilot"
PROJECT_DIR="/home/tan/catkin_ws/src/px4-ros-6dof_project-pid-eso-"
QUAD_SIM_DIR="$PROJECT_DIR/quad_sim"
OFFB_DIR="/home/tan/catkin_ws/src/offboard_test/scripts"
HOLD=${HOLD:-40}
TARGET_Z=2.5
RVIZ=${RVIZ:-true}

log(){ echo "[$(date '+%H:%M:%S')] $*"; }

BACKEND_PID=""; PX4_PID=""; MAVROS_PID=""; MPC_PID=""
cleanup(){
    log "清理..."
    for pid in $MPC_PID $MAVROS_PID $PX4_PID $BACKEND_PID; do
        [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
    done
    pkill -f "mpc_node.py" 2>/dev/null || true
    pkill -f "mavros_node" 2>/dev/null || true
    pkill -f "backend_main" 2>/dev/null || true
    pkill -f "px4.*none_iris" 2>/dev/null || true
    sleep 1; log "清理完成"
}
trap cleanup EXIT INT TERM

source /opt/ros/noetic/setup.bash
source /home/tan/catkin_ws/devel/setup.bash
export DISPLAY="${DISPLAY:-:0}"

rostopic list &>/dev/null || { log "ERR roscore 未运行"; exit 1; }
rosparam set /use_sim_time false 2>/dev/null || true
log "roscore OK"

# 预清理残留
pkill -f "backend_main.py" 2>/dev/null || true
pkill -f "mavros_node" 2>/dev/null || true
pkill -f "mpc_node.py" 2>/dev/null || true
sleep 1

# ── Step 1: Backend (带 rviz 供小龙观察) ──
log "Step1 启动 backend (plant_6dof) rviz=$RVIZ"
roslaunch quad_sim hil_backend.launch rviz:=$RVIZ &>/tmp/smoke_backend.log &
BACKEND_PID=$!
for i in $(seq 1 25); do
    lsof -i :4560 2>/dev/null | grep -q LISTEN && { log "backend 就绪 (4560)"; break; }
    sleep 1
done
sleep 2

# ── Step 2: PX4 SITL none_iris (守护) ──
log "Step2 启动 PX4 SITL none_iris"
cd "$PX4_DIR"
PX4_SIM_HOST_ADDR=127.0.0.1 NO_PXH=1 no_sim=1 make px4_sitl none_iris &>/tmp/smoke_px4.log &
PX4_PID=$!
for i in $(seq 1 30); do
    sleep 1
    kill -0 "$PX4_PID" 2>/dev/null || { log "ERR PX4 退出, 见 /tmp/smoke_px4.log"; tail -15 /tmp/smoke_px4.log; exit 1; }
    pgrep -f "px4.*none_iris" >/dev/null 2>&1 && { log "PX4 SITL 起 (${i}s)"; break; }
done
sleep 2

# ── Step 3: MAVROS ──
log "Step3 启动 MAVROS"
# thrust_scaling=1.0 必设: 裸 rosrun 不读 px4_config.yaml, 不设会丢弃 attitude setpoint (PX4收不到msgid82)
rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14580 _setpoint_raw/thrust_scaling:=1.0 &>/tmp/smoke_mavros.log &
MAVROS_PID=$!
for i in $(seq 1 30); do
    sleep 1
    rostopic echo /mavros/state -n1 2>/dev/null | grep -q "connected: True" && { log "MAVROS 连上 (${i}s)"; break; }
done
sleep 2

# ── Step 4: PX4 参数 (复用调好的) ──
log "Step4 设置 PX4 参数"
rosservice call /mavros/param/pull '{}' >/dev/null 2>&1; sleep 2
set_p(){ rosservice call /mavros/param/set "{param_id: \"$1\", value: {$2}}" 2>&1 | grep -q "success: True" && log "  $1 ✓" || log "  $1 ✗"; }
set_p SYS_HITL "integer: 1"
set_p COM_RCL_EXCEPT "integer: 4"
set_p MPC_XY_P "real: 0.95"
set_p MPC_XY_VEL_P_ACC "real: 1.8"
set_p MPC_XY_VEL_I_ACC "real: 1.2"
set_p MPC_XY_VEL_D_ACC "real: 0.2"
set_p MPC_Z_VEL_P_ACC "real: 4.0"
set_p MPC_Z_VEL_I_ACC "real: 2.0"
set_p MPC_TILTMAX_AIR "real: 45.0"

# ── Step 5: mpc_node (被测对象) ──
log "Step5 启动 mpc_node.py (重构后, 目标 0,0,${TARGET_Z})"
cd "$OFFB_DIR"
PYTHONUNBUFFERED=1 rosrun offboard_test mpc_node.py &>/tmp/smoke_mpc.log &
MPC_PID=$!

# 等解锁
for i in $(seq 1 30); do
    sleep 1
    kill -0 "$MPC_PID" 2>/dev/null || { log "ERR mpc_node 退出, 见 /tmp/smoke_mpc.log"; tail -25 /tmp/smoke_mpc.log; exit 1; }
    rostopic echo /mavros/state -n1 2>/dev/null | grep -q "armed: True" && { log "已解锁 (${i}s)"; break; }
done

# ── Step 6: 悬停判定 ──
log "Step6 悬停判定 ${HOLD}s ..."
Z_OK=0; Z_LAST="?"; SAMPLES=0; Z_SUM=0
for ((t=5; t<=HOLD; t+=5)); do
    sleep 5
    POSE=$(rostopic echo /mavros/local_position/pose -n1 2>/dev/null | grep -E '^\s+[xyz]:' | head -3 | awk '{print $2}')
    PX=$(echo "$POSE" | sed -n 1p); PY=$(echo "$POSE" | sed -n 2p); PZ=$(echo "$POSE" | sed -n 3p)
    Z_LAST="$PZ"
    MODE=$(rostopic echo /mavros/state -n1 2>/dev/null | grep -E '^mode:' | awk '{print $2}')
    ARMED=$(rostopic echo /mavros/state -n1 2>/dev/null | grep -E '^armed:' | awk '{print $2}')
    log "  t=${t}s pos=(${PX:-?}, ${PY:-?}, ${PZ:-?}) mode=${MODE:-?} armed=${ARMED:-?}"
    if [[ -n "$PZ" ]]; then
        SAMPLES=$((SAMPLES+1)); Z_SUM=$(python3 -c "print($Z_SUM + $PZ)")
    fi
done

# 判定 (用最后稳态 z 与目标比较)
log "===== 冒烟判定 ====="
tail -6 /tmp/smoke_mpc.log | sed 's/^/[mpc] /'
if [[ -n "$Z_LAST" && "$Z_LAST" != "?" ]]; then
    VERDICT=$(python3 -c "
z=$Z_LAST; tgt=$TARGET_Z; err=abs(z-tgt)
print('PASS' if err<0.3 else 'WARN', f'z={z:.2f} err={err:.2f}m')
")
    log "VERDICT: $VERDICT"
else
    log "VERDICT: FAIL (无 odom 数据)"
fi
LATEST_CSV=$(ls -t "$OFFB_DIR"/mpc_log_*.csv 2>/dev/null | head -1)
[[ -n "$LATEST_CSV" ]] && log "CSV: $LATEST_CSV ($(wc -l <"$LATEST_CSV") 行)"
log "冒烟结束, 自动清理"
