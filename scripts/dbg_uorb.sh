#!/bin/bash
# 抓 PX4 端 offboard_control_mode uORB — 确认 mavros 发 body_rate 流时是否置标志
set -eo pipefail
PX4_DIR="/home/tan/Desktop/px4rl/PX4-Autopilot"
OFFB_DIR="/home/tan/catkin_ws/src/offboard_test/scripts"
log(){ echo "[$(date '+%H:%M:%S')] $*"; }
B=""; P=""; M=""; C=""
cleanup(){
    for pid in $C $M $P $B; do [[ -n "$pid" ]] && kill "$pid" 2>/dev/null||true; done
    for pid in $(pgrep -f "mpc_node.py") $(pgrep -f mavros_node) $(pgrep -f backend_main); do kill "$pid" 2>/dev/null||true; done
    sleep 1
    for pid in $(pgrep -f none_iris); do kill "$pid" 2>/dev/null||true; done
    sleep 1; log "清理完成"
}
trap cleanup EXIT INT TERM
source /opt/ros/noetic/setup.bash; source /home/tan/catkin_ws/devel/setup.bash
export DISPLAY="${DISPLAY:-:0}"
pkill -f backend_main 2>/dev/null||true; pkill -f mavros_node 2>/dev/null||true; pkill -f mpc_node 2>/dev/null||true; sleep 1

log "backend"; roslaunch quad_sim hil_backend.launch rviz:=false &>/tmp/dbg_backend.log & B=$!
for i in $(seq 1 25); do lsof -i :4560 2>/dev/null|grep -q LISTEN&&break; sleep 1; done; sleep 2
log "PX4"; cd "$PX4_DIR"; PX4_SIM_HOST_ADDR=127.0.0.1 NO_PXH=1 no_sim=1 make px4_sitl none_iris &>/tmp/dbg_px4.log & P=$!
for i in $(seq 1 30); do sleep 1; pgrep -f "px4.*none_iris">/dev/null&&{ log "PX4起(${i}s)";break;}; done; sleep 3
log "MAVROS"; rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14580 &>/tmp/dbg_mavros.log & M=$!
for i in $(seq 1 30); do sleep 1; rostopic echo /mavros/state -n1 2>/dev/null|grep -q "connected: True"&&{ log "MAVROS连(${i}s)";break;}; done; sleep 2
log "params"; rosservice call /mavros/param/pull '{}'>/dev/null 2>&1; sleep 2
for kv in "SYS_HITL 1" "COM_RCL_EXCEPT 4"; do
  k=${kv% *}; v=${kv#* }
  rosservice call /mavros/param/set "{param_id: \"$k\", value: {integer: $v, real: $v}}">/dev/null 2>&1 && log "  $k=$v"
done

log "mpc_node (发 body_rate 流)"; cd "$OFFB_DIR"
PYTHONUNBUFFERED=1 rosrun offboard_test mpc_node.py &>/tmp/dbg_mpc.log & C=$!
log "等 attitude 流稳定 8s..."; sleep 8

# ── 关键: 通过 mavlink_shell 抓 PX4 uORB ──
log "===== 抓 PX4 offboard_control_mode (流发着时) ====="
cd "$PX4_DIR"
# PX4 GCS mavlink: udp 本地口 18570, remote 14550 → shell 连 18570
( sleep 4; printf 'listener offboard_control_mode 2\n'; sleep 2; \
  printf 'listener vehicle_status 1\n'; sleep 2; \
  printf 'commander check\n'; sleep 3 ) | \
  timeout 25 python3 Tools/mavlink_shell.py 127.0.0.1:18570 2>&1 | \
  tee /tmp/dbg_shell.log | \
  grep -iE "offboard|position:|velocity:|accel|attitude:|body_rate|actuator|nav_state|arming|preflight|fail|Ready|TopicNotFound|ERROR|pos_sp|sp_offboard" | head -40
log "shell 原始输出尾:"; tail -15 /tmp/dbg_shell.log | sed 's/^/[shell] /'
log "===== 抓取结束 ====="
log "mpc 握手:"; grep -iE "set_mode|OFFBOARD|预流|未确认" /tmp/dbg_mpc.log | head -8 | sed 's/^/[mpc] /'
