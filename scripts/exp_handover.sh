#!/bin/bash
# 决定性实验: position进OFFBOARD → 交接attitude body_rate, 区分两假说
#  H1 交接空窗/瞬态 → 调大 COM_OF_LOSS_T 可解
#  H2 attitude流不被认作offboard心跳 → 仍 LAND
# 同时监控 PX4 是否收到 attitude setpoint
set -eo pipefail
PX4_DIR="/home/tan/Desktop/px4rl/PX4-Autopilot"
OFFB_DIR="/home/tan/catkin_ws/src/offboard_test/scripts"
log(){ echo "[$(date '+%H:%M:%S')] $*"; }
B=""; P=""; M=""; C=""; MON=""
cleanup(){
    for pid in $MON $C $M $P $B; do [[ -n "$pid" ]] && kill "$pid" 2>/dev/null||true; done
    pkill -f "mpc_node" 2>/dev/null||true; pkill -f "mavros_node" 2>/dev/null||true
    pkill -f "backend_main" 2>/dev/null||true; pkill -f "px4.*none_iris" 2>/dev/null||true
    sleep 1; log "清理完成"
}
trap cleanup EXIT INT TERM
source /opt/ros/noetic/setup.bash; source /home/tan/catkin_ws/devel/setup.bash
export DISPLAY="${DISPLAY:-:0}"
pkill -f backend_main 2>/dev/null||true; pkill -f mavros_node 2>/dev/null||true; pkill -f mpc_node 2>/dev/null||true; sleep 1

log "backend (rviz)"; roslaunch quad_sim hil_backend.launch rviz:=true &>/tmp/exp_backend.log & B=$!
for i in $(seq 1 25); do lsof -i :4560 2>/dev/null|grep -q LISTEN&&break; sleep 1; done; sleep 2
log "PX4"; cd "$PX4_DIR"; PX4_SIM_HOST_ADDR=127.0.0.1 NO_PXH=1 no_sim=1 make px4_sitl none_iris &>/tmp/exp_px4.log & P=$!
for i in $(seq 1 30); do sleep 1; pgrep -f "px4.*none_iris">/dev/null&&{ log "PX4起(${i}s)";break;}; done; sleep 2
log "MAVROS"; rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14580 _setpoint_raw/thrust_scaling:=1.0 &>/tmp/exp_mavros.log & M=$!
for i in $(seq 1 30); do sleep 1; rostopic echo /mavros/state -n1 2>/dev/null|grep -q "connected: True"&&{ log "MAVROS连(${i}s)";break;}; done; sleep 2
log "params"; rosservice call /mavros/param/pull '{}'>/dev/null 2>&1; sleep 2
for kv in "SYS_HITL 1" "COM_RCL_EXCEPT 4" "MPC_XY_VEL_I_ACC 1.2" "COM_RC_IN_MODE 1" "COM_ARM_WO_GPS 1"; do
  k=${kv% *}; v=${kv#* }
  rosservice call /mavros/param/set "{param_id: \"$k\", value: {integer: ${v%.*}, real: $v}}">/dev/null 2>&1 && log "  $k=$v"
done
# 关键: 调大 offboard loss 超时, 给交接留窗口 (默认0.5s → 3s)
rosservice call /mavros/param/set '{param_id: "COM_OF_LOSS_T", value: {real: 3.0}}'>/dev/null 2>&1 && log "  COM_OF_LOSS_T=3.0 (实验)"
OFLT=$(rosservice call /mavros/param/get "{param_id: 'COM_OF_LOSS_T'}" 2>/dev/null | grep real | awk '{print $2}')
log "  COM_OF_LOSS_T 确认=$OFLT"

# 监控 PX4 是否收到 attitude setpoint (target_attitude 是 mavros 回环)
( rostopic hz /mavros/setpoint_raw/target_attitude 2>/dev/null ) &>/tmp/exp_att_hz.log & MON=$!

log "Step mpc_node (混合握手)"; cd "$OFFB_DIR"
PYTHONUNBUFFERED=1 rosrun offboard_test mpc_node.py &>/tmp/exp_mpc.log & C=$!
for i in $(seq 1 60); do
  sleep 2
  kill -0 "$C" 2>/dev/null || { log "mpc 退出"; break; }
  MODE=$(rostopic echo /mavros/state -n1 2>/dev/null|grep '^mode:'|awk '{print $2}')
  PZ=$(rostopic echo /mavros/local_position/pose -n1 2>/dev/null|grep -E '^\s+z:'|head -1|awk '{print $2}')
  log "  ${i}x2s mode=$MODE z=$PZ"
done
log "===== 结果 ====="
log "mpc 握手日志:"; grep -iE "OFFBOARD|交接|到高度|接管|body_rate" /tmp/exp_mpc.log | sed 's/^/[mpc] /'
log "PX4 failsafe:"; grep -iE "failsafe|land|offboard|disarm" /tmp/exp_px4.log | tail -8 | sed 's/^/[px4] /'
log "attitude setpoint 频率 (PX4是否收到):"; tail -3 /tmp/exp_att_hz.log | sed 's/^/[hz] /'
log "实验结束"
