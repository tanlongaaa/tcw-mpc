#!/bin/bash
# 对照诊断: HIL 通路B + pid_baseline (已知能飞), 只看能否进 OFFBOARD + EKF 位置
set -eo pipefail
ROS_DISTRO="${ROS_DISTRO:-noetic}"
PX4_DIR="/home/tan/Desktop/px4rl/PX4-Autopilot"
OFFB_DIR="/home/tan/catkin_ws/src/offboard_test/scripts"
log(){ echo "[$(date '+%H:%M:%S')] $*"; }
B=""; P=""; M=""; C=""
cleanup(){
    for pid in $C $M $P $B; do [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true; done
    pkill -f "pid_baseline" 2>/dev/null||true; pkill -f "mavros_node" 2>/dev/null||true
    pkill -f "backend_main" 2>/dev/null||true; pkill -f "px4.*none_iris" 2>/dev/null||true
    sleep 1; log "清理完成"
}
trap cleanup EXIT INT TERM
source /opt/ros/noetic/setup.bash; source /home/tan/catkin_ws/devel/setup.bash
export DISPLAY="${DISPLAY:-:0}"
pkill -f backend_main 2>/dev/null||true; pkill -f mavros_node 2>/dev/null||true; pkill -f pid_baseline 2>/dev/null||true; sleep 1

log "backend"; roslaunch quad_sim hil_backend.launch rviz:=false &>/tmp/diag_backend.log & B=$!
for i in $(seq 1 25); do lsof -i :4560 2>/dev/null|grep -q LISTEN&&break; sleep 1; done; sleep 2
log "PX4"; cd "$PX4_DIR"; PX4_SIM_HOST_ADDR=127.0.0.1 NO_PXH=1 no_sim=1 make px4_sitl none_iris &>/tmp/diag_px4.log & P=$!
for i in $(seq 1 30); do sleep 1; pgrep -f "px4.*none_iris">/dev/null&&{ log "PX4起(${i}s)";break;}; done; sleep 2
log "MAVROS"; rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14580 &>/tmp/diag_mavros.log & M=$!
for i in $(seq 1 30); do sleep 1; rostopic echo /mavros/state -n1 2>/dev/null|grep -q "connected: True"&&{ log "MAVROS连(${i}s)";break;}; done; sleep 2
log "params"; rosservice call /mavros/param/pull '{}'>/dev/null 2>&1; sleep 2
for kv in "SYS_HITL integer:1" "COM_RCL_EXCEPT integer:4" "MPC_XY_VEL_I_ACC real:1.2"; do
  k=${kv% *}; tv=${kv#* }; t=${tv%:*}; v=${tv#*:}
  rosservice call /mavros/param/set "{param_id: \"$k\", value: {$t: $v}}">/dev/null 2>&1 && log "  $k=$v"
done

# EKF 位置估计就绪?
log "===== EKF/位置诊断 ====="
log "local_position/pose:"; timeout 3 rostopic echo /mavros/local_position/pose -n1 2>/dev/null | grep -E '^\s+[xyz]:' | head -3
EKF=$(timeout 3 rostopic echo /diagnostics -n1 2>/dev/null | grep -i "ekf\|local position" | head -2 || echo "")
log "global_position 有效? $(timeout 3 rostopic list 2>/dev/null | grep -c global_position) topics"

log "Step pid_baseline (对照, 发 position setpoint)"; cd "$OFFB_DIR"
PYTHONUNBUFFERED=1 python3 -u pid_baseline.py &>/tmp/diag_pid.log & C=$!
for i in $(seq 1 25); do
  sleep 1
  MODE=$(rostopic echo /mavros/state -n1 2>/dev/null|grep '^mode:'|awk '{print $2}')
  ARM=$(rostopic echo /mavros/state -n1 2>/dev/null|grep '^armed:'|awk '{print $2}')
  log "  ${i}s mode=$MODE armed=$ARM"
  [[ "$MODE" == *OFFBOARD* ]] && { log ">>> pid_baseline 成功进 OFFBOARD!"; break; }
done
log "pid 启动日志尾:"; tail -12 /tmp/diag_pid.log | sed 's/^/[pid] /'
sleep 3
PZ=$(rostopic echo /mavros/local_position/pose -n1 2>/dev/null|grep -E '^\s+z:'|head -1|awk '{print $2}')
log "pid_baseline 高度 z=$PZ (应爬向2.5)"
log "诊断结束"
