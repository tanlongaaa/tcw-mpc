#!/bin/bash
# ==============================================================================
# smoke_mpc_wind.sh — MPC + 极端湍流 HIL 验证 (通路B)
# ==============================================================================
# 在 smoke_mpc_hil.sh 基础上加 wind_field.py --extreme (固定种子可复现)。
# 流程: 先无风起飞悬停稳定 (WARMUP_HOLD 秒), 再注入极端湍流 (WIND_HOLD 秒),
#       记录抗扰指标: 位置 RMSE / 最大漂移 / 能耗。
# 风经 backend 注入 plant (--no-wrench: wind_field 只发布风数据, backend 算气动力)。
# MPC 同时订阅 /wind_field/velocity 做 TCWP 风预测。
set -eo pipefail
PX4_DIR="/home/tan/Desktop/px4rl/PX4-Autopilot"
PROJECT_DIR="/home/tan/catkin_ws/src/px4-ros-6dof_project-pid-eso-"
QUAD_SIM_DIR="$PROJECT_DIR/quad_sim"
OFFB_DIR="/home/tan/catkin_ws/src/offboard_test/scripts"
WARMUP_HOLD=${WARMUP_HOLD:-25}   # 无风稳定时间
WIND_HOLD=${WIND_HOLD:-40}       # 极端湍流持续时间
TARGET_Z=2.5
RVIZ=${RVIZ:-true}
SEED=${SEED:-42}
U_REF=${U_REF:-12}               # 平均风速 m/s
log(){ echo "[$(date '+%H:%M:%S')] $*"; }

BACKEND_PID=""; PX4_PID=""; MAVROS_PID=""; MPC_PID=""; WIND_PID=""
cleanup(){
    log "清理..."
    for pid in $WIND_PID $MPC_PID $MAVROS_PID $PX4_PID $BACKEND_PID; do
        [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
    done
    pkill -f "wind_field.py" 2>/dev/null || true
    pkill -f "pid_baseline.py" 2>/dev/null || true
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
log "roscore OK | seed=$SEED u_ref=${U_REF}m/s"

pkill -f "backend_main.py" 2>/dev/null || true
pkill -f "mavros_node" 2>/dev/null || true
pkill -f "pid_baseline.py" 2>/dev/null || true
pkill -f "wind_field.py" 2>/dev/null || true
sleep 1

log "Step1 backend (plant_6dof) rviz=$RVIZ"
roslaunch quad_sim hil_backend.launch rviz:=$RVIZ &>/tmp/wind_backend.log &
BACKEND_PID=$!
for i in $(seq 1 25); do lsof -i :4560 2>/dev/null | grep -q LISTEN && { log "backend 就绪"; break; }; sleep 1; done
sleep 2

log "Step2 PX4 SITL none_iris"
cd "$PX4_DIR"
PX4_SIM_HOST_ADDR=127.0.0.1 NO_PXH=1 no_sim=1 make px4_sitl none_iris &>/tmp/wind_px4.log &
PX4_PID=$!
for i in $(seq 1 30); do sleep 1; pgrep -f "px4.*none_iris" >/dev/null 2>&1 && { log "PX4 起 (${i}s)"; break; }; done
sleep 2

log "Step3 MAVROS (thrust_scaling=1.0)"
rosrun mavros mavros_node _fcu_url:=udp://:14540@127.0.0.1:14580 _setpoint_raw/thrust_scaling:=1.0 &>/tmp/wind_mavros.log &
MAVROS_PID=$!
for i in $(seq 1 30); do sleep 1; rostopic echo /mavros/state -n1 2>/dev/null | grep -q "connected: True" && { log "MAVROS 连上"; break; }; done
sleep 2

log "Step4 PX4 参数"
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

log "Step5 pid_baseline.py (目标 0,0,${TARGET_Z})"
cd "$OFFB_DIR"
PYTHONUNBUFFERED=1 rosrun offboard_test pid_baseline.py &>/tmp/wind_pid.log &
MPC_PID=$!
for i in $(seq 1 30); do sleep 1; kill -0 "$MPC_PID" 2>/dev/null || { log "ERR mpc_node 退出"; tail -25 /tmp/wind_pid.log; exit 1; }; rostopic echo /mavros/state -n1 2>/dev/null | grep -q "armed: True" && { log "已解锁 (${i}s)"; break; }; done

log "Step6 无风稳定 ${WARMUP_HOLD}s ..."
for ((t=5; t<=WARMUP_HOLD; t+=5)); do
    sleep 5
    PZ=$(rostopic echo /mavros/local_position/pose -n1 2>/dev/null | grep -E '^\s+z:' | head -1 | awk '{print $2}')
    log "  [无风] t=${t}s z=${PZ:-?}"
done

log "Step7 🌪️ 注入极端湍流 (--extreme seed=$SEED u_ref=${U_REF}) ${WIND_HOLD}s ..."
python3 "$OFFB_DIR/wind_field.py" --extreme --seed "$SEED" --u-ref "$U_REF" --rate 20 &>/tmp/wind_field.log &
WIND_PID=$!
sleep 2
rostopic echo /wind_field/velocity -n1 2>/dev/null | grep -A3 "vector:" | head -4 || log "  (风话题暂无数据)"

# 抗扰采样: 记录 x/y/z 漂移
python3 - "$WIND_HOLD" <<'PYEOF' &
import rospy, sys, time, math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
dur=float(sys.argv[1])
rospy.init_node('wind_probe', anonymous=True)
st={'p':None,'w':None}
rospy.Subscriber('/mavros/local_position/odom', Odometry, lambda m: st.update(p=(m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z)))
rospy.Subscriber('/wind_field/velocity', Vector3Stamped, lambda m: st.update(w=(m.vector.x,m.vector.y,m.vector.z)))
t0=time.time(); xs=[];ys=[];zs=[]; wmax=0
tgt=(0,0,2.5)
while time.time()-t0<dur and not rospy.is_shutdown():
    if st['p']:
        x,y,z=st['p']; xs.append(x);ys.append(y);zs.append(z)
        if st['w']: wmax=max(wmax, math.sqrt(sum(c*c for c in st['w'])))
    time.sleep(0.05)
import statistics as S
def rmse(a,c): return math.sqrt(sum((v-c)**2 for v in a)/len(a)) if a else float('nan')
exy=[math.sqrt((x)**2+(y)**2) for x,y in zip(xs,ys)]
print('\n========== 极端湍流抗扰指标 ==========')
print(f'采样 {len(xs)} 点, 最大风速 {wmax:.1f} m/s')
print(f'XY 平面: RMSE={rmse(exy,0):.3f}m  最大漂移={max(exy) if exy else 0:.3f}m')
print(f'Z 高度:  RMSE={rmse(zs,2.5):.3f}m  范围=[{min(zs):.2f},{max(zs):.2f}]' if zs else 'no z')
print('=====================================')
PYEOF
PROBE=$!
wait $PROBE 2>/dev/null

log "===== PID 末段日志 ====="
tail -6 /tmp/wind_pid.log | sed 's/^/[pid] /'
LATEST_CSV=$(ls -t "$OFFB_DIR"/mpc_log_*.csv 2>/dev/null | head -1)
[[ -n "$LATEST_CSV" ]] && log "CSV: $LATEST_CSV ($(wc -l <"$LATEST_CSV") 行)"
log "极端湍流验证结束, 自动清理"
