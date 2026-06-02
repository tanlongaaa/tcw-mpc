#!/bin/bash
# ──────────────────────────────────────────────────────────
# start_sitl.sh — 一键启动 PX4 SITL + Gazebo + MAVROS + 风场
#
# 三阶段启动:
#   1. Gazebo + 空世界
#   2. PX4 (setarch -R 禁用 ASLR, 绕开 segfault)
#   3. MAVROS + 风场 + 参数
#
# 用法:
#   ./start_sitl.sh                  # 默认
#   ./start_sitl.sh --no-wind        # 不启风场
#   ./start_sitl.sh --headless       # 无 GUI
#   ./start_sitl.sh --no-mavros      # 不启 MAVROS
#
# 按 Ctrl+C 自动清理
# ──────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── 参数 ────────────────────────────────────────────
HEADLESS=false; NO_WIND=false; NO_MAVROS=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless)  HEADLESS=true;  shift ;;
        --no-wind)   NO_WIND=true;   shift ;;
        --no-mavros) NO_MAVROS=true; shift ;;
        *) echo "未知: $1"; exit 1 ;;
    esac
done

source "$SCRIPT_DIR/env.sh"

# ── PX4 启动参数 ─────────────────────────────────────
# env.sh 已设置了 PX4_ROOT, PX4_BUILD; 这里只用脚本局部变量
export PX4_SIM_MODEL=iris
export PX4_ESTIMATOR=ekf2
export PATH="$PX4_BUILD:$PATH"          # rcS 需要 px4-alias.sh

PX4_BIN="$PX4_BUILD/bin/px4"
PX4_ROOTFS="$PX4_BUILD/etc"             # rootfs = build/px4_sitl_default/etc
PX4_RCS="etc/init.d-posix/rcS"          # 相对于 rootfs (sitl_run.sh 同款)
PX4_INSTANCE=0
PX4_WORKDIR="/tmp/px4_sitl_$$"           # ★ 洁净临时目录 (PX4 需在其中创建 etc→rootfs 软链)
IRIS_SDF="$PX4_ROOT/Tools/sitl_gazebo/models/iris/iris.sdf"
EMPTY_WORLD="$PX4_ROOT/Tools/sitl_gazebo/worlds/empty.world"

SIM_PORT=$((4560 + PX4_INSTANCE))
OFFBOARD_PORT_LOCAL=$((14580 + PX4_INSTANCE))
OFFBOARD_PORT_REMOTE=$((14540 + PX4_INSTANCE))

# ── 清理函数 ────────────────────────────────────────
GAZEBO_PID=""; PX4_PID=""; MAVROS_PID=""; WIND_PID=""

cleanup() {
    echo ""
    echo "🛑 正在关闭..."
    [ -n "$WIND_PID" ]   && kill $WIND_PID 2>/dev/null   && echo "   ✅ 风场"
    [ -n "$MAVROS_PID" ] && kill $MAVROS_PID 2>/dev/null && echo "   ✅ MAVROS"
    [ -n "$PX4_PID" ]    && kill $PX4_PID 2>/dev/null    && echo "   ✅ PX4"
    [ -n "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null && echo "   ✅ Gazebo"
    # 彻底清理
    kill $(ps aux | grep 'px4.*bin/px4' | grep -v grep | awk '{print $2}') 2>/dev/null || true
    kill $(ps aux | grep gzserver | grep -v grep | awk '{print $2}') 2>/dev/null || true
    kill $(ps aux | grep gzclient | grep -v grep | awk '{print $2}') 2>/dev/null || true
    rm -rf "$PX4_WORKDIR" 2>/dev/null
    echo "🏁 全部停止"
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── 1. 残留清理 ─────────────────────────────────────
echo "   🧹 清理残留..."
kill $(ps aux | grep 'px4.*bin/px4' | grep -v grep | awk '{print $2}') 2>/dev/null || true
kill $(ps aux | grep gzserver | grep -v grep | awk '{print $2}') 2>/dev/null || true
kill $(ps aux | grep gzclient | grep -v grep | awk '{print $2}') 2>/dev/null || true
sleep 2

# ── 2. 启动 Gazebo ──────────────────────────────────
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🚀 启动仿真环境"
echo "     GUI: $($HEADLESS && echo '关闭' || echo '开启')"
echo "     风场: $($NO_WIND && echo '关闭' || echo '开启')"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

GUI_ARG="gui:=true"
$HEADLESS && GUI_ARG="gui:=false"

echo "   🌍 启动 Gazebo..."
roslaunch gazebo_ros empty_world.launch "$GUI_ARG" world_name:="$EMPTY_WORLD" > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!

# 等待 Gazebo
echo -n "   ⏳ 等待 Gazebo"
for i in $(seq 1 60); do
    if rostopic list 2>/dev/null | grep -q '/clock'; then
        echo " ✅ (${i}s)"; break
    fi
    [ $i -ge 60 ] && { echo " ❌ 超时"; cleanup; exit 1; }
    echo -n "."; sleep 1
done

# ── 3. 生成 iris.sdf ────────────────────────────────
if [ ! -f "$IRIS_SDF" ]; then
    echo "   ⚙  生成 iris.sdf..."
    cd "$PX4_ROOT" && python3 Tools/sitl_gazebo/scripts/jinja_gen.py 2>/dev/null || true
    cd "$SCRIPT_DIR"
fi

# ── 4. 启动 PX4 先 (TCP 4560 listener 就绪后 spawn iris) ─
echo "   🚁 启动 PX4 (ASLR=OFF, instance=$PX4_INSTANCE)..."

# ★ 仿照 sitl_run.sh: PX4 在 CWD 创建 etc→rootfs 软链
#    必须用洁净临时目录，否则会和 PX4_ROOT 已有的 etc 软链冲突
mkdir -p "$PX4_WORKDIR"
cd "$PX4_WORKDIR"

# 关键修复: setarch x86_64 -R 禁用 ASLR + -d daemon 模式
#    避开 PX4 v1.13.3 在 Gazebo 11 + Ubuntu 20.04 下的 segfault
#    rootfs=$PX4_BUILD/etc,  -s etc/init.d-posix/rcS
setarch x86_64 -R "$PX4_BIN" -i "$PX4_INSTANCE" "$PX4_ROOTFS" -s "$PX4_RCS" -d > /tmp/px4.log 2>&1 &
PX4_PID=$!

# 等待 PX4 启动到 TCP listener 就绪
echo -n "   ⏳ 等待 PX4 TCP listener"
for i in $(seq 1 30); do
    if grep -q "Waiting for simulator" /tmp/px4.log 2>/dev/null; then
        echo " ✅ (${i}s)"; break
    fi
    echo -n "."; sleep 1
done

# ── 5. Spawn iris (mavlink_interface 连接 PX4 TCP 4560) ──
echo "   🚁 Spawn iris 模型..."
rosrun gazebo_ros spawn_model -sdf -file "$IRIS_SDF" -model iris \
    -x 0 -y 0 -z 0.5 > /tmp/spawn.log 2>&1
if grep -q "Successfully" /tmp/spawn.log 2>/dev/null; then
    echo "   ✅ iris 已 spawn"
else
    echo "   ⚠️  spawn 可能失败"
fi
sleep 2

# ── 6. 等待 Simulator connected + PX4 初始化 ────────
echo -n "   ⏳ 等待 Simulator 连接 + 初始化"
for i in $(seq 1 30); do
    if grep -q "Simulator connected" /tmp/px4.log 2>/dev/null; then
        echo " ✅ (${i}s)"; break
    fi
    if ! kill -0 $PX4_PID 2>/dev/null; then
        echo " ❌ PX4 已死!"
        grep "Segmentation\|ERROR" /tmp/px4.log | tail -5
        cleanup; exit 1
    fi
    echo -n "."; sleep 1
done

# 额外等待传感器/EKF/Logger 初始化
echo "   ⏳ 传感器 + EKF 初始化 (10s)..."
sleep 10

if ! kill -0 $PX4_PID 2>/dev/null; then
    echo "   ❌ PX4 已死!"
    grep "Segmentation\|ERROR\|FATAL" /tmp/px4.log | tail -5
    cleanup; exit 1
fi

# ── 7. 启动 MAVROS ──────────────────────────────────
if $NO_MAVROS; then
    echo "   🔌 MAVROS: 跳过"
else
    echo "   🔌 启动 MAVROS..."
    roslaunch mavros px4.launch \
        fcu_url:=udp://:${OFFBOARD_PORT_REMOTE}@localhost:${OFFBOARD_PORT_LOCAL} \
        gcs_url:= > /tmp/mavros.log 2>&1 &
    MAVROS_PID=$!

    echo -n "   ⏳ 等待 MAVROS connected"
    for i in $(seq 1 60); do
        if ! kill -0 $PX4_PID 2>/dev/null; then
            echo " ❌ PX4 在 MAVROS 连接时崩溃!"
            grep "Segmentation\|ERROR" /tmp/px4.log | tail -5
            cleanup; exit 1
        fi
        if rostopic echo -n1 /mavros/state 2>/dev/null | grep -q 'connected: True'; then
            echo " ✅ (${i}s)"; break
        fi
        [ $i -ge 60 ] && { echo " ⚠ 超时, 继续..."; break; }
        echo -n "."; sleep 1
    done
    echo "   ✅ MAVROS 就绪"
fi

# ── 8. 设置 PX4 参数 ────────────────────────────────
if ! $NO_MAVROS; then
    echo "   ⚙  设置 PX4 参数..."
    rosrun mavros mavparam set COM_RC_IN_MODE 1 2>/dev/null || echo "      ⚠ COM_RC_IN_MODE"
    rosrun mavros mavparam set COM_ARM_WO_GPS 1  2>/dev/null || echo "      ⚠ COM_ARM_WO_GPS"
    rosrun mavros mavparam set NAV_RCL_ACT 0      2>/dev/null || echo "      ⚠ NAV_RCL_ACT"
    echo "   ✅ 参数已设置"
fi

# ── 9. 启动风场 ────────────────────────────────────
if $NO_WIND; then
    echo "   🌬️  风场: 跳过"
else
    echo "   🌪️  启动极端湍流风场..."
    python3 -u "$SCRIPT_DIR/wind_field.py" &
    WIND_PID=$!
    sleep 2
fi

# ── 10. 就绪 ────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ✅ 环境全部就绪!"
echo ""
echo "  控制脚本:"
echo "    rosrun offboard_test mpc_node.py"
echo "    rosrun offboard_test pid_baseline.py"
echo ""
echo "  日志: /tmp/px4.log  /tmp/mavros.log  /tmp/gazebo.log"
echo "  按 Ctrl+C 停止全部"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# ── 11. 持续监控 PX4 ───────────────────────────────
while kill -0 $PX4_PID 2>/dev/null; do
    sleep 5
done
echo "⚠️  PX4 进程退出, 正在清理..."
cleanup
