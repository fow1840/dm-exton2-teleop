#!/usr/bin/env bash
# =============================================================================
# DM-EXton2 遥操作一键启动（按《天创_真机联调_完整命令清单》T1~T5 顺序）
#   T1 MessageBridge → T2 FK(真机+RViz) → T3 bridge → T4 Puppetry → T5 IK
# 每步等待成功标志后再启动下一步；日志在 $LOGDIR，可随时 tail 查看。
# 停止：./stop_teleop.sh
# =============================================================================
set -u

PKG=/home/tc/DM-EXton2ProductPackage_tianchuang
WS=$PKG/exton_ros2_retargeting_demo
HUMBLE=/opt/ros/humble/setup.bash
LOGDIR=$HOME/桌面/dm-exo/logs
IP_EXO=192.168.61.15        # 外骨骼背包
IP_ROBOT=192.168.61.60      # 双臂机器人控制器
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
export DISPLAY=:0 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority   # RViz 需要桌面

mkdir -p "$LOGDIR"

ok()   { echo "  ✅ $*"; }
fail() { echo "  ❌ $*"; }

# ---------- 等待日志出现成功标志 ----------
wait_log() {                     # $1=logfile $2=pattern(ERE) $3=timeout_s
    local f=$1 pat=$2 t=${3:-25} n=0
    while [ $n -lt $((t * 2)) ]; do
        grep -qE "$pat" "$f" 2>/dev/null && return 0
        sleep 0.5; n=$((n + 1))
    done
    return 1
}

# ---------- 清理上次残留（防自匹配：关键字用[]断开） ----------
cleanup_stale() {
    pkill -f "robot_ik_retargetin[g]"            2>/dev/null
    pkill -x "Puppetr[y]"                        2>/dev/null
    pkill -f "exton_bridg[e].py"                 2>/dev/null
    pkill -f "robot_fk_adapte[r]"                2>/dev/null
    pkill -x "MessageBridg[e]"                   2>/dev/null
    sleep 2
    rm -f /tmp/messagebridge_pub.ipc /tmp/puppetry_pub.ipc
}

cleanup_stale

echo "== DM-EXton2 遥操作一键启动 $(date '+%F %T') =="

# ---------- T1 MessageBridge ----------
LOG=$LOGDIR/T1_messagebridge.log; : > "$LOG"
echo "[T1] MessageBridge 启动中..."
setsid nohup bash -c "
    source $HUMBLE
    cd $PKG/Install/install_ros2/bin/ROS_2 && ./MessageBridge
" >> "$LOG" 2>&1 &
echo $! > "$LOGDIR/T1.pid"
if wait_log "$LOG" "in namespace '/' started" 25; then
    ok "T1 MessageBridge 已启动（连接 $IP_EXO:9899）"
else
    fail "T1 MessageBridge 启动失败，日志尾部："; tail -5 "$LOG"; exit 1
fi
sleep 1

# ---------- T2 FK（真机 + RViz） ----------
LOG=$LOGDIR/T2_fk.log; : > "$LOG"
echo "[T2] FK 真机模式 + RViz 启动中..."
setsid nohup bash -c "
    source $HUMBLE
    source $WS/install/setup.bash
    ros2 launch robot_fk_adapter robot_fk_adapter.launch.py use_sim:=false use_rviz:=true
" >> "$LOG" 2>&1 &
echo $! > "$LOGDIR/T2.pid"
if wait_log "$LOG" "FK adapter active" 30; then
    ok "T2 FK adapter active，RViz 已拉起"
else
    fail "T2 FK 启动失败，日志尾部："; tail -8 "$LOG"; exit 1
fi
sleep 1

# ---------- T3 exton_bridge（连真机控制器） ----------
# 2026-11-06 定版：canfd 流式遥操（movej 走停模式已实测不可用：指令跑飞+保护性故障）；
# use_archive_right_limits=false（档案限位表过时，j5 与真机差 1.22rad，会裁合法位姿）
LOG=$LOGDIR/T3_bridge.log; : > "$LOG"
echo "[T3] exton_bridge 启动中（$IP_ROBOT:30485, movej_canfd, auto_enable:=false, 动态限位关闭档案表）..."
setsid nohup bash -c "
    source $HUMBLE
    python3 $HOME/tcvg/exton_bridge.py --ros-args \
      -p control_mode:=movej_canfd -p move_speed:=15.0 -p auto_enable:=false \
      -p use_archive_right_limits:=false
" >> "$LOG" 2>&1 &
echo $! > "$LOGDIR/T3.pid"
if wait_log "$LOG" "已连接控制器" 20; then
    ok "T3 已连接控制器 $IP_ROBOT:30485（canfd 流式，sent=0，未下发任何指令）"
else
    # 机械臂未开机时不阻断整体启动
    echo "  ⚠️  T3 暂未连上控制器（机械臂未开/JuxieLinker 未起？），后台继续重试，其余步骤照常"
fi
sleep 1

# ---------- T4 Puppetry ----------
LOG=$LOGDIR/T4_puppetry.log; : > "$LOG"
echo "[T4] Puppetry 启动中..."
setsid nohup bash -c "
    source $HUMBLE
    cd $PKG/Install/install_ros2/bin/Puppetry && ./Puppetry
" >> "$LOG" 2>&1 &
echo $! > "$LOGDIR/T4.pid"
if wait_log "$LOG" "Puppetry initialized" 20; then
    ok "T4 Puppetry initialized"
else
    fail "T4 Puppetry 启动失败，日志尾部："; tail -5 "$LOG"; exit 1
fi
sleep 1

# ---------- T5 IK ----------
LOG=$LOGDIR/T5_ik.log; : > "$LOG"
echo "[T5] IK retargeting 启动中（pink venv）..."
setsid nohup bash -c "
    source $WS/pink/bin/activate
    source $HUMBLE
    source $WS/install/setup.bash
    ros2 launch robot_ik_retargeting ik_retargeting.launch.py
" >> "$LOG" 2>&1 &
echo $! > "$LOGDIR/T5.pid"
if wait_log "$LOG" "Reference Frame" 30; then
    ok "T5 IK active"
else
    fail "T5 IK 启动失败，日志尾部："; tail -8 "$LOG"; exit 1
fi

# ---------- 完成汇总 ----------
echo ""
echo "== 全部启动完成 $(date '+%F %T') =="
echo "  试控：①不按 Clutch 真机应不动  ②按住 Clutch 极慢小范围动手臂  ③松开即停"
echo "  日志：tail -f $LOGDIR/T3_bridge.log   停止：$HOME/桌面/dm-exo/stop_teleop.sh"
