#!/usr/bin/env bash
# 恢复 exton_bridge（T3）：movej_canfd 流式模式（2026-11-06 定版），
# 档案限位表关闭（过时，j5 与真机差 1.22rad 会裁合法位姿），与 start_teleop.sh 一致。
# 注意：bridge 启动后会用 get_joint_position 同步真机反馈，同步前不会下发。
set -u
PKG=/home/tc/DM-EXton2ProductPackage_tianchuang
WS=$PKG/exton_ros2_retargeting_demo
HUMBLE=/opt/ros/humble/setup.bash
LOGDIR=$HOME/桌面/dm-exo/logs
IP_EXO=192.168.61.15
IP_ROBOT=192.168.61.60
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0

echo "[PRE] 检查网络与 30485 单连接 ..."
ping -c 3 -W 1 "$IP_EXO" >/dev/null 2>&1 || { echo "❌ $IP_EXO 不可达"; exit 1; }
ping -c 3 -W 1 "$IP_ROBOT" >/dev/null 2>&1 || { echo "❌ $IP_ROBOT 不可达"; exit 1; }

# 先清理失效 bridge，再判断是否有图形界面/其他客户端占用单连接。
pkill -TERM -f "exton_bridg[e].py" 2>/dev/null
for _n in 1 2 3 4 5 6 7 8 9 10; do
    pgrep -f "exton_bridg[e].py" >/dev/null 2>&1 || break
    sleep 0.2
done
if ss -Htnp state established 2>/dev/null | grep -Fq "$IP_ROBOT:30485"; then
    echo "❌ 30485 已被占用，请关闭机械臂图形界面/其他客户端"
    ss -Htnp state established 2>/dev/null | grep -F "$IP_ROBOT:30485" || true
    exit 1
fi

LOG=$LOGDIR/T3_bridge.log; : > "$LOG"
setsid nohup bash -c "
    source $HUMBLE
    exec python3 $HOME/tcvg/exton_bridge.py --ros-args \
      -p control_mode:=movej_canfd -p move_speed:=15.0 -p auto_enable:=false \
      -p use_archive_right_limits:=false
" >> "$LOG" 2>&1 &
bridge_pid=$!
echo "$bridge_pid" > "$LOGDIR/T3.pid"
n=0
while [ $n -lt 40 ]; do
    kill -0 "$bridge_pid" 2>/dev/null || { echo "❌ bridge 进程已退出"; tail -10 "$LOG"; exit 1; }
    if grep -q "已连接控制器" "$LOG" 2>/dev/null \
            && grep -q "已用 get_joint_position 同步" "$LOG" 2>/dev/null \
            && ss -Htnp state established 2>/dev/null | grep -Fq "$IP_ROBOT:30485"; then
        echo "✅ bridge 已恢复：网络正常、30485 ESTAB、反馈已同步（canfd）"
        exit 0
    fi
    sleep 0.5; n=$((n + 1))
done
kill -TERM "$bridge_pid" 2>/dev/null || true
echo "❌ bridge 未通过连接/反馈同步门禁，已停止，日志尾部："; tail -10 "$LOG"
exit 1
