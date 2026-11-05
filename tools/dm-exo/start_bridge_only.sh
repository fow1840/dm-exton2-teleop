#!/usr/bin/env bash
# 恢复 exton_bridge（T3）：movej 模式 v=15，与 start_teleop.sh 一致。
# 注意：bridge 启动后会用 get_joint_position 同步真机反馈，同步前不会下发。
set -u
PKG=/home/tc/DM-EXton2ProductPackage_tianchuang
WS=$PKG/exton_ros2_retargeting_demo
HUMBLE=/opt/ros/humble/setup.bash
LOGDIR=$HOME/桌面/dm-exo/logs
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0

LOG=$LOGDIR/T3_bridge.log; : > "$LOG"
setsid nohup bash -c "
    source $HUMBLE
    python3 $HOME/tcvg/exton_bridge.py --ros-args \
      -p control_mode:=movej -p move_speed:=15.0 -p auto_enable:=false
" >> "$LOG" 2>&1 &
echo $! > "$LOGDIR/T3.pid"
n=0
while [ $n -lt 40 ]; do
    grep -q "已连接控制器" "$LOG" 2>/dev/null && { echo "✅ bridge 已恢复并连接控制器"; exit 0; }
    sleep 0.5; n=$((n + 1))
done
echo "❌ bridge 未连上，日志尾部："; tail -5 "$LOG"
exit 1
