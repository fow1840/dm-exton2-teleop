#!/usr/bin/env bash
# 恢复 exton_bridge（T3）：movej_canfd 流式模式（2026-11-06 定版），
# 档案限位表关闭（过时，j5 与真机差 1.22rad 会裁合法位姿），与 start_teleop.sh 一致。
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
      -p control_mode:=movej_canfd -p move_speed:=15.0 -p auto_enable:=false \
      -p use_archive_right_limits:=false
" >> "$LOG" 2>&1 &
echo $! > "$LOGDIR/T3.pid"
n=0
while [ $n -lt 40 ]; do
    grep -q "已连接控制器" "$LOG" 2>/dev/null && { echo "✅ bridge 已恢复并连接控制器（canfd 模式）"; exit 0; }
    sleep 0.5; n=$((n + 1))
done
echo "❌ bridge 未连上（JuxieLinker 未起？bridge 进程在后台继续重试），日志尾部："; tail -5 "$LOG"
exit 1
