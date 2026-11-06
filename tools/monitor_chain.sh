#!/bin/bash
# 全链路监控：每 2s 采样一行，记录「好→坏」切换时刻各层状态
# 采样层：主机可达 → TCP 30485 → T1/T3/T5 进程 → bridge 收发/拒绝计数
# 日志：~/桌面/dm-exo/logs/monitor_chain.log（只追加，不动现场其他文件）
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
LOG=~/桌面/dm-exo/logs/monitor_chain.log

echo "# start $(date '+%F %T')" >> "$LOG"
while true; do
  ts=$(date '+%F %T')
  ping -c1 -W1 192.168.61.15 >/dev/null 2>&1; p15=$?
  ping -c1 -W1 192.168.61.60 >/dev/null 2>&1; p60=$?
  tcp30485=$(ss -tn state established '( sport = :30485 or dport = :30485 )' 2>/dev/null | tail -n +2 | wc -l)
  t1=$(pgrep -fc "^./MessageBridge$|/MessageBridge" 2>/dev/null)
  t3=$(pgrep -fc "exton_bridg""e.py" 2>/dev/null)
  t5=$(pgrep -fc "robot_ik_retargetin[g]" 2>/dev/null)
  # bridge 最新状态行的关键计数（sent / reject / cmd_rx / ik_moved / fb_age）
  stats=$(grep "状态:" ~/桌面/dm-exo/logs/T3_bridge.log 2>/dev/null | tail -1 \
    | sed 's/.*connected=/connected=/; s/\[INFO\].*//')
  echo "$ts ping15=$p15 ping60=$p60 tcp30485=$tcp30485 T1=$t1 T3=$t3 T5=$t5 $stats" >> "$LOG"
  sleep 2
done
