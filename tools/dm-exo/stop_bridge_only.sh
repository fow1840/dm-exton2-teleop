#!/usr/bin/env bash
# 只停 exton_bridge（T3），其余组件（MessageBridge/FK/Puppetry/IK）不动。
# 用于 joint_probe.py 直连测试；测完用 start_bridge_only.sh 恢复。
set -u
LOGDIR=$HOME/桌面/dm-exo/logs
pkill -TERM -f "exton_bridg[e].py" 2>/dev/null
n=0
while [ $n -lt 16 ]; do
    pgrep -f "exton_bridg[e].py" >/dev/null 2>&1 || { echo "✅ bridge 已停止，30485 已让出"; exit 0; }
    sleep 0.5; n=$((n + 1))
done
pkill -KILL -f "exton_bridg[e].py" 2>/dev/null
sleep 1
echo "✅ bridge 已强停"
