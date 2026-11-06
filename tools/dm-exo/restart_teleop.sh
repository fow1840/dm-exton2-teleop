#!/usr/bin/env bash
# DM-EXton2 遥操作一键重启：清场 -> 启动 -> 运行态终检。
# 只负责软件链路；机器人使能保留人工操作（auto_enable=false）。
set -u

BASE="$HOME/桌面/dm-exo"
LOG="$BASE/logs/T3_bridge.log"
ROBOT_IP=192.168.61.60

echo "== 遥操作一键重启 $(date '+%F %T') =="
"$BASE/stop_teleop.sh" || exit 1
"$BASE/start_teleop.sh" || {
    echo "❌ 软件链启动失败，请看 $LOG"
    exit 1
}

sleep 3
if ! pgrep -f "exton_bridg[e].py" >/dev/null; then
    echo "❌ T3 bridge 已退出"
    tail -20 "$LOG"
    exit 1
fi
if ! ss -Htnp state established 2>/dev/null | grep -Fq "$ROBOT_IP:30485"; then
    echo "❌ T3 存活但 30485 未连接"
    tail -20 "$LOG"
    exit 1
fi
if ! grep -qE "cmd_rx=[1-9][0-9]*" "$LOG"; then
    echo "❌ T3 已连接但没有收到 joint_commands"
    tail -20 "$LOG"
    exit 1
fi

LAST_STATUS=$(grep "状态: connected" "$LOG" | tail -1)
echo "✅ 软件链路已启动：$LAST_STATUS"
echo "⚠️ 机器人使能不会由脚本远程执行（auto_enable=false）。"
echo "   如果按离合后日志出现 program_run_failed：不是 bridge 没起，"
echo "   是控制器未 enable、处于 fault，或电机/CAN 层拒绝；请松开离合并在 5566 检查。"
