#!/usr/bin/env bash
# =============================================================================
# DM-EXton2 遥操作一键停止
# 顺序（与清单一致）：IK → Puppetry → bridge → FK → MessageBridge
# =============================================================================
set -u
LOGDIR=$HOME/桌面/dm-exo/logs

stop_one() {                     # $1=名称 $2=pid文件 $3=pgrep防自匹配模式
    local name=$1 pidf=$2 pat=$3
    local pid=""
    [ -f "$pidf" ] && pid=$(cat "$pidf" 2>/dev/null)
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    fi
    pkill -TERM -f "$pat" 2>/dev/null
}

wait_gone() {                    # $1=pgrep防自匹配模式 $2=timeout_s
    local pat=$1 t=${2:-8} n=0
    while [ $n -lt $((t * 2)) ]; do
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
        sleep 0.5; n=$((n + 1))
    done
    return 1
}

echo "== DM-EXton2 遥操作一键停止 $(date '+%F %T') =="

echo "[1/5] 停 IK ..."
stop_one  IK    "$LOGDIR/T5.pid" "robot_ik_retargetin[g]"
wait_gone "robot_ik_retargetin[g]" || pkill -KILL -f "robot_ik_retargetin[g]"

echo "[2/5] 停 Puppetry ..."
stop_one  Puppetry "$LOGDIR/T4.pid" "Puppetr[y]"
wait_gone "^/.*Puppetr[y]" || pkill -KILL -x "Puppetr[y]"

echo "[3/5] 停 bridge ..."
stop_one  bridge "$LOGDIR/T3.pid" "exton_bridg[e].py"
wait_gone "exton_bridg[e].py" || pkill -KILL -f "exton_bridg[e].py"

echo "[4/5] 停 FK + RViz ..."
stop_one  FK    "$LOGDIR/T2.pid" "robot_fk_adapte[r]"
wait_gone "robot_fk_adapte[r]" || pkill -KILL -f "robot_fk_adapte[r]"
# 残留的 robot_state_publisher 会留下旧 TF 树（幽灵坐标系：曾致 FK adapter /
# Puppetry 参考系错乱、按下离合瞬跳），随 T2 一并清理
pkill -TERM -f "robot_state_publishe[r]" 2>/dev/null

echo "[5/5] 停 MessageBridge ..."
stop_one  MB    "$LOGDIR/T1.pid" "MessageBridg[e]"
wait_gone "^./MessageBridg[e]" || pkill -KILL -x "MessageBridg[e]"

rm -f /tmp/messagebridge_pub.ipc /tmp/puppetry_pub.ipc
rm -f "$LOGDIR"/T?.pid

# ---- 终检 ----
sleep 1
LEFT=$(pgrep -af "MessageBridg[e]|Puppetr[y]|exton_bridg[e]|robot_fk_adapte[r]|ik_nod[e]|rviz[2]|robot_state_publishe[r]" | grep -v "bash -c" || true)
if [ -z "$LEFT" ]; then
    echo "✅ 全部已停止，IPC 已清理"
else
    echo "⚠️ 仍有残留："; echo "$LEFT"
fi
