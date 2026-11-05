# AGENTS.md —— 接手须知（给下一个 AI）

本文件是操作手册，不是背景介绍。背景见 [HANDOFF.md](./HANDOFF.md)，目录说明见 [README.md](./README.md)。

## 0. 接手流程（先做这个，按顺序）

1. 读 `HANDOFF.md` §四（当前阻塞项）→ 读 `TASKS.md`（任务台账，唯一权威）
2. 执行 **T01 现场状态再核对**：上一份快照是 2026-11-05 21:42，现场可能已变。
   核对结果与快照不一致时，先改 `TASKS.md` §一 再干活。
3. 认领任务：把对应任务的 `状态` 从 `TODO` 改成 `DOING`，写上你的名字/日期
4. 干活 → 每完成一个动作就往 `WORKLOG.md` 追加一条（用文件里的模板，必须带证据原文）
5. 收工时：`TASKS.md` 改状态，`WORKLOG.md` 补最后一条，结论有变化就回写 `HANDOFF.md`

**落账硬规则**

- 证据只认原文：日志行、计数、md5、包名。转述不算证据。
- 推测必须标「未验证」，不许写成结论。
- 任务被推翻不要删，状态改 `DEAD` 并写明推翻依据（避免下一手重复踩）。
- `TASKS.md` 改状态，`WORKLOG.md` 只追加；两者不要混写。
- 每次重启进程、改动远端文件，都要在 WORKLOG 的「现场变更」里写清楚，否则下一个人不知道现场被谁动过。

## 1. 你的第一个任务

真机现在**不跟手**：控制器对每一条 movej 都回 `program_run_failed`。
根因高置信是 bridge 以 50Hz 连发 movej、程序重叠被拒（详见 HANDOFF §四），
但**尚未在真机上验证**（松开离合时 bridge 不下发命令，所以「0 拒绝」是假象）。

任务编号见 `TASKS.md`。到现场后先 **T01**，再 **T02**（真机验证，需操作员按住离合）；
人手不够时跳过 T02，直接做 **T03**（`tools/exton_bridge.py` 第 248 行 `_tx_loop` 加 lock-step：
发一帧 → 等该命令带 `status` 的应答 → 再发下一帧，300ms 超时兜底；`get_joint_position` 的应答不能解锁）。

不要在没有新证据的情况下推翻 HANDOFF 的根因结论，也不要先动 `movej_canfd` 方案（有 `-100` 占位轴被拒的坑）。

## 2. 访问方式

```bash
# 跳板 → tc 主机（若 127.0.0.1:2222 没在监听，先建隧道，见 HANDOFF §零）
sshpass -p ' ' ssh -p 2222 tc@127.0.0.1
```

- 远端代码真身：`/home/tc/DM-EXton2ProductPackage_tianchuang/exton_ros2_retargeting_demo/`
- 远端 bridge 真身：`/home/tc/tcvg/exton_bridge.py`
- 远端脚本：`/home/tc/桌面/dm-exo/`
- 本仓库是**只读快照 + 工作副本**，远端才是运行时环境

同步（改完必须推到远端才生效）：

```bash
# 本地 → 远端（改过的单个文件）
sshpass -p ' ' scp -P 2222 \
  ros2_ws/src/robot_ik_retargeting/robot_ik_retargeting/ik_node.py \
  'tc@127.0.0.1:/home/tc/DM-EXton2ProductPackage_tianchuang/exton_ros2_retargeting_demo/src/robot_ik_retargeting/robot_ik_retargeting/'

# 本地 → 远端（bridge）
sshpass -p ' ' scp -P 2222 tools/exton_bridge.py tc@127.0.0.1:/home/tc/tcvg/exton_bridge.py

# 远端 → 本地（拿回现场最新代码/日志尾巴）
sshpass -p ' ' rsync -a -e "ssh -p 2222" \
  tc@127.0.0.1:/home/tc/tcvg/exton_bridge.py tools/
```

**推文件前先备份远端原文件**（`cp xxx.py xxx.py.bak_$(date +%m%d_%H%M)`）。

## 3. 运行 / 重启

```bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0

# 全链路
~/桌面/dm-exo/start_teleop.sh
~/桌面/dm-exo/stop_teleop.sh

# 只重启 T3（改 bridge 后）
pgrep -f "exton_bridg""e.py" | xargs -r kill
cd /home/tc && source /opt/ros/humble/setup.bash
setsid nohup python3 /home/tc/tcvg/exton_bridge.py --ros-args \
  -p control_mode:=movej -p move_speed:=15.0 -p auto_enable:=false \
  >> ~/桌面/dm-exo/logs/T3_bridge.log 2>&1 &

# 只重启 T5（改 ik_node.py 后，硬链接，无需 colcon build）
pkill -f "robot_ik_retargetin[g]"
setsid nohup bash -c "source /home/tc/.../exton_ros2_retargeting_demo/pink/bin/activate && \
  source /opt/ros/humble/setup.bash && \
  source /home/tc/.../exton_ros2_retargeting_demo/install/setup.bash && \
  ros2 launch robot_ik_retargeting ik_retargeting.launch.py" \
  >> ~/桌面/dm-exo/logs/T5_ik.log 2>&1 &
```

## 4. 硬性禁忌（都踩过）

1. **不要写 `pkill -f "xxx.py"` 同时在同一条命令里出现 `xxx.py`** —— 会杀掉自己的 shell。
   用 `pgrep -f "exton_bridg""e.py"` 这种拼接，或按 PID 杀。
2. **不要 `pkill -9` 打 ros2 launch** —— 会孤儿化 robot_state_publisher，制造幽灵 TF 树，
   FK 输出会整体错位（差一个轴置换 + 1.24m），排半天查不出来。
3. **TCP 30485 是单连接**：跑 `joint_probe.py` 前必须先停 T3 bridge。
4. **TF static 是锁存的**：清掉错误的 TF 发布者后，FK adapter 必须重启。
5. **别碰 `move_speed` / `auto_enable`**：`auto_enable:=false` 是安全设定，远程使能需人工在示教器确认。
6. **离核实机时手不要扶在关节上**，J7 现在停在 ±170° 限位附近。

## 5. 怎么判断改对了

不许只看「进程活着 / 日志没报错」。按下列证据链验证：

```bash
# ① bridge 是否真的在发（sent 要增长，且不再是 nodelta）
grep -E "状态: connected" ~/桌面/dm-exo/logs/T3_bridge.log | tail -3

# ② 有没有拒绝
grep -c "program_run_failed" ~/桌面/dm-exo/logs/T3_bridge.log

# ③ IK 是否在发指令（应 ~250Hz）
ros2 topic hz /target_robot/joint_commands

# ④ FK 是否正确（离线 pinocchio FK 对比，偏差应 0.0mm）
#    见 HANDOFF §三 的验证方法

# ⑤ 端到端：按住离合慢动 → 真机跟手；松开 → 立即停
```

离线回归用 `tools/dm-exo/` 里的 `record_compare.py` / `analyze_compare.py`，
历史基线包在远端 `~/桌面/dm-exo/bags/`（20 个事件包）。注意包里有每 10s 的假断档伪影。

## 6. 代码约定

- ROS 2 Humble，Python 3.10，节点文件保持单文件可读，不强拆模块。
- 改 `ik_node.py` 要顺手清 `__pycache__`（硬链接机制下旧 pyc 会干扰）。
- 注释用中文，写「为什么」不写「是什么」。
- 不要新增依赖；`requirements.txt` 里的版本是钉死的（pin/pink 版本和 ROS 源里的不一样）。

## 7. 汇报格式

每次动完真机，回报里必须包含：改了什么文件（含远端路径）、跑了什么命令、
日志里的关键行（拒绝数 / sent / 是否跟手）、下一步建议。
不确定的地方直说「未验证」，不要把推测写成结论。
