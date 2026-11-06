# DM-EXton2 遥操作调试交接文档

- 原始交接时间：2026-11-05 21:41（tc 主机 `~/桌面/dm-exo/HANDOFF.md`）
- 本版整理时间：2026-11-05 21:45，并补充了 21:42 的现场核实结果
- 项目快照：[README.md](./README.md)，操作规矩：[AGENTS.md](./AGENTS.md)
- 接手任务台账：[TASKS.md](./TASKS.md)（唯一权威状态）　工作流水：[WORKLOG.md](./WORKLOG.md)

> 本文只写背景与结论，**不要再往这里加流水**；进度变化写 WORKLOG，任务状态写 TASKS。

---

## 零、怎么连上去（明文凭据）

```
本机 192.168.26.18 (WiFi 192.168.26.0/24)
  └── 跳板机 192.168.26.250  (HSD-RB1021, ARM64 Ubuntu 22.04)
        └── eth1 192.168.61.15 → 内网 192.168.61.0/24
              ├── 192.168.61.14 / .100  tc 主机（同一台双 IP）
              └── 192.168.61.60  双臂机器人控制器（仅 .14 可直达）
```

```bash
# 1) 跳板机
sshpass -p 'DMdaNto' ssh ubuntu@192.168.26.250
# 账号 ubuntu / 密码 DMdaNto（sudo 组）

# 2) 本地隧道（若 127.0.0.1:2222 已在监听则跳过）
sshpass -p 'DMdaNto' ssh -f -N -L 127.0.0.1:2222:192.168.61.14:22 ubuntu@192.168.26.250

# 3) tc 主机（密码是一个空格！）
sshpass -p ' ' ssh -p 2222 tc@127.0.0.1
# 账号 tc / 密码 ' '（单空格），sudo 密码同样是 ' '

# 4) 内网 SOCKS（可选）
sshpass -p 'DMdaNto' ssh -f -N -D 127.0.0.1:1080 ubuntu@192.168.26.250
# socks5h://127.0.0.1:1080  → 可访问 192.168.61.0/24（不含 .60）
```

`192.168.61.60` 的登录凭据未知；它只开放 22 / 53 / 5355 / 5566 / 30485，
其中 30485 是 movej 控制端口（协议 V0.3，帧尾 `#`），5566 是 JuxieLinker Web UI。

---

## 一、系统架构

```
操作员手臂 → 外骨骼背包(192.168.61.15) → T1 MessageBridge → ROS2 (DOMAIN 42)
   ├── /exoskeleton/joint_states (~800Hz, 12关节)
   └── T4 Puppetry → /target_robot/{left,right}_ee/target (~100Hz)
       ⚠️ Puppetry 只在离合按住期间发布目标，松开即停发！
       ⚠️ Puppetry 把 /target_robot/{side}_ee/state（FK adapter 发的真机末端）
          当作"真机当前状态"来锚定目标 bias → FK adapter 坐标系 = Puppetry 坐标系

T2 FK adapter（robot_fk_adapter）: /client_robot/joint_states → /target_robot/{side}_ee/state
   （已验证与 pinocchio FK 偏差 0.0mm，frame=base_link 正确）

T3 exton_bridge（/home/tc/tcvg/exton_bridge.py）: /target_robot/joint_commands → TCP movej
   → 192.168.61.60:30485（协议 V0.3，帧尾'#'，17维：腰+左7+右7+头2）
   ⚠️ TCP 单连接：joint_probe.py 直连前必须先停 bridge

T5 ik_node（pink IK, 250Hz）: 增量遥操核心，离合锚定逻辑都在这
   → /target_robot/joint_commands → bridge 限速步进下发
```

组件管理：`~/桌面/dm-exo/start_teleop.sh` / `stop_teleop.sh`，日志 `~/桌面/dm-exo/logs/T{1..5}_*.log`

**当前接手必读的启动入口（2026-11-06）**：现场恢复遥操使用
`~/桌面/dm-exo/restart_teleop.sh`。它会清场、检查 `.15/.60` 网络、检查 30485 单连接、
启动 T1–T5，并验证 T3 已同步反馈且收到 `joint_commands`。T5 固定左臂 X 镜像
`[-1,1,1]`，每次启动后关闭 `publish_commanded_ee_state`。若 T3 已连接但运动时出现
`program_run_failed`，这是控制器/电机执行层拒绝，不能误判为 bridge 没启动；先由操作员在
5566 检查 fault/enable、关闭图形界面，再重启软件链。`auto_enable=false` 不得绕过。

## 二、今日已修复（全部在 ik_node.py，src↔build 硬链接，改完重启 T5 即生效）

路径：`.../exton_ros2_retargeting_demo/src/robot_ik_retargeting/robot_ik_retargeting/ik_node.py`

| # | 问题 | 根因 | 修复 | 验证 |
|---|------|------|------|------|
| 1 | 按住离合手静关节漂移 | posture task 目标=全零位（launch 里 preferred_angles 全 0），按住期间零空间回拉（Joint7L 实测 65~151 mrad/0.5s） | 按下瞬间 `posture.set_target(当前关节角)` | 手静漂移 → 0.1 mrad ✅ |
| 2 | 按下瞬跳（旧锚点） | Puppetry 松开停发目标，ik 拿上次松开前的旧目标当外骨骼锚 | 断档 >0.2s 目标作废（`await_anchor`） | — 未单独验证 |
| 3 | 按下瞬跳（滤波器陈旧） | OneEuro 滤波器断档期间冻结旧值，恢复后 ~6%/帧爬向新值（76mm / 1200mrad 拖拽） | 断档 >0.2s 重置滤波器（`PoseFilter.reset`） | — 未单独验证 |
| 4 | 按下瞬跳（姿态失配） | ~~断档后回退用真机位姿当外骨骼锚~~ Puppetry 目标姿态与真机手腕姿态可差几十度，一次性套上 → J5 瞬跳 1000+ mrad | 外骨骼锚 = 目标流自身第一帧新鲜目标（位置+姿态同源） | 瞬跳 1075 → 0 mrad（press18-20）✅ |
| 5 | 按下永久冻结 | 安定等待计时基准错误：hist 窗口裁剪（0.3s）把 elapsed 永远压在 min_wait（0.5s）以下 → 永不锚定 | 计时基准改为首帧新鲜目标时刻 `await_t0`，+1.5s 强制锚定兜底 | 离线单元测试 0.55s 锚定 ✅ |

**安定等待设计**（每次按下必经）：按下 → 冻结 → 等 Puppetry 目标流安定（它起流后 bias 渐进生效，
手静时目标姿态仍以 ~23°/s 自转约 1s）→ 0.5s 后且窗口内变化率 <10mm/s、<5°/s 时锚定 → 开始跟随。
**按下后 ~0.5–1s 不动是设计行为，不是 bug。**

## 三、已修复的环境问题

**幽灵 TF 树（重要！）**：16:36 遗留的旧 `tc_urdf` robot_state_publisher（孤儿进程）与当前
hr_up TF 树同名帧并存 → FK adapter TF 查询随机取错树 → 它发的真机末端是错的 →
Puppetry 以它为参考 → 目标坐标系漂移。已清除，并在 `stop_teleop.sh` 增加 robot_state_publisher 清理。
验证方法：pinocchio 离线 FK vs FK adapter 输出（修复前差一个轴循环置换 + 1.244m 偏移，修复后 0.0mm）。

## 四、当前重点：手静时 IK 关节自运动（2026-11-05 22:17 后核对）

最新现场已运行 lock-step bridge，不能再沿用“真机零动作”的旧结论。证据与命令见 WORKLOG 最新条目，任务见 TASKS T10。

- `press04_left_221400.npz` 含精确离合通道：按住 5.32s，左 J7 指令范围 15.99°；其中 0.5s 外骨骼最大变化 0.1774°、上游目标仅 0.297mm / 0.100°，IK J7 却变化 5.9351°。
- 同一 0.5s 的指令 FK 末端变化仅约 0.319mm / 0.0213°，构型接近奇异。完整滤波、死区与控制方法的离线重放已复现：J7 全段范围 16.47°（录包 15.99°），左臂关节轨迹 RMSE 0.639°；手静半秒 -7.48°（录包 -5.94°）。NPZ 缺原始 header 时间戳，使用接收时间近似，因此不是逐帧等价重放。
- **已确认状态话题双发布**：`/target_robot/left_ee/state` 同时由 `fk_adapter` 和 `ik_retargeting` 发布，运行参数 `publish_commanded_ee_state=True`。异常窗口抽样 22 帧 state 全部匹配指令 FK，不匹配实测 FK（中位差 8.417mm）。不要把该话题直接视为真机末端。
- 固定当前末端目标的离线 solve_ik 对照：姿态 cost=0.3 时 J7 +0.216°/0.5s，cost=0 时不动；解释不了本次约 -6°/0.5s。当前左 J7 距模型限位仍约 79°，本次也不支持硬限位回退解释。
- **相同前史、固定真实任务目标的严格对照已完成**：t=221.832 起固定目标，约 0.5s 内原任务 J7 -7.4813°，平移误差 1.059→0.568mm；仅移除 posture 后 -8.0890°，误差 1.059→0.515mm。持续输入变化、滤波余尾及 posture 都不是维持这段运动的必要条件。
- 当前证据最支持：此前目标变化留下残余末端误差；手静时死区保持旧目标，但 IK 仍持续求解；近奇异构型将小误差的追赶转化为大幅关节调整。仅将 damping 从 12 局部改到 120，J7 降至 -0.0662°，但误差仅从 1.059 降至 1.056mm；不能把大阻尼或每拍目标改成当前 FK 直接视为生产修复。
- 下一步验证静止保持及近奇异稳定策略，并单独关闭指令 state 发布做对照。双发布对 Puppetry 漂移的具体因果贡献、松开后真机继续运动的原因及修复有效性仍未验证。
- **2026-11-05 23:01 手静冻结已部署并实测**：IK 指令层手静自运动消除（冻结窗 `ik_net` 全零，探针峰值 16.3mrad vs 修复前 5935mrad，包 `press01_left_230820.npz`）。残余"手静臂动"拆成两个新问题：① 手静但 Puppetry 目标自身滑移最高 ~20mm/s 驱动 IK 合法跟随（疑 `/target_robot/left_ee/state` 双发布回路，未验证，见 T11）；② IK 指令完全冻结时真机 J1L 仍 ~1s 周期 ±2.5° 自振，同期 bridge 拒绝 19→59（见 T12）。远端源文件备份 `ik_node.py.bak_1105_2301`，新参数 `still_lin_speed_mps=0.005` / `still_ang_speed_rps=0.05`。
- **2026-11-05 23:17 T11 实验条件就绪**：`publish_commanded_ee_state` 改为运行时读参（默认仍 True），现场已置 False 并确认生效（IK pid 79431，备份 `ik_node.py.bak_1105_2316`，md5 `2af30692…`，离线回归 OVERALL PASS）。此改动是 T11 单变量对照的必要前提：原代码启动即缓存参数，`ros2 param set` 不生效。效果待新录包验证。
- **2026-11-05 23:30 复测观察（口述，无录包）**：T11 条件下按住离合仍会动，但操作员主观判断「幅度小了些」。**幅度未量化、归因未验证**——无法区分是关闭指令 state 的收益还是手静冻结的固有收益。可确认的是：残余移动仍有至少一条未消除的驱动路径。明天需先确认 IK 仍为 pid 79431 且 `publish_commanded_ee_state=False`（进程重启会回 True），再拉 23:17 之后的新包对比。
- 本轮已按上条部署 ik_node.py（含备份与重启记录，详见 WORKLOG 23:01 与 23:16 条目）；bridge / MessageBridge / Puppetry / FK 未动，未手动下发运动指令。
- **2026-11-06 离线就绪包（ZCode，未部署）**：bridge 下发计划抽成可离线单测的 `StepPlanner`；T14 迟滞 / T17 `step_base=command` 开环斜坡 / R3 反馈看门狗 / R4 重连清快照 / R10 指令流超时全部实现并过 18 项离线回归（`tools/tests/`，含 hunt 复现与 UBT 下垂教训复现）。默认参数=现行为（`step_base=feedback`），上真机只做「备份→推送→参数 A/B→录包对比」，部署步骤见 TASKS T16。**本地 bridge 已领先远端，推送前必须备份**；分析文档三份在 docs/。

### 历史阻塞：机器人不跟手（program_run_failed，21:42 快照）

**现象**：IK 目标正常跟手（实测按住 15.5s，IK 8~21° 连续变化），bridge 在发（sent 增长），
但控制器拒绝：`控制器拒绝运动 movej program_run_failed`（累计 6440+，自 18:54 起），真机零动作。

**根因（高置信，待最终验证）**：18:54 bridge 异步化重构（bak：`exton_bridge.py.bak_blocking_1105_1854`）。
新 tx 线程收到运动槽立即下发，`_flush_commands` 50Hz → **movej 以最高 50Hz 连发，不等上一个
movej 程序跑完**。movej 是控制器阻塞型程序 → 程序重叠直接拒绝。旧阻塞版在回调里 50ms 轮询，
意外把实际发送率压到 ~6.6Hz，所以没暴露。

**已做实验（尚未验证结论）**：21:19 bridge 已用 `-p command_rate_hz:=5.0` 重启（PID 52387），
connected / fb_synced 正常。

**21:42 现场核实（本次补充）**：

- T3 日志里 `program_run_failed` 计数 = **0**，但 `sent=0`、`skip=nodelta`，状态行反复提示
  「IK 与真机接近，暂无有效步进（松开 Clutch 时正常）」
- 说明 **松开离合时根本不会下发 movej，0 拒绝是假象，5Hz 方案还没有被真正检验**
- 其余进程：T1 MessageBridge(pid 21935) / T2 robot_state_publisher(pid 48691) / T3 bridge(pid 52387) /
  T5 ik_node(pid 71634，21:39:28 启动) 均在跑，日志 T3 最后写入 21:42

**下一步（必须先做）**：按住离合、慢动手臂，观察 T3 日志是否再出现拒绝。

- 若跟手了 → 根因实锤，再做永久修复
- 若仍拒绝 → 查 §四末尾「次要疑点」（J7 限位）

**永久修复方案（建议）**：`AsyncTcpClient._tx_loop`（`tools/exton_bridge.py` 第 248 行）加 lock-step：
movej 模式下，发出运动帧后等 rx 线程收到该命令的应答（带 `status` 的帧）再发下一帧，
超时 300ms 兜底防死等；可再叠加 ≥5Hz 下限节拍。实现时注意 rx 收到的 `get_joint_position`
应答（带 `joint` 字段）不应解除锁定。

**备选**：切 `control_mode:=movej_canfd`（流式，无程序重叠问题）——但代码注释明说部分固件对
canfd 中 `-100` 占位轴直接 `program_run_failed`，bridge 当前腰/头槽位发 `-100`，切换前需处理。

**次要疑点**：双臂 J7 停在 ±170°（URDF 限位 ±2.967rad = 170°，bridge 档案软限位 ±152.7°）。
J7 在限位外时控制器是否拒绝 movej 未验证；若 5Hz 后仍偶发拒绝，优先把 J7 手动回到中位再测。

## 五、ik_node 曾无日志挂死（20:38 实例）

用户 20:38 重启的 T5（当时是带冻结 bug 的代码）在 20:39:44 后日志完全停止、不发布指令，
进程 alive（S 状态）。已 kill 重启（21:39:28 新实例，250Hz 正常）。若新代码复现「日志停但进程在」，
查 pink `solve_ik` / OSQP 是否病态卡死（可给 solve_ik 加迭代上限 / 打印）。

## 六、工具

- `clutch_probe.py`：离合触发录包器（环形缓冲 60s，按下 → 松开 +3s 自动落盘 `bags/pressNN_xxx.npz`
  + 即时诊断）。⚠️ 现场正在跑的实例是旧版，包里没有 `/clutch_*` 通道；重启一次即带上（源码已修）。
- `record_compare.py N 前缀` + `analyze_compare.py xx.npz`：全量录制 + 分析（含离合事件分析）。
- `joint_probe.py`：直连控制器探测 / 微动（先停 bridge！TCP 单连接）。

## 七、踩坑清单（给下一个 AI）

1. **pkill 自匹配**：命令文本里同时含 `pkill -f` 的模式和匹配字符串（如 `bash -c` 启动脚本里的
   `exton_bridge.py`）会把自己 shell 杀掉（今天发生 3 次）。杀进程用拼接模式
   `pgrep -f "exton_bridg""e.py"` 或按 PID。
2. **`pkill -9 ros2 launch` 会孤儿化子进程**（robot_state_publisher 曾残留一下午）。
3. **TF static 是锁存的**：杀掉错误 TF 发布者不够，订阅者（FK adapter）必须重启才冲洗缓冲。
4. **`colcon --symlink-install` 下 src 与 build 是硬链接**：改 `ik_node.py` 即生效，但 `__pycache__`
   建议清一下；无需 `colcon build`。
5. 录包器接收时间戳含积压排空伪影：每 10s 的进度打印 + sleep 会造成全话题 ~0.3–0.4s 假断档，
   分析时忽略 x0.0 / x0.7 时刻的同步空洞。
6. 启 bridge / IK 必须 `export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0`。
7. RViz 需要桌面：`export DISPLAY=:0 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority`。

## 八、现场快照（2026-11-05 21:42）

| 组件 | 状态 |
|---|---|
| T1 MessageBridge | ✅ pid 21935，运行 3h17m |
| T2 FK adapter + robot_state_publisher | ✅ pid 48691（20:26 实例，帧正确） |
| T4 Puppetry | ✅ |
| T3 exton_bridge | ✅ pid 52387，`command_rate_hz:=5.0`，connected=True fb_synced=True，拒绝计数 0（但 sent=0） |
| T5 ik_node | ✅ pid 71634（21:39:28 启动，含全部修复），`/target_robot/joint_commands` 250Hz |
| clutch_probe | 后台运行中（旧版，无离合通道） |
| bags/ | 20 个事件包可作回归基线（远端） |

## 九、待办（按优先级）

1. 【需真机操作】5Hz bridge 下**按住离合慢动**测试 → 验证 program_run_failed 根因
2. 【代码】bridge tx 加 movej lock-step（方案见 §四），恢复 10–20Hz 发送率
3. 【验证】重跑 clutch_probe 全流程：瞬跳≈0、手静漂移≈0、跟手正常
4. 【验证】补验修复 #2、#3（旧锚点 / 滤波器陈旧），目前没有独立验证记录
5. 【可选】J7 回中位；probe 重启带上离合通道；canfd 模式评估
6. 【排查】ik_node 无日志挂死（§五）根因未查，若复现优先处理
