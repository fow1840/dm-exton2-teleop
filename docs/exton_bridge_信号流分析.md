# exton_bridge.py 信号流分析

- 日期：2026-11-06（离线分析本仓库快照，未连现场）
- 姊妹篇：[信号链路与离合自运动分析.md](./信号链路与离合自运动分析.md)（本文件把其中 T3 一环展开到线程/时序级）
- 结论分级：✅=代码/日志实证；🔍=机制推断（量级吻合，待验证）；⬜=未验证

---

## 一、文件在哪、有几个版本（✅ md5 核对）

| 路径 | 行数 | md5 | 身份 |
|---|---|---|---|
| `tools/exton_bridge.py` | 775 | `fe58738a…` | **现役分析基线**：V0.3-async2（收发双线程）+ movej lock-step |
| `tools/exton_bridge.py.bak_blocking_1105_1854` | 624 | `d750d22b…` | 18:54 异步化前的**阻塞版**备份 |
| `ros2_ws/scripts/exton_bridge.py` | 624 | `d750d22b…` | 与阻塞版备份**逐字节相同**（厂商原版，AGENTS 同步清单未含此路径） |
| 远端真身 `/home/tc/tcvg/exton_bridge.py` | — | ⬜ | 隧道未通（2026-11-06 尝试：跳板机 192.168.26.250 拒绝，`kex_exchange_identification: Connection closed`），**现场运行版是否与本地基线一致未核对** |

> 下文所有行号均指 `tools/exton_bridge.py`（775 行版）。

## 二、角色定位（✅）

链路最后一环：把 ROS 世界的 `/target_robot/joint_commands`（IK 指令，弧度、URDF 命名）
翻译成控制器私有协议（TCP 30485、JSON+`#`、17 维「腰/L1-7/R1-7/头」、movej 带程序语义），
并把控制器的 `get_joint_position` 反馈翻译回 `/client_robot/joint_states`（URDF 命名）。

**它同时是三个身份**（这是理解一切问题的起点）：
1. **协议网关**：命名映射 + 单位（弧度）+ 17 维槽位填充；
2. **限速整形器**：`max_joint_step_rad=0.20` / `follow_ratio` / `command_eps_rad`；
3. **第二闭环控制器**：以**真机反馈**为底算误差并纠偏（§四），而非简单透传 IK 值。

## 三、线程与锁模型（✅）

```
ROS executor 线程（单线程串行）            tx 线程                    rx 线程
├─ _on_joint_commands @250Hz  ─┐
├─ _flush_commands @cmd_hz     │ 写槽/读快照，永不碰 socket
├─ _publish_joint_states @50Hz ├─ set_motion_target() ──► _motion_slot(最新覆盖)
├─ _ensure_connected @1Hz      │ send_control() ────────► _ctrl_queue(FIFO)
└─ _status_watchdog @2Hz       │ poll_responses() ◄────── _responses(deque64)
                               │ _send_now() ───────────► TCP（_send_lock 串行化）
                               │                          ◄── _rx_loop: select(50ms)→recv→_feed
                                                          _feed: 帧解析→_latest_joint_rad(原子快照)
                                                               → movej+status 应答→ack_seq++/notify
```

五把锁/同步原语：`_cmd_lock`（指令暂存）、`_motion_cv`+`_motion_slot`（运动目标槽，
最新覆盖）、`_state_lock`（反馈快照）、`_send_lock`（socket 写串行）、
`_movej_state_lock`（lock-step 计数）。✅ ROS 回调与 socket 读写严格隔离，与
docstring「回调只做写槽和读快照」一致。

## 四、下发方向：一条 movej 的生命周期（✅ 流程 / 🔍 时序推断）

```
IK 250Hz JointState
  │ ① _on_joint_commands (544-547)：_pending_cmd = msg   ← latest-wins，≈96%帧被覆盖丢弃
  ▼
② _flush_commands @command_rate_hz (549-650)
  │  门禁：msg非空 & TCP已连 & fb_synced (554)
  │  fb = 反馈快照（可能已旧 ≤33ms，见风险R3）
  │  逐关节 (apply_arm 573-596)：
  │     ik_val = clip(IK值, 档案/URDF限位)          (579)
  │     err   = ik_val − fb[idx]                    (581)   ← 以真机反馈为底！
  │     |err|>3.0rad → 跳过该轴（疑似错误初值）      (584)
  │     |err|≤0.001rad → 保持（不发该轴）            (587)
  │     否则 目标 = fb[idx] + clip(err×1.0, ±0.2rad) (590-592)
  │  腰/头 = -100 (603-606)；全帧无变化 → 'nodelta' (620)
  ▼
③ {command:'movej', joint:17维, v:15} → set_motion_target (644) ← 第二层最新覆盖
  ▼
④ tx 线程 _tx_loop (265-335)：
  │  在途锁：waiting_movej_seq 未解锁 → 新槽放回，不发 (301-305)
  │  发送后 deadline = now+300ms (315-317)
  ▼
TCP → 控制器执行 movej（阻塞型 P2P 程序，v=15）
  ▼
⑤ 控制器回 {command:'movej', status:…} → rx 线程：
     ack_seq++ + notify tx (369-374)   ← 解锁下一条
     status 以 failed 结尾 → _status_watchdog 计数 program_run_failed (709-716)
```

**关键量化特征**（🔍，未现场核实全部参数）：

- **双层 latest-wins**：250Hz 输入 → `command_rate_hz` 处理 → slot 覆盖 → tx 节拍发送。
  指令从 IK 到 TCP 的固有延迟 = 0~2 个 flush 周期 + 在途等待。
- **follow_ratio=1.0 时目标=Q_ik**：只要 |err|≤0.2rad，每帧算出的目标**恰好等于 IK 指令值**
  （fb+err=ik_val）——所谓「步进」只在 |err|>0.2rad 时才表现为中间点。这意味着
  IK 冻结时所有纠偏帧都指向同一固定位姿 Q_ik（机制③分析的前提，✅ 由代码可证）。
- **lock-step 节拍 = ack 节拍**：实测 `movej_tx=2092 ack=2092 timeout=25`（23:08 测试）→
  绝大多数 ack 在 300ms 内到达。若 ack=受理（⬜ 待证），lock-step 退化为 ~0 间隔，
  程序重叠拒绝仍会发生（测试期间 +40 次）；若 ack=完成，则发送率被压到运动周期率。

## 五、反馈方向：一条反馈的生命周期（✅）

```
tx 线程 @feedback_rate_hz(30Hz) 发 {command:'get_joint_position'} (320-324)
  → 控制器回 17 维 joint 数组
  → rx _feed (375-384)：_latest_joint_rad = 前 17 维（<17 维时 HOLD 填充兜底 378-384）
  → _publish_joint_states @50Hz (668-698)：
      HOLD(-100) 槽 → 0.0 (681-690)
      name = [Joint1..7, Joint1L..7L]（右先左后），position = 右+左 (696-697)
  → /client_robot/joint_states
      ├→ ik_node.state_cb → q_measured（松开离合时直接变指令）
      ├→ fk_adapter → TF + {side}_ee/state（真机末端，喂 Puppetry）
      └→ robot_state_publisher → TF 树
```

⚠️ 反馈链是**整个系统的物理真值源**：IK 的 `q_measured`、FK 的真机末端、Puppetry 的
锚定参考，全部最终溯源到这里。反馈停更/失真 = 全链路失真（风险 R3）。

## 六、连接与同步状态机（✅）

```
connect() ──► 等首条 get_joint_position 应答 ──► _maybe_sync (524-539)：
                _joint_cmd = 反馈快照；fb_synced=True
                ★ 同步前：不下发（_flush 门禁）、不发布 state —— 防错误初值伤真机 ✅
每 1s _ensure_connected (518-522)：断线重连 → fb_synced=False → 必须重新同步才恢复下发
Ctrl-C (760-767)：clear_motion + stop_motion（给发送线程留 0.2s 窗口）
```

## 七、关键参数表（✅ 默认值 / ⬜ 现场值）

| 参数 | 默认 | 现场值 | 作用 | 备注 |
|---|---|---|---|---|
| `control_mode` | `movej_canfd` | ⬜（重启命令用 `movej`） | movej=阻塞 P2P；canfd=流式 | 两模式风险完全不同：canfd 对 -100 占位轴有拒绝坑（T09） |
| `move_speed` | 15.0 | 15.0（AGENTS 重启命令） | movej 的 v=0~100 | **禁忌参数，不许动** |
| `command_rate_hz` | 50.0 | ⬜（21:19 曾置 5.0，lock-step 版重启后未知） | `_flush_commands` 节拍 | 与 lock-step 叠加决定实际指令率 |
| `feedback_rate_hz` | 30.0 | ⬜ | 查询反馈周期 | 决定 fb 新鲜度上限 |
| `command_eps_rad` | 0.001 (0.057°) | ⬜ | 误差小于此不发 | **hunt 嫌疑参数**（T14 迟滞改造点） |
| `max_joint_step_rad` | 0.20 (11.5°) | ⬜ | 单帧最大步进 | \|err\|≤此值时目标=Q_ik |
| `max_joint_jump_rad` | 3.0 | ⬜ | 错误初值丢弃阈 | 触发即整帧保持+warn |
| `send_only_on_change` | False | ⬜ | 去重 | False=有 err 就发 |
| `auto_enable` | False | false（安全设定） | 远程使能 | **禁忌参数，不许动** |
| `use_archive_right_limits` | True | ⬜ | 右臂用档案软限位 | 左臂始终 URDF ±(π−0.05) |

现场核对命令（到现场先跑，结果写回本表）：
`ros2 param get /exton_bridge command_rate_hz` 等 + `md5sum /home/tc/tcvg/exton_bridge.py`。

## 八、已知问题与风险清单

| 编号 | 级别 | 问题 | 证据 | 处置 |
|---|---|---|---|---|
| R1 | P0 | **纠偏 hunt（机制③）**：以反馈为底 + eps=0.057° + movej P2P 语义 → IK 冻结后仍持续发纠偏帧，疑成 ~1Hz 极限环 | 代码 581-592 ✅；23:08 拒绝 19→59 ✅ | T14（hold+迟滞），先做 T12 判别 |
| R2 | P0 | **ack 语义未定**：300ms 内 ack 到达率 2067/2092，若 ack=受理则 lock-step 不按运动周期串行，程序重叠拒绝继续发生 | `movej_tx=2092 ack=2092 timeout=25` ✅；语义 ⬜ | T12 单步 movej 台架（停 bridge） |
| R3 | P1 | **反馈新鲜度无看门狗**：控制器停回应答但 TCP 存活时，`_latest_joint_rad` 永久陈旧，仍被 50Hz 发布、被 `_flush_commands` 当底 → 全链真值源失真 | 代码 375-384/668-698 无时间戳检查 ✅；是否发生过 ⬜ | 加反馈超时标记（如 >1s 无新帧 → 停下发并告警） |
| R4 | P2 | **重连同步可能用到陈旧快照**：`connect()` 清 `_rx_buffer` 但不清 `_latest_joint_rad`（176-177），重连后 `_maybe_sync` 可能在控制器尚未应答时用**断线前**的旧快照同步 | 代码 ✅；后果 ⬜（机器人断线期间若被动过，首个纠偏基线错） | 同步前强制等一条**新**应答（对比连接时刻） |
| R5 | P2 | `<17 维` 反馈兜底用 HOLD 填充 → 发布时变 0.0 → FK/TF 被 0 角污染 | 代码 378-384 ✅；控制器是否回过短帧 ⬜ | 记录长度异常即告警，不做静默兜底 |
| R6 | P3 | `_responses` deque(64) 溢出静默丢应答；`_status_watchdog` 2s 才消费一次，30Hz 应答下日常余量足够 | 代码 ✅ | 观察即可 |
| R7 | — | `movej_canfd` 备选模式对腰/头 `-100` 直接拒绝（部分固件） | HANDOFF 记录 ⬜（真机未试） | T09 评估，切换前必须先定腰/头策略 |

## 九、版本演进与教训（✅ git/备份可考）

| 版本 | 下发机制 | 反馈机制 | 结局 |
|---|---|---|---|
| 阻塞版（=`ros2_ws/scripts/` 版，d750d22b） | 回调内填槽，`_poll_and_query`@30Hz 在 **ROS executor 里 `poll_recv` 阻塞 50ms** | 同上一进程 → 每秒排队 5.5s 阻塞工作，反馈实测 **6.6Hz** | 反馈/下发双双滞后（docstring 32-34 自记） |
| async2（18:54，`bak_blocking_1105_1854` 起点） | 收发双线程，`_flush_commands`@50Hz **立即逐帧下发** | 30Hz 查询，rx 线程解析 | movej 50Hz 连发 → **程序重叠 → `program_run_failed` 风暴**（6440+ 拒绝，真机零动作） |
| async2+lock-step（现行，fe58738a） | movej 在途锁：发一条→等 status→再发，300ms 兜底 | 不变 | `movej_tx=ack` 无泄漏；**但 hunt（R1）与 ack 语义（R2）未解** |

> 教训一：**第二闭环（fb 底 + 限幅步进）从阻塞版起就存在**，阻塞版的 50ms 轮询
> 恰好把发送率压到 ~6.6Hz，掩盖了程序重叠问题——性能修复揭开了语义问题。
> 教训二：movej 是「程序」不是「流」，任何高于 ack 率的发送都是拒绝；任何低于
> 到位精度的 eps 都是 hunt。这两个约束夹出的可行域就是 T14 迟滞要落的位置。

## 十、未验证清单

1. 现场 `/home/tc/tcvg/exton_bridge.py` 与本地 `tools/exton_bridge.py` 的 md5 一致性（隧道未通）；
2. 现场 `command_rate_hz` / `control_mode` / `feedback_rate_hz` 实际运行值；
3. movej `status` 应答时机 = 受理还是运动完成（T12 台架可一锤定音）；
4. 控制器对单条 movej 的到位过冲量与复现性（R1 极限环的外部条件）；
5. 反馈是否出现过停更/短帧（R3/R5 是否有现实暴露）。
