# TASKS.md —— 接手任务台账

> 这是唯一权威的任务清单。`HANDOFF.md` 只写背景与结论，`WORKLOG.md` 只记流水。
> 状态取值：`TODO` / `DOING` / `BLOCKED`（缺人工或硬件）/ `DONE`（附证据）/ `DEAD`（结论推翻，保留不删）
> 更新规则见 [AGENTS.md](./AGENTS.md) §0。

## 一、接手前必读的现状（2026-11-06 18:20 ZCode 再核对）

- **最新快照（18:19 restart 后）**：T1–T5 全部新进程、T3 `connected=True cmd_rx=1736 … reject=0 max_err=0.000 held=14`（无离合基线 ✅）、30485 唯一 ESTAB 归 T3、`/target_robot/joint_commands`≈244Hz。
- **限位已全项目同步（18:18）**：控制器 get_config 18:10 原文 `upper=[2.79,0.26,2.79,2.26,2.79,1.57,2.79]×2臂`、`lower=[-2.79,-2.3562,-2.79,-0.69,-2.79,-1.57,-2.79]×2臂`（=用户 18:03 UI 设置值）；项目侧统一为该值内缩 0.02rad：URDF 双臂 == bridge `LIVE_LIMITS_RAD`（右臂 URDF 原为 ±2.967，本次补齐，远端 md5 `ca6ab8c0…`，备份 `.bak_1106_1818`）。
- **控制器残留软件错误**：`state=idle, softwareErrors=[34013185]`（电机7 超软限位 0x02070001，17:44 锁存未清）；hardwareErrors 空。全关节当前 ≈0°（J7 已人工归零）。**遥操前必须 5566 清故障 + 人工 enable**。

### 上一轮快照（2026-11-06 17:00 Codex 接手再核对）

- Codex 17:00 核对：MessageBridge 153800 / robot_state_publisher 153908 / bridge 154127 / Puppetry 154222 / IK 154296；30485 唯一 ESTAB 为 bridge 154127。
- 参数原文：`publish_commanded_ee_state=False`，`mirror_xyz_left=[-1.0,1.0,1.0]`；bridge 仍为 `movej_canfd`、`move_speed:=15.0`、`auto_enable:=false`。
- 新只读基线包：`/home/tc/桌面/dm-exo/left_baseline_1106_1701.npz`；8.1s 输出 `/exo:836Hz /ik:226Hz /rob:49Hz /ee_op:835Hz`，左臂 7 轴反馈范围全为 0；松离合静置不能证明运动可执行。
- 本次拉取的 T3 日志 `program_run_failed` 匹配数为 0；状态行原文 `sent=3152 ... +2s: sent=0 ... reject=0 ... max_err=0.000`。真机方向、硬件检修状态待操作员确认。

- 当前进程：MessageBridge 101134 / robot_state_publisher 101204 / bridge 101275 / Puppetry 101313 / IK 101370；均存活。
- bridge 运行参数原文：`control_mode:=movej_canfd -p move_speed:=15.0 -p auto_enable:=false -p use_archive_right_limits:=false`；30485 仅有 T3 `python3 pid=101275` 的 ESTAB 连接。
- 运动窗口日志原文：`connected=True ... sent=100 rx=477 reject=40 fb_age=0.02s max_err=0.644`，并连续出现 `控制器拒绝运动 movej_canfd program_run_failed`；松开离合后 `sent=0 ... reject=0 max_err=0.000`。
- 当前 T3 日志累计：`REJECT_COUNT 152`；现场拒绝累计已达约 2940（日志节流输出），以 T3 累计状态为准。
- **2026-11-06 16:15 更新（ZCode）**：拒绝真因已实锤为**电机7 堵转 → 控制器 state=fault 锁存**（get_device_state 原文见 T28），「50Hz 程序重叠」推测被推翻（lock-step 下发送率仍 50Hz、拒绝依旧）。软件链已连通且 lock-step 上线；堵转 fault 与未使能均已人工恢复（T28）。

### 历史快照（21:42）

| 项 | 结论 |
|---|---|
| 链路 | T1 MessageBridge / T2 FK / T3 bridge / T4 Puppetry / T5 ik_node 全部在跑 |
| 真机 | **不跟手**，控制器对 movej 回 `program_run_failed` |
| 关键误判风险 | 21:19 起 bridge 跑在 `command_rate_hz:=5.0`，日志拒绝数 0 —— 但 `sent=0`，因为松开离合时根本不下发，**这个实验没被检验过** |
| 结论可靠度 | 根因（50Hz 连发导致 movej 程序重叠）为高置信推测，**未实锤** |

---

## 二、任务清单

### T01 现场状态再核对（接手第一步，必做）

- 优先级：P0
- 状态：`DONE`（Codex，2026-11-06 15:47；现场复核，证据见 WORKLOG 最新条目）
- 依赖：无
- 说明：上表是 21:42 的快照，现场可能已经变了（有人重启过、机械臂关过机）。
- 动作：
  ```bash
  ps -eo pid,etime,args | grep -E "exton_bridge|ik_node|Puppetry|MessageBridge|robot_state_publisher" | grep -v grep
  tail -n 20 ~/桌面/dm-exo/logs/T3_bridge.log
  grep -c "program_run_failed" ~/桌面/dm-exo/logs/T3_bridge.log
  ```
- 验收：把实际 PID、`command_rate_hz` 参数值、拒绝计数写进 WORKLOG；与上表不一致要标出来。
- 负责：AI

### T02 真机验证 5Hz 是否仍然拒绝（决定性实验）【DEAD 2026-11-06：被 T28 推翻——拒绝与发送率无关（lock-step 下仍 50Hz 发送、32/2s 恒定拒绝），真因是电机7 堵转 fault 锁存，本实验失去意义】

- 优先级：P0
- 状态：`TODO`
- 依赖：T01；**需要操作员在场按住离合慢动手臂**
- 说明：松开离合时 bridge 的 `skip=nodelta`，不发命令，所以看日志必须先让人按住。
- 动作：
  1. `tail -f ~/桌面/dm-exo/logs/T3_bridge.log`
  2. 操作员按住离合，极慢、小范围动双臂 15s 左右，再松开
  3. 观察三件事：`sent` 是否增长、是否出现 `program_run_failed`、真机是否动作
- 验收：
  - 不再拒绝且真机跟手 → 根因实锤，T03 的验收基准成立
  - 仍拒绝 → 转 T06（J7 限位）先排除，再回到 T03
- 负责：AI 观察 + 人工操作

### T03 bridge tx 加 movej lock-step（永久修复）

- 优先级：P1
- 状态：`DONE`（2026-11-06 16:15 实现并上线 movej+movej_canfd 双模式 lock-step；
  「按离合慢动 sent 稳定、reject=0」的验收因 T28 fault 未清暂缓——见 T28。证据见 WORKLOG 15:58/16:15）
- 依赖：T02 给出方向（可不等待，直接实现后一起验）
- 位置：`tools/exton_bridge.py` → `AsyncTcpClient._tx_loop`（第 248 行）
- 说明：发一帧 movej → 等 rx 收到**该命令**带 `status` 的应答 → 再发下一帧；300ms 超时兜底防死等。
  `get_joint_position` 的应答（带 `joint` 字段）不能解除锁定。可叠加 ≥5Hz 下限节拍。
- 验收：
  - 按住离合慢动时发送率稳定在 10–20Hz（日志/加计数验证），拒绝计数为 0
  - 超时兜底路径被真实触发过一次（断开控制器测试）且不卡死
- 风险：改的是实时发送路径，出错会让整条链路不发命令；改前备份远端原文件
- 负责：AI

### T04 全流程回归（clutch_probe）

- 优先级：P1
- 状态：`TODO`
- 依赖：T03；远端 `~/桌面/dm-exo/bags/` 有 20 个历史事件包作基线
- 验收：
  - 按下瞬间末端跳变 ≈ 0（基线 #4 修复后 0 mrad）
  - 手静 0.5s 关节漂移 ≈ 0（基线 #1 修复后 0.1 mrad）
  - 松开立即停发、真机停止
  - 分析时忽略 x0.0 / x0.7 时刻的同步空洞（录包器每 10s 打印造成的伪影）
- 负责：AI

### T05 补验 ik_node 修复 #2、#3

- 优先级：P2
- 状态：`TODO`
- 依赖：T04 的可复现录包
- 说明：HANDOFF §二的表格里，#2（旧锚点作废）和 #3（OneEuro 滤波器断档重置）验证列还是空的。
  用「松开 >0.2s 再按」的场景专门跑一轮，确认没有 76mm / 1200mrad 级别的拖拽。
- 验收：WORKLOG 里给出对应的事件包编号与量化数据
- 负责：AI

### T06 J7 限位排查

- 优先级：P2
- 状态：`DOING`（2026-11-06 17:24 Codex：**左臂已修**——URDF Joint1L~7L 改为 get_config
  实测值（J7L ±2.77rad=158.71°，与 bridge LIVE_LIMITS_RAD 同值）并部署远端；
  2026-11-06 18:18 ZCode：**右臂 URDF Joint1~7 也已补齐为同表**（get_config 18:10 原文双臂对称、
  内缩 0.02rad，md5 `ca6ab8c0…`），至此 URDF 双臂 == bridge 表，两臂都不再有「IK 越过裁剪线」失配）
- 依赖：T02 若仍偶发拒绝则提升为 P0
- 说明：原左臂 URDF 全轴 ±2.967rad（≈170°），bridge 裁剪 ±158.71°，IK 目标越过裁剪线
  → 真机 J7L 停在 +158.719°、IK 却给到 +169.997°（`left_direction_1106_live01.npz` 实测）。
  J5L 本轮极值 −155.25°，不是 146.629°（后者是松手时的静止位）。
- 验收（未做）：按住离合把左 J5/J7 推到边界，确认 IK 指令不再超过 ±2.77rad，
  且 bridge `reject` 仍为 0。
- 动作：先把 J7 手动回中位，重跑 T02 的测试；对比拒绝率
- 验收：给出「回中位前 / 后」的拒绝计数对比
- 负责：人工调姿 + AI 观察

### T07 clutch_probe 重启，带上 /clutch_* 通道

- 优先级：P2
- 状态：`TODO`
- 依赖：无
- 说明：现场跑的是旧版进程，录出来的包没有离合通道，事后无法精确对齐按下/松开时刻。源码已修，只需重启。
- 验收：新录的包里能 `ros2 bag info` 看到 `/clutch_*` 话题
- 负责：AI

### T08 ik_node 无日志挂死根因

- 优先级：P2
- 状态：`TODO`
- 依赖：复现（20:38 出现过一次：日志停止、进程 S 状态、不发指令）
- 说明：怀疑 pink `solve_ik` / OSQP 病态卡死。可先给 solve_ik 加迭代上限与耗时打印。
- 验收：要么复现并定位到卡死点，要么给出加过监控后长时间不再挂死的记录
- 负责：AI

### T09 canfd 流式验证（**优先验证任务**，用户 2026-11-06 裁定）

- 优先级：**P0**（跟手性第一瓶颈的直接根治验证：movej 有效下发率 ~1–3Hz 走停 →
  canfd 50Hz+ 流式；**零开发量**——`control_mode` 参数与 `_build_motion_payload:880`
  均为桥原生支持，缺的只是验证）
- 状态：`DOING`（2026-11-06 真机：协议层三步全通——零增量/50Hz×10 零拒绝/**-100 占位正确解析**；扫位姿全程 canfd 流式驱动成功=平滑性软验证 ✅；**freq 上限与遥操场景实测待做**，且需肘检修后）
- 依赖：T16/T19 部署后；与 T20/T15 同一真机窗口执行
- 台架清单（joint_probe / 专用小脚本，先停 bridge，TCP 单连接）：
  1. **`-100` 占位行为**：发一条腰/头槽 = -100 的 movej_canfd，看是否
     `program_run_failed`（HANDOFF 记载「部分固件」拒绝，协议文档没写该限制——
     可能过时）；若拒绝 → 定腰/头策略（实发值或该轴真实反馈值）；
  2. **canfd 有效频率与平滑度**：从 50Hz 起连发小幅增量帧（**小幅、慢速、
     操作员知情**），测控制器实际接受率与运动平滑度，逐步试探上限（50→100Hz）；
  3. **回滚路径**：canfd 异常即切回 movej（param set 后重启 T3），确认恢复行为。
- 验收：三步结论写 WORKLOG；canfd 可用 → 与 T15 基线做同场景跟手对比
  （端到端滞后、stop-go 消失与否），并把「canfd 模式 + 实测 freq 上限」写进
  HANDOFF 固化为新基线；不可用 → 结论写 HANDOFF（原验收要求保留）。
- 负责：AI 工具 + 人工观察（真机小运动需操作员在场确认）

### T22 反馈链提频评估（30→60–100Hz 查询，容量验证优先）

- 优先级：P2（当前反馈用途对 30Hz 不敏感；为 T09 canfd / T17 command 模式后的
  闭环使用与 leash 监控做准备）
- 状态：`TODO`（2026-11-06 频率分析新立项，此前无对应任务——纠正「已安排」的误判）
- 说明：feedback_rate_hz=30 是旧阻塞架构时代遗留（当时反馈实测 6.6Hz）；async2 后
  查询本身不再是瓶颈，但控制器对更高查询率的容量从未测过。
- 动作：
  1. joint_probe 台架：get_joint_position 以 60/100/200Hz 连发，测应答率与
     movej 并发时的相互影响；
  2. 容量确认后 `ros2 param set /exton_bridge feedback_rate_hz` A/B（运行时可改）；
  3. 用 T15 基线法对比提频前后 (F)→(D) 滞后变化（chain_monitor T21 在线观测更佳）。
- 验收：控制器查询容量上限数字 + 提频前后滞后对比写 WORKLOG。
- 负责：AI 工具 + 人工观察

---

### T10 手静自运动：最新录包离线诊断

- 优先级：P0
- 状态：`DONE`（Codex，2026-11-05；只读分析已完成，修复和真机对照另行验证）
- 证据：`press04_left_221400.npz`，t=221.831–222.331s 外骨骼最大范围 0.1774°，目标 0.297mm / 0.100°，IK J7 范围 5.9351°。
- 已确认：state 双发布且掺入指令 FK；异常构型接近奇异；完整滤波、死区与控制方法重放复现 J7 全段范围 16.4747°（录包 15.9863°），左臂轨迹 RMSE 0.6394°。使用接收时间近似目标 header 时间，属于近似重放。
- 严格对照：相同前史，在 t=221.832 固定真实末端任务目标；随后约 0.5s，原任务 J7 -7.4813°、平移误差 1.059→0.568mm；仅移除 posture 后 -8.0890°、误差 1.059→0.515mm；仅 damping 12→120 后 -0.0662°、误差 1.059→1.056mm。
- 当前机制判断：死区保持旧目标但持续求解，残余末端误差在近奇异构型下驱动大幅关节调整。持续输入变化、滤波余尾及 posture 都不是维持该段运动的必要条件。大阻尼压低运动同时牺牲误差收敛，尚不能作为生产修复。
- 下一步：验证静止保持及近奇异稳定策略；单独关闭 `publish_commanded_ee_state` 做对照；检查 bridge/控制器松开后仍运动问题。
- 未验证：双发布对 Puppetry 目标漂移的因果贡献、修复有效性与真机回归。
- **2026-11-05 23:01 修复已部署实测**：手静冻结（`still_lin_speed_mps=0.005`/`still_ang_speed_rps=0.05`）上线，IK 指令层手静自运动消除（冻结窗 `ik_net` 全零，探针峰值 16.3mrad vs 修复前 5935mrad）。但操作员仍观察到手臂动，定位出两个后续问题 → T11 / T12。

### T11 手静但 Puppetry 目标自身滑移（上游目标漂移）

- 优先级：P0
- 状态：`BLOCKED`（Codex，2026-11-05 23:30；缺现场与录包，明天继续）
- 依赖：需操作员在场复测 + 拉取新录包
- 证据：`press01_left_230820.npz` 手静窗（手 p2p≤0.65°）内目标滑移最高 10.2mm/0.5s≈20mm/s（w=327.992），高于冻结阈值 → IK 合法跟随 → 手臂移动；全按住段 IK J1 净变化 -11.09°。
- 已做：`publish_commanded_ee_state` 改为运行时读参（默认仍 True），现场已 `ros2 param set ... false` 并确认 `Boolean value is: False`；IK 重启 pid 79431（备份 `ik_node.py.bak_1105_2316`，md5 `2af30692…`）；离线回归 OVERALL PASS。详见 WORKLOG 23:16 条目。
- 复测观察（操作员口述，无录包）：「按住离合的也会移动，但是确实是移动的幅度小了些。」→ 未完全消除，幅度下降但**归因未验证**。
- 下一步（需现场）：
  1. 确认 IK 仍为 pid 79431 且参数仍 False（重启会回 True）；
  2. 拉 23:17 之后的新包，用同一判据对比 `press01_left_230820.npz`；
  3. 目标滑移未降 → Puppetry 自身漂移；已降但仍动 → 转 T12 查下游；
  4. 补测右臂。
- 验收：关闭前后同判据目标滑移对比写 WORKLOG。

### T12 IK 指令冻结时真机 J1L 自振（下游）

- 优先级：P0
- 状态：`TODO`
- 依赖：需操作员配合复测
- 证据：同一包内 IK 全零冻结窗，真机 J1L 以 ~1s 周期 ±2.2~2.9° 交替振荡（w=331.99~336.99）；全程 J1 实测净 -4.19° vs 指令 -11.09°；测试期间 bridge `program_run_failed` 19→59。
- 机制分析（2026-11-06，详证见 docs/信号链路与离合自运动分析.md 机制③）：bridge `_flush_commands` 是以真机反馈为底的第二闭环（err=IK−fb，`exton_bridge.py:581-593`），|err|>0.001rad(0.057°) 就发 movej——**IK 冻结≠bridge 停发**，拒绝数 19→59 是冻结窗内仍在发的直接证据；每条 movej 为完整 P2P 程序（v=15，小行程 ~1s 节拍，与振荡周期吻合），疑为纠偏 hunt 极限环。ack 通常 <300ms 返回（timeout 仅 25/2092），疑为「受理」而非「完成」应答（未验证）。23:30 复测（state 双发布已关）仍有残余移动 → 机制③嫌疑权重上升。
- 动作：① 对齐 T3 日志 movej 发送/拒绝时间戳与振荡窗口；② 有条件做单步 movej 台架（停 bridge，joint_probe 发一条 3° 外固定位姿 movej，看 status 时刻 vs 反馈稳定时刻 vs 是否过冲）——单步干净则 hunt 为 bridge 连发驱动，单步过冲则控制器侧问题；③ 若确认 bridge 驱动 → T14（hold+迟滞）。
- 验收：给出振荡与拒绝的因果方向或排除证据。

### T13 Puppetry 对 {side}_ee/state 的依赖刻画（离线注入，不动机器人）

- 优先级：P1
- 状态：`TODO`
- 依赖：无（需停 bridge 释放 TCP/避免干扰；不按踏板、不下发真机命令）
- 说明：Puppetry 是 vendor 二进制，其对 `/target_robot/{side}_ee/state` 的消费方式
  （按下瞬间采样锚定 or 按住期间连续参考）决定 T11 实锤后的永久修复选型
  （分析文档第四节方案 a/b/c）。
- 动作：停 bridge；`ros2 topic pub /clutch/left std_msgs/Bool true`；回放录包外骨骼流
  （或保持外骨骼静止）；人工给 `/target_robot/left_ee/state` 发已知轨迹（如 10mm 正弦）；
  观察 `/target_robot/left_ee/target` 是否跟随 state 移动、相位/增益如何。
- 验收：WORKLOG 给出「target 是否随 state 动 + 量级」的原文记录；据此在分析文档
  第四节三方案中选定一方案。
- 风险：注入 state 会同时被 MessageBridge 转发，确认 bridge 已停、真机侧无命令下发。

### T14 bridge hold+迟滞（防纠偏 hunt，机制③修法）——离线已完成，待真机验证

- 优先级：P1（T12 确认 bridge 驱动后升 P0）
- 状态：`DOING`（ZCode，2026-11-06；代码+离线回归完成，见 WORKLOG 同日条目，**真机未验证**）
- 需求：`command_eps_rad=0.001rad(0.057°)` 远小于 movej 到位能力 → IK 冻结后 bridge 持续
  纠偏（hunt，机制③/R1）。加迟滞：|err|≤`hold_enter_rad`(0.01) 该轴停发；目标重新活动
  （`ik_move_eps_rad=0.002`）或 |err|>`hold_rearm_rad`(0.05) 才恢复。
  **UBT 实测教训（pink_bridge_safety.hpp:106-115，已按其实现）**：保持值必须是冻结的
  最后指令/计划，严禁退化为「周期重发当前实测位」——否则复现其 2026-10-13 下垂回归。
- 离线证据（tools/tests/test_bridge_offline.py，18/18 OK）：
  - `test_a2_hysteresis_works_for_small_overshoot`：过冲 0.005 < enter → ≤2 拍停发 ✅
  - `test_a1_hysteresis_insufficient_for_big_overshoot`：**过冲 0.035 ≥ enter 时 err 恒带外，
    迟滞不足仍 hunt（≥45/50 拍在发）**——文档化边界：根治靠 T17 command 底
  - `test_c_following_not_blocked_by_hold`：目标持续变化时 20/20 拍全发，迟滞不卡跟随 ✅
- 真机验证步骤（配合 T16 部署后）：
  1. 基线等价：`ros2 param set /exton_bridge hold_enter_rad 0.001`（≈旧行为）→ 按住离合
     手静 8s×2，确认行为与改前一致；
  2. 开迟滞：`hold_enter_rad 0.01` / `hold_rearm_rad 0.05` → 同场景录包，判据：
     手静窗 `movej_tx` 停止增长、J1L 振荡（±2.2~2.9°/1s）幅度下降或消失；
  3. 对比 `press01_left_230820.npz` 同判据数据写 WORKLOG。
- 参数（运行时可 `ros2 param set` A/B，不重启）：`hold_enter_rad`/`hold_rearm_rad`/
  `ik_move_eps_rad`。风险：改实时发送路径；改前必须备份远端原文件（见 T16 步骤）。

### T15 端到端滞后与运动平滑度量化（跟手性基线测量，先测后调）

- 优先级：P1（优化路线 C1，先测量后调参——UBT P-007 方法论）
- 状态：`TODO`
- 依赖：操作员在场按住离合；`record_compare.py`；20 个历史包可对照
- 动作：按住离合做慢动/快动各 15s，录包 + 取 T3 日志 2s 状态行，量化：
  ① 外骨骼变化 → 真机变化的端到端滞后（ms 级分布）；
  ② 指令/反馈的 stop-go 周期（movej 程序节奏）；
  ③ 有效指令率（sent 增量 ÷ 时间窗）；
  ④ 快动时 IK 指令与真机的最大关节滞后。
- 验收：四个数字写入 WORKLOG（含包名），作为 T14/T17 改造前后的对照基线。
- 负责：AI 分析 + 人工操作

### T16 bridge 健壮性加固 + 离线回归框架（R3/R4/R10）——离线已完成，待部署

- 优先级：P1
- 状态：`DOING`（ZCode，2026-11-06；本地代码与测试完成，**未推远端**）
- 需求：
  - R3 反馈新鲜度看门狗：控制器停回应答但 TCP 存活时，陈旧 fb 被当纠偏底、全链真值源
    失真 → `feedback_timeout_s`(1.0) 内无新反馈即停发+告警；
  - R4 重连陈旧快照：`connect()` 清空 `_latest_joint_rad/_ts`，重连后同步必等新应答；
  - R10 指令流活性：IK 挂死（HANDOFF §五前科）时 rx 计数停止 → `command_timeout_s`(1.0)
    停发+告警（判据用 rx 计数非消息内容：手静时 IK 仍发同值，流活着）；
  - 结构改造：下发计划抽成纯逻辑类 `StepPlanner`（无 ROS/socket，可离线单测），
    `_flush_commands` 变门禁+副作用薄壳；控制语义参数每帧实时读（T11 教训：启动缓存
    会让现场单变量对照变成重启对照）。
- 离线证据：`test_reconnect_drops_stale_snapshot` ✅ / `test_sync_and_feedback_age` ✅ /
  `test_lockstep_accept_mode_high_rate`（受理即回：10/10 全发）✅ /
  `test_lockstep_complete_mode_paced_and_rejects_overlap`（执行完才回：300ms 超时触发、
  重叠拒绝发生——R2 两种语义的行为刻画，供 T12 现场对照）✅
- **真机部署步骤（改远端前必须备份）**：
  ```bash
  # 1) 备份
  sshpass -p ' ' ssh -p 2222 tc@127.0.0.1 \
    'cp /home/tc/tcvg/exton_bridge.py /home/tc/tcvg/exton_bridge.py.bak_$(date +%m%d_%H%M)'
  # 2) 推送
  sshpass -p ' ' scp -P 2222 tools/exton_bridge.py tc@127.0.0.1:/home/tc/tcvg/exton_bridge.py
  # 3) 重启 T3（参数与 start_teleop.sh 一致，见 AGENTS §3；先 pgrep 拼接防自杀）
  # 4) 验证：connected/fb_synced 正常、md5sum 与本地一致、
  #    按住离合慢动 → sent 增长、拒绝不新增、手静表现与改前一致（默认参数=旧行为）
  ```
  新参数默认值不改变现行为（`step_base=feedback`），迟滞默认开启但可用
  `hold_enter_rad 0.001` 退化为旧行为——现场先跑 1 验证等价再开 2。
- 验收：WORKLOG 记录远端 md5、重启后状态行、按住离合慢动的 sent/拒绝计数、
  手静窗对比；离线回归在推送前重跑一次全绿。

### T17 bridge 下发语义对齐 UBT（step_base=command 开环斜坡）——离线已完成，待真机 A/B

- 优先级：P2（跟手性根治候选，依赖 T15 基线 + T16 部署）
- 状态：`DOING`（ZCode，2026-11-06；`step_base` 参数门控已实现并离线回归，**默认 feedback**
  即现行为，真机 A/B 只是 `ros2 param set`，无需再改代码）
- 说明（详证 docs/exton_bridge_对比UBT与优化路线.md §四/§五）：
  `step_base=command` 时步进底=上一拍计划值（开环斜坡），到位判据 `|IK−计划|` 与反馈
  无关 → 收敛即停发，hunt 从构造上消失；配套 `tracking_leash_rad`(0.6) 指令-实测偏离
  护栏（超限整帧冻结，0.8×leash 内自动恢复）+ 拒绝时 `invalidate()` 有限重试
  （否则指令被拒后桥会认为已收敛而不补发）。
- 离线证据（test_bridge_offline.py）：
  - `test_a3_command_mode_immune_to_big_overshoot`：2° 过冲下 ≤2 拍停发（feedback 底为 ≥45）✅
  - `test_b_sag_model_feedback_hunts_command_stops`：UBT 下垂模型，feedback 底 ≥45 拍
    「追着下垂走」、command 底 ≤2 拍停——复现 UBT 实测教训 ✅
  - `test_command_mode_ramp_from_last_plan`：0.5rad 目标 3 拍斜坡收敛（0.2/拍）✅
  - `test_invalidate_rebases`：拒绝后重估重试 ✅
- 真机验证步骤：T15 基线录完后 `ros2 param set /exton_bridge step_base command` →
  同场景录包对比：端到端滞后、stop-go 周期、手静窗 movej_tx、拒绝计数；
  异常即 `step_base feedback` 回滚（不重启）。
- 验收：跟手段端到端滞后较 T15 基线显著下降；手静窗零纠偏帧；无新增拒绝。
- 风险：动的是下发核心语义；leash/invalidate 行为已离线覆盖，真机异常直接参数回滚。

### T18 双臂场景容量与耦合测量（右臂首测 + 双臂同动）

- 优先级：P0（UBT 三次教训全部在双臂场景暴露，而本项目双臂从未测过）
- 状态：`TODO`
- 依赖：操作员在场；T16 部署后做（bridge 新状态行可用）
- 依据：docs/左右臂独立计算分析.md——UBT C-001（双臂抖动）/P-007（双臂细颤）/P-016 M4.5
  （GIL 串行化，顺序双臂 5.7ms > 本项目 250Hz 的 4ms 预算）全是「单臂正常、双臂暴露」。
- 动作：
  1. **右臂单臂首测**：按住右离合慢动/手静/松开即停，录包（右臂从未测过）；
  2. **双臂同按住同动** 15s：`ros2 topic hz /target_robot/joint_commands` 观察是否
     从 250Hz 掉频（判据：≥225Hz 合格；同栈基准顺序双臂 5.7ms > 4ms 预算，预判会掉）；
  3. **双臂交叠按下**：右臂跟手中按下左离合——观察右臂是否顿/跳（T19 修复前应为
     可见跳变；修复后应无）；
  4. T3 日志回查：T12 拒绝窗（19→59）内右臂槽位是否同帧停发（协议连坐实证）。
- 验收：四个场景的量化数据（频率分布 / 跳变幅度 / 连坐证据）写 WORKLOG；
  据此裁定 T19 的 per-arm 进程拆分是否立项。
- 负责：AI 分析 + 人工操作

### T19 per-arm 隔离改造包——离线已完成，待真机验证

- 优先级：P1（T18 数据决定是否追加进程拆分）
- 状态：`DOING`（ZCode，2026-11-06；两项修复已实现并过离线回归，**未推远端**）
- 需求（详证 docs/左右臂独立计算分析.md §二）：
  - **B 修复（ik_node 按臂重置）**：原任一臂离合上升沿 `q_commanded = q_measured.copy()`
    + `last_velocity.fill(0)` 重置两臂全量——右臂跟手中按左离合会让右臂指令跳回
    实测位（bridge 限幅追认 → 可见顿跳）。改为只重置本臂 7 轴；posture 锚也只锚本臂
    （另一臂锚保持其自己按下时的值，跟中不被扰动）。单臂行为不变（另一臂本就在实测位）。
  - **D 修复（StepPlanner per-arm leash）**：原 leash worst 跨两臂取 max、整帧冻结——
    左臂超限连坐右臂。改为按臂判定：超限臂 7 轴冻结在计划值，另一臂照常
    （UBT ArmSafetyGate×2 同款）；全部使能臂超限才整帧冻结（保留原日志语义）。
- 离线证据：test_bridge_offline.py **19/19 OK**，新增
  `test_leash_per_arm_isolation`（左臂超限冻结、右臂照常更新）+
  `test_leash_hold_and_release` 改按臂语义断言。
- 部署注意：
  - ik_node 本地已领先远端（远端 2af30692）。推前备份 + 清 `__pycache__` + 重启 T5；
    **重启后 `publish_commanded_ee_state` 回默认 True，T11 对照若还在进行需重新置 False**；
  - bridge（T16 包）与 ik_node（本包）可一次真机窗口一起推，验证序列：
    T16 等价验证 → T14 A/B → T18 双臂测量（此时 B 修复已生效）→ T17 command A/B。
- 验收：T18 场景 3（交叠按下）右臂指令无 >0.01rad 级跳变；双臂同动无新增异常。
- 风险：ik_node 改的是离合沿处理，属安全相关路径；单臂回归（离线重放脚本在远端
  /tmp）+ 真机先单臂后双臂。

### T20 URDF 与真机运动学对账（明早现场执行清单，工具已就绪）

- 优先级：P0（今晚离线预备已完成，见下；T10 近奇异/T06 限位疑点均可能同源）
- 状态：`DONE`（判决完成，2026-11-06 真机 6 样本：位置残差 292~1895mm/姿态 100~167°，符号枚举救不回 → **结构性差异不可修补**；路线=厂商原生 URDF/D-H（档案手眼平台有「高度一致」先例）或数值重建（样本不足，如需另立项 T25）。get_config 真机限位已取得并暴露 T23）
- 今晚离线侦察新发现（证据见 WORKLOG 同日条目）：
  1. **协议 V0.3 有 `get_tcp_pose`**（双臂 TCP 位姿，pose[14]=left7+right7，
     **四元数 qw-first 与 ROS 相反**；「单位：弧度」疑文档笔误，xyz 应为米）→
     对账不需要物理测量；
  2. **协议有 `get_config`**（upperLimits/lowerLimits）→ 真机软限位直接查询，不必微动试探；
  3. **档案 docx**：右臂软限位表（2026-07-06）与 URDF ±170° 一刀切**全轴结构性差异**
     （J3 差 130.5°、J5 差 80°，报告已出）；**末端工具长度 0.145m**（TCP 含工具偏移，
     对比必须做工具假设）；关节 14（右臂腕部）有**反复堵转**故障史（右臂首测注意）；
  4. **协议 movej 应答 status=`program_run_finish`** → R2 的 ack 语义倾向「执行完才回」
     （仍需 T12 台架最终确认）；
  5. URDF 源 CSV 显示左右臂关节 origin rpy **非简单镜像**（Joint1 z=-π vs Joint1L z=0）
     ——UBT「镜像建模」隐患形态，待真机数据判决。
- 已有候选结论（待真机确认）：**左臂在 bridge 里用 URDF ±177° 裁剪、无档案限位表**，
  而真机左臂软限位大概率与右臂同构（UBT 经验：驱动接口双臂统一）→ 左臂 IK 可指令到
  真机不可达区间 → 未解释行为的候选来源；修法候选=从 get_config 动态取真机限位
  喂给 bridge 两臂（替换静态档案表）。
- 工具（已备，本地 tools/dm-exo/urdf_fk_calib/，推到 tc 主机跑）：
  - `collect_samples.py`：--probe（先验协议）/ --config（拉软限位）/ --collect N（交互采
    N 组 q17+tcp14，**只读不发运动**，操作员摆位姿）
  - `compare_fk.py`（tc 主机 pink venv 跑）：URDF FK vs TCP，位置+姿态双维度残差、
    工具 0.145m 两假设（raw / 沿工具 z -0.145）、s∈{±1}^7 符号枚举可救性判决
  - 四元数 qw-first 数学已离线自检（恒等/绕z90°/绕x180°）；pinocchio 部分无法本地测，
    首跑注意 import/帧名错误即修
- **明早执行序列（约 1h，先于 T14/T15/T16；T09 已裁定为优先验证任务，与本序列共用「停 bridge」窗口）**：
  1. （10min）停 T3 bridge（TCP 单连接）→ `collect_samples.py --probe` 验证三条命令
     真实应答 → `--config` 拉真机软限位存盘；
  1.5 （15min）**T09 canfd 台架**（见 T09 条目）：-100 占位 → 50Hz 小幅连发 → 试探
     上限 → 回滚确认（全程小幅慢速、操作员知情确认）；
  2. （20min）操作员摆位姿，`--collect 20`：覆盖工作空间（高位/低位/近身/外缘/
     近奇异构型各几组）；**左臂优先**（T10 异常臂），右臂顺带 5 组（注意 J14 堵转史，
     慢动避极限）；每臂采数时另一臂保持不动（compare_fk 按单臂判决）；
  3. （10min）`compare_fk.py` 判决：位置+姿态残差 → <20mm/2° 门限判定 → 符号枚举
     可救性分支；
  4. （10min）判决落地：可救 → 约定修正（映射或重建 URDF，走 UBT 同源重建路线）；
     不可救 → 向厂商索控制器 D-H/运动学参数；顺带评估「get_config 动态限位喂
     bridge/IK」改造（替换静态档案表+URDF ±170°）；
  5. 恢复现场：重启 T3 bridge；若 T16/T19 包已部署按新参数核对。
- 验收：WORKLOG 记录①真机软限位表 vs URDF/档案三方对比；②20 组残差表
  （位置 mm+姿态°，两种工具假设）；③符号枚举判决与路线决定。
- 负责：AI 工具 + 操作员摆位姿
- 关联：T06（限位，get_config 直接解决大半）、T10（近奇异构型真实性）、T13、T16/T19（部署）

### T21 链路监视节点 chain_monitor（只读外挂观测，P1）

- 优先级：P1（S2/S3 vendor 黑盒不可加日志的外挂替代，UBT `pink_signal_chain_monitor`
  同款思路；评估见 docs/日志观测性评估.md）
- 状态：`DOING`（Codex，2026-11-06 16:25 上线常驻只读运行：远端
  `~/桌面/dm-exo/chain_monitor.py`，输出 `flow_1106_1630.csv/.log`；已实测捕获一次控制器失联。验收「手静无误报」与「目标滑移速度对账」仍待完整遥操段）
- 需求：vendor 组件（MessageBridge/Puppetry）无法内部加日志，本周 T11/T12 归因
  全靠录包离线分析；需要一个独立只读进程在线回答跨模块问题。
- 规格：新节点 `tools/dm-exo/chain_monitor.py`（仅订阅，零发布，不碰任何控制路径）：
  - 订阅 (C) target×2 / (E) state×2 / (F) commands / (D) client_joint_states / (B) clutch×2；
  - 2s 打印：**目标流速度**（手静窗滑移 mm/s=T11 在线判据）、(E) 发布者数与
    指令FK-vs-实测FK 残差（双发布=机制②报警）、(F)→(D) 关节级指令-反馈滞后
    （跟随延迟在线版=T15 连续观测）、clutch 事件时间线（跨日志对齐基准）。
- 验收：手静按住时监视器直接打出目标滑移速度并与录包离线值一致（±20%）；
  全链路静置时无告警误报。
- 风险：零（纯只读）；随 T16 部署窗口一起上（挂进 start_teleop.sh 或单独跑）。

### T27 左臂遥操方向反转——离线定责（上游无罪；断点=URDF↔真机镜像 + 限位触发电机死锁）

- 优先级：P0（遥操可用性的两个 blockers 各归其位：方向归 T20 翻转表，死锁归 T26+T23）
- 状态：`DONE`（定责完成，ZCode，2026-11-06 14:54；修复分属 T20/T23/T26，见 WORKLOG 同日条目）
- 证据（dir01_left.npz，24.5MB，远端 ~/桌面/dm-exo/）：段#1 操作员前推，外骨骼末端 x=+0.2603m ↔
  目标末端 x=+0.2603m **corr=+1.000**（段#2 回拉同样 +1.000）→ 上游映射无罪；URDF-FK(指令)↔目标
  corr=+0.999 → IK 按 URDF 自洽；真机段#1 仅半执行（比值 0.45–0.64，J1/J3/J6≈0），
  t=66.43s 起反馈位级冻结（与首条 program_run_failed 同秒），此后 5 次按压全死 → 断点=S7 电机层
  （T26 复发）；死亡时刻 J4L 距真机软限位 6.8°、J7L 余 11.3°，IK 正按 URDF ±177° 向越界驱动。
- 结论：①「前推后移」= T20 结构性镜像的遥操表现，修法=翻转表（urdf_axis_probe.py 复跑），
  不是上游改帧；②「臂死」= T26 复发，疑似被「宽限位驱动逼近真机软限位」触发（未实锤因果），
  恢复后不修限位（T23）必复发。
- 下一步：电机复活后 ① 部署 T23 或临时把 bridge 左臂限位（tools/exton_bridge.py:681
  `self.left_limits = URDF_LIMITS_RAD`）改为 get_config 实测值；② 复跑 urdf_axis_probe.py 建翻转表。

### T23 bridge/IK 换 get_config 动态限位（现场证据已实锤，P1→建议升 P0：左臂遥操恢复前置条件）

- 优先级：P1（2026-11-06 真机当场抓获：档案表 j5=±1.57rad vs 真机 get_config j5=±2.79rad，
  差 1.22rad；用户合法手摆位姿被 bridge 裁剪触发永久纠偏循环——**bridge 会把臂从合法
  位姿往过时档案限位里拉**，历史「手臂自己动」的候选贡献者之一，未定量归因）
- 状态：`TODO`（临时止血已生效：bridge 以 `use_archive_right_limits:=false` 运行；
  2026-11-06 18:18 ZCode：静态表侧已全同步——URDF 双臂 == LIVE_LIMITS_RAD == get_config−0.02rad，
  右臂 URDF ±2.967 缺口补齐；get_config 动态拉取（本地 use_live_limits 代码）仍未部署）
- 动作：① 启动时/周期性发 get_config，取真机限位喂 bridge 两臂（替换 ARCHIVE_RIGHT_LIMITS_DEG
  与 URDF_LIMITS_RAD）；② get_config 失败时的兜底策略（沿用当前值+告警）；③ IK 侧
  ConfigurationLimit 同步换真机限位；④ 顺手处理左臂同问题（现在左臂用 URDF ±177°）。
- 验收：手动摆任意限内位姿 → bridge max_err≈0 零纠偏；离线回归含限位注入用例。
- 风险：改限位来源=安全相关路径，先离线回归再上；注意 get_config 只回 14 轴（腰/头无限位）。

### T24 左肘电机 4 厂家检修（硬件，用户渠道）

- 优先级：P0（硬件）
- 状态：`TODO`（用户渠道）
- 证据：2026-11-06 两次独立堵转（错误码 17039362/电机 detail=8），第一次采数扫位姿
  （收臂组合）、第二次多轴换向承力；clear→enable 立即复现，断电重启可临时恢复——
  与档案「电机 14 反复堵转」处置记录完全同型（根治=巨蟹智能单机检修）。
- 动作：报修时附两次堵转的位姿与负载描述（WORKLOG 同日条目）；**检修前禁用遥操**
  （肘持续承力必复现）；档案电机 14 或可合并报修。
- **随报修一并向厂商索取的资料清单（2026-11-06 ZCode 立项；1+2 优先，拿到即触发 T20 重新对账）**：
  1. 控制器内部运动学：双臂各自 base→法兰 D-H（杆长/扭角/偏置）+ 关节零位偏置 +
     **每关节正方向定义**（电机正转对应关节角正号的物理转向——这正是 T30 翻转表要探的内容，厂商能直接告知）；
  2. 关节序/轴映射表：控制器 1..7 ↔ 物理关节 ↔ URDF Joint1..7(L)；左右臂安装关系
     （是否镜像）与各自 base 系原点/朝向；
  3. 与固件 dual_arm_app 0.7.88 同版本的原生 URDF 或 D-H 模型文件；
  4. TCP 定义：法兰→TCP 变换（档案载 0.145m 工具偏移，需含方向）+ `get_tcp_pose`
     的基系约定与四元数序（qw-first 已实测，需书面确认）；
  5. （有则最好）exo 侧：MessageBridge/Puppetry 的 `{side}_ee/state` 基系约定文档
     ——有了它 sixdir 标定可以从"实测拟合"升级为"解析计算+实测验证"。
  **验收门（拿到先离线判决，不上真机）**：用 6 组现存 (q, get_tcp_pose) 样本跑
  `compare_fk.py`，残差 <20mm/2° 才允许部署；不达标 = 资料与固件不符，退回。
- 验收：厂家检修报告；检修后轻载验证肘部全行程。

### T25 JuxieLinker 服务不随臂上电自启（硬件/站点层，P0 阻塞）

- 优先级：P0（阻塞一切真机操作）
- 状态：`DONE`（2026-11-06 结案+衍生新问题 T26）。根因=控制器磁盘 100% 满（日志 24G）导致
  固件启动崩溃 + 固件无自启机制。恢复链已固化：清磁盘 → can_init →
  `DUAL_ARM_SDK_CONFIG=/home/root/DualArm/usr/etc` + LD_LIBRARY_PATH 启动 interface_node →
  网页 5566 登录（密码 123456）→ 网页 enable。衍生问题见 T26。
- 问题：臂断电重启后，.60 主机存活（ping 通、22 开）但 JuxieLinker 控制服务未起
  （5566 Web UI 与 30485 控制端口均关）。与档案「固件随开机自动启动」矛盾——
  可能本次断电方式（只断臂电源、控制程序宿主未断）导致服务未触发重启，或自启机制
  损坏。此前多次断电后服务都能自行恢复，本次例外。
- 已知信息（档案）：固件 dual_arm_app 0.7.88（ARM aarch64）装于控制器
  `/home/root/DualArm/usr/`；**控制器 SSH：192.168.61.60 root/wanghuo（tc 主机可达）**。
- 动作（需操作员裁定是否远程处置）：
  1. SSH 上 .60 只读诊断：进程列表（dual_arm_app 是否在）、服务/自启配置状态；
  2. 手动拉起固件（按厂商安装布局 /home/root/DualArm/usr/bin 下找入口）；
  3. 验证 5566/30485 开放、canfd 零增量帧接受；
  4. 若手动拉起成功 → 固化「自启修复/启动检查」建议给厂商；失败 → 联系巨蟹智能。
- 验收：30485 恢复监听、bridge 自动接入、遥操可用；根因（为何本次不自启）写入档案级记录。
- 负责：AI 远程处置 + 操作员现场确认

### T26 电机 CAN 零应答（臂动力层，P0 阻塞）

- 优先级：P0（臂完全不可动）
- 状态：`DOING`（2026-11-06；**当日第二次复发，且死亡瞬间首次被录包捕获**，证据见 WORKLOG 14:54 条目）
- Codex 15:56–15:59 只读复核：180 秒包 `monitor_1106_1556.npz` 已完成；两路 CAN 在 5 秒内 TX 各 +2499、RX 均 0。录到两次左离合下目标/IK 活动而反馈样本为 0；bridge 同时被其他操作者重启，须分开分析。具体 fault、触发原因及恢复后跟手仍未验证，详见 WORKLOG 对应条目。
- 现象：固件正常监听、协议全命令「成功」（含 enable/clear_fault/反馈查询），但 can0/can1
  **TX ~500帧/s 持续、RX=0 冻结**（TX_ERR=RX_ERR=0——收发器 ACK 但电机主控不应答）；
  TCP canfd/movej 帧受理不执行；固件 GUI 自身 movej 也不动。
- **复发（14:42，新证据）**：电机上午恢复后曾在遥操中执行运动（dir01_left.npz 段#1 反馈在变，
  14:26 press02 全轴忠实跟随比值 0.93–1.05），14:42:07 遥操进行中突然批量拒绝 canfd
  （首条 `累计1` @1793947327）+ 左臂反馈自 66.43s 起位级冻结 172s；现测 can0 RX=0 TX=946,762 /
  can1 RX=0 TX=920,810 / stats RXF=0。**触发相关性**：死亡时 J4L=-32.75°（距真机软限位 -39.5°
  余 6.8°）、J7L=-148.6°（距 -159.9° 余 11.3°），IK 正按 URDF ±177° 宽限位向越界方向驱动
  （因果方向未实锤，但时间耦合精确到秒）。
- 已排除：急停（已旋出）、GUI 占用（已关）、拖动模式（已查）、断路器（操作员已合，待复核是否
  正确一路）、协议使能（成功但无效）。
- 已知线索：CAN RX 曾有 1224 帧历史计数（电机曾短暂应答）；GUI↔协议读数可疑偏差
  （J5R 17°/J7L 55°，未同步复核）；今日三次臂断电、一次整机重启。
- 动作：① 操作员核实臂供电链（基座电源开关/指示灯/正确断路器）；② GUI↔协议同步读数对比
  （验反馈活性）；③ 若供电确认正常仍零应答 → 报巨蟹智能（并入 T24 报修包：肘堵转×2 +
  电机零应答 + 磁盘满不自启 + 档案限位过时；报修材料新增「遥操中死亡」实录包 dir01_left.npz）；
  ④ **恢复后左臂遥操的前置条件=T27/T23（限位不修必复发）**。
- 验收：CAN RX 恢复跳动、get_joint_position 反馈活性确认、驱动测试位移>0。

### T29 末端增量方向标定 + 遥操降敏（交接下一 AI，未实施）

- 优先级：P0（当前可遥操但操作员体感“方向十分混乱、过于灵敏”）
- 状态：`DOING`（2026-11-06 Codex；用户已改为直接上手测试，见下；限位已先行修正）
- 限位前置（2026-11-06 17:24 已部署）：左臂 URDF Joint1L~7L = 控制器 get_config 实测限位
  （J1/J3/J5/J7 ±2.77，J2 [−2.3362,0.24]，J4 [−0.67,2.24]，J6 ±1.55，单位 rad），
  与 exton_bridge.py 的 LIVE_LIMITS_RAD 同值；远端 src/install 同一文件（install 为软链），
  md5 `abf816f429b09e954f314b1737cbff07`；备份 `*.bak_1106_1723`。整链 17:24:16 重启完成。
- 已确认控制语义：T5 不是外骨骼关节到真机关节的直接映射；它在离合锚定后使用
  Puppetry 末端位姿相对增量：
  `dp=(p_exo-p_exo0)*mirror`，`p_des=p_robot0+dp`，
  `R_des=(R_exo*R_exo0^-1)*R_robot0`。即平移/姿态末端增量遥操。
- 当前运行参数（16:5x 复核）：左平移符号 `[-1,+1,+1]`、右 `[+1,+1,+1]`，
  `ee_position_cost=300`、`ee_orientation_cost=40`；没有位置/姿态比例参数，平移与姿态均
  1:1；bridge `movej_canfd/50Hz`、`max_joint_step_rad=0.20`、`follow_ratio=1.0`。
- 只读证据：`monitor_1106_1556.npz` 中左 target 与 `/ee_poses` 左末端位置对角相关约
  x=0.993、y=0.955、z=0.998，证明上游确实传末端位姿；该包没有纯单轴动作，不能据此
  求完整方向矩阵。历史 `dir01_left.npz` 已证明外骨骼 x→target x corr=1、指令 FK→target
  corr=0.999，而真机方向仍反，断点在 URDF/真机映射层。
- 根因分层：
  1. 当前 `mirror_xyz` 只能同轴翻符号，不能修正轴交换或任意坐标旋转；左 X 翻转只是临时止血；
  2. 姿态相对旋转未做坐标变换和比例缩放，会带出难理解的多关节动作；
  3. T20 已实锤当前 URDF 与控制器运动学结构性不一致（位置 292~1895mm、姿态 100~167°），
     因此仅改 mirror 不能完成真机方向校准。
- 推荐实现顺序（先离线测试，真机需 donghai 确认）：
  1. 新增每臂 `position_scale`（初值 0.30）与 `orientation_scale`（初值 0.20）；
  2. 新增每臂完整 `3×3 position_map`，用外骨骼纯 ±X/±Y/±Z 与控制器真实 TCP 数据拟合，
     不再只用三个符号；
  3. 姿态用同一坐标变换 `C*R_delta*C^T`，再以旋转向量乘 `orientation_scale`；
  4. 校准初期将姿态 cost 置 0，仅低倍率做平移六方向；平移验收后逐轴恢复姿态；
  5. bridge 的 `max_joint_step_rad` 建议从 0.20 降到 0.02 作为关节速度第二层保护；
     保持 canfd 50Hz，不靠降低频率降敏；`move_speed=15` 在 canfd 模式不生效。
- **2026-11-06 19:00 已实现并部署（ZCode）**：`master_slave_map.py`（ArmPoseMap：3×3 映射+姿态
  换基含 det 符号+缩放+门禁+JSON 存取+Procrustes）+ ik_node 接线（calib_file_* 参数）+
  `tools/tests/test_master_slave_map.py` 15 项离线回归 + `tools/dm-exo/sixdir_calib.py`
  （segment/fit measured|desired/verify/selftest）+ `docs/方向诊断与校准系统.md`。
  **默认参数==旧行为（离线逐位断言），部署零行为变化**；远端 md5 `1ea272d1/caf170c6`，
  运行态 pose_map 日志==diag(mirror) 证实。剩余=真机六方向标定+姿态层+T30 探针配合。
- 验收建议：外骨骼单轴移动 50mm，在 scale=0.30 时真机 TCP 对应方向移动约 15mm，
  正负方向一致、其他轴泄漏可量化且 <3mm；外骨骼单轴旋转 20°，scale=0.20 时真机约 4°；
  左右臂分别验收，松离合立即停，reject=0。
- 关键风险：在厂商匹配 URDF/D-H 到位前，输入坐标矩阵校准只能改善操作者方向感，不能从根本
  修复 IK 模型到真机的结构性错误；优先索取厂商 URDF/D-H，或用 `(q,get_tcp_pose)` 数值重建。

### T30 翻转表真机探针复跑（urdf_axis_probe，方向对齐唯一数据源）

- 优先级：P0（T27/T29 方向修复前置：符号层 [1,-1,1,-1,1,-1,1] 离线已高置信锁定但置换不可辨，
  翻转表只有真机单轴微扰能定——17:2x 会话被 fault 阻断，本任务接续）
- 状态：`DOING`（ZCode，2026-11-06 17:5x；操作员确认现场可摆姿，窗口执行；
  19:0x T29 映射系统已部署、链路健康（reject=0），探针随时可跑——J7 已归零，
  新限位 ±159.9°，超软限位 fault 需先在 5566 再清一次（18:10 探针仍见 softwareErrors 34013185））
- 离线预研（2026-11-06 ZCode，`tools/dm-exo/offline_direction_audit.py`，证据见 WORKLOG 同日条目）：
  9 包 4435 样本回归证实左臂 ik↔rob **数值层**全轴正跟踪无反演、健康窗全轴执行（增益 0.62~0.99），
  跟踪滞后 240~400ms——数值/固件层嫌疑排除，翻转表确认只能由本任务真机探针给出；
  右臂现存录包无健康基准（仅贫执行抓拍），右臂探针/基线一并排进窗口。
- 动作：`pink/bin/python /home/tc/tcvg/urdf_fk_calib/urdf_axis_probe.py <hr_up URDF> left`
  （每关节 +0.12rad 去回微扰，约 3 分钟）；左臂完成后视情况再做 right（J14 有堵转史，需操作员点头）。
- 验收：逐关节 ALIGNED/INVERTED/UNCERTAIN 判定 + 翻转表原文落 WORKLOG；UNCERTAIN 关节
  换构型补测或加大样本。
- 后续（修复落地，另记）：按表改 URDF 左链 axis（RViz/FK/IK 同步生效）+ bridge 左臂
  收发两侧同表符号映射（先 record_compare 离线回归再真机）。
- 负责：AI 工具 + 操作员监护

## 三、别接的活（明确出界）

- 改 `move_speed` / `auto_enable`（安全设定，需人工在示教器确认）
- 动 `192.168.61.60` 的凭据与固件
- 重装 `pink` venv（ARM64，版本钉死；出问题先问人）
- 把明文凭据写进任何会外传的地方（Git / 云端 / 聊天工具）

### T28 电机堵转 fault 锁存 → 全拒不下发（2026-11-06 真机实锤）

- 优先级：P0
- 状态：`DONE`（2026-11-06 16:25 用户确认：就是第五关节堵转导致无法遥操；fault 已清+enable 后恢复。探针证据 motor_id=7 与「第五关节」的电机号↔关节号映射未定，已在持久记忆记录签名）
- 现象：开机后能遥操一会，随后真臂完全不动；TCP/反馈/IK 全链路正常，bridge 50Hz 下发，控制器恒回 `program_run_failed`（每 2s 恰 32 条错误应答，疑有节流）。
- 证据（2026-11-06 16:10，probe_device_state.py 原文）：
  `{"command":"get_device_state","erroeCode":1,"hardwareErrors":[17235970],"motorErrorDetails":[{"general_desc":"电机7 报错","hardware_error_code":17235970,"motor_detail_code":8,"motor_detail_desc":"堵转","motor_id":7}],"softwareErrors":[],"state":"fault"}`
  当前关节角 J7=-0.043 rad（不在限位），堵转发生在遥操过程中（J7 被指令顶死→过流→锁 fault）。
- 处置：操作员松离合 → 5566 Web UI 清故障（clear_fault）→ 人工 enable → 重启 T3 → 按离合验证。
- 遗留风险（新任务候选）：bridge 启动即追 IK 目标与实臂的大偏差（max_err≈1.15），纠偏帧风暴可能诱发堵转/fault；建议加「启动后首个目标偏差>阈值则 hold，等首次按离合再追」。
- 负责：操作员 + AI
