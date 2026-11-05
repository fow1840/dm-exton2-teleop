# CHANGELOG —— 本地仓库变更记录

> 仓库：`/home/dm/下载/dm-exton2-teleop`（本地专用，无远端）。
> 每笔提交的「为什么」详见对应 WORKLOG 条目与分析文档；本文件只做变更索引。

## ⚠️ 安全边界

- 本仓库含**明文凭据**（AGENTS/HANDOFF 中的跳板机与 tc 账号密码、内网 IP）——这是
  现场操作文档的固有内容。**仅限本地保存，禁止 push 到任何外部/云端远端**（AGENTS §三
  明文规定）。如需外传，先整体替换凭据字段。

## 提交索引（时间序）

### `811e6de` 初始快照（2026-11-05 现场基线）
- 内容：AGENTS/HANDOFF/TASKS/WORKLOG 四台账 + ros2_ws 工作副本（ik_node.py 含
  T10 手静冻结与 T11 运行时读参修复）+ tools/（bridge、探针、脚本）+
  厂商包源码部分（DM-EXton2ProductPackage_tianchuang 的 src/Install/文档）。
- 排除（.gitignore）：pink venv（588MB）、colcon build/install/log、*.zip/*.apk、
  录包 *.npz/*.db3/*.mcap、__pycache__。
- 基线核对：ik_node.py 与远端 md5 一致（WORKLOG 21:45 条目）。

### `44d9ba8` 现场台账更新 + 信号链路根因分析
- 并入另一会话（Codex）23:30 的现场条目：T11 复测（口述）「按住离合仍动，幅度小了些」
  → T11 转 BLOCKED。
- 新增 `docs/信号链路与离合自运动分析.md`：三闭环结构（IK 内环 / bridge 反馈纠偏环 /
  Puppetry state 锚定环），机制①②③拆解，UBT-walker-s2 经验对照表。
- TASKS 新增：T13（Puppetry state 依赖离线注入）、T14（bridge hold+迟滞）。

### `b82586c` 文档补 UBT 台账绝对路径
- 仅一行：分析文档头补 `/home/dm/文档/UBT-walker-s2/` 路径。

### `046508e` exton_bridge.py 信号流专项分析
- 新增 `docs/exton_bridge_信号流分析.md`：线程/锁模型、下发/反馈双向生命周期、
  参数表（现场值待核对）、风险清单 R1~R7、阻塞版→async2→lock-step 演进史。
- 三份 bridge 拷贝 md5 核对：`ros2_ws/scripts/` 与 18:54 阻塞版备份逐字节相同（d750d22b），
  `tools/` 为现役 async2+lock-step 版（fe58738a）；远端真身隧道未通未核对。
- 顺手入库两份协议参考原件：`机械臂通信协议说明_V0.3.pdf`、
  `机器人手眼标定平台系统信息档案.docx`（原在工作区未跟踪，一并纳管）。

### `3ba39ab` bridge 对比 UBT 与优化路线
- 新增 `docs/exton_bridge_对比UBT与优化路线.md`：10 维架构对比（核心差异=步进底：
  EX 以真机反馈闭环纠偏 vs UBT 以上一拍指令开环斜坡）、跟手性延迟预算分解
  （250–500ms 级、有效指令率 ~1–3Hz 推断）、新问题 R8 走停步进 / R9 无会话保活 /
  R10 无输入超时 / R11 无偏差锁存、A/B/C 优化路线定序。
- UBT 冻结保持实测教训（pink_bridge_safety.hpp:106-115）抄录进 T14 作硬约束。
- TASKS 新增 T15（跟手性基线测量）、T17（下发语义对齐）。

### `0682960` bridge 离线就绪包（代码 + 测试）
- `tools/exton_bridge.py` 重构（+309/−70）：
  - 下发计划抽成纯逻辑类 `StepPlanner`（无 ROS/socket，可离线单测），
    `_flush_commands` 变「门禁 + 副作用」薄壳；
  - T14 迟滞：`hold_enter_rad`(0.01)/`hold_rearm_rad`(0.05)/`ik_move_eps_rad`(0.002)；
  - T17 `step_base`（默认 `feedback`=现行为）：`command` 模式=开环斜坡 + `tracking_leash_rad`
    护栏 + 拒绝后 `invalidate()` 有限重试；
  - R3 反馈新鲜度看门狗（`feedback_timeout_s=1.0`）；R4 重连清旧快照；
    R10 指令流活性超时（`command_timeout_s=1.0`，rx 计数判据）；
  - 语义类参数每帧实时读（T11 教训：现场单变量对照不再需要重启）。
- 新增 `tools/tests/`：`fake_controller.py`（协议 V0.3 假控制器：ack=受理/完成两模式、
  过冲/下垂模型）+ `test_bridge_offline.py`（L1 纯逻辑 / L2 闭环仿真 / L3 真 socket，
  stdlib unittest 无第三方依赖）。
- 回归结果：**18/18 OK**。关键证据：hunt 复现（旧行为 50 拍≥45 发）、command 底免疫
  （≤2 发）、UBT 下垂教训复现、迟滞不卡跟随（20/20）、R4 重连清快照、
  R2 两语义行为刻画（accept=10/10 全发；complete+0.5s=超时触发+重叠拒绝）。
- 过程中修掉两个真缺陷：`invalidate()` 未清 hold 锁（拒绝后重试发不出）；
  `ik_moved` 当帧被 enter 条件重新锁住（慢速跟随被卡）。
- TASKS：T14/T16/T17 转 `DOING`（离线完成、真机未验证），T16 含远端部署步骤
  （备份→scp→重启→验证）。

### `dfe7238` HANDOFF §四 补离线就绪包结论
- 一行：离线包就绪状态、默认参数=现行为、**本地 bridge 领先远端，推送前必须备份**。


### 本轮追加（2026-11-06，见 WORKLOG 同日条目）
- `docs/左右臂独立计算分析.md`：UBT per-arm 决策链回放（C-001/P-007/P-016 M4.5 GIL 基准）
  + DM-EXton2 对照（ik_node 双臂全量重置 bug、250Hz vs 顺序双臂 5.67ms 容量缺口、协议连坐）。
- T19 离线修复：ik_node 离合沿按臂重置（q_commanded/last_velocity/posture 锚）；
  StepPlanner per-arm leash。回归 19/19 OK。TASKS 立 T18（双臂测量，P0）。

- `docs/UBT_URDF对账方法论.md`：UBT URDF-vs-真机 发现/校准全过程回放（七条可迁移
  规则：真值源先行/权威分级/限位表便宜检查/枚举拟合判可救性/同源重建+双维度回归/
  X_map 帧标定/数据纪律）；TASKS 立 T20。

## 当前状态与下一步

- 本地 vs 远端：`tools/exton_bridge.py` **领先远端**（远端仍是 fe58738a 版）；
  ik_node 本地与远端一致（2af30692）。
- 待真机序列（TASKS T16→T14→T15→T17）：
  1. T16 部署 bridge（先备份远端；`hold_enter_rad 0.001` 验证与旧行为等价）
  2. T14 参数 A/B：开迟滞 → 手静窗 movej_tx 停增、J1L 振荡消失
  3. T15 基线测量：端到端滞后 / stop-go 周期 / 有效指令率
  4. T17 A/B：`ros2 param set step_base command` → 对比 T15 基线，异常即参数回滚
- 并行待办：T11 补录包复测（现场参数已置 False）、T12 判别（日志对齐 + 单步 movej
  台架）、T13 Puppetry 离线注入。
- 未验证清单：见 `docs/exton_bridge_对比UBT与优化路线.md` §六 与 WORKLOG 2026-11-06 条目。
