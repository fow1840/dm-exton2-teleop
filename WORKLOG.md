# WORKLOG.md —— 接手工作流水账

> 只追加、不改写、不删。每条必须带**证据**（命令输出 / 日志行 / 包名），
> 推测必须标注「未验证」。任务状态回到 [TASKS.md](./TASKS.md) 改，不要在这里改状态。

## 模板（复制这段用）

```
## YYYY-MM-DD HH:MM | T0x 任务名 | AI / 人工 / 双方

- 动作：具体做了什么（文件、远端路径、命令）
- 证据：关键日志行、计数、md5、包名；贴原文，不要转述
- 结论：成立/不成立；成立才写，否则写「未验证」
- 未验证：还有什么没证到
- 下一步：指向哪个任务号
- 现场变更：重启了哪些进程 / 改动了哪些文件（没有就写「无」）
```

---

## 2026-11-05 21:42 | T01 现场状态核对 | AI

- 动作：在 tc 主机（经 127.0.0.1:2222 隧道）执行 `ps`、`grep -c program_run_failed`、`tail T3_bridge.log`
- 证据：
  - 进程：MessageBridge pid 21935 / robot_state_publisher pid 48691 / exton_bridge pid 52387 / ik_node pid 71634（21:39:28 启动）
  - `grep -c "program_run_failed" T3_bridge.log` → `0`
  - T3 日志末行：`状态: connected=True cmd_rx=347799 sent=0 fb_synced=True skip=nodelta`
    以及 `IK 与真机接近，暂无有效步进（松开 Clutch 时正常）。请按住 Clutch 再慢动手臂`
- 结论：5Hz 实验**未被检验**。拒绝数 0 是因为 `sent=0`（松开离合不下发），不是修复生效。
- 未验证：5Hz 下按住离合是否仍拒绝（T02）；J7 在限位外是否有影响（T06）
- 下一步：T02，需操作员在场
- 现场变更：无（只读）

## 2026-11-05 21:45 | 项目落库 | AI

- 动作：把 tc 主机上的代码与脚本抽成独立项目 `/home/dm/下载/dm-exton2-teleop/`
- 证据：`ik_node.py` 本地与远端 md5 均为 `c1ea3c71d86910baf43e6b47cea9a17a`；
  `joint_to_pose_adapter.py` 均为 `97620a4c9cbf63b5415b38d8d1315338`
- 结论：快照与现场代码一致，可直接作为改动基线
- 未验证：无
- 下一步：T01
- 现场变更：无

## 2026-11-05 22:17 起 | T01 / T10 最新录包手静自运动诊断 | Codex

- 动作：SSH 只读核对现场，复制最新录包及现场 IK 到 `/tmp/exton_bags/`；本地执行 `python3 /tmp/exton_bags/check_drift.py`；远端 pink 环境执行 `/tmp/replay_ik_diagnosis.py`，只调用离线 FK / solve_ik，不创建 ROS 节点，不连接控制端口。
- 证据原文：
  - `CLOCK 2026-11-05T22:17:34+08:00`
  - PID：MessageBridge `75283`，robot_state_publisher `75358`，bridge `75430`，Puppetry `75465`，IK `75524`。
  - `REJECT_COUNT 19`
  - `状态: connected=True cmd_rx=116511 sent=787 fb_synced=True skip=nodelta movej_tx=520 ack=520 timeout=9`
  - 包：`press01_right_221254.npz`、`press02_right_221259.npz`、`press03_left_221328.npz`、`press04_left_221400.npz`。
  - 第4包：`HOLD 218.33130860328674 223.65092158317566`。
  - `WINDOW 221.831 222.331 /exo range_deg=0.1774 joint=6 net_deg=-0.1774; /ik range_deg=5.9351 joint=7 net_deg=-5.9351; /rob range_deg=0.5603 joint=3 net_deg=0.5548; /ee_left_tgt pos_mm=0.297 rot_deg=0.100`。
  - 全按住段：`/ik range_deg=15.9863 joint=7`，`/rob range_deg=2.0435 joint=3`。
  - `ros2 topic info /target_robot/left_ee/state --verbose`：`Publisher count: 2`，`Node name: ik_retargeting`，`Node name: fk_adapter`，订阅者 `message_bridge`。
  - `ros2 param get /ik_retargeting publish_commanded_ee_state`：`Boolean value is: True`。
  - `CONDITION 220.5 11395.409831258225 min_sv 0.00015353703061852134`（混合平移/旋转单位的雅可比数值，只用于当前模型同尺度比较）。
  - `RECORDED_HALF_SECOND FK_mm 0.3193913038805089 FK_deg 0.021318753004841363`（t=221.84–222.34，指令 FK）。
  - `STATE_NEAREST_FK samples 22 command_match_lt_0.2mm 22 feedback_match_lt_0.2mm 0 median_error_mm [2.82502532e-03 8.41720557e+00]`。
  - 固定目标、姿态 cost=0.3：`joint_delta_deg [0.037557 -0.009445 -0.107742 -0.082046 -0.105769 0.009137 0.216339] EE_mm 0.006578100893511498 EE_deg 0.00020610341910708922`；cost=0 时全零。
  - `LIMIT_DISTANCE_DEG [168.681 167.954 158.045 168.228 160.383 158.653 78.773]`。
- 结论：本次异常已出现在 IK 指令层；大幅关节变化伴随微小末端变化，构型接近奇异；state 话题混有指令 FK 已确认。当前构型不支持左 J7 硬限位驱动回退的解释。旧全零姿态回拉不能单独解释本次异常方向与幅度。
- 未验证：近奇异放大的完整目标重放；双发布对 Puppetry 的具体因果贡献；松开后实测关节继续变化是否来自控制器未完成动作。没有声称修复完成。
- 下一步：关闭指令 state 发布做单变量对照；再处理 IK 近奇异稳定性与 bridge 停止语义。
- 现场变更：无运行进程/运行代码变更，无运动指令；仅新增远端 `/tmp/replay_ik_diagnosis.py` 离线分析脚本。更新本地台账与 handoff。

## 2026-11-05 | T10 完整控制方法重放与固定真实目标严格对照 | Codex

- 动作：从现场 IK 源码 AST 提取原始滤波器、`target_cb`、`control_loop`、`_side_joint_indices`、`_incremental_desired`，构造无 ROS 发布与控制器连接的 Harness；重放 `press04_left_221400.npz`，dt=0.004s，t=218.332–223.65。NPZ 不含目标原始 header 时间，使用接收时间近似。
- 脚本：本地 `/tmp/exton_bags/replay_full.py`、`/tmp/exton_bags/replay_strict.py`；远端 `/tmp/replay_full_diagnosis.py`、`/tmp/replay_strict_diagnosis.py`。
- 执行：远端 `source /opt/ros/humble/setup.bash && /home/tc/DM-EXton2ProductPackage_tianchuang/exton_ros2_retargeting_demo/pink/bin/python /tmp/replay_strict_diagnosis.py`，exit 0。出现 SparseConversionWarning，未出现求解失败。
- 第一轮完整重放输出原文（RMSE 为左臂关节轨迹相对原录包的复现差，不是末端跟随精度）：

```text
MODE baseline anchor 218.83599999999882 J7_total_deg 16.474737144239054 J7_halfsec_deg -7.48132650764951 baseline_RMSE_deg 0.6394375103568019
MODE freeze_raw J7_total_deg 16.617005040016988 J7_halfsec_deg -7.48132650764951 baseline_RMSE_deg 0.6516053578755633
MODE no_posture J7_total_deg 17.242596161496284 J7_halfsec_deg -8.079606672136672 baseline_RMSE_deg 0.8944744164627975
MODE hold_current J7_total_deg 4.0556052285999655 J7_halfsec_deg 0.34921391042918093 baseline_RMSE_deg 2.6521801939353478
MODE damping120 J7_total_deg 1.3731300199095864 J7_halfsec_deg 0.029954753889160506 baseline_RMSE_deg 3.7002213593916844
```

- 第一轮局限：`damping120` 和 `no_posture` 从整段开始干预，异常前路径不同；`hold_current` 每拍将目标设为当前 FK，会放弃原目标，仅用于诊断。随后补充相同前史的严格对照。
- 严格实验：三个 fixed 模式在 t=221.832 前均保持基线，从该时刻复制并固定真实 FrameTask 目标；再分别保留原任务、仅移除 posture、仅 damping 12→120。误差统计至 t=222.328，单位 mm / deg。
- 严格输出原文：

```text
TASK_ERROR_AT_STILL [ 2.11794482e-04 -3.04486584e-05  1.03712965e-03  7.46091667e-04
  7.66765204e-04 -8.48880163e-05]
FIXED_TARGET_ERROR_FIRST_LAST_mm_deg
(221.83199999999184, 1.0589721104025318, 0.06149061508500177)
(222.32799999999068, 0.567869200991175, 0.05710986422961901)
MODE fixed anchor 218.83599999999882 J7_total_deg 16.617005040016988 J7_halfsec_deg -7.48132650764951 baseline_RMSE_deg 0.6516053578755633
FIXED_TARGET_ERROR_FIRST_LAST_mm_deg
(221.83199999999184, 1.0589721104025318, 0.06149061508500177)
(222.32799999999068, 0.5152288693277272, 0.05386556739285059)
MODE fixed_no_posture anchor 218.83599999999882 J7_total_deg 17.34410178350808 J7_halfsec_deg -8.08895644532759 baseline_RMSE_deg 0.8016534315533078
FIXED_TARGET_ERROR_FIRST_LAST_mm_deg
(221.83199999999184, 1.0589721104025318, 0.06149061508500177)
(222.32799999999068, 1.0556575561484391, 0.06195450366872289)
MODE fixed_damping120 anchor 218.83599999999882 J7_total_deg 4.299973174488215 J7_halfsec_deg -0.06616958821624831 baseline_RMSE_deg 2.411060901539776
```

- 结论：基线近似复现录包异常。即便任务目标完全固定，关节仍大幅运动并降低残余末端误差；移除 posture 仍运动且幅度更大。持续输入变化、滤波余尾及 posture 不是维持该段运动的必要条件。结合前次雅可比与 FK 证据，当前最支持“小幅残余末端误差持续追赶 + 近奇异构型放大”的机制；当前死区只保持目标，不冻结关节。
- 阻尼代价：局部增大阻尼将 J7 半秒运动从 -7.4813° 压到 -0.0662°，但平移误差只减少约 0.0033mm（基线减少约 0.4911mm）。不能将此诊断干预直接称为生产修复。
- 未验证：重放不是原始消息时序的逐帧等价复现；state 双发布的上游因果贡献；静止保持与近奇异稳定策略的修复有效性；松开后真机继续运动的原因。
- 下一步：验证静止保持及近奇异稳定策略，并独立验证关闭指令 state 发布后的效果。
- 现场变更：仅新增上述两个远端 `/tmp` 离线脚本；未修改运行控制代码、未重启运行进程、未下发机器人运动指令。本地更新 HANDOFF、TASKS、WORKLOG。

## 2026-11-05 23:01–23:10 | T10 手静冻结修复：部署 + 真机录包对照 | Codex + 操作员

- 修复内容（仅 ik_node.py 一处控制语义）：`target_cb` 内按滤波后目标自身速度判定手静（`still_lin_speed_mps=0.005`、`still_ang_speed_rps=0.05`，新声明参数，未动 move_speed/auto_enable/死区/姿态/阻尼）；手静时 `control_loop` 跳过该臂 solve_ik 并清零其速度滤波残留，目标一动即恢复求解。断档>0.2s 重置判定状态。
- 离线验收（远端 `/tmp/replay_hold_test.py`，TDD：先跑旧代码确认 FAIL 再改代码）：

```text
A0 修复前基线 J7_half_deg=-7.4813 max_step_deg=0.0643 still_frac=0.000
A1 修复后 J7_half_deg=-0.3286 max_step_deg=0.0438 travel_deg=41.68 still_frac=0.323
B 运动组 travel_base=297.02 travel_fix=297.02 ratio=1.000 ee_base_mm=52.49 ee_fix_mm=52.49 ee_ratio=1.000 still_frac=0.003
PASS A0 基线能复现异常(>3deg) -7.481
PASS A1 手静半秒J7<=0.5deg -0.329
PASS A1 手静段无脉冲<=0.1deg 0.0438
PASS B 关节总行程>=70% 1.000
PASS B 末端净位移>=80% 1.000
OVERALL PASS
```

- 部署：远端源文件备份 `ik_node.py.bak_1105_2301`；新文件 md5 `036320ec868e66d819016843dfb12846`（src 与 build 硬链接一致）；清 robot_ik_retargeting `__pycache__`；kill 旧 IK（75524）后按 start_teleop.sh 同参数重启，新 pid 78569；`ros2 param get` 两参数返回 0.005 / 0.05；`/target_robot/joint_commands` 250.149Hz；bridge connected=True。
- 操作员实测：按住左离合 32.82s（含手静段），录包 `press01_left_230820.npz`（探针快诊：按住中手静窗口 IK漂移峰值 16.3mrad ✅，修复前 press04 同判据为 5935mrad）。
- 对比分析（本地 `/tmp/exton_bags/compare_after_fix.py`，HOLD 320.492–353.311s）：

```text
/exo hold_range_deg max=36.692 joint=3
/ik hold_range_deg max=11.104 joint=0 net=[-11.087 -1.895 -6.57 -4.054 -6.516 -3.324 3.655]
/rob hold_range_deg max=9.992 joint=0 net=[-4.186 0.088 -3.554 -3.95 -3.433 -1.758 4.949]
w=329.992 hand=0.198deg ik_rng=0.000 ik_net_deg=[0 0 0 0 0 0 0] rob_net_deg=[0.088 -0.077 -0.082 0.011 -0.06 0.071 -0.071]
w=331.992 hand=0.145deg ik_rng=0.034 rob_rng=2.175 rob_net_deg=[-2.175 0.099 -0.198 -0.181 -0.258 -0.489 0.297]
w=332.492 hand=0.089deg ik_rng=0.206 rob_rng=2.302 rob_net_deg=[2.252 -0.104 0.253 0.225 0.253 0.472 -0.286]
w=333.492 hand=0.168deg ik_rng=0.000 rob_rng=2.164 rob_net_deg=[-2.164 0 -0.22 -0.286 -0.286 -0.747 0]
w=333.992 hand=0.099deg ik_rng=0.144 rob_rng=2.411 rob_net_deg=[2.406 0 0.308 0.346 0.33 0.802 0]
w=334.992 hand=0.100deg ik_rng=0.000 rob_rng=2.186 rob_net_deg=[-2.186 -0.181 -0.286 -0.302 -0.335 -0.846 -0.264]
w=335.492 hand=0.098deg ik_rng=0.000 rob_rng=2.593 rob_net_deg=[2.593 0.225 0.396 0.374 0.417 1.005 0.346]
w=327.992 hand=0.635deg ik_rng=0.705 tgt_rng_mm=10.243
w=326.992 hand=0.583deg ik_rng=0.741 tgt_rng_mm=8.893
```

- bridge 同期：`grep -c program_run_failed` = **59**（本轮测试前为 19，即测试期间新增约 40 次拒绝）；`sent=3274 movej_tx=2092 ack=2092 timeout=25`。
- 结论：
  1. **IK 指令层的手静自运动已修复**：多窗 `ik_net` 全零、探针峰值 16.3mrad，冻结在线上真实生效；残余末端误差追赶机制不再驱动关节。
  2. **用户仍观察到手臂动，来自两个新问题**：
     - 问题A（上游目标漂移）：手静但 Puppetry 目标自身仍滑移（最高 10.2mm/0.5s ≈ 20mm/s），高于冻结阈值 → IK 正常跟随 → 手臂移动。press01 全按住段 IK J1 净变化仍有 -11.09°。怀疑与 `/target_robot/left_ee/state` 双发布（IK 指令 FK 掺入）形成的目标-指令回路有关，**未验证**。
     - 问题B（真机自振）：IK 指令完全冻结的窗口内，真机 J1L 仍以 ~1s 周期 ±2.2~2.9° 振荡（上表 w=331.99~336.99 交替正负），且全程 J1 实测净变化 -4.19° 与指令 -11.09° 差距大；同期 bridge 拒绝从 19 涨到 59。振荡与拒绝的因果方向**未验证**。
- 未验证：问题A 的 state 双发布闭环；问题B 的控制器/bridge 侧根因；右臂未测。
- 下一步：T11（单变量关闭 `publish_commanded_ee_state` 复测问题A）、T12（真机 J1 振荡排查：先看拒绝日志时间戳与振荡窗口是否对齐）。
- 现场变更：远端 ik_node.py 已替换（备份 bak_1105_2301），IK 进程重启（75524→78569）；新增远端 `/tmp/replay_hold_test.py`、`/tmp/ik_node.py`、`/tmp/bag_motion_stats.py`；clutch_probe 以新实例运行（pid 78802，900s 限时）。bridge / MessageBridge / Puppetry / FK 未动。

## 2026-11-05 23:16–23:17 | T11 参数设置：指令 state 发布改为运行时读参 | Codex

- 动作：为实现「不重启进程、纯单变量」地开关指令 state 发布，把 `publish_commanded_ee_state` 从启动时缓存属性改为 `control_loop` 每拍 `get_parameter` 实时读取。默认值仍为 True，现场由 `ros2 param set` 置 False 做对照。
- 改动（`ros2_ws/src/robot_ik_retargeting/robot_ik_retargeting/ik_node.py`）：
  - 删除 `__init__` 中的 `self.publish_commanded_ee_state = bool(...)` 缓存赋值；
  - `control_loop` 末尾条件改为 `if bool(self.get_parameter('publish_commanded_ee_state').value) and clutch_any:`。
  - 未改其它逻辑；手静冻结参数、死区、cost、move_speed、auto_enable 均未动。
- 离线回归（`/tmp/replay_hold_test.py`，需给 Harness 补 `get_parameter` 桩）输出原文：

```text
A1 修复后 J7_half_deg=-0.3286 max_step_deg=0.0438 travel_deg=41.68 still_frac=0.323
B 运动组 travel_base=297.02 travel_fix=297.02 ratio=1.000 ee_base_mm=52.49 ee_fix_mm=52.49 ee_ratio=1.000 still_frac=0.003
PASS A0 基线能复现异常(>3deg) -7.481
PASS A1 手静半秒J7<=0.5deg -0.329
PASS A1 手静段无脉冲<=0.1deg 0.0438
PASS B 关节总行程>=70% 1.000
PASS B 末端净位移>=80% 1.000
OVERALL PASS
```

- 部署：远端备份 `ik_node.py.bak_1105_2316`；新文件 md5 `2af3069280134d02d32eb5acb10f1ee5`（src 与 build 硬链接一致）；清 `__pycache__`；kill 旧 IK（78569）确认 `IK_STOPPED` 后按 start_teleop.sh 同参数重启，新 pid 79431。
- 参数设置原文：

```text
$ ros2 param set /ik_retargeting publish_commanded_ee_state false
Set parameter successful
$ ros2 param get /ik_retargeting publish_commanded_ee_state
Boolean value is: False
$ ros2 param get /ik_retargeting still_lin_speed_mps
Double value is: 0.005
$ ros2 topic hz /target_robot/joint_commands
average rate: 250.762
```

- 录包：旧探针（900s）到期，重启为 1800s 窗口，新 pid 79635，日志 `~/桌面/dm-exo/logs/probe_1105_2317.log`。
- 结论：T11 实验条件已就绪（指令 state 发布关闭，其余与 23:08 那次实测一致）。**修复效果尚未验证**——等待同一手静场景的新录包。
- 未验证：关闭指令 state 后 Puppetry 目标滑移是否下降；问题B（真机 J1 自振）与 T11 无关，仍独立存在。
- 下一步：操作员按住左离合手静约 8s 两次 → 拉新包，用同一判据对比目标滑移与 IK J1 净变化。
- 现场变更：远端 ik_node.py 已替换（备份 bak_1105_2316），IK 进程重启（78569→79431），运行参数 `publish_commanded_ee_state` 已置 False；clutch_probe 重启（78802→79635）。bridge / MessageBridge / Puppetry / FK 未动。

## 2026-11-05 23:30 | T11 复测观察（仅口述，无录包） | 操作员 + Codex

- 动作：操作员在 T11 条件（`publish_commanded_ee_state=False`、手静冻结生效）下按住离合实测；随后离开机器人现场，**未能拉取新录包**。
- 证据（操作员口述，非录包/日志原文，按规矩标注）：
  - 「按住离合的也会移动，但是确实是移动的幅度小了些。」
- 结论：
  - 手静冻结 + 关闭指令 state 的组合**没有完全消除**松下手臂移动，但主观幅度下降。**方向与预期一致，幅度无法量化**。
  - 因缺新包，无法判断下降来自 T11（state 双发布回路被切断）还是仅来自手静冻结的固有收益，**归因未验证**。
  - 仍存在残余移动，说明至少还有一条未消除的驱动路径；排除项只能到「不是纯 IK 静默追赶」这一步（该机制已由 23:08 包证伪为已消除）。
- 未验证：残余移动幅度、周期、来源（Puppetry 目标自漂 / 真机 J1 自振 / 控制器队列）；右臂完全未测。
- 下一步（明天，需现场）：
  1. 先确认现场 IK 进程仍是 79431 且 `publish_commanded_ee_state` 仍为 False（进程若被重启会回到 True）。
  2. 拉取 `~/桌面/dm-exo/bags/` 里 23:17 之后的新包，用同一判据（手静窗目标滑移、IK J1 净变化、IK 冻结窗内真机 J1L 振幅）对比 `press01_left_230820.npz`。
  3. 若目标滑移未降 → 漂移来自 Puppetry 自身；若已降但手臂仍动 → 转 T12 排查下游控制器/bridge。
  4. 补测右臂。
- 现场变更：无（本轮未改代码、未重启进程、未下发运动）。clutch_probe（pid 79635，1800s）可能已自然到期；IK 与其余组件保持 23:17 状态。



## 2026-11-06 00:1x | 信号链路全面分析 + 本地 git 建立 | ZCode

- 动作：① `git init` 建本地仓库（.gitignore 排除 pink venv / build / install / zip / apk / 录包），
  首个提交 = 2026-11-05 21:42 快照 + T10/T11/T12 修复版代码；② 通读 ik_node.py（582 行工作副本）、
  exton_bridge.py、joint_to_pose_adapter.py、Puppetry.yaml、ik_retargeting.launch.py，
  结合 UBT-walker-s2 台账（P-007 / P-012 / P-016 / C-002 / P-014）做链路与根因分析，
  产出 `docs/信号链路与离合自运动分析.md`；③ TASKS.md 增 T13（Puppetry state 依赖离线注入）、
  T14（bridge hold+迟滞），T12 条目补机制分析。
- 证据（代码行锚点，均已在本仓库快照核对）：
  - `joint_to_pose_adapter.py:47-48` `if self.yield_when_clutch and any(self.clutch.values()): return`
    → 按住离合期间 FK adapter 停发 state，由 ik_node 接管；
  - `ik_node.py:540-541` `if bool(self.get_parameter('publish_commanded_ee_state').value) and clutch_any: self._publish_ee_states(self.q_commanded)`
    → 接管后发的是**指令 FK**（q_commanded），非真机实测；
  - `exton_bridge.py:581-593` `err = ik_val - fb[idx]; ... joints[idx] = fb[idx] + step`
    → bridge 以**真机反馈**为底闭环纠偏，`command_eps_rad=0.001rad`（第 409 行），
    IK 冻结 ≠ bridge 停发；
  - `ik_node.py:501-507` solve_ik 未传 limits= → 核对 UBT P-016 澄清记录 + 本仓
    requirements.txt `pin-pink==3.5.0`：pink≥3.0 默认即应用 Configuration/VelocityLimit，
    **no-op，不立任务**（防误判）。
- 结论：
  1. 链路里叠加三个串联闭环：IK 内环(250Hz) / bridge 反馈纠偏外环 / Puppetry state 锚定环；
  2. 机制②（T11 目标自漂移）回路结构具体化：按住期间 state=指令FK，若 Puppetry 连续参考
     则 target=state+C 永追不上，漂移速率=IK 闭环收敛速率，与实测 ~20mm/s 量级吻合（推断，待 T13/T11 量化）；
  3. 机制③（T12 自振）新提出：bridge 第二闭环在 IK 冻结后仍以 0.057° 判据持续发纠偏 movej
     （23:08 测试拒绝 19→59 为冻结窗内仍发的直接证据），每条 movej 为 ~1s P2P 程序，
     节拍与 ~1s 振荡周期吻合，疑 hunt 极限环（推断）；
  4. 23:30 口述复测（state 双发布已关仍动、幅度变小）与上述「②有贡献 + ③独立存在」一致。
- 未验证：Puppetry 对 state 的连续依赖（T13）；ack=受理还是完成、movej 到位过冲量（T12 台架）；
  现场 bridge `command_rate_hz` 实际值（5 或 50，需 `ros2 param get` 现场确认）。
- 下一步：按分析文档第四节顺序——T11 补录包复测 与 T12 日志对齐并重；T13 可并行。
- 现场变更：无（纯离线分析 + 本地 git；未连真机、未改远端文件）。git 提交两笔：
  快照基线 + 本次台账/分析（含 23:30 现场 Codex 条目一并入库）。

## 2026-11-06 00:4x | exton_bridge.py 信号流专项分析 | ZCode

- 动作：核对仓库内三份 bridge 拷贝（md5）→ 尝试建隧道核对远端真身（失败）→ 通读
  tools/exton_bridge.py（775 行）与阻塞版备份，产出 docs/exton_bridge_信号流分析.md
  （线程/锁模型、下发/反馈双向生命周期、参数表、风险清单 R1~R7、版本演进）。
- 证据原文：
  - `md5sum`：tools/exton_bridge.py = `fe58738accb7bd416f5dd59ce95307f4`（775 行）；
    tools/exton_bridge.py.bak_blocking_1105_1854 = ros2_ws/scripts/exton_bridge.py =
    `d750d22bec3b1d909cf0fb93f170bab6`（624 行，两文件逐字节相同）。
  - 隧道尝试：`sshpass -p 'DMdaNto' ssh … ubuntu@192.168.26.250` →
    `kex_exchange_identification: Connection closed by remote host`，nc 2222 不通。
  - 阻塞版关键行：`create_timer(1.0/fb_hz, self._poll_and_query)`（备份 315 行）+
    `poll_recv` 内 `sock.recv`（166/173 行）——反馈 6.6Hz 根源与 docstring 自述一致；
    `_flush_commands` 的 fb 底步进内核两版相同。
- 结论：bridge 同时是协议网关/限速整形器/第二闭环控制器；R1（纠偏 hunt）与 R2（ack
  语义未定）为 P0 待验证项；R3（反馈无新鲜度看门狗）、R4（重连可能用陈旧快照同步）
  为新发现（代码实证，现实暴露未验证）。所有「目标=Q_ik（|err|≤0.2rad 时）」推理已由
  apply_arm 代码锚定（581-592 行）。
- 未验证：远端 md5、现场 command_rate_hz/control_mode 实际值、ack=受理或完成、
  单条 movej 过冲量、反馈停更是否发生过。
- 下一步：现场先跑参数核对命令（分析文档第七节末）再动 T14；T12 台架一锤定音 R2。
- 现场变更：无（未连现场；仅本地新增 docs/exton_bridge_信号流分析.md 并入库）。

## 2026-11-06 01:1x | exton_bridge vs UBT pink_cmd_bridge 对比分析 | ZCode

- 动作：通读 UBT `walker_s2_fk_node/src/walker_s2_pink_cmd_bridge.cpp`（530 行）与
  `pink_bridge_safety.hpp`（230 行），与本桥逐维度对比，产出
  docs/exton_bridge_对比UBT与优化路线.md（10 维对比表 + 延迟预算 + R8~R11 新问题 + A/B/C 优化路线）；TASKS 增 T15（跟手性基线测量）、T17（下发语义对齐），T14 补 UBT 冻结保持教训。
- 证据原文（代码锚点）：
  - UBT 步进底=指令非反馈：`command_[i] += std::clamp(target_[i]-command_[i], -max_step, max_step)`（safety.hpp:172-174），max_step=0.8rad/s×clamp(dt,≤0.02)（169-170）→ 每拍 ≤0.016rad 匀速斜坡；
  - UBT 500Hz 流式保活：`publish_rate` 默认 500（cpp:63），Step 拒发时 `HoldCommand(now)` 顶上（cpp:308-314）；
  - UBT 冻结保持教训（safety.hpp:106-115 注释原文）：「保持命令必须是冻结值，严禁每周期取实时 measured_：若目标=当前实测位，运控零位置误差→零保持力矩→重力下拉→命令追着下垂走→手臂缓慢下垂（2026-10-13 实测回归）」——与本项目 T12 hunt 同构的实测案例；
  - UBT 偏差锁存：`MaxAbsDifference(command_, measured_) > 0.6 → fault_latched_`（safety.hpp:151-155），离合循环才解除；四路输入超时各 0.2s（135-139）；
  - EX 桥步进底=反馈：`err = ik_val - fb[idx]; joints[idx] = fb[idx] + step`（exton_bridge.py:581-592）。
- 结论：
  1. 架构根差一处：EX=反馈闭环比例纠偏（程序串行），UBT=开环匀速斜坡（500Hz 流式）。EX 的 hunt（R1）、走停步进（R8 新）、无保活（R9 新）、无输入超时（R10 新）、无偏差锁存（R11 新）均由此派生；
  2. 跟手性影响（推断，待 T15 量化）：有效指令率被 lock-step×movej 程序时长钳制约 1–3Hz、每条之间完全停止、端到端延迟约 250–500ms 量级；UBT 流式低一个量级；
  3. 优化路线定序：C1 测量基线 → B（T14 迟滞 + 反馈看门狗）→ A（T17 开环斜坡 / T09 canfd 流式 / 保活评估）。
- 未验证：现场 command_rate_hz、ack 语义、单条 movej 过冲量、空闲会话行为、有效指令率实测值、快速运动最大滞后。
- 下一步：T15（需操作员）与 C2 参数核对并行；T14 在 T12 判别后实施。
- 现场变更：无（未连现场；本地新增对比文档并入库）。

## 2026-11-06 01:5x | T14/T16/T17 离线实现 + 离线回归框架 | ZCode

- 动作：① `tools/exton_bridge.py` 重构——下发计划抽成纯逻辑类 `StepPlanner`（无 ROS/socket），
  `_flush_commands` 改门禁+副作用薄壳；实现 T14 迟滞（`hold_enter_rad`/`hold_rearm_rad`/
  `ik_move_eps_rad`）、T17 `step_base=command` 开环斜坡 + `tracking_leash_rad` 护栏 +
  拒绝 `invalidate()` 重试、R3 反馈看门狗（`feedback_timeout_s`）、R4 重连清旧快照、
  R10 指令流活性超时（`command_timeout_s`，rx 计数判据）；语义参数每帧实时读（T11 教训）。
  ② 新建 `tools/tests/`：`fake_controller.py`（V0.3 协议假控制器，ack=受理/完成两模式、
  过冲/下垂模型）+ `test_bridge_offline.py`（L1 纯逻辑单测 / L2 闭环仿真 / L3 真 socket）。
- 证据原文：`Ran 18 tests in 3.338s / OK`（18/18），关键项：
  - `test_feedback_mode_matches_old_apply_arm`：feedback 模式与旧 apply_arm 逐位一致（回归基线）
  - `test_a0_old_behaviour_hunts_with_overshoot`：2° 过冲 + 旧行为 → 50 拍 ≥45 发（hunt 复现）
  - `test_a1_hysteresis_insufficient_for_big_overshoot`：过冲≥enter 时迟滞不足（≥45 发，边界文档化）
  - `test_a3_command_mode_immune_to_big_overshoot`：command 底 ≤2 发（hunt 免疫）
  - `test_b_sag_model_feedback_hunts_command_stops`：下垂模型 feedback 底 ≥45 发「追着下垂走」、
    command 底 ≤2 发（复现 UBT 2026-10-13 实测回归）
  - `test_c_following_not_blocked_by_hold`：目标连续变化 20/20 发（迟滞不卡跟随）
  - `test_lockstep_complete_mode_paced_and_rejects_overlap`：执行完才回 + 0.5s 执行 →
    300ms 超时触发、程序重叠拒绝发生（R2 两语义行为刻画，供 T12 现场对照）
  - `test_reconnect_drops_stale_snapshot`：重连后旧快照为 None（R4）✅
  - 过程中修掉两个真缺陷：`invalidate()` 未清 hold 锁（拒绝后重试发不出）；`ik_moved`
    当帧被 enter 条件重新锁住（慢速跟随会被卡）
- 结论：T14/T16/T17 的代码与离线回归全部就绪，默认参数=现行为（step_base=feedback、
  迟滞可用 hold_enter_rad=0.001 退化关闭），上真机只做「备份→推送→参数 A/B→录包对比」，
  不需要再改代码。**真机一切未验证。**
- 未验证：真机 hunt 幅度/过冲量（决定迟滞是否足够，T12 判别）；ack 语义；command 模式
  跟手性收益（T15 基线对比）。
- 下一步：现场按 T16 部署步骤推 bridge → T14 步骤 1/2 参数 A/B → T15 基线录制 →
  T17 command 模式 A/B。
- 现场变更：无（未连现场；本地 tools/exton_bridge.py 已领先远端，**推送前必须按 T16 备份**）。

## 2026-11-06 02:2x | 左右臂独立计算溯源分析 + T19 per-arm 修复 | ZCode

- 动作：回放 UBT 决策链（C-001/P-003/P-007/P-016 M2~M5），对照本项目代码逐项核实，
  产出 docs/左右臂独立计算分析.md；实现两项 per-arm 修复并入离线回归；TASKS 立
  T18（双臂测量）/T19（per-arm 隔离包）。
- 证据原文：
  - UBT 三次「单臂正常、双臂暴露」：C-001 双臂使能发布线程饿出 20~130ms 间隙→剧烈抖动
    （LEDGER-ARCHIVE C-001）；P-007 四组真机对照定案「双臂轮转挤占单求解线程→周期超限+
    限速器自激极限环」；P-016 M4.5 基准四档：单臂 3.1ms / 顺序双臂 5.67ms（理论上限
    176Hz）/ 两线程被 GIL 反拖 101~104Hz / **两进程 253.7Hz**（docs/diagnostics/
    2026-10-13-pink-m4-5-realtime-fix.md）。
  - 本项目同构证据：`ik_node.py:386-393`（改前）任一臂离合上升沿
    `self.q_commanded = self.q_measured.copy(); self.last_velocity.fill(0.0)` 重置两臂
    全量——与 UBT M2 审查发现同型且更重（UBT 只发现 last_velocity 一半）；
    ik 目标 250Hz（预算 4ms）< 顺序双臂 5.67ms（同栈基准）→ 双臂同动必掉频（🔍 待 T18 实测）。
  - 协议连坐：本机 17 维一条 movej = 两臂同程序，整帧拒绝（T12 拒绝 19→59 期间右臂
    是否同窗停发待回查）。
  - 右臂零数据：WORKLOG 23:01 明记「右臂未测」，press01~04 与 20 个基线包全为单臂。
- 改动（本地，未推远端）：
  - `ik_node.py`：离合上升沿改只重置本臂 `_side_joint_indices(side)` 的
    q_commanded/last_velocity；posture 锚只锚本臂（另一臂锚保持其按下时的值）；
  - `exton_bridge.py` StepPlanner：leash 改 per-arm（`_leash_hold` 按臂，超限臂 7 轴
    冻结在计划值，另一臂照常；全臂超限才整帧冻结）；invalidate 注释明确协议级连坐语义。
- 回归：test_bridge_offline.py **19/19 OK**（新增 test_leash_per_arm_isolation；修复
  test_leash_hold_and_release 断言——释放帧 held=6 是 Joint2~7 收敛进 hold 的正确行为）。
- 结论：UBT 的四类原因（算力/状态耦合/故障隔离/消息语义）在 DM-EXton2 都有对应物；
  差异是本机协议 17 维一条把隔离上限压低；最大风险=双臂场景零测试。A（进程拆分）
  等 T18 数据裁定。
- 未验证：双臂同动实际频率、交叠按下右臂跳变幅度、臂轴 -100 降级行为、修复在真机
  双臂交叠下的效果。
- 下一步：T18 四场景测量（右臂首测优先）；T19 随 T16 一起部署（注意重启后
  publish_commanded_ee_state 回 True，T11 对照需重设）。
- 现场变更：无（未连现场；本地 ik_node.py 现已领先远端，推前必须备份）。

## 2026-11-06 02:4x | UBT URDF 对账方法论梳理 + T20 立项 | ZCode

- 动作：回放 UBT「URDF vs 真机不匹配」全过程（P-008 真值源 → M0 四证据链判决 →
  同源重建 → M5 姿态盲区 → X_map 帧标定 → 数据纪律），产出
  docs/UBT_URDF对账方法论.md（七条可迁移规则）；TASKS 立 T20。
- 证据原文（UBT 侧原始记录）：
  - HANDOVER §九 48 行：「执行器/robot_state 双肘弯曲为负（R 肘实限位 [-2.62, 0.01]）；
    Puppetry URDF 的右肘符号是错的」；53 行：「URDF 零位=垂臂，SDK 零位=双臂前伸，
    符号+偏移拟合不达标，『右肘符号错』只是症状。正解=按 SDK 头文件 D-H 参数直写
    （sdk_fk.py，θ=q+init 标准链，20 组真值 0.0008mm 一致）」。
  - M0 诊断文档四证据链：限位表两臂逐项相同（权威）；两库存 URDF md5 逐字节相同且
    均镜像建模；s∈{±1}^7 枚举 × LM 拟合 max 321~388mm（门限 20mm）→ 结构性不可救；
    合成 URDF 同 20 组验证腕点 0.001mm/肘点 0.000mm。
  - M5 ②：合成 URDF 末端漏 Rz(π/2)，位置验证通过姿态漏检 → 真机「手腕持续旋转到限位」，
    pinocchio worst_rot 90.000000°→0.000000°。
  - M5 ①：X_map 缺失 ~1m 偏差被软启动压缩 = 离合瞬间猛动；fk_probe 零位单点+21 组复核
    残差 0.0000mm，烘成常量+启动门禁。
  - 数据纪律坑：P-008 数据第 7/8 组配对抄串制造「176mm」假阳性；教训=异常残差先查配对。
- 本项目对照结论：URDF 对账从未做过（0.0mm 验证是同 URDF 自洽检查）；三个未解释疑点
  （J7 限位差、T10 近奇异、mirror_xyz 声明）可能同源 → T20 立项。
- 未验证：本机控制器是否暴露自有末端位姿；真机实达限位范围；外骨骼帧-机器人帧是否
  固定变换。
- 下一步：T20 步骤 1（限位三方对比，可与 T06 合并现场执行）。
- 现场变更：无（纯离线分析）。

## 2026-11-06 03:0x | T20 明早作业包：协议/档案离线侦察 + 采数对账工具 | ZCode

- 动作：① 提取协议 V0.3 PDF 与档案 docx 全文检索；② 解析 URDF 限位并与档案软限位
  三方对比（报告 /tmp/limit_report.txt 原文已录下条）；③ 写采数器与对账器
  （tools/dm-exo/urdf_fk_calib/）；④ TASKS T20 重写为明早执行清单。
- 证据原文：
  - 协议 PDF（pdftotext）：`get_tcp_pose` 应答 `pose:[x,y,z,qw,qx,qy,qz, …]×2`
    「left TCP + right TCP」（**qw-first**；「单位：弧度」疑笔误）；`get_config` 应答
    `upperLimits/lowerLimits`；`get_device_state` 错误码含「0x02010001 第一个电机超软限位」；
    movej/movej_canfd「-100 表示该关节不移动」；**movej 应答 status=`program_run_finish`**
    （R2 倾向「执行完才回」）。
  - 档案 docx：右臂软限位表（2026-07-06）j1 ±179.9 / j2 -148.97~154.70 / j3 -39.53~129.49 /
    j4 -148.97~154.70 / j5 ±89.95 / j6 ±154.70 / j7 ±154.70；**末端工具长度 0.145 米**；
    「关节 14（右臂腕部翻滚）反复堵转」故障史；手眼外参占位未标定（与本项目无关）。
  - URDF 解析：Joint1..7 与 Joint1L..7L **全部 ±170° 一刀切** → 与档案表全轴结构性差异
    （J3 Δ130.5°、J5 Δ80.0°）；且 bridge 左臂 limits=URDF(±177°)、右臂才用档案表 →
    左臂可被指令到真机不可达区（候选 bug 来源，待 get_config 实证左臂真机限位）。
  - URDF 源 CSV：左右臂关节 origin rpy 非简单镜像（Joint1 rpy=(0,0,-3.1416) vs
    Joint1L=(0,0,0)；Joint2L 多 -π）——UBT 镜像建模隐患形态。
- 工具：`collect_samples.py`（--probe/--config/--collect，只读不发运动，交互采
  q17+tcp14 存 jsonl）+ `compare_fk.py`（pinocchio FK vs TCP：位置+姿态双维度、
  工具 0.145m 两假设、s∈{±1}^7 零偏移枚举可救性判决，UBT 规则 3/4/5 落地）。
- 自检：两脚本 py_compile OK；qw-first 四元数数学三例自检 OK（恒等/绕z90°/绕x180°）；
  pinocchio 路径本地无依赖无法测，标注首跑即修。
- 结论：T20 对账方式从「可能需要物理测量」升级为「协议自带 TCP 位姿直接采数」；
  明早序列 1h 内可完成采数+判决。T06 的限位问题大半可由 get_config 直接回答。
- 未验证：get_tcp_pose/get_config 在真机固件上的真实可用性与应答格式（--probe 先验）；
  TCP 位姿参考系（base? waist?）——若残差整体平移恒定即坐标系问题，再帧标定（UBT 规则 6）；
  左臂真机软限位。
- 下一步：明早按 TASKS T20 执行序列跑采数与判决。
- 现场变更：无（未连现场；新工具在本地，届时 scp 到 tc 主机）。

## 2026-11-06 03:2x | 信号链路模块定位图成文 | ZCode

- 动作：重新抓取 ik_node.py / exton_bridge.py 当日快照的函数级行号锚点（两文件今日
  均有改动，防旧行号误导），产出 docs/信号链路模块定位图.md（S1~S7 环节 ×
  锚点表 × 话题字典 × 日志判读 × 按症状反查表 × 定位工具箱）。
- 证据原文（锚点核对命令与输出节选）：
  - ik_node.py：target_cb=320、control_loop=378（按臂重置 392-397 / await_anchor 424-462 /
    死区 476-488 / 手静冻结 494-499 / solve_ik 501-509 / 积分 524-534 / 发布 540 /
    指令FK条件发 540-545）、_publish_ee_states=547；
  - exton_bridge.py：StepPlanner.plan=190 / invalidate=182 / connect=316 / _tx_loop=425 /
    _feed=516 / _flush_commands=753 / _build_motion_payload=880 / _publish_joint_states=896 /
    _status_watchdog=931 / main=984；
  - ROS 接线：ik_node 274-285（订 (D)(C)(B)，发 (F)(E)+TF+250Hz timer）；
    fk_adapter 26-34（yield_when_clutch 20-22 行）；bridge 660-675（四 timer）。
- 结论：每个环节（含 vendor 组件的配置文件/二进制路径）都有了「文件:行号:职责」
  锚点与首查命令；行号以函数名为准（远端部署前行号会漂）。
- 未验证：无（纯文档化，全部锚点经当日 grep 核对）。
- 现场变更：无（本地新增 docs/信号链路模块定位图.md 并入库）。

## 2026-11-06 03:3x | 日志观测性评估 + A/B 项实现 | ZCode

- 动作：逐节点盘点日志现状（grep 计数），产出 docs/日志观测性评估.md；实现 A（ik_node
  运行期状态行+四类事件）与 B（bridge 状态行窗口化）；TASKS 立 T21（chain_monitor）。
- 证据原文：
  - 盘点：`grep -c get_logger ik_node.py` = **3（全部在启动期，运行期日志为零）**；
    bridge = 20（较全但计数器为自启动累计值）。
  - 实现：ik_node 新增 2s `IK_STAT clutch=… still=… frozen=L/R await=L/R solve=N/ticks
    win=…` 状态行（与 T3 状态行同节拍可对齐）+ 事件日志 `clutch ON/OFF` /
    `anchor settled after X.XXs（超时兜底标注）` / `freeze ENGAGE/RELEASE`（边沿触发不刷屏）；
    bridge 状态行追加 `| +2s: sent=N rx=N reject=N fb_age=X.XXs max_err=X.XXX held=N
    jump=N ik_moved=N`（保留原累计字段，历史证据链兼容）。
  - 回归：py_compile 双文件 OK；test_bridge_offline.py **19/19 OK**（纯日志添加，
    控制语义未动）。
- 结论：评估成立——① T5 运行期零日志是最大观测黑洞（T12「冻结了还在发吗」当时靠
  录包数小时，今后两条状态行对照即知）；② vendor 黑盒用只读外挂监视器补（T21）；
  ③ 日志回答「有没有/多频繁」，录包回答「逐帧因果」，分层不互替。
- 未验证：新日志在真机上的实际输出格式与刷屏程度（250Hz 循环只累计计数器，
  2s 一条，理论无压力）；chain_monitor 未写（T21）。
- 下一步：随 T16/T19 部署窗口上真机看状态行实貌；T21 择机实现。
- 现场变更：无（本地 ik_node.py/exton_bridge.py 领先远端，推前必须备份）。

## 2026-11-06 03:5x | 频率链分析 + 台账缺口补全（T09 升级 / T22 立项） | ZCode

- 动作：① 全链路频率盘点（配置值 vs 有效值）；② 确认用户「反馈链与 movej 是否已
  安排」的对账——movej 侧成立、反馈侧有缺口；③ T09 升级并具体化台架清单，新立 T22。
- 证据原文（频率清单，全部为代码/文档/日志可查事实）：
  - 下行有效下发率：lock-step（_tx_loop:425 在途锁）+ movej P2P 程序语义（v=15，
    ~150-300ms/条，条间完全停）→ **~1-3Hz**，对照 UBT 500Hz 连续流；协议原生
    movej_canfd(freq=50) 流式通道未启用（现场跑 movej，桥文件头自注「遥操作透传用
    movej_canfd」）。
  - command_rate_hz 现场实际值未核实（默认 50，21:19 曾手动 5Hz）。
  - 反馈：feedback_rate_hz=30（旧阻塞架构遗留，当时实测 6.6Hz）；publish 50Hz；
    IK 250Hz 输出 96-98% 被 latest-wins 丢弃（设计行为）。
- 结论：
  1. movej 侧：治 hunt（T14/T17）已实现待上机 ✅；治走停根治（T09 canfd）任务已立、
     零开发量、纯待台架验证（-100 行为 + 有效频率 + 回滚）✅；
  2. 反馈侧：R3 看门狗治可信度、async2 治架构阻塞，**「30→60-100Hz 提频」此前
     无任务无代码** → T22 立项补缺（容量验证优先于改参数）；
  3. T09 从 P3「暂不实施」升级 P1「正式验证项」并写明与 T20/T15 同窗口顺带执行。
- 未验证：控制器查询容量上限；canfd 在当前固件的 -100 行为与有效频率；现场
  command_rate_hz 实际值。
- 现场变更：无（仅 TASKS 台账更新）。

## 2026-11-06 04:0x | T09 裁定为优先验证任务 | 用户裁定 + ZCode 落账

- 动作：依据「movej 走停四道闸」分析（程序重叠/lock-step/flush 错位为语义性约束，
  100Hz 在 movej 语义下物理不可达，唯一未知数=固件 canfd 接受率），用户裁定 T09
  为优先验证任务；TASKS T09 升级 P0 并排入明早序列（步骤 1.5，与 T20 共用停 bridge 窗口）。
- 结论：canfd 验证通过后 50–100Hz 流式为纯参数切换（control_mode:=movej_canfd），
  无需新代码；失败则结论写 HANDOFF 固化。
- 未验证：canfd 三步台架全部待真机（-100 行为 / 50→100Hz 有效频率与平滑度 / 回滚）。
- 现场变更：无（台账更新）。

## 2026-11-06 09:33–10:0x | T16/T19 部署 + T20.1/T09 真机执行 + 左臂肘堵转事故 | ZCode + 操作员

- 动作与证据（时间序）：
  1. **T01 核对**：进程与昨晚一致（IK 79431 未重启，`publish_commanded_ee_state=False` 保持 ✅）；
     bridge md5 fe58738a（旧版）；**command_rate_hz=50 实锤**（非 5）；control_mode=movej；
     拒绝数隔夜 59→105（部分可由 23:30 复测解释，未逐条归因）。
  2. **部署**：远端备份 `exton_bridge.py.bak_1106_0936` / `ik_node.py.bak_1106_0936`；推送新 bridge
     （md5 01597472…）+ 新 ik_node（md5 b326d305…，build 硬链接同 md5 确认）+ urdf_fk_calib 工具；
     清 __pycache__。**事故**：远程脚本里 `pkill -f "robot_ik_retargetin[g]"` 与脚本自身的字面
     `robot_ik_retargeting`（find/launch 行）自匹配，杀掉远端脚本 shell（AGENTS 禁忌#1 变体：
     括号技巧只防模式串本身，不防脚本其他行的字面串）；桥已停、旧 IK 已停、新 IK 未起。
  3. **IK 重启修正**：首起重启漏 `export ROS_DOMAIN_ID=42`（domain 0，param 找不到节点），
     杀掉重起补上；新 IK pid 82430，`Reference Frame 'base_link' active`，
     `publish_commanded_ee_state=False` 已恢复；IK_STAT 状态行未出现为预期（bridge 停→无
     joint_states→is_initialized=False）。bridge 保持停止（让出 30485）。
  4. **T20.1 协议探测 ✅**：get_joint_position / get_tcp_pose（含额外 poseRPY 字段，qw-first 确认，
     四元数换算与 poseRPY 互验一致）/ get_config 三命令全部可用。
  5. **get_config 真机软限位**：左右臂完全相同（UBT「执行器双臂统一」再证）：
     J1/J3/J5/J7=±2.79rad，J2=[-2.356,+0.26]，J4=[-0.69,+2.26]，J6=±1.57——与档案表、
     URDF ±170° 均不同；**左臂 bridge 用 URDF ±177° 裁剪 = 可指令到真机不可达区，坐实**。
  6. **T09 canfd 台架**：①零增量帧 accepted（program_run_finish）；②50Hz×10 帧零拒绝、
     零运动；③**-100 占位轴（腰/头）被正确解析**（accepted+零运动）——HANDOFF「部分固件
     拒绝 -100」的顾虑对本机不成立，canfd 可按现槽位策略启用（用户现场确认安全后执行）。
  7. **左臂肘堵转事故（T20.2 采数驱动中）**：canfd 程序化驱臂扫位姿，L01 目标（肩 J2=-2.1 内收
     + 肘 J4=+1.2）触发**电机 4（左肘）堵转**（hardware_error_code 17039362，detail=8 堵转，
     state=fault），实际 J4 被偏转到 -1.428，随后全部目标超时。促成因素：a) 驱动循环本地
     位置模型未与真机反馈同步，帧可能被拒后继续盲发（已定位为脚本缺陷）；b) 收臂类位姿
     是肘部高危区（档案「关节反复堵转」前科 + UBT C-003「肘保持力矩不足」+ UBT P-008
     「收臂段撞机身」同型）；c) L01 是否存在机身接触未目检确认（操作员目检后选「清故障+慢速回安全位」）。
  8. **故障恢复受阻**：clear_fault → ready，但 canfd/movej 全部 program_run_failed（拒发一切运动）；
     enable_robot → **堵转立即复现**（state=fault，同错误码）——与档案「电机 14 反复堵转」
     处置记录完全同型：「临时恢复=整机断电重上电（断电前须托稳臂，抱闸会松）；仅重启控制器
     不一定清除；根治需巨蟹智能单机检修」。等待操作员断电重启。
  9. **T20 首批数据 FK 对账（2 样本预览）**：残差位置 1204/1757mm、姿态 110/155°——符号枚举
     128 组合救不回；Δpos 非恒定 → **结构性轴系差异实锤**（URDF J2 正方向与真机相反：
     真机 J2 负=臂下收，URDF 同值算出臂朝天，L01 URDF z=1.56 超过臂展上限）。数字级诊断：
     ctrl L00 TCP [0.426,-0.527,0.136] vs URDF Link7L [-0.148,-0.271,1.164]。
     **档案「URDF FK 与控制器反馈高度一致」指其手眼标定平台自带 URDF（厂商系）**——
     厂商存在与控制器一致的 URDF/D-H，同源重建路线有源可索。另档案载明：机械臂绝对定位
     精度 ~20mm（标定残差下限），T20 验收门限应设 ≥20mm。
- 结论：
  1. T09 canfd：三步台架中①②③协议层全通（-100 正确解析）；**运动平滑度/频率上限未测**
     （被事故打断）；启用 canfd 的代码条件已具备；
  2. T20：对账判决已可下——**本仓库 hr_up_7dof_fixed URDF 与控制器运动学结构性不一致，
     不可用符号/偏移修补**，路线=向厂商索控制器原生 URDF/D-H（档案平台已有「高度一致」
     先例）或从足量 (q,TCP) 数据数值重建；验收门限 20mm（机构绝对定位精度）；
  3. 观测性：IK_STAT/状态行窗口化部署完成（IK_STAT 待 bridge 恢复后可见）；
  4. 新风险入册：**肘关节（电机 4 左/电机 14 右）为堵转薄弱环节，收臂位姿列禁区**；
     canfd 驱动循环必须逐帧同步真机反馈+禁用本地模型外推。
- 未验证：canfd 运动平滑度与频率上限；断电重启后电机 4 是否恢复；T20 完整 20 组采数
  （已采 2 组，位姿集需避开肘负载收臂组合重设）；厂商 URDF/D-H 索取渠道。
- 下一步：①操作员断电重启（托稳臂）；②重启后清故障+使能验证+慢速回安全位；
  ③改写安全采数集（J2∈[-1.6,+0.11]、肘避开 +1.2 以上弯折组合、每帧同步反馈、
  位姿前查 device_state）续采；④厂商 URDF/D-H 索取（用户渠道）。
- 现场变更：bridge/ik_node 新版已部署（bridge 进程停止中）；IK pid 82430 domain42
  param=False；控制器经历 fault/clear/enable 循环，**待断电重启**；左臂停在故障收拢位。

## 2026-11-06 10:0x–10:5x | 断电重启恢复 + 安全采数 + T20 终审 + 档案限位 bug 当场抓获 | ZCode + 操作员

- 动作与证据：
  1. **断电重启后恢复验证**：state=idle、错误清零、canfd 零增量 accepted——电机 4 堵转经
     断电重启清除（与档案剧本一致）；臂由操作员手动回到站立位（TCP z=0.881m）。
  2. **采数驱动 v2**（sweep_v2.py，事故教训全量落实）：逐帧真机反馈同步（每 4 帧批同步，
     漂移上界 0.02rad）、步进 0.005、位姿集避开肘负载收臂组合（J2≥-1.5 且 J4≤+1.0）、
     每位姿前查 device_state。迭代修复三处：①单位姿 25s 超时不足（查询节奏 ~350ms/帧，
     有效 0.04rad/s）→ 批量帧+60s；②长行程末段掉速（J7 1.8rad 走不完）→ >0.9rad 自动
     拆半程路点；③**控制器队列灌满**（连续拒绝、臂停跟）→ 连续拒绝≥3 即 stop_motion
     清队自愈（复现了此前 L02 卡死机理）。
  3. **左肘第二次堵转（L204）**：大幅多轴换向中电机 4 再堵（17039362/8），J4 被顶到
     -0.699（软限位 -0.69 边界外）。两次独立实锤=硬件薄弱，用户裁定**停止真机运动**。
  4. **档案限位 bug 当场抓获**：bridge 重启后持续追 0.414rad「误差」并无限被拒——IK 指令
     与反馈逐位一致，差异来自 bridge 硬编码档案限位 j5=±1.57rad 把合法位姿（用户手摆
     J5R=-1.949，**真机 get_config 限位 ±2.79**）裁剪成 -1.57 → 永久纠偏循环。
     **bridge 会把臂从合法位姿往过时档案限位里拉**——历史「手臂自己动」的又一贡献者
     （未定量归因）。止血：`use_archive_right_limits:=false` 重启（URDF ±177° 不裁 J5），
     重启后 max_err=0.000/held=14/reject=0 静默。治本=T23（get_config 动态限位）。
     过程再踩 pkill 自匹配变体（启动行含字串），教训同晨间条目。
  5. **T20 终审（6 样本）**：位置残差 292~1895mm（median 764mm raw / 724mm tool），
     姿态 99.6~167°（median 130°）；s∈{±1}^7 枚举最优 median=742mm → **结构性差异终审：
     不可修补**。工具 0.145m 补偿不改变结论。样本存档 tools/dm-exo/urdf_fk_calib/data/。
  6. **系统现状**：bridge 新版运行（use_archive_right_limits=false，movej/v15/auto_enable=false），
     IK pid 82430（domain42，publish_commanded_ee_state=False），IK_STAT/窗口化状态行
     上线（250Hz 确认）；**左臂保持 fault（抱闸持住）**，待操作员空闲时断电重启恢复；
     遥操未测（肘未检修前不建议）。
- 结论：
  1. T20 判决完成：URDF 不可用 → 路线=厂商原生 URDF/D-H（档案手眼平台有先例）或
     数值重建（样本仅 6 组且肘受限，不足；如需另立项）；
  2. T09 协议层三步全通 + 扫位姿全程 canfd 流式驱动成功（平滑性软验证 ✅；freq 上限未测）；
  3. 新增 P1：T23 bridge/IK 换 get_config 动态限位（今日现场证据：档案表 j5 与真机差
     1.22rad 且已实际触发纠偏循环）；T24 肘电机厂家检修（用户渠道）。
- 未验证：canfd freq 上限；遥操（肘未检修）；数值重建所需样本量；左臂肘检修后状态。
- 下一步：操作员断电重启恢复臂位（安全位摆放避开档案限位禁区，或直接上 T23 后再摆）；
  向厂商索取原生 URDF/D-H + 肘电机检修（用户渠道）；T23 实现（可离线备好）。
- 现场变更：bridge 第二次重启（use_archive_right_limits=false）；左臂 fault 待重启；
  其余组件未动。

## 2026-11-06 11:0x | 断电重启恢复确认 + T23 实现落账（测试挂起未解） | ZCode

- 动作与证据：
  1. **断电重启恢复确认**：state=idle、错误清零、canfd 零增量 accepted；操作员手动
     摆新站立位（左 J3L=+0.354 等，右臂保持 J5R=-1.949）。
  2. **T23 实现**：本地 bridge 新增 get_config 动态限位（与另一并行会话协作：其提供
     `parse_live_limits()` 解析函数——14 维→双臂 7 轴、2° 内缩安全带、失败返回 None 回退；
     本会话接线：rx `_feed` 解析缓存 `_live_limits`、`get_live_limits()` 访问器、
     `use_live_limits` 参数（默认 True）、看门狗每 2s 发 get_config 并在取得后替换
     planner 双臂限位+打日志）。本地文件另有并行会话未提交改动已一并保留。
  3. **离线测试**：FakeController 加 get_config 应答（真机实测限位值）；新增 2 测试
     （动态限位获取+解析、J5R=-1.949 不裁不追验收核心）。**当前 21 测试中 2 新测试
     挂起超时未解**（get_config 请求-应答路径疑似阻塞，未定位），其余 19 全绿。
     **未部署**——真机 bridge 仍跑早晨部署版（01597472，use_archive_right_limits=false）。
- 结论：T23 代码基本就绪但测试未过、未部署；挂起定位与部署推迟到遥操作观察之后。
- 未验证：T23 真机验收（J5R=-1.949 位姿静默）；2 个挂起测试的根因。
- 下一步：遥操作效果观察（用户指令）→ 回头解 T23 测试挂起 → 部署验收。
- 现场变更：无（本地代码未推；真机 bridge/IK 维持上午状态）。

## 2026-11-06 11:3x–11:5x | canfd 遥操验证成功（里程碑）+ 新问题：肩膀方向相反 | ZCode + 操作员

- 动作与证据：
  1. **第三次故障恢复**：clear_fault → ready，但 canfd 仍拒（同前剧本）；enable_robot 成功
     （本次故障类型=「未知错误」非堵转，enable 可过）。恢复后 state=idle。
  2. **第三次故障签名（新增）**：电机 3/5/7（左臂 J3/J5/J7）同报 0x0200「未知错误」
     （码 512），非电机 4 堵转（码 8）。机理：movej 模式下 IK 指令跑飞（max_err 1.858rad）
     → 触发控制器保护。锚定窗口 IK_STAT：3130-3134 solve=328→448→470/拍（全速解算），
     臂只跟了 J1L+0.19/J7L+0.12 就死。
  3. **切 canfd 模式**：bridge 以 `control_mode:=movej_canfd` 重启（PID 杀法避自匹配）。
  4. **canfd 遥操验证（里程碑，全部量化）**：
     - 拒绝：2340（movej）→ **0**
     - 跟手误差：跑飞 1.858rad → **峰值 0.043rad 且收敛**（0.043→0.029→0.011）
     - 有效指令率：1-3Hz → **40-50Hz 连续流**（+2s sent=78~100）
     - 手静：锤拒不停 → **sent=0、frozen=500**（T14 实战生效，即停零漂移）
     - 锚定：1.5s 超时 → 0.50s 正常安定
     - 操作员体感原话：「体感十分好，这个确实是需要canfd的模式进行遥操」
  5. **新问题：肩膀移动方向与遥操方向相反**（操作员报告）——与 T20 终审的 URDF J2 轴向
     反向实测互为印证：URDF 与真机的关节轴方向系统性不一致，体现在 IK 解的雅可比列
     符号上；T20 位置残差（300~1800mm）是零位/几何层问题，方向反转是符号层问题——
     遥操增量模式下符号层直接可见，几何层大部分被增量语义抵消。
- 结论：
  1. **T09 canfd 遥操场景验证通过**，movej 模式判定不可用于遥操（跑飞+保护性故障），
     canfd+现有 T14 迟滞=可用基线；
  2. 方向反转=T20 结构性差异的符号层表现，修复路径=实测每关节雅可比列方向
     （微扰实测）→ 翻转 URDF 对应关节轴 → 三消费者（IK/rsp/fk）共享同文件一致受益。
- 未验证：肩膀反转的具体关节集合（待微扰实测表）；URDF 轴翻转后 T20 残差改善幅度；
  右臂/其余关节是否也有反转。
- 下一步：落账→微扰实测建方向翻转表→改 URDF→部署（T2+T5 重启，TF 锁存教训）→
  遥操复测方向+T15 基线。
- 现场变更：bridge 重启为 canfd 模式（use_archive_right_limits=false）；
  控制器经 clear+enable 恢复（第三次故障）。

## 2026-11-06 11:2x–11:3x | 一键脚本固化今日优化 + 全栈重启验证 | ZCode + 操作员

- 动作与证据：
  1. **微扰实测受阻**：urdf_axis_probe.py 两次 ConnectionRefused——第三次臂断电重启后
     JuxieLinker 服务（5566/30485）未随上电自动启动（.60 主机 ping 通、22 开，
     5566/30485 关）。等操作员手动启动 JuxieLinker。
  2. **一键脚本固化今日优化**（tools/dm-exo/start_teleop.sh + start_bridge_only.sh）：
     T3 参数改为 `control_mode:=movej_canfd`（movej 走停模式今日实测不可用：跑飞+
     保护性故障）+ `use_archive_right_limits:=false`（档案限位过时，j5 差 1.22rad 会裁
     合法位姿）。远端备份 `start_teleop.sh.bak_1106_1129` / `start_bridge_only.sh.bak_1106_1129`
     后推送，bash -n 语法过。
  3. **全栈重启验证（新脚本首次实战）**：stop_teleop 清场 ✅ → start_teleop 全链：
     T1 ✅ / T2 ✅（fk+RViz）/ T4 ✅ / T5 ✅ / T3 ⚠️ 未连上（30485 未起，后台 1s 重试——
     脚本非阻断设计生效）。重启后 `publish_commanded_ee_state=False` 已恢复（T11 条件）。
     IK_STAT 未出现为预期（反馈链未通，bridge 连上即现）。
- 结论：canfd + 关档案限位已是脚本级基线，重启不再回退；JuxieLinker 启动后 bridge
  自动接入即可遥操。
- 未验证：微扰翻转表（等 JuxieLinker）；T23 两个挂起测试。
- 下一步：操作员启动 JuxieLinker → bridge 自动连 → 跑微扰翻转表 → 改 URDF →
  T2/T5 重启 → 遥操方向复测。
- 现场变更：全栈重启（新脚本）；远端 start 脚本更新（原版已备份）；T3 后台重试中。

## 2026-11-06 11:4x | T25 立项：JuxieLinker 服务不随臂上电自启 | ZCode + 操作员

- 现象：臂断电重启（第三次）后 .60 存活（ping 通/22 开）但 5566、30485 均关；
  操作员再次重启臂仍无法恢复。今日早些时候的断电（第一、二次）服务均自行恢复，本次例外。
- 与档案矛盾：档案明载「固件随开机自动启动，升级后重启即加载」（dual_arm_app 0.7.88，
  /home/root/DualArm/usr/）。本次未自启的触发条件未知（疑与「只断臂电源不断控制宿主」
  的断电方式有关——前两次也是同样方式，但都恢复了；差异待查）。
- 恢复路径：档案提供控制器 SSH（192.168.61.60 root/wanghuo，tc 主机可直达），
  可远程诊断/拉起（见 TASKS T25，待操作员裁定是否远程处置）。
- 影响：T25 未解决前，遥操/微扰实测/URDF 修复验证全部阻塞。
- 现场变更：无。

## 2026-11-06 12:3x | 新 URDF v2 对账分析（用户提供的 download/hr_up_7dof_fixed） | ZCode

- 动作：对新提供的 v2 URDF 包（含 README_v2.md 与 docs/URDF检查报告_2026-11-06.md）做
  运动学级逐位对比 + 报告通读。
- 证据原文：
  - 关节级对比脚本输出：14 关节 type/axis/origin_xyz/origin_rpy/limits **全部一致、零差异**；
  - 惯性差异连杆：Link1L~Link6L（本次唯一修复：左臂惯性坐标系未随网格 180° 旋转补偿同步，
    Link5L 质心曾超出包围盒 85.5mm）；
  - 报告自认：「实机映射和 TCP 仍待真实资料确认」「当前检查不能证明模型与实机运动学一致」
    「后续待办：统一每轴限位、正方向及零位」；
  - 报告 §3：左臂 Joint1L~7L origin 被重写（软件镜像构造，与厂商原始左臂不同）——与
    肩膀方向反向互证。
- 结论：v2 = 惯性修复包（动力学/重力补偿/RViz 有用），**运动学与真机映射不一致的问题
  原样保留**，遥操方向问题不在其修复范围；方向修复仍依赖微扰翻转表（探针运行中）。
- 未验证：探针翻转表（运行中）；URDF 轴翻转后残差改善幅度。
- 现场变更：无（bridge 停止中给探针让路）。

## 2026-11-06 12:5x–13:4x | 臂「帧受理不执行」排查链 + T25 结案 + 新问题：电机 CAN 零应答 | ZCode + 操作员

- 排查时间线（每步带证据）：
  1. **新 URDF v2 对账**（用户提供 download/hr_up_7dof_fixed）：14 关节定义逐位一致零差异，
     仅左臂 Link1L-6L 惯性坐标系修复——运动学映射不一致原样保留，方向问题不在其修复范围（commit 2da7e0d）。
  2. **遥操复测失败**：离合注册✅锚定✅但 bridge sent=0/ik_moved=0——Puppetry 新实例目标流
     暂态交替（两姿态 z 0.57↔-0.65 互跳）；稳定后目标流 ~90Hz 平滑跟手 ✅，但臂位移 0.000，
     bridge +2s sent=100（帧全部受理）、max_err 曾达 1.669rad（IK 指令跑离不动的臂）。
  3. **「受理不执行」排除法**：网页关闭✗、急停旋出✗、网页 enable 点击✗、协议 enable✗、
     拖动模式已查、固件 GUI 自身 movej 也**无法运动** → 断点收敛到**固件↔电机层**。
  4. **T25 破案（JuxieLinker 不自启根因）**：控制器根分区 **26G/26G 100% 满**（/home/root/logs
     505 个会话目录 24G CSV 日志）→ 固件启动时建日志目录失败 → abort（`cannot create
     directory: No space left on device`）。清理后仅 2.5G（11%）。**手动启动固件的关键**：
     `DUAL_ARM_SDK_CONFIG=/home/root/DualArm/usr/etc` 环境变量（缺失即 abort：
     `basic_string::_M_construct null not valid`，用 LD_PRELOAD getenv 钩子抓到）；
     启动序列=can_init.sh → (cd DualArm/usr) LD_LIBRARY_PATH=./lib:/usr/local/lib 启动
     interface_node → 30485/5566 监听 → 网页登录（**密码 123456**）→ 网页内 enable。
  5. **固件 GUI 截图三张**：限位页与 get_config 逐项一致（J2=[-135°,+14.9°] J4=[-39.5°,+129.5°]
     J6=±90° 其余 ±159.9°）——**7 月档案限位表确认过时**；示教页有 movej/enable/disable/清错/回零
     按钮；GUI 关节读数与协议反馈存在可疑偏差（J5R：-94.9° vs -111.7°、J7L：-87.3° vs -32.4°，
     读数时间不同步未定论，需同步读数复核——若坐实则存在协议↔GUI 换算层，对 T20 对账重要）。
  6. **新问题（未解）：电机 CAN 零应答**。can0/can1 UP、TX ~500帧/s 持续猛发、
     **RX=0 完全冻结**（TX_ERR=RX_ERR=0——收发器有 ACK 但电机主控不应答）；
     协议 enable/clear_fault/反馈查询全部「成功」（反馈值疑似缓存）。
     物理检查已做：急停旋出✅、GUI 关闭✅、断路器合上✅——电机仍零应答。
- 当前链路状态：T1/T2/T4/T5 运行（canfd 桥接，publish_commanded_ee_state=False），
  固件手动启动运行中（30485/5566 监听），**电机无应答，臂不可动**。
- 下一步排查候选：① 确认断路器是否真给臂供电（合上后臂上有无指示灯/上电声）；
  ② 臂基座独立电源开关检查；③ GUI↔协议读数同步对比（验反馈活性）；
  ④ 对照 5 月厂商启动顺序（有无遗漏步骤）；⑤ 联系巨蟹智能（积累的问题已够一次打包报修：
  左肘两次堵转 + 电机 CAN 零应答 + 固件不自启/磁盘满设计缺陷 + 档案限位表过时）。
- 现场变更：控制器磁盘清理（24G 日志删除，保留 3 会话）；固件手动启动运行中；
  桥/IK 等按一键脚本重启；微扰探针被迫中止（无翻转表产出）。

## 2026-11-06 13:5x | T26 续：受控上电测试无效果，定性为硬件层问题 | ZCode + 操作员

- 测试与证据：
  1. **反馈活性测试**：J1L 连续 10 采样（1Hz）位级冻结（0.0314466062 ×10，极差 0）——
     固件返回**缓存值**（电机启动瞬间应答过一次：1224 帧 RX + 初始位置），此后电机零发送。
  2. **受控上电测试**：CAN RX 90 秒监控（3s 采样），现场断电 10s 再上电——全程 RX=0，
     电机未上线。
  3. **CAN 物理层**：TX ~500帧/s 持续、TX_ERR=RX_ERR=0（收发器 ACK 正常但电机主控
     零发送）——电机 CAN 收发器有电、主控板无响应，最符合「电机主控欠压/未上电/保护静默」。
  4. 附带发现：控制器时钟不准（显示 11:10，实际 ~13:50，RTC 漂移，小问题记录备查）。
- 定性：**硬件/供电层问题，软件侧无可再挖**。已排除（当日证据）：急停、GUI 占用、拖动模式、
  网页 enable、协议 enable/clear_fault、断路器合闸、受控断电重启。
- 移交现场/厂家的清单（今日积累，足够一次打包报修）：
  ① 左肘电机 4 两次堵转（码 17039362/8）；② 电机 CAN 零应答（TX 正常 RX=0，收发器 ACK）；
  ③ 固件不自启 + 日志无限累积撑爆 26G 盘（设计缺陷）；④ 档案限位表与固件 GUI/get_config
  不一致；⑤ 需厂商提供：固件官方启动流程、web 口令官方说明、控制器原生 URDF/D-H（T20 用）。
- 现场待做（操作员）：找到臂的**真正供电开关**（基座开关/指示灯）确认电机主控供电；
  若供电确认正常仍零应答 → 厂家检修（T24+T26 合并报修）。
- 现场变更：无新变更（固件保持手动启动运行态）。

## 2026-11-06 14:2x–14:3x | 启动门禁修复 + CANFD 遥操恢复 + 手静漂移取证 | Codex + 操作员

- 启动脚本修复（本地与远端已同步）：
  - `tools/dm-exo/start_teleop.sh`、`start_bridge_only.sh` 增加 `.15/.60` ping 门禁、
    30485 单连接占用检查、T3 存活/连接/反馈同步/ROS 指令接收验证；T3 失败时不再继续
    宣称全链成功；清场补充 `robot_state_publisher`/`rviz2`。
  - 远端原文件备份后推送；远端 SHA256：`start_teleop.sh`
    `22cfccc157c1989c5f15e20a9e40e2ef156e5a29233da365a482949791559e59`，
    `start_bridge_only.sh`
    `9c1eab6685c749ad3d7e49eed66873b4bd09e3d96978c78b2c44c622d22`。
- 全链启动原文证据：T1/T2/T3/T4/T5 全部启动；T3 日志确认“已连接控制器”、
  “已用 get_joint_position 同步”，随后 `cmd_rx>0`；30485 ESTAB 的进程为 T3 Python；
  仅一个 robot_state_publisher。参数保持 `control_mode=movej_canfd`、`move_speed=15`、
  `auto_enable=false`、`use_archive_right_limits=false`。
- 真机验证：`press02_left_142621.npz` 左离合按住 4.2s，外骨骼移动 698mrad，
  IK 瞬跳 0，真机移动 78mrad，bridge reject=0。操作员随后确认设备已经可以正常遥控，
  证明 ROS→bridge→30485 CANFD→真机链路恢复。
- 风险取证：`press03_left_142721.npz` 长按约 47.5s；0.5s 手静窗口最坏出现
  外骨骼 6.2mrad、IK Joint7L 114.0mrad、真机 86.2mrad，而末端目标仅
  4.6mm/0.46°。证据支持“上游小变化/残余误差→近奇异 IK 冗余轴漂移→CANFD 正确跟随”，
  不是 CAN 通讯拒绝；该风险尚未修复。
- 安全处置：长按测试后已运行 `stop_bridge_only.sh`，T3 停止；离合最终回到 false，
  录包器与监视器停止，其余 T1/T2/T4/T5 保留运行。再次启动遥操前必须清场并确认
  30485 唯一连接；测试先短时、低速，手静自运动立即松开离合。

## 2026-11-06 14:31 | 操作员要求恢复遥操，全链安全重启 | Codex + 操作员

- 执行：远端依次运行 `stop_teleop.sh`、`start_teleop.sh`，先清掉重复 MessageBridge 与
  旧进程，再按新门禁启动全链。
- 启动原文：`.15/.60 网络正常`、`30485 未被本机其他客户端占用`、T1–T5 全部成功、
  `T3 已收到 joint_commands，ROS → canfd 桥接链路完整`。
- 启动后无运动基线：30485 唯一 ESTAB 进程为 T3 `python3 pid=96132`；
  robot_state_publisher 数量=1；T3 状态 `connected=True`、`fb_synced=True`、
  `cmd_rx=2369`、`sent=0`、`reject=0`、`max_err=0.000`、`ik_moved=0`。
- 现场状态：遥操已启动，等待操作员按住离合做短时、极慢、小范围效果检查。

## 2026-11-06 14:54 | 左臂方向反转离线定责 + T26 复发（电机死亡时刻首次被录包捕获） | ZCode + 操作员

- 动作：拉取远端 press02/03_left_142x 与新录 dir01_left.npz（本机 14:41 起用 record_compare.py 240s
  只读录包，恰覆盖事发窗）；离线逐关节/末端三段链路分析；pexpect 经 tc 主机只读控制器 CAN 统计。
- 证据（原文/数字）：
  - **上游无罪**：dir01_left.npz 段#1 外骨骼末端左位移 x=+0.2603m（操作员前推），目标末端 x=+0.2603m，
    `exo末端x vs tgt末端x: corr=+1.000`；段#2 回拉 x=-0.2157 ↔ 目标 -0.2163，同样 +1.000。
  - **IK 按 URDF 自洽**：URDF-FK(指令) ↔ 目标 corr x:+0.999。
  - **真机半执行后死亡**：段#1（t=61.1–70.0）左臂净跟踪比值 J4L=0.526 J5L=0.447 J7L=0.644，
    J1L/J3L/J6L≈0（J1L corr=-0.194）；对照 press02_left_142621.npz（14:26）比值 0.93–1.05 全轴忠实 →
    14:26→14:42 之间机器劣化。
  - **死亡时刻**：`左臂反馈最后一次变化: t=66.43s (wall≈1793947328)`；本 T3 会话（14:31 起）首条拒绝
    `[ERROR] [1793947327.080945998] 控制器拒绝运动 movej_canfd program_run_failed (累计1)`——同一秒。
    此后 172s 跨 5 次按压左臂 7 轴段首值逐位相同（J1L +26.95° J4L -33.48° J7L -148.64°，极差 0.00 mrad）
    =固件返回锁存缓存。右臂同期仅 0.096mrad 量化噪声。会话累计拒绝 ~920（46 行×20 节流）。
  - **限位余量**：拒绝起点 J4L=-32.75°（GUI 限位 -39.5°，余 6.8°）、J7L=-148.6°（限 ±159.9°，余 11.3°）；
    而 IK 段#1 净指令 J4L -0.607rad(-34.8°) J7L -0.789rad(-45.2°)——按真机限位双双越界；bridge 左臂
    用 URDF_LIMITS_RAD=±(3.14-0.05)rad 裁剪（tools/exton_bridge.py:91/681），T23 动态限位未部署。
  - **控制器 CAN 现况**：can0 RX=0 TX=946,762；can1 RX=0 TX=920,810（TX_ERR=RX_ERR=0）；
    /proc/net/can/stats RXF=0——电机零应答（T26 同签名），但今天 14:42 前电机确认执行过运动
    （段#1 反馈在变），即「电机在遥操中死亡」首次被完整录包捕获。
- 结论：
  1. 方向反转定责：上游映射与 IK 均无罪（corr 1.000/0.999），反向发生在 URDF→真机运动学映射层
     （T20 结构性不一致的遥操表现），与「肩膀方向翻转表待建」一致——未验证项=逐轴翻转量（探针跑通才有）。
  2. 手臂不动断点：S7 电机/CAN 层再次零应答（T26 复发）；触发条件与「按 URDF 宽限位驱动 J4L/J7L
     逼近真机软限位」强相关（时间耦合精确到秒），但因果方向未实锤。
- 未验证：GUI 限位页数值是否即左臂真机限位（需 get_config 复核左臂 7 轴）；半执行(~50%)机制
  （canfd freq 钳制?）；越界→电机保护跳的因果；「向后」体感与镜像预测的逐轴对账。
- 下一步：① 操作员按 T26 恢复链处理（断电重启/查 5566 故障码/查供电）；② 恢复后**先别跑左臂遥操**——
  限位不修必复发，建议 T23 部署或临时把 bridge 左臂限位改为 get_config 值；③ 电机活了先重跑
  urdf_axis_probe.py 建翻转表（T20 路线）；④ T26 报修包加「遥操中死亡」实录证据。
- 现场变更：启动过一次只读录包（已自动结束，产物 /home/tc/桌面/dm-exo/dir01_left.npz，24.5MB）；
  经 tc 主机 pexpect 只读读取控制器 CAN 统计（ip -s link show can0/can1、/proc/net/can/stats）。
  未重启/杀任何进程，未改任何远端文件。

## 2026-11-06 15:2x | T27 修复部署：bridge 真机限位补丁 + T5 左臂 x 镜像 + 全链重启 | ZCode

- 动作：
  1. 停 T3 bridge（stop_bridge_only.sh）→ `collect_samples.py --config` 拉真机软限位
     （**首次落盘**：/home/tc/tcvg/urdf_fk_calib/config_limits_1106_150922.json）
  2. bridge 限位补丁：远端 /home/tc/tcvg/exton_bridge.py 新增 LIVE_LIMITS_RAD（get_config 实测值
     + 0.02rad 内缩），替换左右臂 URDF_LIMITS_RAD 引用（原 `self.left_limits = URDF_LIMITS_RAD`
     → `list(LIVE_LIMITS_RAD)`；右臂 else 分支同改）。备份 exton_bridge.py.bak_1506，
     **新 md5 95f18a791f544e5e0136d053bba8cbce**（基线 01597472 的增量补丁，T23 仍未整体部署）
  3. stop_teleop.sh + start_teleop.sh 全链重启（一次失败：补丁误删 URDF_LIMITS_RAD 定义行
     NameError，当轮修复后 8 门全绿）
  4. T5 重启带 `mirror_xyz_left:="[-1.0, 1.0, 1.0]"`（launch 日志原文：
     `mirror L=[-1.0, 1.0, 1.0] R=[1.0, 1.0, 1.0]`），启动脚本 /tmp/launch_t5_mirror.sh
  5. `ros2 param set /ik_retargeting publish_commanded_ee_state False`（T11 条件，重启后已重设）
- 证据：
  - get_config 原文：lowerLimits=[-2.79,-2.3562,-2.79,-0.69,-2.79,-1.57,-2.79, …同右臂]，
    upperLimits=[2.79,0.26,2.79,2.26,2.79,1.57,2.79, …]——**左右臂完全对称**，与 GUI 截图逐项吻合
    （J2=[-135°,+14.9°] J4=[-39.5°,+129.5°] J6=±90° 其余 ±159.9°）；J4 下限 -0.69rad 即昨日
    电机死亡时 J4L 逼近值（死时 -33.48°）
  - T5 健康原文：`IK_STAT clutch=L:0,R:0 … solve=0/501 win=2.00s`（2s 一条）；
    `ros2 param get mirror_xyz_left` → `array('d', [-1.0, 1.0, 1.0])`
  - T3 原文：`cmd_rx=39243 … rx=488 reject=0 fb_age=0.03s held=14`（IK 250Hz 发布、控制器应答正常）
  - can0 现况：RX=0 / TX=877,435——**电机仍未应答，待 enable**
- 过程坑（两次踩雷，均有现场证据）：
  1. 临时 launcher 漏 `export ROS_DOMAIN_ID=42` → ik_node 起在 domain 0：init 行照打、
     IK_STAT 也有（20% CPU 在跑），但域 42 图上隐形（param get "Node not found"、cmd_rx 冻结）。
     修复=launcher 首行补 export（对照实验证实与 mirror 参数无关）
  2. `pgrep -f`/`kill $(pgrep -f)` 与含明文 `robot_ik_retargeting` 的命令同 ssh 调用 → 两次
     杀掉自己会话（exit 255）——AGENTS 禁忌①的实录变体
- 未验证：enable 后 CAN RX 是否恢复（电机层 T26）；镜像后左臂方向是否正确（待操作员真机验证）；
  限位补丁下桥不会把臂推过真机软限位（逻辑推断，未实测）
- 下一步：操作员 5566 登录 enable → **关闭网页** → 短测试；方向验证 OK 后把 mirror 参数固化进
  start_teleop.sh（当前是临时 launcher，**下次 start_teleop.sh 重启 T5 会回 [1,1,1]**）
- 现场变更：bridge 限位补丁（备份 bak_1506）；全链两次重启；T5 现以 /tmp/launch_t5_mirror.sh
  运行（镜像 [-1,1,1]）；publish_commanded_ee_state=False 已设；get_config 首次落盘。
  未改 start_teleop.sh / ik_node.py / URDF。

## 2026-11-06 15:5x | “又不动”现场断点 + 一键重启脚本固化 | Codex + 操作员

- 当前断点证据：T3 `pid=98474` 存活并独占 `192.168.61.60:30485`（ESTAB）；
  `/target_robot/joint_commands` 为 1 pub / 1 sub，`/client_robot/joint_states` 为
  1 pub / 3 sub；T3 持续 `connected=True`、`fb_synced=True`、`cmd_rx` 增长。
  因此本次不是 bridge 未启动。
- 运动失败证据：T3 在运动窗口累计收到约 300 条
  `控制器拒绝运动 movej_canfd program_run_failed`；松开离合后恢复 `sent=0`、
  `max_err=0`。当前只能确认控制器/电机执行层拒绝，未在本条记录中把“未 enable”或
  “fault”任一候选写成已证实根因。
- 脚本改动：
  1. `start_teleop.sh` 固化 T27 的 `mirror_xyz_left=[-1,1,1]`，并在每次 T5 重启后强制
     `publish_commanded_ee_state=false`；任一参数设置失败即整链失败。
  2. 新增 `restart_teleop.sh`：一键执行完整清场→启动→终检；要求 T3 进程、30485 ESTAB、
     `cmd_rx>0` 三项同时成立，并明确区分“软件链已起”和“控制器拒绝运动”。
  3. 保留 `auto_enable=false` 安全门禁；脚本不会远程使能机械臂。
- 部署：远端 `~/桌面/dm-exo/` 已更新并赋可执行权限，原 start 脚本已时间戳备份；
  `bash -n` 通过。SHA256：start=`7c12a814bc6133c95c5b35dedc5ab111fbd7aa02c0a9eedd723e4828bcad0ad3`，
  restart=`7962abd5a96edc969ce096bb6313c745aa69b0fbb4e9def2036008595798c986`。

## 2026-11-06 15:34–15:35 | 操作员要求整链重启，一键脚本现场修正并通过 | Codex + 操作员

- 首次执行 `restart_teleop.sh`：T1–T5 实际均已启动，但 `start_teleop.sh` 主 shell 未 source
  ROS 环境，执行 `ros2 param set` 报 `timeout: 无法运行命令 "ros2": 没有那个文件或目录`，
  脚本正确以失败退出。修复为参数设置子 shell 显式 source Humble 并导出 domain 42；远端备份、
  同步、`bash -n` 通过，新 start SHA256=
  `f38108dea01e0e0a65eb007f4575015c94dc734f8bb9cf42e78cbbf04789a1b2`。
- 第二次完整清场重启成功：网络与 30485 门禁通过；T1–T5 全绿；左臂 X 镜像与
  `publish_commanded_ee_state=false` 固化成功；T3 收到 joint_commands。
- 无离合终检原文：`connected=True cmd_rx=790 sent=0 fb_synced=True skip=nodelta`，
  `+2s: sent=0 rx=500 reject=0 fb_age=0.01s max_err=0.000 held=14 jump=0 ik_moved=0`。
  软件链路当前正常；机器人仍保持 `auto_enable=false`，运动执行需控制器人工使能且无 fault。

## 2026-11-06 15:4x | 将一键启动规则写入接手 AI 必读文档 | Codex

- 更新 `AGENTS.md`：新增强制使用 `~/桌面/dm-exo/restart_teleop.sh` 的接手规则、网络/30485
  门禁、T1–T5 成功证据、T5 镜像与参数固化、`program_run_failed` 的分层判读，以及
  `auto_enable=false` 安全要求。
- 更新 `HANDOFF.md`：在系统架构入口处增加当前启动脚本和“bridge 连接不等于控制器可运动”
  的醒目标注，指向 5566 enable/fault 检查流程。
- `git diff --check` 通过；本次只修改本地交接文档，未改变真机运行进程。
