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


