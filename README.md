# DM-EXton2 双臂遥操作 —— 交接项目（2026-11-05 快照）

把 `tc` 主机（192.168.61.14）上正在调试的遥操作链路，抽成一个**最小可接手项目**：
只含源码、运行脚本、调试工具和交接文档（背景 + 任务台账 + 工作流水），
不含说明书 PDF / APK / ARM 架构 venv / 编译产物。

> 当前状态：**链路能跑通，但真机不跟手（控制器拒绝 movej）**，具体见 [HANDOFF.md](./HANDOFF.md) §四。
> 接手前请先读 [AGENTS.md](./AGENTS.md)，里面是操作规矩和踩坑清单。

---

## 一、目录结构

```
dm-exton2-teleop/
├── README.md                  # 本文件：项目说明
├── HANDOFF.md                 # 交接文档：架构、已修项、未解决问题、现场快照
├── AGENTS.md                  # 给接手 AI 的操作手册（流程、权限、禁忌、验证方法）
├── TASKS.md                   # 任务台账：接手要干的活，唯一权威状态
├── WORKLOG.md                 # 工作流水账：每步动作 + 证据，只追加
│
├── ros2_ws/                   # ROS 2 工作空间（源码层，可直接 colcon build）
│   ├── requirements.txt       # pin/pink 等算法库版本（必须 pip 装，别用 apt 的）
│   ├── src/
│   │   ├── hr_up_7dof_fixed/       # 机械臂 URDF / mesh / rviz / launch
│   │   ├── robot_fk_adapter/       # T2：关节角 → 末端位姿（正解适配器）
│   │   └── robot_ik_retargeting/   # T5：ik_node.py，增量遥操核心（今天所有修复都在这）
│   └── scripts/
│       ├── exton_bridge.py         # 旧版（阻塞式，与 .bak_blocking 一致，仅作对照）
│       └── test_wanghuo_tcp.py
│
└── tools/
    ├── exton_bridge.py             # ⚠️ 现场正在运行的版本（V0.3-async2）
    ├── exton_bridge.py.bak_blocking_1105_1854   # 18:54 异步化改造前的阻塞版
    └── dm-exo/                     # 现场调试脚本（远端 ~/桌面/dm-exo）
        ├── start_teleop.sh         # T1~T5 一键启动
        ├── stop_teleop.sh          # 一键停止（含 robot_state_publisher 清理）
        ├── start_bridge_only.sh    # 只启 bridge
        ├── stop_bridge_only.sh
        ├── clutch_probe.py         # 离合触发录包器（环形缓冲 60s）
        ├── joint_probe.py          # 直连控制器探测（必须先停 bridge）
        ├── record_compare.py       # 全量录制
        ├── analyze_compare.py      # 录包分析
        └── clutch_test*_summary.txt
```

## 二、两条路径别搞混

| | 本地（这台 x86 开发机） | 远端 tc 主机（192.168.61.14，ARM64） |
|---|---|---|
| 用途 | 读代码、改代码、静态分析 | 编译、运行、连真机 |
| 路径 | `/home/dm/下载/dm-exton2-teleop/` | `/home/tc/DM-EXton2ProductPackage_tianchuang/exton_ros2_retargeting_demo/` |
| 运行脚本 | 仅存档 | `/home/tc/桌面/dm-exo/` |
| bridge | `tools/exton_bridge.py`（副本） | `/home/tc/tcvg/exton_bridge.py`（**真身**） |

改完代码要生效，必须同步回远端；**远端 `src` 与 `build` 是硬链接**（`colcon --symlink-install`），
所以改 `ik_node.py` 后只需重启 T5，不用重新 build。

## 三、环境

- 本地：ROS 2 Humble + Python 3.10.12（已具备，可做 `colcon build` 之外的静态检查）
- 远端：Ubuntu 22.04 / aarch64 / ROS 2 Humble，`pink` venv 在 `ros2_ws/pink`（ARM64，**未随本项目拷贝**）
- 算法依赖见 `ros2_ws/requirements.txt`：`pin-pink==3.5.0`、`pin==2.7.0`、`qpsolvers==4.8.2`、`osqp==1.0.5` 等

## 四、远端运行

```bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
~/桌面/dm-exo/start_teleop.sh      # T1 MessageBridge → T2 FK → T3 bridge → T4 Puppetry → T5 IK
tail -f ~/桌面/dm-exo/logs/T3_bridge.log
~/桌面/dm-exo/stop_teleop.sh
```

单独重启 bridge（改完 bridge 代码时）：

```bash
pgrep -f "exton_bridg""e.py" | xargs -r kill
cd /home/tc && source /opt/ros/humble/setup.bash
setsid nohup python3 /home/tc/tcvg/exton_bridge.py --ros-args \
  -p control_mode:=movej -p move_speed:=15.0 -p auto_enable:=false \
  >> ~/桌面/dm-exo/logs/T3_bridge.log 2>&1 &
```

## 五、看什么

**适配与排障入口**：[外骨骼适配 FAQ 与故障排查指南](./docs/外骨骼适配FAQ与故障排查指南.md)（30 个 FAQ、12 张故障卡、只读取证、验收与报修模板）。本 README 的运行状态和命令含历史快照，现场执行以当前 TASKS、批准流程与实际运行版本为准。

0. **接手先按 [AGENTS.md](./AGENTS.md) §0 走流程**：核对现场（T01）→ 认领任务（[TASKS.md](./TASKS.md)）→ 每步落账（[WORKLOG.md](./WORKLOG.md)）。
1. **先看 [HANDOFF.md](./HANDOFF.md) §四** —— 当前唯一阻塞项（program_run_failed）的根因分析与修复方案。
2. **[TASKS.md](./TASKS.md)** —— 9 条接手任务，带优先级、依赖、验收标准和负责人（哪些必须人工配合）。
3. **[AGENTS.md](./AGENTS.md)** —— pkill 自匹配、TCP 单连接、TF 锁存等今天踩过的坑，能省好几小时。
4. **[docs/机器人现场调试手册.md](./docs/机器人现场调试手册.md)** —— 现场调试工作方式：快速实验驱动模式（假设 → 单变量最小修改 → 真机验证 → 保留/回退），实验记录模板见其 §4.1。
5. 代码入口：
   - IK：`ros2_ws/src/robot_ik_retargeting/robot_ik_retargeting/ik_node.py`
   - bridge：`tools/exton_bridge.py`（`AsyncTcpClient._tx_loop` 在第 248 行，待改 lock-step）
   - FK：`ros2_ws/src/robot_fk_adapter/robot_fk_adapter/joint_to_pose_adapter.py`

## 六、没进本项目的东西

- `ros2_ws/pink`（571 MB，ARM64 venv，本地无法用）
- `build/ install/ log/`（编译产物，远端重新 `colcon build` 即可）
- 说明书 PDF、APP APK、`Install/`（真实产品包，留在远端）
- `bags/` 20 个事件包 + 日志（体积大，留在远端 `~/桌面/dm-exo/bags`、`logs/`）
- 完整产品包副本：`/home/dm/下载/DM-EXton2ProductPackage_tianchuang/`（173 MB）
