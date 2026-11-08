# T39：基于厂家 CSV 生成左臂候选 URDF

## 目标与边界

以 SolidWorks 导出的 CSV 为基准，生成一份**独立、不可部署**的左臂候选 URDF（C），与当前工作区 URDF（W）比较。它用于验证“CSV 左臂坐标定义是否比当前镜像修复版更符合已有数据”，不预设 CSV 等于控制器 D-H，也不替代真机方向验证。

T39 是离线任务，可与 T31 现场 ROS 只读采集并行。T34 仍需 T31 通过、T39 候选模型可复核、控制器协议/反馈/限位/停止行为核实，以及现场操作员明确批准。

## 已知输入

- W：`ros2_ws/src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf`
- CSV：`ros2_ws/src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.csv`
- 离线审计：`docs/URDF左右臂关节轴离线审计.md`
- 历史样本：`tools/dm-exo/urdf_fk_calib/data/calib_samples_L_1106_0943.jsonl`、`calib_L_drive.jsonl`、`calib_L2_drive.jsonl`

T32 证据：四份 URDF 的 14 个机械臂关节运动学字段相同；CSV 左臂 J2L/J3L/J4L/J6L 的统一 base_link 轴向与 W 相反。CSV 的关节限位字段为 0，不能导入候选模型。

## 生成器

`tools/dm-exo/urdf_fk_calib/csv_urdf_candidate.py`

默认只替换 Joint1L～Joint7L 的：

- `origin xyz`
- `origin rpy`
- `axis xyz`（归一化）

以下内容全部保留自 W：

- 右臂全部关节；
- 左右臂现有 lower / upper / velocity / effort 限位；
- `base_link_to_base` 固定安装变换；
- links、visual、collision、inertial、mesh URI。

可选 `--link-frames csv` 只生成一份额外的**显示诊断变体**：把左侧 Link1L～Link7L 的 visual/collision origin 改为 CSV 值，mesh URI 和 inertial 仍保留 W。该变体仅帮助判断 CAD 导出 link frame 与当前显示 frame 的关系，不参与 FK 差异结论。

生成器会拒绝缺失/重复关节行、错误 parent/child、非 revolute 关节、零轴、非有限值，以及不含 `candidate` 和 `NOT_FOR_ROBOT` 的输出文件名。它不会连接控制器、发 ROS 消息或修改生产 URDF。

## Ubuntu 22 / Python 3.10 离线运行

在仓库根目录：

~~~bash
python3 tools/dm-exo/urdf_fk_calib/csv_urdf_candidate.py
~~~

默认输出：

- `tools/dm-exo/urdf_fk_calib/candidates/HR-UP-7DOF-3KG-R-B01382-CSV-left-kinematics-candidate-NOT_FOR_ROBOT.urdf`
- `tools/dm-exo/urdf_fk_calib/candidates/HR-UP-7DOF-3KG-R-B01382-CSV-left-candidate-report.json`

生成 CSV visual-frame 诊断变体：

~~~bash
python3 tools/dm-exo/urdf_fk_calib/csv_urdf_candidate.py \
  --link-frames csv \
  --output tools/dm-exo/urdf_fk_calib/candidates/HR-UP-7DOF-3KG-R-B01382-CSV-left-visual-candidate-NOT_FOR_ROBOT.urdf \
  --report tools/dm-exo/urdf_fk_calib/candidates/HR-UP-7DOF-3KG-R-B01382-CSV-left-visual-report.json
~~~

测试：

~~~bash
python3 tools/tests/test_t32_urdf_csv_audit.py -v
python3 tools/tests/test_t39_csv_urdf_candidate.py -v
~~~

## 首轮 CI 对比结果（2026-11-08）

完整数值见 [T39 首轮离线对比结果](./T39_CSV候选URDF离线对比结果_20261108.md)。

- T39 CI run 42457377948：15/15 PASS；T32：6/6 PASS。
- 11 条历史原始记录去重为 7 个左臂姿态。
- 全部 21 对姿态：W 相对旋转误差中位数/最大值 23.94°/140.37°；CSV 候选 C 为 29.52°/75.80°。
- 排除 L01 后 15 对姿态：W 为 23.94°/140.37°；C 为 20.10°/75.80°。
- 直接 FK/TCP 位置与姿态残差在当前未验证绝对 frame 假设下，C 没有优于 W。
- Pink 合成目标 IK 对照（W FK 目标，不是真机 ground truth）：W 6/6 收敛（位置/姿态中位误差 0.115 mm/0.059°），C 0/6 收敛（206.314 mm/70.315°）。这说明在当前共同 base/frame 假设下 W/C 运动学差异很大，不证明 W 就是真机正确模型。
- 结论：CSV 候选值得保留为诊断模型，但不能直接选为最终 URDF；必须结合 T31 和 T34 继续核验。

## Pink IK 对比（与 FK 报告分开解读）

工具：`tools/dm-exo/urdf_fk_calib/compare_pink_ik.py`。

它使用项目当前算法栈（Pink 3.5.0 / Pinocchio 2.7.0），并复用 `ik_node.py` 的任务配置：

- FrameTask：position cost 300、orientation cost 40；
- PostureTask：cost 0.3；
- DampingTask：cost 12；
- OSQP，dt = 1/250 s。

为避免把尚未核实的 controller TCP/frame 当成真值，IK 对比不直接把历史 tcp14 当目标。它从每个历史 q 计算 **W 模型的 Link7L FK 目标**，然后让 W 与 C 从同一个轻微扰动 seed 求解同一目标，记录最终位置/姿态残差、迭代数和关节解偏移。L01 因 J4L 超出当前模型限位而排除。

这只能回答“两个模型的 Pink IK 对同一组 W 合成目标表现如何”，不能回答“哪个模型更接近真实机械臂”。它是离线求解器敏感性对照，不是物理方向或真机精度证明。

运行：

~~~bash
python -m pip install -r ros2_ws/requirements.txt
python tools/dm-exo/urdf_fk_calib/compare_pink_ik.py \
  --baseline ros2_ws/src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf \
  --candidate tools/dm-exo/urdf_fk_calib/candidates/HR-UP-7DOF-3KG-R-B01382-CSV-left-kinematics-candidate-NOT_FOR_ROBOT.urdf \
  --output tools/dm-exo/urdf_fk_calib/candidates/pink-ik-comparison.json
~~~

## 离线报告如何解读

报告包含：

1. W / C / CSV SHA256；
2. 左臂逐关节 origin 差异、local/world axis 和轴向 dot；
3. 左臂限位保留、右臂不变、base 固定安装不变检查；
4. 所有历史样本去重后的 W/C 对控制器 TCP 直接位置/姿态残差；
5. 相对旋转角差异（对固定 base rotation 和固定 tool rotation 不变），分别报告全部样本与排除异常 L01 后的结果。

**证据限制**：

- 历史 `tcp14` 是控制器报告的 TCP，不是独立外部测量；
- `base_link` 与控制器 base 相同、`Link7L` 与 TCP 相同、关节顺序和符号直接匹配，均是待 T31 核实的假设；
- 直接位置/姿态残差在绝对 frame / tool 未核实时只能作线索；
- 相对旋转角误差可消除固定 base/tool 旋转影响，但仍依赖样本的 q 与 TCP 配对正确；
- CSV 模型是 CAD 假设，不是真机 D-H 证明。

已有 T32 的初步相对旋转角对比：全样本 21 对时 W 中位数/最大值约 23.94°/140.37°，CSV 约 29.52°/75.80°；排除 L01 后 W 约 23.94°/140.37°，CSV 约 20.10°/75.80°。这只说明 CSV 候选在某些子集上更好、某些指标上并非全面更好，T39 脚本会重新生成可复现报告。

## RViz 隔离预览（不连接真机）

先确保候选模型与网格包已在本地工作区可解析。三个终端都使用隔离 ROS 域，避免加入现场机器人正在使用的 ROS_DOMAIN_ID：

~~~bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=97
export ROS_LOCALHOST_ONLY=1
MODEL="$PWD/tools/dm-exo/urdf_fk_calib/candidates/HR-UP-7DOF-3KG-R-B01382-CSV-left-kinematics-candidate-NOT_FOR_ROBOT.urdf"
~~~

终端 1，启动独立 robot_state_publisher：

~~~bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat "$MODEL")"
~~~

终端 2，启动关节滑块（仅发布本地 joint_states）：

~~~bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args \
  -p robot_description:="$(cat "$MODEL")"
~~~

终端 3：

~~~bash
rviz2
~~~

在 RViz 添加 RobotModel，Fixed Frame 设为 `base_link`。需要看 CSV visual-frame 变体时，只替换 MODEL 路径并重新启动这三个本地进程。

**禁止**在此预览流程中启动 exton_bridge、IK、Puppetry、MessageBridge，或连接 TCP 30485。不要将候选文件复制到生产 URDF 路径，不要用它驱动真机。

## 与 T31 / T34 的关系

- T39：离线生成 W/C 并对比历史样本。
- T31：并行核实当前 ROS 运行态、frame_id、反馈发布源和控制器坐标约定。
- T34：两者准备完成后，在现场批准下做真实关节方向五点试采。采集到的实际关节角变化与实测 TCP 旋转分别对照 W/C 的同姿态 Jacobian 预测。
- T35：只有 T34 结果可复核后，才建立唯一关节坐标转换方案。

无论 CSV 候选离线指标是否更好，T34 的真机方向验证都不能跳过。
