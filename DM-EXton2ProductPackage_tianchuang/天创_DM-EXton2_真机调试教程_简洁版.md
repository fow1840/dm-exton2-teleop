# DM-EXton2 × 天创双臂 真机调试教程（简洁版）

> 详细版：`天创_DM-EXton2_真机调试详细教程.md`  
> **当前现场：全部 ROS 跑在 PC-A（Humble）**；tetrabot 无 ROS，不参与联调。

适用：`DM-EXton2 Product Package_tianchuang` + `tc_urdf`  
链路：外骨骼 ↔ **PC-A** ↔ wanghuo `192.168.61.60:30485` ↔ 真机  
协议：V0.3（TCP-JSON，帧尾 `#`，关节**弧度**）

---

## 1. 拷什么 / 跑什么（全在 PC-A）

| 拷贝到 PC-A | 运行 |
|-------------|------|
| `Install/install_ros2/` | MessageBridge、Puppetry |
| `exton_ros2_retargeting_demo/` | FK、IK、`exton_bridge`、RViz |

```
外骨骼 ←→ PC-A（MessageBridge + Puppetry + FK + bridge + IK）
              ↕ 30485
           wanghuo → 真机
```

---

## 2. 开干前检查

```bash
ping -c 3 <外骨骼IP>
ping -c 3 192.168.61.60      # 必须通
# 关闭占用 30485 的机械臂图形界面
source /opt/ros/humble/setup.bash && ros2 --help
```

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

改 yaml（两处外骨骼 IP **必须相同**）：

- `.../ROS_2/config/MessageBridge.yaml` → `monitoring_ip`，`base_frame: base_link`
- `.../Puppetry/config/Puppetry.yaml` → `puppeteer_monitoring_ip`

编译 demo：

```bash
cd ~/exton_ros2_retargeting_demo
python3 -m venv pink && source pink/bin/activate
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
cp scripts/exton_bridge.py ~/tcvg/exton_bridge.py
```

---

## 3. 数据流 & 何时能动

```
反馈：真机 → bridge → joint_states → FK → ee/state → MessageBridge
控制：Clutch+动手臂 → Puppetry/MB → ee/target → IK → joint_commands → bridge → 真机
```

**按住 Clutch 再动手臂，机器人才跟；松开即停。**

---

## 4. 真机启动顺序（全在 PC-A）

每个终端先：

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/exton_ros2_retargeting_demo/install/setup.bash   # T2~T5 需要
```

| 顺序 | 终端 | 命令 |
|------|------|------|
| 1 | 外骨骼 | 开机自检 |
| 2 | T1 | `cd ~/Install/install_ros2/bin/ROS_2 && ./MessageBridge` |
| 3 | T2 | `ros2 launch robot_fk_adapter robot_fk_adapter.launch.py use_sim:=false use_rviz:=true` |
| 4 | T3 | `python3 ~/tcvg/exton_bridge.py --ros-args -p control_mode:=movej -p move_speed:=15.0` |
| 5 | T4 | `cd ~/Install/install_ros2/bin/Puppetry && ./Puppetry` |
| 6 | T5 | `source pink/bin/activate && ros2 launch robot_ik_retargeting ik_retargeting.launch.py` |

试控：不按不动 → 按住 Clutch 小范围慢动 → 松开停止。

---

## 5. 快速验收

- [ ] ping 外骨骼、wanghuo  
- [ ] MessageBridge / Puppetry OK；Clutch 正常  
- [ ] 左右 ee/state 有数据  
- [ ] bridge 连上 30485；`/client_robot/joint_states` ≥ 30Hz（`joint*`/`jointL*`）  
- [ ] 按 Clutch 有 joint_commands；真机小范围跟随  

```bash
ros2 topic echo /clutch/left
ros2 topic hz /target_robot/left_ee/state
ros2 topic hz /client_robot/joint_states
ros2 topic echo /target_robot/joint_commands --once
```

---

## 6. 常见卡点

| 现象 | 处理 |
|------|------|
| MessageBridge 连不上 | 外骨骼 IP / 自检 / ping |
| 无 ee/state | FK+bridge；`use_sim:=false` |
| 按 Clutch 无动作 | 是否起 Puppetry / IK |
| 30485 失败 | 关图形界面；PC-A→61.60 |
| 猛抖 | 对齐后再 Clutch；降 `move_speed` |
| Pink 双臂异常 | 换客户自研 IK |
| tetrabot 无 ros2 | **忽略 tetrabot，全用 PC-A** |

---

## 7. 名称速查

| 项目 | 值 |
|------|-----|
| 右臂 | `Link7` / `joint1`~`joint7`（协议 R1~R7） |
| 左臂 | `LinkL7` / `jointL1`~`jointL7`（协议 L1~L7） |
| 基座 | `base_link`（IK root：`base`） |

排错细节见详细版分阶段调试。
