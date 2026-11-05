# DM-EXton2 外骨骼 × 天创双臂 真机调试详细教程（只讲怎么做）

适用：`DM-EXton2 Product Package_tianchuang` + `tc_urdf`（HR-UP-7DOF 双臂）  
现场架构（当前）：**全部 ROS 程序跑在 PC-A**（Ubuntu 22.04 + Humble）  
原因：tetrabot 无 ROS 2 环境，FK / IK / bridge 无法在其上运行  
控制器：wanghuo `192.168.61.60:30485`（协议 V0.3，TCP-JSON，帧尾 `#`，关节单位**弧度**）  
原则：只改 yaml / launch 参数 / 桥接配置，不改 SDK 源码  
前提：PC-A 能 ping 通外骨骼与 wanghuo；**先关闭占用 30485 的机械臂图形界面**（单连接）

---

## 一、先搞清楚整体架构

### 1.1 设备关系

```
[DM-EXton2 外骨骼]  ←→  [PC-A 工控机 / Ubuntu 22.04 + Humble]
                              │  MessageBridge、Puppetry
                              │  FK、IK、exton_bridge、RViz
                              ↕ TCP 30485（JSON + '#'）
                        [wanghuo 192.168.61.60]
                              ↕
                        [双臂实体机器人]
```

说明：tetrabot（192.168.61.11）若仍无 ROS，**不参与本联调**。只要 PC-A 能访问 wanghuo 即可。

### 1.2 PC-A 上跑什么

| 程序 | 作用 |
|------|------|
| MessageBridge | 连外骨骼；进出 ROS 话题（关节、Clutch、ee/state、ee/target） |
| Puppetry | 遥操作增量/Clutch 逻辑（与 MessageBridge 本机 IPC，必须同机） |
| FK launch | `/client_robot/joint_states` → TF → `/target_robot/*/ee/state` |
| exton_bridge | 30485 双向：joint_commands ↔ 真机；发 `/client_robot/joint_states` |
| IK launch（或客户自研 IK） | ee/target → `/target_robot/joint_commands` |
| RViz（可选） | 看模型 |

### 1.3 两条数据链（必须理解）

**反馈链（机器人 → 外骨骼）：**

```
真机 → TCP get_joint_position（17 维，弧度）
     → exton_bridge → /client_robot/joint_states（tc_urdf 关节名）
     → robot_state_publisher (TF)
     → FK → /target_robot/left_ee/state、/target_robot/right_ee/state
     → MessageBridge → 外骨骼
```

**控制链（外骨骼 → 机器人）：**

```
穿外骨骼、按 Clutch
         → MessageBridge / Puppetry → ee/target
         → IK → /target_robot/joint_commands
         → exton_bridge 填 17 维 → movej_canfd / movej
         → wanghuo 30485 → 真机动
```

缺任何一环都不行。常见错误：只跑 FK+IK+bridge，没跑 MessageBridge 和 Puppetry；或 30485 被图形界面占用。

**机器人何时动：** 全链路已起，**并且按住 Clutch** 再动手臂。不按不应动；松开应停。

---

## 二、产品包目录（知道文件在哪）

全部拷到 **PC-A**：

```
DM-EXton2 Product Package_tianchuang/
├── Install/install_ros2/
│   └── bin/
│       ├── ROS_2/
│       │   ├── MessageBridge              ← PC-A 运行
│       │   └── config/MessageBridge.yaml  ★ 必改 IP / base_frame
│       └── Puppetry/
│           ├── Puppetry                   ← PC-A 运行
│           └── config/Puppetry.yaml       ★ 必改 IP
└── exton_ros2_retargeting_demo/           ← 也拷到 PC-A
    ├── requirements.txt
    ├── scripts/
    │   └── exton_bridge.py                ← 建议放到 ~/tcvg/ 或直接 scripts 下运行
    └── src/
        ├── tc_urdf/
        ├── robot_fk_adapter/
        └── robot_ik_retargeting/
```

本包 FK/IK launch **已按 `tc_urdf` 预配**，现场以核对参数为主。

---

## 三、部署前准备（全在 PC-A）

### 3.1 网络

```bash
ping -c 3 <外骨骼实际IP>     # 出厂常见 192.168.2.240，以现场为准
ping -c 3 192.168.61.60      # wanghuo（bridge 必需）
```

外骨骼必须与 PC-A 互通；wanghuo 必须与 PC-A 互通。

### 3.2 ROS 环境（Humble）

```bash
source /opt/ros/humble/setup.bash
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
source ~/.bashrc
```

同机运行时 DOMAIN_ID 仍建议统一为 42。

### 3.3 拷贝与权限

```bash
# 示例路径，按现场调整
mkdir -p ~/Install ~/tcvg
# 将 Install/install_ros2 拷到 ~/Install/install_ros2
# 将 exton_ros2_retargeting_demo 拷到 ~/exton_ros2_retargeting_demo
cp ~/exton_ros2_retargeting_demo/scripts/exton_bridge.py ~/tcvg/
chmod +x ~/Install/install_ros2/bin/ROS_2/MessageBridge
chmod +x ~/Install/install_ros2/bin/Puppetry/Puppetry
chmod +x ~/tcvg/exton_bridge.py
```

### 3.4 编译 exton_ros2_retargeting_demo（Humble）

```bash
cd ~/exton_ros2_retargeting_demo
sudo apt install -y ros-humble-desktop ros-humble-robot-state-publisher ros-humble-tf2-tools
python3 -m venv pink
source pink/bin/activate
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3.5 客户侧

1. **关闭占用 30485 的机械臂图形界面**
2. wanghuo 已上电：`ping 192.168.61.60`
3. 急停可用、工作空间无人

---

## 四、配置修改（只改 yaml）

### 4.1 MessageBridge.yaml

路径：`Install/install_ros2/bin/ROS_2/config/MessageBridge.yaml`

```yaml
network:
  monitoring_ip: "<外骨骼实际IP>"   # 必改
frame_ids:
  base_frame: "base_link"            # 与 tc_urdf 一致
```

### 4.2 Puppetry.yaml

路径：`Install/install_ros2/bin/Puppetry/config/Puppetry.yaml`

```yaml
communication:
  puppeteer_monitoring_ip: "<外骨骼实际IP>"  # 必须与 MessageBridge 相同
mirroring:
  enable_mirroring: false
```

不要改：`use_ipc: true`、`messagebridge_tcp_ip: "127.0.0.1"`

---

## 五、天创 URDF 关键信息

| 项目 | 名称 |
|------|------|
| 基座（消息/FK） | `base_link` |
| IK root | `base` |
| 右臂末端 / 关节 | `Link7` / `joint1`~`joint7` |
| 左臂末端 / 关节 | `LinkL7` / `jointL1`~`jointL7` |

---

## 六、Launch / Bridge 核对

### 6.1 FK

- 包：`tc_urdf`，URDF：`urdf/tc.urdf`
- `left_ee_frame=LinkL7`，`right_ee_frame=Link7`
- remapping：`/joint_states` → `/client_robot/joint_states`
- 真机：`use_sim:=false`

### 6.2 IK

- `root_link_name=base`
- `target_link_names=["Link7","LinkL7"]`（右、左）
- `preferred_joints`：`joint1~7` + `jointL1~L7`（与 URDF 匹配）

说明：Pink `ik_node` 用关节名是否含 `left`/`right` 分流；天创是 `jointL*`，**可能双臂驱动异常**。优先客户自研 IK。

### 6.3 exton_bridge（协议 V0.3）

| 项 | 值 |
|----|-----|
| 地址 | `192.168.61.60:30485` |
| 帧尾 | `#` |
| 数组 | `"joint"`，17 维，**弧度** |
| 顺序 | 腰 / L1~L7 / R1~R7 / 摇头 / 点头 |
| 保持 | `-100` |
| L1~L7 | `jointL1`~`jointL7` |
| R1~R7 | `joint1`~`joint7` |

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/exton_ros2_retargeting_demo/install/setup.bash
python3 ~/tcvg/exton_bridge.py
```

首次真机建议：

```bash
python3 ~/tcvg/exton_bridge.py --ros-args \
  -p control_mode:=movej \
  -p move_speed:=15.0 \
  -p auto_enable:=false
```

---

## 七、分阶段调试（严格按顺序）

每个终端先执行：

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/exton_ros2_retargeting_demo/install/setup.bash   # 跑 demo 时需要
```

### 阶段 1：FK 仿真（不接外骨骼、不接真机）

```bash
ros2 launch robot_fk_adapter robot_fk_adapter.launch.py \
  use_sim:=true use_rviz:=true
```

另开终端：

```bash
ros2 topic hz /client_robot/joint_states
ros2 topic echo /target_robot/left_ee/state --once
ros2 topic echo /target_robot/right_ee/state --once
```

通过：`joint_states` 约 30~50Hz；左右 ee/state 有数；RViz Fixed Frame=`base_link`。

### 阶段 2：外骨骼 + MessageBridge

1. 外骨骼开机，自检完成  
2. `ping` 通外骨骼 IP  

```bash
cd ~/Install/install_ros2/bin/ROS_2
./MessageBridge
```

另开终端：

```bash
ros2 topic hz /exoskeleton/joint_states
ros2 topic echo /clutch/left
```

通过：MessageBridge started；关节有频率；按侧键 Clutch 为 true/false。

### 阶段 3：本机话题互通（FK 仿真仍开着）

```bash
ros2 topic hz /target_robot/left_ee/state
ros2 topic hz /exoskeleton/joint_states
```

同机应很快通。不通：`echo $ROS_DOMAIN_ID`，`ros2 daemon stop && ros2 daemon start`。

### 阶段 4：Puppetry + IK 仿真

```bash
cd ~/Install/install_ros2/bin/Puppetry
./Puppetry
```

```bash
source ~/exton_ros2_retargeting_demo/pink/bin/activate
source ~/exton_ros2_retargeting_demo/install/setup.bash
ros2 launch robot_ik_retargeting ik_retargeting.launch.py
```

操作：按住 Clutch → 慢动手臂 → 看：

```bash
ros2 topic hz /target_robot/left_ee/target
ros2 topic hz /target_robot/joint_commands
```

通过：不按 Clutch 无效；按住有 target / joint_commands；RViz 有跟随趋势。

### 阶段 5：bridge 测 30485（停 use_sim）

关掉仿真 FK。关图形界面。

```bash
python3 ~/tcvg/exton_bridge.py --ros-args \
  -p control_mode:=movej -p move_speed:=10.0
```

应看到：已连接控制器 `192.168.61.60:30485`

```bash
ros2 topic hz /client_robot/joint_states
ros2 topic echo /client_robot/joint_states --once
```

通过：连接成功；`name` 含 `joint*`/`jointL*`；`position` 为弧度；stamp 非 0。

### 阶段 6：真机完整联调

**安全：** 低速、小范围、有人监护、急停可用。

**启动顺序（严格）：**

| 顺序 | 操作 |
|------|------|
| 1 | 外骨骼开机 |
| 2 | `./MessageBridge` |
| 3 | FK：`use_sim:=false use_rviz:=true` |
| 4 | `python3 ~/tcvg/exton_bridge.py`（建议先 movej 低速） |
| 5 | `./Puppetry` |
| 6 | IK launch（或客户自研 IK） |

```bash
ros2 launch robot_fk_adapter robot_fk_adapter.launch.py \
  use_sim:=false use_rviz:=true
```

`use_sim:=false` **必须**。

试控：

1. 不按 Clutch → 不动  
2. 按住一侧 Clutch → 极慢小范围动手臂  
3. 松开 → 停止  
4. 再测另一侧  

---

## 八、终端布局（最终联调 · 全在 PC-A）

```
┌─ PC-A（唯一 ROS 工控机）─────────────────────────┐
│ T1: ./MessageBridge                              │
│ T2: FK launch  use_sim:=false                    │
│ T3: python3 ~/tcvg/exton_bridge.py               │
│ T4: ./Puppetry                                   │
│ T5: IK launch（或自研 IK）                         │
│ T6: ros2 topic echo /clutch/left（可选监控）       │
│ T7: ros2 topic echo /target_robot/joint_commands │
│     RViz 可由 FK launch 自带                      │
└──────────────────────────────────────────────────┘
```

每个终端都要：

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
# 跑 demo / bridge / IK / FK 的终端再：
source ~/exton_ros2_retargeting_demo/install/setup.bash
```

---

## 九、验收检查表

### 网络与环境

- [ ] PC-A ping 通外骨骼  
- [ ] PC-A ping 通 wanghuo `192.168.61.60`  
- [ ] Humble 可用：`source /opt/ros/humble/setup.bash && ros2 --help`  
- [ ] `exton_ros2_retargeting_demo` 已 `colcon build`  
- [ ] 30485 图形界面已关闭  

### 外骨骼

- [ ] MessageBridge started  
- [ ] Puppetry initialized  
- [ ] 两处 yaml 外骨骼 IP 一致  
- [ ] `/exoskeleton/joint_states` 有数据  
- [ ] Clutch 正常；`base_frame=base_link`  

### 机器人软件

- [ ] FK：左右 ee/state 正常  
- [ ] bridge：30485 连接成功  
- [ ] `/client_robot/joint_states` ≥ 30Hz，URDF 关节名  
- [ ] 按 Clutch 后 `/target_robot/joint_commands` 有输出  

### 真机

- [ ] 不按 Clutch 不动  
- [ ] 按住小范围跟随  
- [ ] 松开停止  
- [ ] 无上使能瞬间大幅抖动  

---

## 十、常见问题

| 现象 | 处理 |
|------|------|
| MessageBridge 连不上 | 改外骨骼 IP；等自检完成 |
| Puppetry 异常 | 与 MessageBridge IP 不一致 |
| 无 ee/state | FK/bridge 未起；`use_sim` 冲突 |
| 按 Clutch 无 target | 未起 Puppetry |
| 有 commands 真机不动 | 30485/bridge；关界面；使能 |
| 30485 连接失败 | 图形界面占端口；PC-A 到 61.60 不通 |
| 上使能抖动 | 先对齐再 Clutch；`movej` + 低 `move_speed` |
| Pink 双臂不跟 | 换客户自研 IK |
| tetrabot 无 ROS | **正常，本教程不依赖 tetrabot** |

---

## 十一、两种集成方式

**方式一（推荐）：** 客户自研 IK，订阅 ee/target，输出 joint_commands（或直连控制器）；仍要 MessageBridge、Puppetry、FK、bridge。  

**方式二：** Pink IK demo 联调；真机前充分测试，官方不保证可直接控真机。

---

## 十二、建议时间线

| 天 | 内容 |
|----|------|
| Day 1 | PC-A 拷包、编译 demo、阶段 1 FK 仿真 |
| Day 2 | 改 yaml、阶段 2 MessageBridge |
| Day 3 | 阶段 3~4 Puppetry + IK 仿真 |
| Day 4 | 阶段 5 测 30485 |
| Day 5 | 阶段 6 真机低速试控 |
| Day 6 | 双臂优化、preferred_angles / 速度 |

---

## 十三、现在最该做的 3 件事

1. **PC-A：** 确认 Humble；ping 通外骨骼与 wanghuo；改两个 yaml 的 IP  
2. **PC-A：** 编译 `exton_ros2_retargeting_demo`，跑通阶段 1 FK 仿真  
3. **PC-A：** 关 30485 界面 → 跑 `exton_bridge` → 确认 `/client_robot/joint_states`  

某一阶段卡住：发该阶段完整终端日志和 `ros2 topic list`。
