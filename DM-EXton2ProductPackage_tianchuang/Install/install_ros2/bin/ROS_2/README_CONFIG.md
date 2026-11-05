# MessageBridge 配置说明

## 概述

MessageBridge 支持通过 YAML 配置文件和命令行参数自定义参数，方便客户根据实际需求修改配置。

## 配置文件位置

配置文件位于：`config/MessageBridge.yaml`

## 配置优先级

配置参数的优先级从高到低为：

1. **命令行参数**（最高优先级）
2. **YAML配置文件**
3. **代码默认值**（最低优先级）

例如：如果同时在命令行和YAML中指定了 `monitoring_ip`，则使用命令行的值。

## 可配置参数

### 1. 网络配置 (network)

- **monitoring_ip**: 外骨骼的IP地址
  - 默认值: `"127.0.0.1"`
  - 用途: 连接到Puppeteer主控板接收数据

- **advertise_ip**: 本机对外通告的IP地址
  - 默认值: `""` (空字符串表示自动检测)
  - 用途: 向Service Hub注册时使用的本机IP

- **bridge_pub_port**: MessageBridge发布服务的端口号
  - 默认值: `9900`
  - 用途: ZMQ Publisher绑定的端口

### 2. ROS话题配置 (ros_topics)

所有ROS话题名称都可以通过配置文件修改：

- **joints**: 关节角度话题
  - 默认值: `"/joints"`
  
- **ee_poses**: 末端位姿话题
  - 默认值: `"/ee_poses"`
  
- **ee_trigger**: 扳机位置数据话题
  - 默认值: `"/position_data_hand"`
  
- **joint_state**: 外骨骼关节状态话题
  - 默认值: `"/exoskeleton/joint_states"`
  
- **tact_force**: 触觉力反馈话题
  - 默认值: `"/tact_force"`
  
- **cmd_vel**: 速度控制话题
  - 默认值: `"/custom/cmd_vel"`
  
- **custom_a**: 自定义参数A话题
  - 默认值: `"/custom/custome_a"`

## 使用方法

### 方法1：使用YAML配置文件（推荐）

1. 编辑 `config/MessageBridge.yaml` 文件
2. 修改需要的参数值
3. 保存文件
4. 启动程序：`./MessageBridge`

### 方法2：使用命令行参数

```bash
./MessageBridge --monitoring-ip 192.168.1.100
```

可用的命令行参数：
- `--monitoring-ip`: 外骨骼设备IP地址
- `--advertise-ip`: 本机对外通告的IP地址（docker中需要填宿主ip）
- `--bridge-port`: MessageBridge发布服务的端口号
- `--service-name`: 注册到Service Hub的服务名称
- `--wait-master-ms`: 等待ROS Master启动的最大毫秒数（仅ROS1，0表示不等待）

### 方法3：混合使用（命令行优先级更高）

在YAML中设置常用配置，需要临时修改时使用命令行参数覆盖：

```bash
# YAML中配置了monitoring_ip=192.168.1.100
# 命令行临时改为192.168.1.200（命令行优先）
./MessageBridge --monitoring-ip 192.168.1.200
```

## 配置示例

### 修改Puppeteer主板IP地址

**YAML方式：**
```yaml
network:
  monitoring_ip: "192.168.1.100"  # 修改为实际的主板IP
  advertise_ip: ""
  bridge_pub_port: 9900
```

**命令行方式：**
```bash
./MessageBridge --monitoring-ip 192.168.1.100
```

### 修改ROS话题名称

**YAML方式：**
```yaml
ros_topics:
  joints: "/my_robot/joints"           # 自定义话题名
  ee_poses: "/my_robot/ee_poses"
  ee_trigger: "/my_robot/trigger"
  joint_state: "/my_robot/joint_states"
  tact_force: "/my_robot/tactile"
  cmd_vel: "/my_robot/cmd_vel"
  custom_a: "/my_robot/custom_a"
```

### 等待ROS Master启动（仅ROS1）

```bash
# 等待最多10秒让ROS Master启动
./MessageBridge --wait-master-ms 10000
```

如果不想等待，ROS Master未就绪时直接退出：
```bash
./MessageBridge --wait-master-ms 0
```

## 日志输出

程序启动时会在日志中显示加载的配置信息：

```
[INFO] 命令行参数: monitoring_ip=127.0.0.1, advertise_ip=, bridge_port=9900, ...
[INFO] 成功加载YAML配置文件: config/MessageBridge.yaml
[INFO]   monitoring_ip: 192.168.1.100
[INFO] 使用YAML配置覆盖monitoring_ip: 192.168.1.100
[INFO] === MessageBridge 最终配置 ===
[INFO]   monitoring_ip: 192.168.1.100
[INFO]   advertise_ip: 
[INFO]   bridge_pub_port: 9900
[INFO] ROS发布者已创建，使用话题:
[INFO]   joints: /joints
[INFO]   ee_poses: /ee_poses
...
```

## 注意事项

1. 如果配置文件不存在或加载失败，程序会使用默认配置继续运行
2. 命令行参数的优先级高于YAML配置
3. 修改YAML配置后需要重启程序才能生效
4. 话题名称必须符合ROS命名规范（以`/`开头）
5. IP地址必须是有效的IPv4格式
6. 端口号范围：1024-65535

## 故障排除

### 配置文件未生效

- 检查配置文件路径是否正确：`config/MessageBridge.yaml`
- 检查YAML语法是否正确（注意缩进）
- 查看日志确认配置是否成功加载
- 确认没有使用命令行参数覆盖了YAML配置

### 找不到配置文件

程序会输出警告信息：
```
[WARN] 无法加载YAML配置文件: config/MessageBridge.yaml, 将使用默认配置
```

这种情况下程序会使用代码中的默认值继续运行。

### ROS Master未就绪（仅ROS1）

如果看到以下错误：
```
[ERROR] ROS master not detected, please start roscore or use --wait-master-ms to specify wait time
```

解决方法：
1. 先启动 `roscore`
2. 或使用 `--wait-master-ms 10000` 让程序等待ROS Master启动

### 命令行参数和YAML配置冲突

记住：**命令行参数优先级更高**，会覆盖YAML配置。

例如：
- YAML中设置 `monitoring_ip: "192.168.1.100"`
- 命令行使用 `--monitoring-ip 127.0.0.1`
- 最终使用: `127.0.0.1`（命令行的值）
