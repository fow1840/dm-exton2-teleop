# 双臂 URDF 修复包 v2（2026-11-06）

本包取自当前项目实际使用的 src/hr_up_7dof_fixed，根坐标系为 base_link。
包含 URDF、15 个 STL、ROS 2 包配置、RViz 配置、启动文件和原始 CSV。
已修复 Link1L～Link6L 的质心及惯性坐标系，使其与网格旋转补偿一致。
原始质量和惯性张量数值保留；关节运动学和限位未修改。

真实限位、实机零位/正方向、TCP 仍待标定，详见 docs/URDF检查报告_2026-11-06.md。
历史独立包采用 base_mount；本 v2 包采用当前工程的 base_link，请使用包内配套 RViz 配置。

## 编译与显示（ROS 2 Humble）

将此 hr_up_7dof_fixed 目录放到 ROS 2 工作空间 src/ 下，在工作空间根目录运行：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m colcon build --packages-select hr_up_7dof_fixed --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch hr_up_7dof_fixed display.launch.py
```

需安装包内 package.xml 声明的依赖。该 display 启动文件显示模型，不包含上一层工程 robot_fk_adapter 提供的滑块 GUI。

## 惯性校验

在解压后的 hr_up_7dof_fixed 目录中执行（依赖系统 Python numpy）：

```bash
/usr/bin/python3 tools/validate_urdf_inertia.py urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf
```

CSV 保留原始 SolidWorks 导出数据，用于验证坐标转换，不代表修复后的 link 坐标。
