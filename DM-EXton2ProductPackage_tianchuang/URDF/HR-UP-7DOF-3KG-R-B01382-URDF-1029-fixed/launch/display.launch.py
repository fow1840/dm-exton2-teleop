import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hr_up_7dof_fixed')
    urdf_path = os.path.join(pkg_share, 'urdf', 'HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'hr_up_fixed.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # 关节状态发布 (零位), robot_state_publisher 订阅后计算 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str),
            }],
        ),
        # 交互滑块 GUI 请使用 teleop_ws 的 hr_up_7dof_description 包 (joint_slider_gui.py)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        # RViz 图形界面
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
