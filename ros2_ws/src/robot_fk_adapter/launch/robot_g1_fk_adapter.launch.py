from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('g1_description')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'g1_29dof_mode_11.urdf'])
    # 默认加载的 RViz 配置路径
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'urdf.rviz'])
    
    # 参数定义 
    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false', description='Set to false for real robot'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # 1. Robot State Publisher (始终运行)
        # 负责将全量关节状态转换为 TF 树供适配器查询
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['cat ', urdf_path])}],
            remappings=[('/joint_states', '/client_robot/joint_states')]
        ),

        # 2. FK Pose Adapter (始终运行)
        # 采用订阅触发模式，反馈频率与关节状态话题同步
        Node(
            package='robot_fk_adapter',
            executable='joint_to_pose_adapter',
            name='fk_adapter',
            parameters=[{
                'base_frame': 'pelvis',
                'left_ee_frame': 'left_wrist_yaw_link',
                'right_ee_frame': 'right_wrist_yaw_link'
            }]
        ),

        # 3. Dummy Robot Driver (仅在仿真模式启动)
        # 提供初始 0.0 关节状态并接收 IK 指令 
        Node(
            condition=IfCondition(use_sim),
            package='robot_fk_adapter',
            executable='dummy_robot_driver',
            name='dummy_driver',
            parameters=[{'urdf_path': urdf_path, 'publish_frequency': 500.0}]
        ),

        # 4. RViz 可视化
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz_monitor',
            arguments=['-d', default_rviz_config_path],
        )
    ])