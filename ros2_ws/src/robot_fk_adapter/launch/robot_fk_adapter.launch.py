from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('hr_up_7dof_fixed')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf'])
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'hr_up_fixed.rviz'])

    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)
            }],
            remappings=[('/joint_states', '/client_robot/joint_states')]
        ),

        # FK Pose Adapter
        Node(
            package='robot_fk_adapter',
            executable='joint_to_pose_adapter',
            name='fk_adapter',
            output='screen',
            parameters=[{
                'base_frame': 'base_link',
                'left_ee_frame': 'Link7L',
                'right_ee_frame': 'Link7',
                # 未按 Clutch：发真机 ee/state；按下后让位给 ik_node 指令状态
                'publish_ee_state': True,
                'yield_when_clutch': True,
            }]
        ),

        # Dummy Driver (仿真用)
        Node(
            condition=IfCondition(use_sim),
            package='robot_fk_adapter',
            executable='dummy_robot_driver',
            name='dummy_driver',
            output='screen',
            parameters=[{
                'urdf_path': urdf_path,
                'publish_frequency': 30.0
            }]
        ),

        # RViz
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz_config_path],
        )
    ])