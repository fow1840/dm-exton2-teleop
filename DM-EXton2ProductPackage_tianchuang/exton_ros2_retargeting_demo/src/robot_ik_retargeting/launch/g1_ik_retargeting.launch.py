import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ================= 参数定义 =================
    pkg_name_arg = DeclareLaunchArgument(
        'description_package',
        default_value='g1_description', 
        description='Name of the robot description package'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_relative_path',
        default_value='urdf/g1_29dof_mode_11.urdf',
        description='Relative path to the URDF file'
    )
    
    # 1. 目标 Link (外骨骼控制的末端)
    default_target_links = [
        "right_wrist_yaw_link", "left_wrist_yaw_link"
    ]

    # 2. 拟人化姿态配置 - 关节名称列表
    # 这里列出您希望“干预姿态”的关节，通常是肩膀和肘部
    preferred_joints_default = [
        # --- 左臂 ---
        "left_shoulder_roll_joint",  
        "left_shoulder_yaw_joint",   
        "left_elbow_joint",          
        # --- 右臂 ---
        "right_shoulder_roll_joint", 
        "right_shoulder_yaw_joint",
        "right_elbow_joint"
    ]

    # 3. 拟人化姿态配置 - 对应的期望角度 (弧度)
    # 注意：顺序必须与上面的列表严格对应！
    # 假设：向外张开是正/负，肘部弯曲是 -1.5 (请根据实际电机方向微调符号)
    preferred_angles_default = [
        # --- 左臂 ---
        0.3,   # Left Shoulder Roll 
        -0.451,   # Left Shoulder Yaw  
        0,  # Left Elbow        
        # --- 右臂 ---
        -0.3,  # Right Shoulder Roll
        0.451,   # Right Shoulder Yaw
        0   # Right Elbow       
    ]

    # ================= 路径解析 =================
    urdf_path_resolved = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration('description_package')),
        LaunchConfiguration('urdf_relative_path')
    ])

    # ================= 节点配置 =================
    ik_node = Node(
        package='robot_ik_retargeting',
        executable='ik_node',
        name='ik_retargeting',
        output='screen',
        parameters=[{
            'urdf_path': urdf_path_resolved,
            'root_link_name': 'pelvis',   # TF 父坐标系 & IK 基准
            'target_link_names': default_target_links,
            
            # 传入拟人化参数
            'preferred_joints': preferred_joints_default,
            'preferred_angles': preferred_angles_default,
        }]
    )

    return LaunchDescription([
        pkg_name_arg,
        urdf_file_arg,
        ik_node
    ])