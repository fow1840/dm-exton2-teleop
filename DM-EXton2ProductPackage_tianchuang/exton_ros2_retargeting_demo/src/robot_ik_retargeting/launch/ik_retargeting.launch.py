from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_xyz(raw: str, name: str):
    vals = [float(x) for x in raw.strip().strip('[]').split(',') if x.strip()]
    if len(vals) != 3:
        raise RuntimeError(f'{name} 需 3 个数，当前={raw!r}，例: {name}:="[-1.0, -1.0, -1.0]"')
    return vals


def _launch_setup(context, *args, **kwargs):
    default_target_links = ["Link7", "Link7L"]

    preferred_joints_default = [
        "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "Joint7",
        "Joint1L", "Joint2L", "Joint3L", "Joint4L", "Joint5L", "Joint6L", "Joint7L",
    ]
    preferred_angles_default = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ]

    mirror_left = _parse_xyz(
        LaunchConfiguration('mirror_xyz_left').perform(context), 'mirror_xyz_left'
    )
    mirror_right = _parse_xyz(
        LaunchConfiguration('mirror_xyz_right').perform(context), 'mirror_xyz_right'
    )
    pos_cost = float(LaunchConfiguration('ee_position_cost').perform(context))
    orn_cost = float(LaunchConfiguration('ee_orientation_cost').perform(context))

    ik_node = Node(
        package='robot_ik_retargeting',
        executable='ik_node',
        name='ik_retargeting',
        output='screen',
        parameters=[{
            'urdf_path': PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('description_package')),
                LaunchConfiguration('urdf_relative_path'),
            ]),
            'root_link_name': 'base_link',
            'ref_link_name': 'base_link',
            'target_link_names': default_target_links,
            'preferred_joints': preferred_joints_default,
            'preferred_angles': preferred_angles_default,
            'mirror_xyz_left': mirror_left,
            'mirror_xyz_right': mirror_right,
            'ee_position_cost': pos_cost,
            'ee_orientation_cost': orn_cost,
        }],
    )
    return [ik_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'description_package',
            default_value='hr_up_7dof_fixed',
            description='Name of the robot description package',
        ),
        DeclareLaunchArgument(
            'urdf_relative_path',
            default_value='urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf',
            description='Relative path to the URDF file',
        ),
        DeclareLaunchArgument(
            'mirror_xyz_left',
            default_value='[1.0, 1.0, 1.0]',
            description='Left arm teleop delta signs [x,y,z]. hr_up_7dof_fixed: raw target already in robot frame, keep [1,1,1].',
        ),
        DeclareLaunchArgument(
            'mirror_xyz_right',
            default_value='[1.0, 1.0, 1.0]',
            description='Right arm teleop delta signs [x,y,z].',
        ),
        DeclareLaunchArgument(
            'ee_position_cost',
            default_value='300.0',
            description='IK end-effector position task cost',
        ),
        DeclareLaunchArgument(
            'ee_orientation_cost',
            default_value='40.0',
            description='IK orientation cost (restore orientation following for incremental teleop)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
