import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description = get_package_share_directory('chatboat_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'chatboat_uvms.urdf.xacro')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='girona500',
        description='Robot namespace'
    )

    robot_description = Command(['xacro ', xacro_file,
                                 ' namespace:=', LaunchConfiguration('namespace')])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ['/girona500/joint_states']}],
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        robot_state_publisher,
        joint_state_publisher,
    ])
