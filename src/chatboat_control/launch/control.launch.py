import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_control = get_package_share_directory('chatboat_control')
    controller_params = os.path.join(pkg_control, 'config', 'controller_params.yaml')

    gripper_service = Node(
        package='chatboat_control',
        executable='gripper_service',
        name='gripper_service',
        output='screen',
    )

    test_commander = Node(
        package='chatboat_control',
        executable='test_commander',
        name='test_commander',
        output='screen',
        parameters=[controller_params],
    )

    return LaunchDescription([
        gripper_service,
        test_commander,
    ])
