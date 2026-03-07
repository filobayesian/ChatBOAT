import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_stonefish = get_package_share_directory('chatboat_stonefish')
    pkg_description = get_package_share_directory('chatboat_description')

    scenario_file = os.path.join(pkg_stonefish, 'scenarios', 'chatboat_scenario.scn.xml')
    data_dir = os.path.join(pkg_stonefish, 'data')

    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate', default_value='300.0',
        description='Stonefish simulation rate (Hz)'
    )

    # Stonefish simulator node — expects scenario file and data dir as CLI args
    stonefish_node = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        output='screen',
        arguments=[scenario_file, data_dir],
        parameters=[{
            'simulation_rate': LaunchConfiguration('simulation_rate'),
        }],
    )

    # Include robot description (robot_state_publisher)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'description.launch.py')
        ),
    )

    return LaunchDescription([
        simulation_rate_arg,
        stonefish_node,
        description_launch,
    ])
