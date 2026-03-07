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
    sim_params_file = os.path.join(pkg_stonefish, 'config', 'sim_params.yaml')

    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate', default_value='300.0',
        description='Stonefish simulation rate (Hz)'
    )

    window_res_x_arg = DeclareLaunchArgument(
        'window_res_x', default_value='1200',
        description='Window resolution X'
    )

    window_res_y_arg = DeclareLaunchArgument(
        'window_res_y', default_value='900',
        description='Window resolution Y'
    )

    # Stonefish simulator node
    stonefish_node = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        output='screen',
        parameters=[
            sim_params_file,
            {
                'scenario_description': scenario_file,
                'simulation_rate': LaunchConfiguration('simulation_rate'),
                'window_res_x': LaunchConfiguration('window_res_x'),
                'window_res_y': LaunchConfiguration('window_res_y'),
            }
        ],
    )

    # Include robot description (robot_state_publisher)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'description.launch.py')
        ),
    )

    return LaunchDescription([
        simulation_rate_arg,
        window_res_x_arg,
        window_res_y_arg,
        stonefish_node,
        description_launch,
    ])
