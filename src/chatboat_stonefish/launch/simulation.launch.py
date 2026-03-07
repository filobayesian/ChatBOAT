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

    # stonefish_simulator expects 6 CLI args:
    #   argv[1]=data_dir argv[2]=scenario argv[3]=rate
    #   argv[4]=window_w argv[5]=window_h argv[6]=quality
    stonefish_node = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        output='screen',
        arguments=[
            data_dir,
            scenario_file,
            LaunchConfiguration('simulation_rate'),
            '1200',   # window width
            '900',    # window height
            'medium', # rendering quality
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
        stonefish_node,
        description_launch,
    ])
