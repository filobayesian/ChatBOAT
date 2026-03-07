import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('chatboat_bringup')
    pkg_stonefish = get_package_share_directory('chatboat_stonefish')
    pkg_control = get_package_share_directory('chatboat_control')

    rviz_config = os.path.join(pkg_bringup, 'config', 'rviz_config.rviz')

    run_demo_arg = DeclareLaunchArgument(
        'run_demo', default_value='false',
        description='Launch the cube stacking demo automatically'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz for visualization'
    )

    # Simulation (includes description + Stonefish)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stonefish, 'launch', 'simulation.launch.py')
        ),
    )

    # Gripper service (always runs)
    gripper_service = Node(
        package='chatboat_control',
        executable='gripper_service',
        name='gripper_service',
        output='screen',
    )

    # Test commander (only if run_demo=true)
    test_commander = Node(
        package='chatboat_control',
        executable='test_commander',
        name='test_commander',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_demo')),
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        run_demo_arg,
        use_rviz_arg,
        simulation_launch,
        gripper_service,
        test_commander,
        rviz_node,
    ])
