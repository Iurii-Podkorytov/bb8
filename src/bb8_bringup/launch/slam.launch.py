from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Required for launching nodes

def generate_launch_description():
    # Include the existing navigation launch file
    bb8_pkg = FindPackageShare('bb8_bringup')
    bb8_launch_path = PathJoinSubstitution([bb8_pkg, 'launch', 'bb8.launch.py'])
    bb8_navigation_pkg = FindPackageShare('bb8_navigation')
    nav2_pkg = FindPackageShare('nav2_bringup')

    bb8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bb8_launch_path)
    )

    # Navigation parameters and map
    nav2_bringup_params_file = PathJoinSubstitution([
        bb8_navigation_pkg,
        'config', 'nav2_params.yaml'
    ])
    
    nav2_map_file = PathJoinSubstitution([
        bb8_navigation_pkg,
        'maps', 'map.yaml'
    ])

    # Include BB8 base launch
    bb8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bb8_launch_path)
    )

    # Include Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_pkg, 'launch', 'bringup_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_bringup_params_file,
            'map': nav2_map_file,
            'autostart': 'True',
            'slam': 'False',
        }.items()
    )


    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[PathJoinSubstitution([
                    FindPackageShare('bb8_navigation'),
                    'config', 'slam_params.yaml'
            ])],
        output='screen'
    )

    return LaunchDescription([
        bb8_launch,
        nav2_bringup,
        slam_node,
    ])