from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Paths to packages
    bb8_pkg = FindPackageShare('bb8_bringup')
    nav2_pkg = FindPackageShare('nav2_bringup')
    bb8_navigation_pkg = FindPackageShare('bb8_navigation')

    # Paths to launch files
    bb8_launch_path = PathJoinSubstitution([bb8_pkg, 'launch', 'bb8.launch.py'])
    
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

    return LaunchDescription([
        # Launch BB8 first
        bb8_launch,
        
        # Then launch Nav2
        nav2_bringup,
    ])