from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Required for launching nodes

def generate_launch_description():
    # Include the existing navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bb8_bringup'),
                'launch', 'navigation.launch.py'
            ])
        ]),
    )

    # Add the red_circle_follower node
    red_circle_follower_node = Node(
        package='bb8_navigation',
        executable='red_circle_follower',
        name='red_circle_follower',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        # First bring up navigation
        navigation_launch,
        
        # Then launch the red circle follower
        red_circle_follower_node
    ])