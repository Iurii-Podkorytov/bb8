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

    bb8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bb8_launch_path)
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
        slam_node
    ])