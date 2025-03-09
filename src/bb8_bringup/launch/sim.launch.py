from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # URDF Path
    pkg_share = FindPackageShare('bb8_description').find('bb8_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'bb8.urdf.xacro')
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', xacro_file
    ])

    # Controller Parameters
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("bb8_controllers"),
        "config", "controllers.yaml"
    ])

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bb8', '-z', '0.3'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        TimerAction(period=3.0, actions=[joint_broad_spawner]),
        TimerAction(period=5.0, actions=[diff_drive_spawner]),

    ])