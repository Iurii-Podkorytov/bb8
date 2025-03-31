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

    world_file = PathJoinSubstitution([
        FindPackageShare('bb8_bringup'),
        'worlds', 'test.sdf'             # Path to your .sdf file
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
        launch_arguments=[
            ('pause', 'false'),
            ('world', world_file)
            ]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bb8', '-z', '0.26'],
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

    head_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller"],
    )

    sphere_tf_publisher = Node(
        package="bb8_description",
        executable="p3d_tf_broadcast",
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=['src/slam_params.yaml'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        joint_broad_spawner,
        diff_drive_spawner,
        head_controller_spawner,
        sphere_tf_publisher,
        slam_node,
    ])