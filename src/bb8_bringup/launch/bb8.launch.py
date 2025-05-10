from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # URDF Path
    pkg_share_description = FindPackageShare('bb8_description').find('bb8_description')
    xacro_file = os.path.join(pkg_share_description, 'urdf', 'bb8.urdf.xacro')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', xacro_file
    ])

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': True}]
    )

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false',
            'world': PathJoinSubstitution([
                FindPackageShare('bb8_bringup'),
                'worlds', 'test_world.sdf'
            ])
        }.items()
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bb8'],
        output='screen'
    )

    # Controller Spawners
    wheels_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheels_controller"],
        parameters=[{'use_sim_time': True}],
    )

    joint_broad_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[{'use_sim_time': True}],
    )

    head_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller"],
        parameters=[{'use_sim_time': True}],
    )

    head_z_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_z_controller"],
        parameters=[{'use_sim_time': True}],
    )

    # Static transforms for sphere visualization in RViz
    sphere_tf_publisher_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'sphere_center', 'sphere_yaw'],
            output='screen',
            name='sphere_yaw_tf_publisher',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'sphere_yaw', 'sphere_pitch'],
            output='screen',
            name='sphere_pitch_tf_publisher',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'sphere_pitch', 'sphere'],
            output='screen',
            name='sphere_tf_publisher',
            parameters=[{'use_sim_time': True}]
        ),
    ]


    base_tf_publisher_node = Node(
        package="bb8_description",
        executable="base_transform",
        parameters=[{"use_sim_time": True}]
    )

    head_pid_controller_node = Node(
        package="bb8_controllers",
        executable="head_pid_controller",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('bb8_controllers'),
                'config', 'controllers_params.yaml'
            ]),
            {"use_sim_time": True}
        ],
        output='screen'
    )

    hamster_controller_node = Node(
        package="bb8_controllers",
        executable="hamster_controller",
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    wheels_odom_node = Node(
        package="bb8_controllers",
        executable="wheels_odom",
        name="wheel_odometry_publisher",
        parameters=[{
            "use_sim_time": True,
            "publish_tf": False,
        }],
        remappings=[
            ('/odom', '/odom_wheels') # Remap its output from /odom to /odom_wheels
        ],
        output='screen'
    )

    imu_covariance_adder_node = Node(
        package='bb8_description',
        executable='imu_covariance_adder',
        name='imu_covariance_adder',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('imu_raw', '/imu_base'),
            ('imu_with_covariance', '/imu_base_cov')
        ]
    )

    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('bb8_controllers'),
                'config', 'ekf_params.yaml'
            ]),
            {'use_sim_time': True}
        ],
        remappings=[
            ('odometry/filtered', '/odom')
        ]
    )

    scan_filter_node = Node(
        package="bb8_navigation",
        executable="scan_filter",
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch_include,
        spawn_entity_node,
        *sphere_tf_publisher_nodes,
        base_tf_publisher_node,

        # Controllers and Spawners
        joint_broad_spawner_node,
        wheels_controller_spawner_node,
        head_controller_spawner_node,
        head_z_controller_spawner_node,
        head_pid_controller_node,
        hamster_controller_node,

        # Odometry Pipeline
        wheels_odom_node,
        imu_covariance_adder_node,
        ekf_localization_node,

        scan_filter_node,
    ])