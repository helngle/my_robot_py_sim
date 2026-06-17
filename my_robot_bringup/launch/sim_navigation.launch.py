import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

NAMESPACE = 'mobile_manipulator'


def generate_launch_description():
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_navigation = get_package_share_directory('my_robot_navigation')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    urdf_file = os.path.join(pkg_description, 'urdf', 'mobile_manipulator.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    world_file = os.path.join(pkg_description, 'worlds', 'door_world.sdf')
    slam_config = os.path.join(pkg_navigation, 'config', 'slam_toolbox.yaml')
    safety_shell_config = os.path.join(pkg_navigation, 'config', 'safety_shell.yaml')

    # Start Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': f'-r {world_file}'}.items(),
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=NAMESPACE,
        output='screen',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True}],
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_robot',
        arguments=['-topic', f'/{NAMESPACE}/robot_description',
                   '-name', 'mobile_manipulator',
                   '-x', '-2.0', '-y', '0', '-z', '0.0'],
        output='screen',
    )

    # RViz2
    rviz_config = os.path.join(pkg_description, 'rviz', 'view_robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # Bridge for /clock from Gazebo
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/model/mobile_manipulator/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        remappings=[('/model/mobile_manipulator/cmd_vel', '/cmd_vel')],
        output='screen',
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/model/mobile_manipulator/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        remappings=[('/model/mobile_manipulator/odometry', '/odom')],
        output='screen',
    )

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/world/door_passage_test/model/mobile_manipulator/joint_state'
            '@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        remappings=[
            (
                '/world/door_passage_test/model/mobile_manipulator/joint_state',
                f'/{NAMESPACE}/joint_states',
            )
        ],
        output='screen',
    )

    lidar_points_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_points_bridge',
        arguments=['/lidar/points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
        remappings=[('/lidar/points/points', '/lidar/points')],
        output='screen',
    )

    lidar_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_tf',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'lidar_link',
            'mobile_manipulator/base_footprint/head_gpu_lidar',
        ],
        output='screen',
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            'min_height': 0.20,
            'max_height': 0.85,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,
            'scan_time': 0.125,
            'range_min': 0.45,
            'range_max': 8.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/lidar/points'),
            ('scan', '/scan'),
        ],
        output='screen',
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen',
    )

    odom_to_tf = Node(
        package='my_robot_py_sim',
        executable='odom_to_tf',
        name='odom_to_tf',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
        output='screen',
    )

    safety_shell = Node(
        package='my_robot_py_sim',
        executable='safety_shell_marker',
        name='safety_shell_marker',
        parameters=[{
            'use_sim_time': True,
            'topic': '/safety_shell_array',
            'config_file': safety_shell_config,
            'padding': 0.02,
            'alpha': 0.22,
        }],
        output='screen',
    )

    footprint_marker = Node(
        package='my_robot_py_sim',
        executable='footprint_marker',
        name='footprint_marker',
        parameters=[{
            'use_sim_time': True,
            'topic': '/base_footprint_marker',
            'frame_id': 'base_footprint',
            'length': 0.72,
            'width': 0.79,
            'z_offset': 0.025,
        }],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        gazebo,
        clock_bridge,
        cmd_vel_bridge,
        odom_bridge,
        joint_state_bridge,
        lidar_points_bridge,
        lidar_frame_tf,
        pointcloud_to_scan,
        slam_toolbox,
        odom_to_tf,
        robot_state_pub,
        safety_shell,
        footprint_marker,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[rviz]),
    ])
