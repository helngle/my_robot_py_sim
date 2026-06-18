import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'mobile_manipulator'


def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot_py_sim')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map = os.path.expanduser('~/ros2_ws/maps/my_first_map/my_first_map.yaml')
    map_file = LaunchConfiguration('map')
    use_route = LaunchConfiguration('use_route')
    nav2_params = LaunchConfiguration('nav2_params')
    use_25d_avoidance = LaunchConfiguration('use_25d_avoidance')

    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'mobile_manipulator.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    world_file = os.path.join(pkg_my_robot, 'worlds', 'door_world.sdf')
    amcl_config = os.path.join(pkg_my_robot, 'config', 'amcl.yaml')
    nav2_config = os.path.join(pkg_my_robot, 'config', 'nav2_navigation.yaml')
    route_config = os.path.join(pkg_my_robot, 'config', 'nav2_route.yaml')
    route_graph = os.path.join(pkg_my_robot, 'config', 'route_graph.geojson')
    routes_file = os.path.join(pkg_my_robot, 'config', 'routes.yaml')
    safety_shell_config = os.path.join(pkg_my_robot, 'config', 'safety_shell.yaml')
    rviz_config = os.path.join(pkg_my_robot, 'rviz', 'view_robot.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': f'-r {world_file}'}.items(),
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=NAMESPACE,
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-topic', f'/{NAMESPACE}/robot_description',
            '-name', 'mobile_manipulator',
            '-x', '-2.0',
            '-y', '0',
            '-z', '0.0',
        ],
        output='screen',
    )

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

    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        arguments=['/world/door_passage_test/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[('/world/door_passage_test/pose/info', '/gazebo_pose_info')],
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

    gazebo_pose_odom = Node(
        package='my_robot_py_sim',
        executable='gazebo_pose_odom',
        name='gazebo_pose_odom',
        parameters=[{
            'use_sim_time': True,
            'pose_topic': '/gazebo_pose_info',
            'model_name': 'mobile_manipulator',
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_tf': True,
        }],
        output='screen',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file,
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config],
    )

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': nav2_params,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    navigation_2d = TimerAction(
        period=10.0,
        condition=UnlessCondition(use_25d_avoidance),
        actions=[navigation],
    )

    navigation_25d = TimerAction(
        period=15.0,
        condition=IfCondition(use_25d_avoidance),
        actions=[navigation],
    )

    route_server = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        condition=IfCondition(use_route),
        output='screen',
        parameters=[
            route_config,
            {
                'use_sim_time': True,
                'graph_filepath': route_graph,
            },
        ],
    )

    route_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_route',
        condition=IfCondition(use_route),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['route_server'],
        }],
    )

    route_manager = Node(
        package='my_robot_py_sim',
        executable='route_manager',
        name='route_manager',
        parameters=[{
            'use_sim_time': True,
            'route_file': routes_file,
            'marker_topic': '/route_markers',
            'publish_rate': 1.0,
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
            'length': 1.86,
            'width': 1.86,
            'z_offset': 0.025,
        }],
        output='screen',
    )

    static_25d_forbidden_grid = Node(
        package='my_robot_py_sim',
        executable='static_25d_forbidden_grid',
        name='static_25d_forbidden_grid',
        condition=IfCondition(use_25d_avoidance),
        parameters=[{
            'use_sim_time': True,
            'world_file': world_file,
            'urdf_file': urdf_file,
            'config_file': safety_shell_config,
            'grid_topic': '/static_25d_forbidden_grid',
            'frame_id': 'map',
            'base_frame': 'base_footprint',
            'resolution': 0.10,
            'origin_x': -7.0,
            'origin_y': -5.0,
            'size_x': 14.0,
            'size_y': 10.0,
            'yaw_samples': 16,
            'planning_padding': 0.0,
        }],
        output='screen',
    )

    safety_forbidden_grid = Node(
        package='my_robot_py_sim',
        executable='safety_forbidden_grid',
        name='safety_forbidden_grid',
        condition=IfCondition(use_25d_avoidance),
        parameters=[{
            'use_sim_time': True,
            'cloud_topic': '/lidar/points',
            'grid_topic': '/safety_forbidden_grid',
            'config_file': safety_shell_config,
            'fixed_frame': 'odom',
            'base_frame': 'base_footprint',
            'resolution': 0.05,
            'grid_size': 12.0,
            'forward_offset': 1.0,
            'max_points': 5000,
            'planning_padding': 0.0,
            'self_filter_padding': 0.03,
            'obstacle_keep_time': 0.8,
            'min_obstacle_z': 0.25,
        }],
        output='screen',
    )

    grid_to_pointcloud = Node(
        package='my_robot_py_sim',
        executable='grid_to_pointcloud',
        name='grid_to_pointcloud',
        condition=IfCondition(use_25d_avoidance),
        parameters=[{
            'use_sim_time': True,
            'grid_topic': '/safety_forbidden_grid',
            'cloud_topic': '/safety_forbidden_cloud',
            'occupied_threshold': 50,
            'point_z': 0.5,
            'publish_rate': 10.0,
        }],
        output='screen',
    )

    planning_map_fusion = Node(
        package='my_robot_py_sim',
        executable='planning_map_fusion',
        name='planning_map_fusion',
        condition=IfCondition(use_25d_avoidance),
        parameters=[{
            'use_sim_time': True,
            'map_topic': '/map',
            'static_safety_grid_topic': '',
            'safety_grid_topic': '/safety_forbidden_grid',
            'planning_map_topic': '/planning_map',
            'occupied_threshold': 50,
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_route', default_value='false'),
        DeclareLaunchArgument('nav2_params', default_value=nav2_config),
        DeclareLaunchArgument('use_25d_avoidance', default_value='false'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        gazebo,
        clock_bridge,
        cmd_vel_bridge,
        pose_bridge,
        joint_state_bridge,
        lidar_points_bridge,
        lidar_frame_tf,
        pointcloud_to_scan,
        gazebo_pose_odom,
        robot_state_pub,
        safety_shell,
        footprint_marker,
        route_manager,
        safety_forbidden_grid,
        grid_to_pointcloud,
        planning_map_fusion,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=7.0, actions=[
            map_server,
            amcl,
            localization_lifecycle,
        ]),
        navigation_2d,
        navigation_25d,
        TimerAction(period=11.0, actions=[
            route_server,
            route_lifecycle,
        ]),
        TimerAction(period=12.0, actions=[rviz]),
    ])
