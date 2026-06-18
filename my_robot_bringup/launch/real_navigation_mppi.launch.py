import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

NAMESPACE = 'mobile_manipulator'


def generate_launch_description():
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_navigation = get_package_share_directory('my_robot_navigation')
    pkg_maps = get_package_share_directory('my_robot_maps')
    pkg_vmr_base = get_package_share_directory('vmr_base_bridge')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_livox_driver = get_package_share_directory('livox_ros_driver2')
    pkg_orbbec_camera = get_package_share_directory('orbbec_camera')

    urdf_file = os.path.join(pkg_description, 'urdf', 'mobile_manipulator.urdf')
    rviz_config = os.path.join(pkg_description, 'rviz', 'view_robot.rviz')
    nav2_config = os.path.join(pkg_navigation, 'config', 'real_nav2_no_odom_mppi.yaml')
    routes_file = os.path.join(pkg_navigation, 'config', 'routes.yaml')
    livox_config = os.path.join(pkg_livox_driver, 'config', 'MID360s_config.json')
    default_map = os.path.join(pkg_maps, 'maps', 'Test052601', 'Test052601.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    use_rviz = LaunchConfiguration('use_rviz')
    use_map = LaunchConfiguration('use_map')
    use_base_driver = LaunchConfiguration('use_base_driver')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    cmd_vel_enabled = LaunchConfiguration('cmd_vel_enabled')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_max_linear_x = LaunchConfiguration('cmd_vel_max_linear_x')
    cmd_vel_max_linear_y = LaunchConfiguration('cmd_vel_max_linear_y')
    cmd_vel_max_angular_z = LaunchConfiguration('cmd_vel_max_angular_z')
    cmd_vel_speed_factor = LaunchConfiguration('cmd_vel_speed_factor')
    use_scan_conversion = LaunchConfiguration('use_scan_conversion')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_orbbec_camera = LaunchConfiguration('use_orbbec_camera')
    orbbec_color_width = LaunchConfiguration('orbbec_color_width')
    orbbec_color_height = LaunchConfiguration('orbbec_color_height')
    orbbec_color_fps = LaunchConfiguration('orbbec_color_fps')
    orbbec_depth_width = LaunchConfiguration('orbbec_depth_width')
    orbbec_depth_height = LaunchConfiguration('orbbec_depth_height')
    orbbec_depth_fps = LaunchConfiguration('orbbec_depth_fps')
    nav2_delay = LaunchConfiguration('nav2_delay')
    map_file = LaunchConfiguration('map')
    lidar_source = LaunchConfiguration('lidar_source')
    pose_topic = LaunchConfiguration('pose_topic')
    laser_cloud_topic = LaunchConfiguration('laser_cloud_topic')
    stamped_laser_cloud_topic = LaunchConfiguration('stamped_laser_cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    use_livox_lidar = PythonExpression(["'", lidar_source, "' == 'livox'"])
    selected_laser_cloud_topic = PythonExpression([
        "'/livox/lidar' if '",
        lidar_source,
        "' == 'livox' else '",
        laser_cloud_topic,
        "'",
    ])

    base_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vmr_base, 'launch', 'vmr_base_bridge.launch.py')
        ),
        condition=IfCondition(use_base_driver),
        launch_arguments={
            'cmd_vel_enabled': cmd_vel_enabled,
            'cmd_vel_topic': cmd_vel_topic,
            'cmd_vel_max_linear_x': cmd_vel_max_linear_x,
            'cmd_vel_max_linear_y': cmd_vel_max_linear_y,
            'cmd_vel_max_angular_z': cmd_vel_max_angular_z,
            'cmd_vel_speed_factor': cmd_vel_speed_factor,
        }.items(),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=NAMESPACE,
        condition=IfCondition(use_joint_state_publisher),
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=NAMESPACE,
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
        }],
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        condition=IfCondition(use_livox_lidar),
        output='screen',
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': livox_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    livox_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_static_tf',
        condition=IfCondition(use_livox_lidar),
        arguments=[
            '0.3', '0.0', '0.35',
            '0.0', '0.02', '0.0',
            'base_footprint', 'livox_frame',
        ],
        output='screen',
    )

    orbbec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_orbbec_camera, 'launch', 'gemini435_le.launch.py')
        ),
        condition=IfCondition(use_orbbec_camera),
        launch_arguments={
            'camera_name': 'camera',
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'true',
            'color_width': orbbec_color_width,
            'color_height': orbbec_color_height,
            'color_fps': orbbec_color_fps,
            'depth_width': orbbec_depth_width,
            'depth_height': orbbec_depth_height,
            'depth_fps': orbbec_depth_fps,
        }.items(),
    )

    orbbec_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='orbbec_static_tf',
        condition=IfCondition(use_orbbec_camera),
        arguments=[
            '0.25', '-0.10', '1.00',
            '0.0', '0.0', '0.0',
            'base_footprint', 'camera_link',
        ],
        output='screen',
    )

    sdk_pose_to_map_tf = Node(
        package='my_robot_localization',
        executable='sdk_pose_to_map_tf',
        name='sdk_pose_to_map_tf',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'pose_topic': pose_topic,
            'estimated_pose_topic': '/estimated_pose',
            'estimated_odom_topic': '/estimated_odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'publish_rate': 30.0,
            'max_pose_age': 1.0,
            'stamp_with_current_time': True,
        }],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        condition=IfCondition(use_map),
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_file,
        }],
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_real_no_odom_map',
        condition=IfCondition(use_map),
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    pointcloud_restamper = Node(
        package='my_robot_perception',
        executable='pointcloud_restamper',
        name='real_laser_pointcloud_restamper',
        condition=IfCondition(use_scan_conversion),
        parameters=[{
            'use_sim_time': False,
            'input_topic': selected_laser_cloud_topic,
            'output_topic': stamped_laser_cloud_topic,
        }],
        output='screen',
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='real_no_odom_mppi_pointcloud_to_laserscan',
        condition=IfCondition(use_scan_conversion),
        parameters=[{
            'use_sim_time': False,
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.5,
            'min_height': 0.10,
            'max_height': 1.00,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872665,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', stamped_laser_cloud_topic),
            ('scan', scan_topic),
        ],
        output='screen',
    )

    route_manager = Node(
        package='my_robot_tools',
        executable='route_manager',
        name='route_manager',
        parameters=[{
            'use_sim_time': False,
            'route_file': routes_file,
            'marker_topic': '/route_markers',
            'publish_rate': 1.0,
        }],
        output='screen',
    )

    footprint_marker = Node(
        package='my_robot_tools',
        executable='footprint_marker',
        name='footprint_marker',
        parameters=[{
            'use_sim_time': False,
            'topic': '/base_footprint_marker',
            'frame_id': 'base_footprint',
            'length': 0.80,
            'width': 0.70,
            'z_offset': 0.025,
        }],
        output='screen',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': nav2_config,
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/lidar/points', stamped_laser_cloud_topic),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_map', default_value='true'),
        DeclareLaunchArgument('use_base_driver', default_value='true'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='true'),
        DeclareLaunchArgument('use_scan_conversion', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument(
            'use_orbbec_camera',
            default_value='false',
            description='Start the Orbbec Gemini 435Le camera and publish its base TF.',
        ),
        DeclareLaunchArgument('orbbec_color_width', default_value='640'),
        DeclareLaunchArgument('orbbec_color_height', default_value='400'),
        DeclareLaunchArgument('orbbec_color_fps', default_value='10'),
        DeclareLaunchArgument('orbbec_depth_width', default_value='640'),
        DeclareLaunchArgument('orbbec_depth_height', default_value='400'),
        DeclareLaunchArgument('orbbec_depth_fps', default_value='10'),
        DeclareLaunchArgument('nav2_delay', default_value='8.0'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'lidar_source',
            default_value='vmr',
            description='Point cloud source for /scan conversion: vmr or livox.',
        ),
        DeclareLaunchArgument('pose_topic', default_value='/vmr_base_bridge/pose'),
        DeclareLaunchArgument('laser_cloud_topic', default_value='/vmr_base_bridge/laser/points'),
        DeclareLaunchArgument(
            'stamped_laser_cloud_topic',
            default_value='/selected_lidar/points_stamped',
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_enabled', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_max_linear_x', default_value='0.3'),
        DeclareLaunchArgument('cmd_vel_max_linear_y', default_value='0.3'),
        DeclareLaunchArgument('cmd_vel_max_angular_z', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_speed_factor', default_value='1.0'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        base_driver,
        joint_state_publisher,
        robot_state_publisher,
        livox_driver,
        livox_static_tf,
        orbbec_camera,
        orbbec_static_tf,
        sdk_pose_to_map_tf,
        pointcloud_restamper,
        pointcloud_to_scan,
        route_manager,
        footprint_marker,
        TimerAction(period=2.0, actions=[
            map_server,
            map_lifecycle,
        ]),
        TimerAction(period=nav2_delay, actions=[navigation]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
