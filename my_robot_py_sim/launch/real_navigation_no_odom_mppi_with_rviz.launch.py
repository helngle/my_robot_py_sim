import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'mobile_manipulator'


def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot_py_sim')
    pkg_vmr_base = get_package_share_directory('vmr_base_bridge')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'mobile_manipulator.urdf')
    rviz_config = os.path.join(pkg_my_robot, 'rviz', 'view_robot.rviz')
    nav2_config = os.path.join(pkg_my_robot, 'config', 'real_nav2_no_odom_mppi.yaml')
    routes_file = os.path.join(pkg_my_robot, 'config', 'routes.yaml')
    default_map = os.path.expanduser('~/ros2_ws/maps/Test052601/Test052601.yaml')

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
    nav2_delay = LaunchConfiguration('nav2_delay')
    map_file = LaunchConfiguration('map')
    pose_topic = LaunchConfiguration('pose_topic')
    laser_cloud_topic = LaunchConfiguration('laser_cloud_topic')
    stamped_laser_cloud_topic = LaunchConfiguration('stamped_laser_cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')

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

    sdk_pose_to_map_tf = Node(
        package='my_robot_py_sim',
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
        package='my_robot_py_sim',
        executable='pointcloud_restamper',
        name='real_laser_pointcloud_restamper',
        condition=IfCondition(use_scan_conversion),
        parameters=[{
            'use_sim_time': False,
            'input_topic': laser_cloud_topic,
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
            'min_height': -0.15,
            'max_height': 0.35,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872665,
            'scan_time': 0.1,
            'range_min': 0.05,
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
        package='my_robot_py_sim',
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
        package='my_robot_py_sim',
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
        DeclareLaunchArgument('nav2_delay', default_value='8.0'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('pose_topic', default_value='/vmr_base_bridge/pose'),
        DeclareLaunchArgument('laser_cloud_topic', default_value='/vmr_base_bridge/laser/points'),
        DeclareLaunchArgument(
            'stamped_laser_cloud_topic',
            default_value='/vmr_base_bridge/laser/points_stamped',
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
