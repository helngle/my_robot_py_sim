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
    pkg_my_robot = get_package_share_directory('my_robot_py_sim')
    pkg_vmr_base = get_package_share_directory('vmr_base_bridge')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'mobile_manipulator.urdf')
    rviz_config = os.path.join(pkg_my_robot, 'rviz', 'view_robot.rviz')
    amcl_config = os.path.join(pkg_my_robot, 'config', 'real_amcl.yaml')
    nav2_config = os.path.join(pkg_my_robot, 'config', 'real_nav2_navigation.yaml')
    default_map = os.path.expanduser('~/ros2_ws/Test052601/Test052601.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    use_rviz = LaunchConfiguration('use_rviz')
    use_map = LaunchConfiguration('use_map')
    use_base_driver = LaunchConfiguration('use_base_driver')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    publish_static_map_to_odom = LaunchConfiguration('publish_static_map_to_odom')
    map_file = LaunchConfiguration('map')
    cmd_vel_enabled = LaunchConfiguration('cmd_vel_enabled')
    cmd_vel_max_linear_x = LaunchConfiguration('cmd_vel_max_linear_x')
    cmd_vel_max_angular_z = LaunchConfiguration('cmd_vel_max_angular_z')
    cmd_vel_speed_factor = LaunchConfiguration('cmd_vel_speed_factor')
    use_scan_conversion = LaunchConfiguration('use_scan_conversion')
    use_amcl = LaunchConfiguration('use_amcl')
    use_sdk_pose_tf = LaunchConfiguration('use_sdk_pose_tf')
    use_nav2 = LaunchConfiguration('use_nav2')
    nav2_delay = LaunchConfiguration('nav2_delay')
    nav2_params = LaunchConfiguration('nav2_params')
    use_laser_static_tf = LaunchConfiguration('use_laser_static_tf')
    laser_cloud_topic = LaunchConfiguration('laser_cloud_topic')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_parent_frame = LaunchConfiguration('laser_parent_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    scan_target_frame = LaunchConfiguration('scan_target_frame')

    base_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vmr_base, 'launch', 'vmr_base_bridge.launch.py')
        ),
        condition=IfCondition(use_base_driver),
        launch_arguments={
            'cmd_vel_enabled': cmd_vel_enabled,
            'cmd_vel_max_linear_x': cmd_vel_max_linear_x,
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

    odom_to_tf = Node(
        package='my_robot_py_sim',
        executable='odom_to_tf',
        name='real_odom_to_tf',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_topic': '/vmr_base_bridge/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'stamp_with_current_time': True,
        }],
    )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        condition=IfCondition(PythonExpression([
            "'", publish_static_map_to_odom, "' == 'true' and '",
            use_amcl, "' == 'false' and '",
            use_sdk_pose_tf, "' == 'false'",
        ])),
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'map',
            'odom',
        ],
        output='screen',
    )

    sdk_pose_to_map_odom_tf = Node(
        package='my_robot_py_sim',
        executable='sdk_pose_to_map_odom_tf',
        name='sdk_pose_to_map_odom_tf',
        condition=IfCondition(use_sdk_pose_tf),
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'pose_topic': '/vmr_base_bridge/pose',
            'odom_topic': '/vmr_base_bridge/odom',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_rate': 20.0,
            'max_pose_age': 1.0,
            'max_odom_age': 1.0,
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
        name='lifecycle_manager_real_map',
        condition=IfCondition(PythonExpression([
            "'", use_map, "' == 'true' and '", use_amcl, "' == 'false'",
        ])),
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        condition=IfCondition(use_amcl),
        output='screen',
        parameters=[amcl_config],
    )

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_real_localization',
        condition=IfCondition(use_amcl),
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        condition=IfCondition(use_laser_static_tf),
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            laser_parent_frame,
            laser_frame,
        ],
        output='screen',
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='real_pointcloud_to_laserscan',
        condition=IfCondition(use_scan_conversion),
        parameters=[{
            'use_sim_time': False,
            'target_frame': scan_target_frame,
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
            ('cloud_in', laser_cloud_topic),
            ('scan', scan_topic),
        ],
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
            'params_file': nav2_params,
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
            ('/lidar/points', '/vmr_base_bridge/laser/points'),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_map', default_value='true'),
        DeclareLaunchArgument('use_base_driver', default_value='true'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='true'),
        DeclareLaunchArgument('publish_static_map_to_odom', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('cmd_vel_enabled', default_value='false'),
        DeclareLaunchArgument('cmd_vel_max_linear_x', default_value='0.1'),
        DeclareLaunchArgument('cmd_vel_max_angular_z', default_value='0.3'),
        DeclareLaunchArgument('cmd_vel_speed_factor', default_value='0.2'),
        DeclareLaunchArgument('use_scan_conversion', default_value='true'),
        DeclareLaunchArgument('use_amcl', default_value='false'),
        DeclareLaunchArgument('use_sdk_pose_tf', default_value='false'),
        DeclareLaunchArgument('use_nav2', default_value='false'),
        DeclareLaunchArgument('nav2_delay', default_value='30.0'),
        DeclareLaunchArgument('nav2_params', default_value=nav2_config),
        DeclareLaunchArgument('use_laser_static_tf', default_value='false'),
        DeclareLaunchArgument('laser_cloud_topic', default_value='/vmr_base_bridge/laser/points'),
        DeclareLaunchArgument('laser_frame', default_value='laser_1'),
        DeclareLaunchArgument('laser_parent_frame', default_value='base_footprint'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_target_frame', default_value='base_footprint'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        base_driver,
        joint_state_publisher,
        robot_state_publisher,
        odom_to_tf,
        static_map_to_odom,
        sdk_pose_to_map_odom_tf,
        laser_static_tf,
        pointcloud_to_scan,
        TimerAction(period=2.0, actions=[
            map_server,
            map_lifecycle,
            amcl,
            localization_lifecycle,
        ]),
        TimerAction(period=nav2_delay, actions=[navigation]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
