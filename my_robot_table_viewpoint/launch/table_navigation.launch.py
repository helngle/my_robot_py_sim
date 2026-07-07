import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')
    maps_share = get_package_share_directory('my_robot_maps')
    table_share = get_package_share_directory('my_robot_table_viewpoint')

    default_map = os.path.join(
        maps_share,
        'maps',
        'Test052601_table_viewpoint',
        'Test052601_table_viewpoint.yaml',
    )
    real_navigation_launch = os.path.join(
        bringup_share,
        'launch',
        'real_navigation_mppi.launch.py',
    )
    table_viewpoint_launch = os.path.join(
        table_share,
        'launch',
        'table_viewpoint.launch.py',
    )

    map_file = LaunchConfiguration('map')
    lidar_source = LaunchConfiguration('lidar_source')
    use_base_driver = LaunchConfiguration('use_base_driver')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_scan_conversion = LaunchConfiguration('use_scan_conversion')
    use_orbbec_camera = LaunchConfiguration('use_orbbec_camera')
    use_rgbd_goal = LaunchConfiguration('use_rgbd_goal')
    use_table_rviz = LaunchConfiguration('use_table_rviz')
    input_mode = LaunchConfiguration('input_mode')
    bbox_topic = LaunchConfiguration('bbox_topic')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_navigation_launch),
        launch_arguments={
            'map': map_file,
            'lidar_source': lidar_source,
            'use_base_driver': use_base_driver,
            'use_nav2': use_nav2,
            'use_scan_conversion': use_scan_conversion,
            'use_orbbec_camera': use_orbbec_camera,
            'use_orbbec_pointcloud': 'false',
            'use_rgbd_goal': use_rgbd_goal,
            'rgbd_goal_auto_send': 'false',
            'rgbd_enable_target_localization': 'false',
            'use_rviz': 'false',
        }.items(),
    )
    table_viewpoint = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(table_viewpoint_launch),
        launch_arguments={
            'use_table_rviz': use_table_rviz,
            'input_mode': input_mode,
            'bbox_topic': bbox_topic,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('lidar_source', default_value='livox'),
        DeclareLaunchArgument('use_base_driver', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_scan_conversion', default_value='true'),
        DeclareLaunchArgument('use_orbbec_camera', default_value='true'),
        DeclareLaunchArgument('use_rgbd_goal', default_value='true'),
        DeclareLaunchArgument('use_table_rviz', default_value='true'),
        DeclareLaunchArgument('input_mode', default_value='topic'),
        DeclareLaunchArgument(
            'bbox_topic',
            default_value='/target_bbox_3d',
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        navigation,
        table_viewpoint,
    ])
