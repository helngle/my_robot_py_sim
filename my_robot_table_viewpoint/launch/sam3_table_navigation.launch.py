import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
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
    sam3_launch = os.path.join(
        table_share,
        'launch',
        'sam3_table_bbox.launch.py',
    )
    table_viewpoint_launch = os.path.join(
        table_share,
        'launch',
        'table_viewpoint.launch.py',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_navigation_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'lidar_source': LaunchConfiguration('lidar_source'),
            'use_base_driver': LaunchConfiguration('use_base_driver'),
            'use_nav2': LaunchConfiguration('use_nav2'),
            'use_scan_conversion': LaunchConfiguration('use_scan_conversion'),
            'use_orbbec_camera': 'true',
            'use_orbbec_pointcloud': 'false',
            'use_rgbd_goal': 'false',
            'rgbd_goal_auto_send': 'false',
            'use_rviz': 'false',
        }.items(),
    )
    sam3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sam3_launch),
        condition=IfCondition(LaunchConfiguration('use_sam3')),
        launch_arguments={
            'sam3_model': LaunchConfiguration('sam3_model'),
            'sam3_prompt': LaunchConfiguration('sam3_prompt'),
            'sam3_device': LaunchConfiguration('sam3_device'),
            'sam3_imgsz': LaunchConfiguration('sam3_imgsz'),
            'sam3_confidence': LaunchConfiguration('sam3_confidence'),
            'bbox_topic': LaunchConfiguration('bbox_topic'),
        }.items(),
    )
    table_viewpoint = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(table_viewpoint_launch),
        launch_arguments={
            'use_table_rviz': LaunchConfiguration('use_table_rviz'),
            'input_mode': 'topic',
            'bbox_topic': LaunchConfiguration('bbox_topic'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('lidar_source', default_value='livox'),
        DeclareLaunchArgument('use_base_driver', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_scan_conversion', default_value='true'),
        DeclareLaunchArgument('use_table_rviz', default_value='true'),
        DeclareLaunchArgument('use_sam3', default_value='true'),
        DeclareLaunchArgument(
            'sam3_model',
            default_value='/home/jensen/ros2_ws/sam3.pt',
        ),
        DeclareLaunchArgument('sam3_prompt', default_value='office desk'),
        DeclareLaunchArgument('sam3_device', default_value='cuda'),
        DeclareLaunchArgument('sam3_imgsz', default_value='644'),
        DeclareLaunchArgument('sam3_confidence', default_value='0.25'),
        DeclareLaunchArgument('bbox_topic', default_value='/target_bbox_3d'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        navigation,
        sam3,
        table_viewpoint,
    ])
