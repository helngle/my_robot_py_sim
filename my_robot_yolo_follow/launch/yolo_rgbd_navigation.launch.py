import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')
    maps_share = get_package_share_directory('my_robot_maps')
    perception_share = get_package_share_directory('my_robot_perception')

    real_navigation_launch = os.path.join(
        bringup_share,
        'launch',
        'real_navigation_mppi.launch.py',
    )
    default_map = os.path.join(
        maps_share,
        'maps',
        'Test052601',
        'Test052601.yaml',
    )
    default_tracker = os.path.join(
        perception_share,
        'config',
        'bytetrack_person.yaml',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_navigation_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'lidar_source': LaunchConfiguration('lidar_source'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_base_driver': LaunchConfiguration('use_base_driver'),
            'use_nav2': LaunchConfiguration('use_nav2'),
            'use_scan_conversion': LaunchConfiguration('use_scan_conversion'),
            'use_orbbec_camera': 'true',
            'use_orbbec_pointcloud': 'false',
            'use_rgbd_goal': 'true',
            'rgbd_goal_auto_send': LaunchConfiguration('rgbd_goal_auto_send'),
            'rgbd_target_detector': 'yolo',
            'rgbd_target_class': LaunchConfiguration('target_class'),
            'rgbd_yolo_model': LaunchConfiguration('yolo_model'),
            'rgbd_yolo_device': LaunchConfiguration('yolo_device'),
            'rgbd_yolo_confidence': LaunchConfiguration('yolo_confidence'),
            'rgbd_process_rate_hz': LaunchConfiguration('process_rate_hz'),
            'rgbd_use_tracking': LaunchConfiguration('use_tracking'),
            'rgbd_yolo_tracker': LaunchConfiguration('yolo_tracker'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('lidar_source', default_value='livox'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_base_driver', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_scan_conversion', default_value='true'),
        DeclareLaunchArgument('rgbd_goal_auto_send', default_value='false'),
        DeclareLaunchArgument('target_class', default_value='person'),
        DeclareLaunchArgument(
            'yolo_model',
            default_value='/home/jensen/ros2_ws/yolo11n.pt',
        ),
        DeclareLaunchArgument('yolo_device', default_value='cuda:0'),
        DeclareLaunchArgument('yolo_confidence', default_value='0.10'),
        DeclareLaunchArgument('process_rate_hz', default_value='8.0'),
        DeclareLaunchArgument('use_tracking', default_value='true'),
        DeclareLaunchArgument('yolo_tracker', default_value=default_tracker),
        navigation,
    ])
