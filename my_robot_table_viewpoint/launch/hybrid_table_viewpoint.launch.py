import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('my_robot_table_viewpoint')
    navigation_share = get_package_share_directory('my_robot_navigation')
    maps_share = get_package_share_directory('my_robot_maps')

    default_params = os.path.join(
        package_share,
        'config',
        'hybrid_table_viewpoint.yaml',
    )
    default_database = os.path.join(
        maps_share,
        'maps',
        'Test052601_table_viewpoint',
        'tables.yaml',
    )
    default_rviz = os.path.join(
        package_share,
        'rviz',
        'table_viewpoint.rviz',
    )
    short_bt = os.path.join(
        navigation_share,
        'behavior_trees',
        'navigate_short_omni_replanning_if_path_invalid.xml',
    )
    long_bt = os.path.join(
        navigation_share,
        'behavior_trees',
        'navigate_long_forward_replanning_if_path_invalid.xml',
    )
    fine_bt = os.path.join(
        navigation_share,
        'behavior_trees',
        'navigate_fine_omni_replanning_if_path_invalid.xml',
    )
    sam3_launch = os.path.join(
        package_share,
        'launch',
        'sam3_table_bbox.launch.py',
    )

    sam3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sam3_launch),
        condition=IfCondition(LaunchConfiguration('use_sam3')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'sam3_model': LaunchConfiguration('sam3_model'),
            'sam3_prompt': LaunchConfiguration('sam3_prompt'),
            'sam3_device': LaunchConfiguration('sam3_device'),
            'sam3_imgsz': LaunchConfiguration('sam3_imgsz'),
            'bbox_topic': LaunchConfiguration('bbox_topic'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('database_file', default_value=default_database),
        DeclareLaunchArgument('input_mode', default_value='topic'),
        DeclareLaunchArgument('bbox_topic', default_value='/target_bbox_3d'),
        DeclareLaunchArgument('use_table_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_orchestrator',
            default_value='false',
            description='Enable only when Nav2 runs on this same machine.',
        ),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('use_sam3', default_value='true'),
        DeclareLaunchArgument(
            'sam3_model',
            default_value='/home/jensen/ros2_ws/sam3.pt',
        ),
        DeclareLaunchArgument('sam3_prompt', default_value='office desk'),
        DeclareLaunchArgument('sam3_device', default_value='cuda'),
        DeclareLaunchArgument('sam3_imgsz', default_value='644'),
        DeclareLaunchArgument('long_bt_xml', default_value=long_bt),
        DeclareLaunchArgument('short_bt_xml', default_value=short_bt),
        DeclareLaunchArgument('fine_bt_xml', default_value=fine_bt),
        Node(
            package='my_robot_table_viewpoint',
            executable='table_viewpoint_planner',
            name='table_viewpoint_planner',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'database_file': LaunchConfiguration('database_file'),
                    'input_mode': LaunchConfiguration('input_mode'),
                    'bbox_topic': LaunchConfiguration('bbox_topic'),
                },
            ],
        ),
        Node(
            package='my_robot_table_viewpoint',
            executable='hybrid_viewpoint_orchestrator',
            name='hybrid_viewpoint_orchestrator',
            condition=IfCondition(LaunchConfiguration('use_orchestrator')),
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'bbox_topic': LaunchConfiguration('bbox_topic'),
                    'long_bt_xml': LaunchConfiguration('long_bt_xml'),
                    'short_bt_xml': LaunchConfiguration('short_bt_xml'),
                    'fine_bt_xml': LaunchConfiguration('fine_bt_xml'),
                },
            ],
        ),
        sam3,
        Node(
            package='rviz2',
            executable='rviz2',
            name='hybrid_table_viewpoint_rviz',
            condition=IfCondition(LaunchConfiguration('use_table_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
        ),
    ])
