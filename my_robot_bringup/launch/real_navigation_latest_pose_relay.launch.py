import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')

    input_pose_topic = LaunchConfiguration('relay_input_pose_topic')
    output_pose_topic = LaunchConfiguration('relay_output_pose_topic')
    publish_rate = LaunchConfiguration('relay_publish_rate')
    stats_period = LaunchConfiguration('relay_stats_period')
    drop_regressed = LaunchConfiguration('relay_drop_regressed')

    latest_pose_relay = Node(
        package='my_robot_localization',
        executable='latest_sdk_sample_relay',
        name='latest_sdk_sample_relay',
        output='screen',
        parameters=[{
            'input_pose_topic': input_pose_topic,
            'output_pose_topic': output_pose_topic,
            'publish_rate': ParameterValue(publish_rate, value_type=float),
            'stats_period': ParameterValue(stats_period, value_type=float),
            'drop_regressed': ParameterValue(
                drop_regressed, value_type=bool
            ),
            'relay_odom': False,
        }],
    )

    stable_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                'launch',
                'real_navigation_mppi.launch.py',
            )
        ),
        launch_arguments={
            'pose_topic': output_pose_topic,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'relay_input_pose_topic',
            default_value='/vmr_base_bridge/pose',
            description='Raw VMR pose input for the experimental relay.',
        ),
        DeclareLaunchArgument(
            'relay_output_pose_topic',
            default_value='/vmr_base_bridge/latest_pose',
            description=(
                'Latest-only pose consumed by the stable localization node.'
            ),
        ),
        DeclareLaunchArgument(
            'relay_publish_rate',
            default_value='30.0',
            description='Maximum latest-pose forwarding rate in Hz.',
        ),
        DeclareLaunchArgument(
            'relay_stats_period',
            default_value='5.0',
            description='Seconds between relay statistics logs.',
        ),
        DeclareLaunchArgument(
            'relay_drop_regressed',
            default_value='true',
            description=(
                'Drop samples whose SDK header timestamp moves backward.'
            ),
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        latest_pose_relay,
        stable_navigation,
    ])
