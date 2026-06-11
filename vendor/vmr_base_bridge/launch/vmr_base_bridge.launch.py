from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('vmr_base_bridge')
    params_file = os.path.join(package_share, 'config', 'vmr_base_bridge.yaml')

    sdk_config_file = LaunchConfiguration('sdk_config_file')
    site_mapping_file = LaunchConfiguration('site_mapping_file')
    cmd_vel_enabled = LaunchConfiguration('cmd_vel_enabled')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_timeout = LaunchConfiguration('cmd_vel_timeout')
    cmd_vel_rate_hz = LaunchConfiguration('cmd_vel_rate_hz')
    cmd_vel_speed_factor = LaunchConfiguration('cmd_vel_speed_factor')
    cmd_vel_max_linear_x = LaunchConfiguration('cmd_vel_max_linear_x')
    cmd_vel_max_linear_y = LaunchConfiguration('cmd_vel_max_linear_y')
    cmd_vel_max_angular_z = LaunchConfiguration('cmd_vel_max_angular_z')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sdk_config_file',
            default_value=os.path.join(package_share, 'config', 'vmr_sdk.ini'),
            description='Path to the VMR SDK ini config file',
        ),
        DeclareLaunchArgument(
            'site_mapping_file',
            default_value=os.path.join(package_share, 'config', 'site_mapping.yaml'),
            description='Path to the named navigation target mapping file',
        ),
        DeclareLaunchArgument(
            'cmd_vel_enabled',
            default_value='false',
            description='Enable realtime SDK speed control from /cmd_vel',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Twist command topic for realtime SDK speed control',
        ),
        DeclareLaunchArgument(
            'cmd_vel_timeout',
            default_value='0.5',
            description='Seconds before stale cmd_vel commands become zero speed',
        ),
        DeclareLaunchArgument(
            'cmd_vel_rate_hz',
            default_value='30.0',
            description='SDK speed command publish rate; must be greater than 20 Hz',
        ),
        DeclareLaunchArgument(
            'cmd_vel_speed_factor',
            default_value='0.3',
            description='VMR SDK speed factor from 0.0 to 1.0',
        ),
        DeclareLaunchArgument(
            'cmd_vel_max_linear_x',
            default_value='0.2',
            description='Maximum absolute linear x speed in m/s',
        ),
        DeclareLaunchArgument(
            'cmd_vel_max_linear_y',
            default_value='0.0',
            description='Maximum absolute linear y speed in m/s',
        ),
        DeclareLaunchArgument(
            'cmd_vel_max_angular_z',
            default_value='0.5',
            description='Maximum absolute angular z speed in rad/s',
        ),
        Node(
            package='vmr_base_bridge',
            executable='vmr_base_bridge_node',
            name='vmr_base_bridge_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'sdk_config_file': sdk_config_file,
                    'navigation.site_mapping_file': site_mapping_file,
                    'cmd_vel.enabled': ParameterValue(cmd_vel_enabled, value_type=bool),
                    'topics.cmd_vel': cmd_vel_topic,
                    'cmd_vel.timeout': ParameterValue(cmd_vel_timeout, value_type=float),
                    'cmd_vel.rate_hz': ParameterValue(cmd_vel_rate_hz, value_type=float),
                    'cmd_vel.speed_factor': ParameterValue(cmd_vel_speed_factor, value_type=float),
                    'cmd_vel.max_linear_x': ParameterValue(cmd_vel_max_linear_x, value_type=float),
                    'cmd_vel.max_linear_y': ParameterValue(cmd_vel_max_linear_y, value_type=float),
                    'cmd_vel.max_angular_z': ParameterValue(cmd_vel_max_angular_z, value_type=float),
                },
            ],
        ),
    ])
