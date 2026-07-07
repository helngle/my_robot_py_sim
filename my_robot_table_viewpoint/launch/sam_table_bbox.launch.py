import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'my_robot_table_viewpoint'
    )
    default_params = os.path.join(
        package_share,
        'config',
        'table_viewpoint.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
        ),
        DeclareLaunchArgument(
            'sam_checkpoint',
            default_value='',
            description='Path to SAM checkpoint, for example sam_vit_b.pth.',
        ),
        DeclareLaunchArgument(
            'sam_device',
            default_value='cuda',
            description='SAM device, usually cuda or cpu.',
        ),
        DeclareLaunchArgument(
            'bbox_topic',
            default_value='/target_bbox_3d',
        ),
        Node(
            package='my_robot_table_viewpoint',
            executable='sam_table_bbox_node',
            name='sam_table_bbox_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'sam_checkpoint': LaunchConfiguration('sam_checkpoint'),
                    'sam_device': LaunchConfiguration('sam_device'),
                    'bbox_topic': LaunchConfiguration('bbox_topic'),
                },
            ],
        ),
    ])
