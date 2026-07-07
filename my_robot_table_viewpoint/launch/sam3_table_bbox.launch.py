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
            'sam3_model',
            default_value='',
            description='Path to SAM3 .pt model, for example sam3.pt.',
        ),
        DeclareLaunchArgument(
            'sam3_prompt',
            default_value='office desk',
            description='Text prompt used by SAM3 concept segmentation.',
        ),
        DeclareLaunchArgument(
            'sam3_device',
            default_value='cuda',
            description='SAM3 device, usually cuda or cpu.',
        ),
        DeclareLaunchArgument(
            'sam3_imgsz',
            default_value='644',
            description='SAM3 inference size. Lower this when GPU memory is limited.',
        ),
        DeclareLaunchArgument(
            'sam3_confidence',
            default_value='0.25',
            description='SAM3 confidence threshold.',
        ),
        DeclareLaunchArgument(
            'bbox_topic',
            default_value='/target_bbox_3d',
        ),
        Node(
            package='my_robot_table_viewpoint',
            executable='sam3_table_bbox_node',
            name='sam3_table_bbox_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'sam3_model': LaunchConfiguration('sam3_model'),
                    'sam3_prompt': LaunchConfiguration('sam3_prompt'),
                    'sam3_device': LaunchConfiguration('sam3_device'),
                    'sam3_imgsz': LaunchConfiguration('sam3_imgsz'),
                    'sam3_confidence': LaunchConfiguration('sam3_confidence'),
                    'bbox_topic': LaunchConfiguration('bbox_topic'),
                },
            ],
        ),
    ])
