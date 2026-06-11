import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'mobile_manipulator'


def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot_py_sim')
    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'mobile_manipulator.urdf')
    rviz_config = os.path.join(pkg_my_robot, 'rviz', 'view_robot.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    use_rviz = LaunchConfiguration('use_rviz')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    use_base_driver = LaunchConfiguration('use_base_driver')
    base_driver_package = LaunchConfiguration('base_driver_package')
    base_driver_executable = LaunchConfiguration('base_driver_executable')
    base_driver_name = LaunchConfiguration('base_driver_name')

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

    base_driver = Node(
        package=base_driver_package,
        executable=base_driver_executable,
        name=base_driver_name,
        condition=IfCondition(use_base_driver),
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='true'),
        DeclareLaunchArgument('use_base_driver', default_value='false'),
        DeclareLaunchArgument('base_driver_package', default_value=''),
        DeclareLaunchArgument('base_driver_executable', default_value=''),
        DeclareLaunchArgument('base_driver_name', default_value='base_driver'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        joint_state_publisher,
        robot_state_publisher,
        base_driver,
        rviz,
    ])
