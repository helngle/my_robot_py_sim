import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

NAMESPACE = 'mobile_manipulator'


def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot_py_sim')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'mobile_manipulator.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    world_file = os.path.join(pkg_my_robot, 'worlds', 'door_world.sdf')

    # Start Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': f'-r {world_file}'}.items(),
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=NAMESPACE,
        output='screen',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True}],
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_robot',
        arguments=['-topic', f'/{NAMESPACE}/robot_description',
                   '-name', 'mobile_manipulator',
                   '-x', '-2.0', '-y', '0', '-z', '0.0'],
        output='screen',
    )

    # RViz2
    rviz_config = os.path.join(pkg_my_robot, 'rviz', 'view_robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # Bridge for /clock from Gazebo
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/model/mobile_manipulator/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        remappings=[('/model/mobile_manipulator/cmd_vel', '/cmd_vel')],
        output='screen',
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/model/mobile_manipulator/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        remappings=[('/model/mobile_manipulator/odometry', '/odom')],
        output='screen',
    )

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/world/door_passage_test/model/mobile_manipulator/joint_state'
            '@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        remappings=[
            (
                '/world/door_passage_test/model/mobile_manipulator/joint_state',
                f'/{NAMESPACE}/joint_states',
            )
        ],
        output='screen',
    )

    odom_to_tf = Node(
        package='my_robot_py_sim',
        executable='odom_to_tf',
        name='odom_to_tf',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
        output='screen',
    )

    safety_shell = Node(
        package='my_robot_py_sim',
        executable='safety_shell_marker',
        name='safety_shell_marker',
        parameters=[{
            'use_sim_time': True,
            'topic': '/safety_shell_array',
            'padding': 0.06,
            'alpha': 0.22,
        }],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '23'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        gazebo,
        clock_bridge,
        cmd_vel_bridge,
        odom_bridge,
        joint_state_bridge,
        odom_to_tf,
        robot_state_pub,
        safety_shell,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[rviz]),
    ])
