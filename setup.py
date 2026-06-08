import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_robot_py_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.geojson')),
        ('share/' + package_name + '/urdf',
            ['urdf/mobile_manipulator.urdf']),
        ('share/' + package_name + '/worlds',
            ['worlds/door_world.sdf']),
        ('share/' + package_name + '/rviz',
            ['rviz/view_robot.rviz']),
        ('share/' + package_name + '/launch', [
            'launch/view_robot.launch.py',
            'launch/gazebo_world.launch.py',
            'launch/sim_with_rviz.launch.py',
            'launch/localization_with_rviz.launch.py',
            'launch/navigation_with_rviz.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jensen',
    maintainer_email='jensen@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gazebo_pose_odom = my_robot_py_sim.gazebo_pose_odom:main',
            'odom_to_tf = my_robot_py_sim.odom_to_tf:main',
            'footprint_marker = my_robot_py_sim.footprint_marker:main',
            'full_body_clearance_checker = my_robot_py_sim.full_body_clearance_checker:main',
            'planning_map_fusion = my_robot_py_sim.planning_map_fusion:main',
            'safety_forbidden_grid = my_robot_py_sim.safety_forbidden_grid:main',
            'safety_shell_marker = my_robot_py_sim.safety_shell_marker:main',
            'grid_to_pointcloud = my_robot_py_sim.grid_to_pointcloud:main',
            'route_commander = my_robot_py_sim.route_commander:main',
            'route_recorder = my_robot_py_sim.route_recorder:main',
            'route_manager = my_robot_py_sim.route_manager:main',
            'route_cli = my_robot_py_sim.route_cli:main',
        ],
    },
)
