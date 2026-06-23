import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jensen',
    maintainer_email='jensen@todo.todo',
    description='Point cloud, occupancy grid, and planning-map helper nodes.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'full_body_clearance_checker = my_robot_perception.full_body_clearance_checker:main',
            'grid_to_pointcloud = my_robot_perception.grid_to_pointcloud:main',
            'planning_map_fusion = my_robot_perception.planning_map_fusion:main',
            'pointcloud_restamper = my_robot_perception.pointcloud_restamper:main',
            (
                'rgbd_goal_finder = '
                'my_robot_perception.rgbd_goal_finder:main'
            ),
            'safety_forbidden_grid = my_robot_perception.safety_forbidden_grid:main',
            'static_25d_forbidden_grid = my_robot_perception.static_25d_forbidden_grid:main',
        ],
    },
)
