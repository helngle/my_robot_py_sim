import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_robot_table_viewpoint'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jensen',
    maintainer_email='jensen@todo.todo',
    description='Click-guided 3D table localization and viewpoint planning.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            (
                'table_viewpoint_planner = '
                'my_robot_table_viewpoint.table_viewpoint_planner:main'
            ),
        ],
    },
)
