from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pose_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Pose estimator and interpolator node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_estimator_node = pose_estimator.pose_estimator_node:main',
        ],
    },
)
