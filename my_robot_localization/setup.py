from setuptools import find_packages, setup

package_name = 'my_robot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jensen',
    maintainer_email='jensen@todo.todo',
    description='Pose, odometry, and TF helper nodes for robot localization.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gazebo_pose_odom = my_robot_localization.gazebo_pose_odom:main',
            'odom_to_tf = my_robot_localization.odom_to_tf:main',
            'sdk_pose_to_map_odom_tf = my_robot_localization.sdk_pose_to_map_odom_tf:main',
            'sdk_pose_to_map_tf = my_robot_localization.sdk_pose_to_map_tf:main',
        ],
    },
)
