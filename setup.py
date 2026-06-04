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
            'odom_to_tf = my_robot_py_sim.odom_to_tf:main',
            'safety_shell_marker = my_robot_py_sim.safety_shell_marker:main',
        ],
    },
)
