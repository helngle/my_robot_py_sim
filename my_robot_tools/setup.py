from setuptools import find_packages, setup

package_name = 'my_robot_tools'

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
    description='Route, marker, and operator helper tools for the robot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'footprint_marker = my_robot_tools.footprint_marker:main',
            'route_cli = my_robot_tools.route_cli:main',
            'route_commander = my_robot_tools.route_commander:main',
            'route_insert_editor = my_robot_tools.route_insert_editor:main',
            'route_manager = my_robot_tools.route_manager:main',
            'route_recorder = my_robot_tools.route_recorder:main',
            'safety_shell_marker = my_robot_tools.safety_shell_marker:main',
        ],
    },
)
