# my_robot_py_sim

ROS 2 simulation package for a mobile manipulator in Gazebo and RViz2.

## Features

- Mobile manipulator URDF with base, wheels, torso, head, and fixed arms
- Gazebo world with door frame, boundary walls, and mixed obstacles
- Keyboard velocity control through `/cmd_vel`
- Odometry TF bridge from `odom` to `base_footprint`
- RViz2 visualization with robot model, TF, grid, and safety shell

## Run

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_robot_py_sim sim_with_rviz.launch.py
```

Keyboard control:

```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=1
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
