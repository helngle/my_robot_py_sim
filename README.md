# ROS 2 Mobile Manipulator Workspace Sources

This repository contains the ROS 2 source packages used by the mobile manipulator project.

## Packages

- `my_robot_py_sim`: simulation, Nav2 configuration, route tools, and real-robot launch files
- `vmr_base_bridge`: ROS 2 bridge for the VMR chassis SDK
- `pose_estimator`: optional pose interpolation and odometry estimation experiments

Build from the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:$PATH colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```
