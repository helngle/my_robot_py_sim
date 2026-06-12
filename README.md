# ROS 2 Mobile Manipulator

ROS 2 Humble mobile-manipulator workspace containing simulation, real-robot
navigation, route tools, and the VMR chassis SDK bridge.

## Repository Layout

```text
src/
|-- my_robot_py_sim/   # Gazebo/RViz2, Nav2, robot model, and route tools
|-- vmr_base_bridge/   # ROS 2 bridge for the VMR chassis SDK
`-- pose_estimator/    # Optional pose interpolation experiments
```

The primary real-robot navigation path is:

```text
VMR SDK pose + LiDAR point cloud
        |
        v
map -> base_footprint, /estimated_odom, /scan
        |
        v
Nav2 global planner + MPPI omni controller
        |
        v
/cmd_vel -> vmr_base_bridge -> physical chassis
```

The current Nav2 footprint is a `0.70 m x 0.60 m` rectangle. The MPPI
configuration keeps omni-directional motion available, but strongly prefers
aligning the vehicle front with the route and moving forward. Local and global
costmaps use a `0.45 m` inflation radius.

## Build

ROS 2 Humble uses Python 3.10. If Anaconda is active, force the system Python
when building:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:$PATH colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## Quick Start

Simulation with Gazebo, SLAM, and RViz2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_robot_py_sim sim_with_rviz.launch.py
```

Real robot with the MPPI omni controller:

```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0

ros2 launch my_robot_py_sim real_navigation_no_odom_mppi_with_rviz.launch.py
```

The real launch expects the vehicle network and a saved map. Its default map is
`~/ros2_ws/maps/Test052601/Test052601.yaml`; override it with:

```bash
ros2 launch my_robot_py_sim real_navigation_no_odom_mppi_with_rviz.launch.py \
  map:=/absolute/path/to/map.yaml
```

See [`my_robot_py_sim/README.md`](my_robot_py_sim/README.md) for route
recording, route editing, validation commands, and navigation details.

## Packages

- **my_robot_py_sim**: robot URDF, Gazebo world, RViz2 configuration, Nav2
  parameters, map/scan integration, route recording, and route execution.
- **vmr_base_bridge**: wraps the vendor VMR SDK, publishes chassis state and
  LiDAR topics, accepts `/cmd_vel`, and exposes task-style motion services.
- **pose_estimator**: optional experimental package for interpolating an
  external pose stream. It is not part of the primary real-navigation launch.

## Safety

Test real-robot changes at low speed with an operator and emergency stop
available. Confirm the loaded map, footprint, TF, and obstacle data before
sending an RViz2 navigation goal.
