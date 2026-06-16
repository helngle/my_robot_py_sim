# ROS 2 Mobile Manipulator

ROS 2 Humble mobile-manipulator workspace containing simulation, real-robot
navigation, route tools, and the VMR chassis SDK bridge.

## Repository Layout

```text
src/
|-- my_robot_py_sim/   # Gazebo/RViz2, Nav2, robot model, and route tools
|-- vmr_base_bridge/   # ROS 2 bridge for the VMR chassis SDK
|-- livox_ros_driver2/ # ROS 2 driver for the Livox MID360 LiDAR
`-- pose_estimator/    # Optional pose interpolation experiments
```

The primary real-robot navigation path is:

```text
VMR SDK pose + VMR or Livox LiDAR point cloud
        |
        v
map -> base_footprint, /estimated_odom
PointCloud2 restamp -> /scan
        |
        v
Nav2 global planner + MPPI omni controller
        |
        v
/cmd_vel -> vmr_base_bridge -> physical chassis
```

The physical base is approximately `0.70 m x 0.60 m`. The current Nav2 hard
collision footprint is an expanded `0.80 m x 0.70 m` rectangle. The MPPI
configuration keeps omni-directional motion available, but strongly prefers
aligning the vehicle front with the route and moving forward. Local and global
costmaps use a `0.45 m` inflation radius.

The default real launch still uses the VMR SDK LiDAR. Pass
`lidar_source:=livox` to use the Livox MID360 instead. Livox points are
converted to a 2D `/scan` for Nav2 using a filtered height band:

```text
min_height: 0.10 m
max_height: 1.00 m
range_min: 0.45 m
range_max: 20.0 m
```

This filters floor returns and near-field self hits before projecting the 3D
point cloud into the 2D costmaps.

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

The Livox ROS driver depends on Livox-SDK2 installed into `/usr/local`.
`Livox-SDK2/` is intentionally ignored by Git because it is only used as local
SDK source. Install it once before building `livox_ros_driver2`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

If you only need to build the Livox ROS driver and do not need lint checks:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:$PATH colcon build --packages-select livox_ros_driver2 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
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

Real robot using Livox MID360 for obstacle sensing:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0

ros2 launch my_robot_py_sim real_navigation_no_odom_mppi_with_rviz.launch.py \
  lidar_source:=livox
```

Real robot with Livox and the Orbbec Gemini 435Le camera:

```bash
ros2 launch my_robot_py_sim real_navigation_no_odom_mppi_with_rviz.launch.py \
  lidar_source:=livox \
  use_orbbec_camera:=true
```

The camera defaults to `640x400 @ 10 FPS`. Override color or depth resolution
with launch arguments such as `orbbec_color_width:=1280`,
`orbbec_color_height:=800`, and `orbbec_color_fps:=10`.

Useful Livox checks:

```bash
ros2 topic info /livox/lidar
ros2 topic hz /livox/lidar
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_footprint livox_frame
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
- **livox_ros_driver2**: vendored ROS 2 driver for Livox MID360. The real
  navigation launch can start it and convert `/livox/lidar` to `/scan`.
- **OrbbecSDK_ROS2**: vendored ROS 2 driver for the Orbbec Gemini 435Le depth
  camera. The real navigation launch can start it, publish the camera TF, and
  show color images plus RGB depth points in RViz2.
- **pose_estimator**: optional experimental package for interpolating an
  external pose stream. It is not part of the primary real-navigation launch.

## Safety

Test real-robot changes at low speed with an operator and emergency stop
available. Confirm the loaded map, footprint, TF, and obstacle data before
sending an RViz2 navigation goal.
