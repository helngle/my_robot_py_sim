# my_robot_py_sim

`my_robot_py_sim` contains the mobile-manipulator model, Gazebo/RViz2
simulation, Nav2 configurations for the real chassis, and route-management
tools.

## Current Capabilities

- Gazebo simulation with keyboard `/cmd_vel`, 3D LiDAR, projected `/scan`,
  SLAM Toolbox, and RViz2.
- Real-robot RViz2 navigation using the VMR SDK pose and LiDAR point cloud.
- Nav2 global planning with `NavfnPlanner`.
- Omni-directional local control with MPPI.
- MPPI prefers rotating the vehicle front toward the route and moving forward;
  lateral movement remains available when needed.
- Static-map global avoidance and live `/scan` local avoidance.
- Route recording, display selection, waypoint execution, and RViz2-assisted
  insertion of intermediate points.

## Navigation Geometry

The real-robot Nav2 configurations use:

```text
Footprint:         0.70 m x 0.60 m
Inflation radius:  0.45 m
```

The footprint is used by both global and local costmaps. The URDF base model is
`0.70 m x 0.60 m x 0.35 m`, but standard Nav2 collision checking is 2D and
does not use the base height.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:$PATH colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## Simulation

Start Gazebo, SLAM, and RViz2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_robot_py_sim sim_with_rviz.launch.py
```

Keyboard control in another terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=1

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Important simulation topics:

- `/lidar/points`: Gazebo 3D LiDAR point cloud
- `/scan`: projected 2D LaserScan
- `/odom`: Gazebo odometry
- `/map`: SLAM occupancy grid
- `/safety_shell_array`: full-body visualization shell

## Real-Robot MPPI Navigation

Connect to the vehicle network first, then start:

```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0

ros2 launch my_robot_py_sim real_navigation_no_odom_mppi_with_rviz.launch.py
```

The launch uses:

- `/vmr_base_bridge/pose` as the global vehicle pose
- `/vmr_base_bridge/laser/points` as the LiDAR source
- `/scan` for the local obstacle layer
- `/cmd_vel` for real-time chassis control
- `config/real_nav2_no_odom_mppi.yaml` for Nav2

The default saved map is:

```text
~/ros2_ws/maps/Test052601/Test052601.yaml
```

Use another map without changing source code:

```bash
ros2 launch my_robot_py_sim real_navigation_no_odom_mppi_with_rviz.launch.py \
  map:=/absolute/path/to/map.yaml
```

After RViz2 opens, use **Nav2 Goal** to select a destination.

## Validate Real Navigation

Check the SDK topics:

```bash
ros2 topic echo --once /vmr_base_bridge/pose
ros2 topic hz /vmr_base_bridge/laser/points
ros2 topic hz /scan
```

Check TF and Nav2:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list | grep navigate
```

Check the loaded collision footprint and obstacle margin:

```bash
ros2 param get /global_costmap/global_costmap footprint
ros2 param get /local_costmap/local_costmap footprint
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
```

Expected footprint:

```text
[[0.350, 0.300], [0.350, -0.300], [-0.350, -0.300], [-0.350, 0.300]]
```

Observe whether MPPI is moving forward or laterally:

```bash
ros2 topic echo /cmd_vel
```

`linear.x` is forward motion, `linear.y` is lateral motion, and `angular.z`
rotates the chassis.

## Route Management

Routes are stored in:

```text
config/routes.yaml
```

Start navigation before using route commands so that `/route_manager` and the
Nav2 action servers are available.

List and inspect routes:

```bash
ros2 run my_robot_py_sim route_cli list
ros2 run my_robot_py_sim route_cli info patrol_a
```

Choose which routes RViz2 displays:

```bash
ros2 run my_robot_py_sim route_cli display patrol_a
ros2 run my_robot_py_sim route_cli display patrol_a patrol_b
ros2 run my_robot_py_sim route_cli display_none
ros2 run my_robot_py_sim route_cli display_all
```

Execute route targets:

```bash
ros2 run my_robot_py_sim route_cli go_to patrol_a wp_4
ros2 run my_robot_py_sim route_cli follow patrol_a --start wp_1 --goal wp_4
ros2 run my_robot_py_sim route_cli follow_nearest patrol_a --goal wp_4
```

Rename or delete routes:

```bash
ros2 run my_robot_py_sim route_cli rename patrol_a patrol_test
ros2 run my_robot_py_sim route_cli delete patrol_test --yes
```

### Record A Driven Route

While navigation and TF are running, start the recorder:

```bash
ros2 run my_robot_py_sim route_recorder --ros-args \
  -p route_name:=route_test \
  -p min_point_spacing:=0.35
```

Drive the robot along the desired path. Press `Ctrl+C` to save the route into
`config/routes.yaml`.

### Insert Intermediate Points In RViz2

Start the editor for an existing route:

```bash
ros2 run my_robot_py_sim route_insert_editor --ros-args \
  -p source_route:=route_test \
  -p output_route:=route_test_edit
```

In RViz2, select **Publish Point** and click near the desired route segment.
Each click inserts a waypoint into the nearest segment. Press `Ctrl+C` to save
the edited route.

## Main Configuration Files

- `config/real_nav2_no_odom_mppi.yaml`: primary real-robot MPPI navigation
- `config/real_nav2_no_odom_omni.yaml`: real-robot DWB omni alternative
- `config/nav2_navigation.yaml`: simulation Nav2 parameters
- `config/slam_toolbox.yaml`: simulation SLAM parameters
- `config/routes.yaml`: saved named routes
- `urdf/mobile_manipulator.urdf`: RViz2/Gazebo robot model
- `rviz/view_robot.rviz`: shared RViz2 display configuration

## Notes

- Real navigation uses the SDK global pose to publish `map -> base_footprint`
  and `/estimated_odom`; it does not depend on the unreliable raw chassis odom
  as its primary localization source.
- The simulation robot model is useful for visualization and algorithm
  development, but real avoidance behavior is determined by the Nav2 footprint
  and sensor data.
- Keep an emergency stop available during real-robot tests.
