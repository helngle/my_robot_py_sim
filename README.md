# ROS 2 Mobile Manipulator

[中文文档](README_zh.md)

ROS 2 Humble workspace for a mobile manipulator with real-robot navigation,
YOLO RGB-D following, SAM3 table viewpoint planning, simulation tools, and a
VMR chassis SDK bridge.

## Main Workflows

All real-robot workflows share the same navigation backbone:

```text
VMR SDK pose + Livox/VMR point cloud
        |
        v
/estimated_pose + /estimated_odom + map -> base_footprint
PointCloud2 -> /scan
        |
        v
Nav2 SmacPlanner2D + MPPI Omni controller
        |
        v
/cmd_vel -> vmr_base_bridge -> physical chassis
```

The shared entry point is
`my_robot_bringup/launch/real_navigation_mppi.launch.py`. YOLO following and
SAM3 table navigation include this launch file, so changes to
`my_robot_navigation/config/real_nav2_no_odom_mppi.yaml` affect all three
workflows unless another `nav2_params_file` is supplied explicitly.

The default stack is pure MPPI. It does not use the archived hybrid,
distance-split, or RotationShim controllers. Its behavior tree clears
costmaps and waits during recovery; loading the Nav2 spin plugin does not mean
that Spin recovery is executed.

## Repository Layout

```text
src/
|-- my_robot_bringup/        # Shared real and simulation launch entry points
|-- my_robot_navigation/     # Nav2, behavior tree, route, and safety config
|-- my_robot_description/    # URDF, RViz2, and simulation assets
|-- my_robot_localization/   # SDK pose, odometry, and TF helpers
|-- my_robot_perception/     # RGB-D and point-cloud processing nodes
|-- my_robot_yolo_follow/    # YOLO RGB-D follow workflow
|-- my_robot_table_viewpoint/# SAM3 detection and table viewpoint workflow
|-- my_robot_maps/           # Versioned maps
|-- my_robot_tools/          # Route, marker, and operator tools
|-- vmr_base_bridge/         # VMR SDK ROS 2 bridge
|-- livox_ros_driver2/       # Livox MID360 ROS 2 driver
`-- OrbbecSDK_ROS2/          # Orbbec Gemini 435Le ROS 2 driver
```

Older hybrid and distance-split implementations are archived outside the Git
repository under `~/ros2_ws/archive/current_main_chain_backup_20260709` and
are not part of the default startup path.

## Build

ROS 2 Humble uses Python 3.10. If Anaconda is active, force the system Python:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:$PATH colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

`livox_ros_driver2` requires Livox-SDK2 installed under `/usr/local`. Keep its
source outside the ROS workspace:

```bash
mkdir -p ~/vendor
cd ~/vendor
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

## Environment

Use the following environment in each terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0
```

## Normal Navigation

Livox is the default LiDAR. The Orbbec camera and RGB-D goal finder remain off
for normal navigation:

```bash
ros2 launch my_robot_bringup real_navigation_mppi.launch.py \
  lidar_source:=livox \
  use_orbbec_camera:=false \
  use_rgbd_goal:=false \
  use_rviz:=true
```

Use `lidar_source:=vmr` to select the VMR SDK point cloud. Override the saved
map with `map:=/absolute/path/to/map.yaml`.

The Livox conversion is intentionally lightweight: a one-degree scan is built
from the `0.10-0.80 m` height band with a maximum range of `10 m`. Normal RViz
does not display the raw Livox cloud, reducing CPU and GPU load.

## YOLO RGB-D Follow

```bash
ros2 launch my_robot_yolo_follow yolo_rgbd_navigation.launch.py \
  lidar_source:=livox \
  target_class:=person \
  use_rviz:=true
```

Following does not start automatically. Use the services below:

```bash
ros2 service call /send_rgbd_goal std_srvs/srv/Trigger "{}"
ros2 service call /start_rgbd_follow std_srvs/srv/Trigger "{}"
ros2 service call /stop_rgbd_follow std_srvs/srv/Trigger "{}"
ros2 service call /unlock_rgbd_target std_srvs/srv/Trigger "{}"
```

See [my_robot_yolo_follow/README.md](my_robot_yolo_follow/README.md) for package
details.

## SAM3 Table Viewpoint

The combined launch starts the shared navigation stack, Orbbec camera, SAM3
detector, viewpoint planner, and table RViz configuration:

```bash
ros2 launch my_robot_table_viewpoint sam3_table_navigation.launch.py \
  lidar_source:=livox \
  sam3_model:=/home/jensen/ros2_ws/sam3.pt \
  sam3_prompt:="office desk" \
  sam3_device:=cuda
```

SAM3 detection runs on demand using the latest RGB-D frame:

```bash
ros2 service call /detect_sam3_table std_srvs/srv/Trigger "{}"
```

The detector publishes `/target_bbox_3d`, `/sam3_table_mask/debug`, and
`/sam3_table_bbox_marker`. See
[my_robot_table_viewpoint/README.md](my_robot_table_viewpoint/README.md) for
standalone launches, input modes, calibration, and manual goal services.

## Navigation Behavior

The physical base is approximately `0.70 x 0.60 m`; Nav2 uses an expanded
`0.80 x 0.70 m` rectangular footprint. The controller retains Omni motion but
prefers vehicle-forward path following outside the near-goal region. Inside
`1.2 m`, forward/path-angle preferences stop dominating so lateral and angular
corrections can converge on the goal.

Current goal and progress checks are:

```yaml
progress_checker:
  required_movement_radius: 0.05
  movement_time_allowance: 30.0

goal_checker:
  stateful: true
  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.08
```

The stateful goal checker remembers when position tolerance has been reached,
allowing final heading correction without repeatedly reacquiring XY. The
progress threshold is small enough not to interrupt normal near-goal motion
prematurely. Velocity deadbands `[0.015, 0.015, 0.05]` suppress tiny commands
that otherwise make the steering modules hunt near the endpoint.

## Pose and Timing

Raw SDK topics `/vmr_base_bridge/pose` and `/vmr_base_bridge/odom` may appear
delayed because the vendor timestamp is not synchronized with ROS time. The
navigation chain uses `sdk_pose_to_map_tf.py` with
`stamp_with_current_time: true`, publishing `/estimated_pose`,
`/estimated_odom`, and `map -> base_footprint`. Nav2 and the velocity smoother
consume `/estimated_odom`.

## Diagnostics

Basic checks:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /scan
ros2 topic hz /estimated_odom
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_footprint livox_frame
```

For a complete real-navigation capture, use the installed diagnostic script:

```bash
ros2 run my_robot_navigation record_nav_diagnostics.sh
```

It records velocity commands, raw and estimated pose/odometry, scan and
costmap data, TF, CPU usage, and a configuration snapshot.

## Safety

Test real-robot changes at low speed with an operator and emergency stop
available. Before sending a goal, verify the map, footprint, TF tree, `/scan`,
and both costmaps. Change one control parameter group at a time and preserve a
diagnostic log for comparison.
