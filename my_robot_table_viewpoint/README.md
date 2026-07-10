# Table Viewpoint Planner

This package is intentionally separate from `my_robot_perception` and the
existing real-navigation launch. It adds no nodes to the normal robot startup.

The planner accepts a runtime `vision_msgs/msg/Detection3D` bounding box,
transforms it to `map`, and uses the received center, orientation, and size to
generate camera viewpoints. It checks complete tabletop framing and robot
footprint safety against the global/local costmaps before sending the selected
Nav2 goal.

The default input mode is `topic`, listening on `/target_bbox_3d`. It does not
load the historical `tables.yaml`, so startup alone does not create or send a
goal. The YAML workflow remains available as an explicit fallback mode.
Calibration is stored in the separate
`Test052601_table_viewpoint/tables.yaml` map sidecar; the original map is not
modified.

Collision safety and complete framing remain hard requirements. Remaining
candidates receive a weighted score that strongly favors projected area (70%)
over horizontal centering (10%), vertical centering (8%), costmap clearance
(10%), and straight-line travel distance (2%). Score scales and weights are configurable in
`config/table_viewpoint.yaml`.
Projected area is normalized adaptively: when every valid candidate is below
the configured area ceiling, the largest current candidate receives full area
score. This preserves discrimination when the visible tabletop polygon is
small in absolute pixel area.

## Run

The normal real-robot workflow is available as one combined command. It uses
the separate table map, starts Livox, Orbbec, the current
`real_navigation_mppi.launch.py` Nav2 backbone, the viewpoint planner, and the
dedicated RViz configuration:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_table_viewpoint table_navigation.launch.py
```

The SAM3 workflow uses the same navigation backbone and starts the detector in
the same launch:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_table_viewpoint sam3_table_navigation.launch.py \
  sam3_model:=/home/jensen/ros2_ws/sam3.pt \
  sam3_prompt:="office desk" \
  sam3_device:=cuda
```

The lower-level standalone launch below remains available when the existing
navigation stack is already running.

Start the existing navigation stack with the Orbbec camera, then start this
standalone launch in a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0
ros2 launch my_robot_table_viewpoint table_viewpoint.launch.py
```

The standalone launch opens a dedicated RViz configuration automatically. It
already contains the map, global/local costmaps, robot, projected scan, table
viewpoint markers, and `/rgbd_debug_image`. Green candidate markers fit the
complete tabletop in the camera and are footprint-safe; the blue arrow is the
selected goal.

The default operating mode is automatic: once a valid 3D bbox arrives and
Nav2's `bt_navigator` lifecycle node is active, the node submits the goal and
verifies that Nav2 accepted it. Rejected startup requests are retried. The
final navigation result is also written to the node log. The runtime table's
blue box and selected blue goal arrow are displayed automatically; noisy
candidate dots stay hidden.

## Input modes

Default dynamic bbox mode:

```bash
ros2 launch my_robot_table_viewpoint table_navigation.launch.py \
  input_mode:=topic bbox_topic:=/target_bbox_3d
```

The bbox must have a valid `header.frame_id`, center pose, and positive size.
Its local X/Y dimensions are normalized so the longer horizontal dimension is
used as the table long axis. Near-identical repeated detections are ignored;
meaningful position, yaw, or size updates trigger replanning. A repeated bbox
also starts a new task when the robot has moved more than 0.35 m away from the
previous viewpoint. Set `repeat_bbox_retriggers_goal:=false` for continuously
streaming detectors that should not reclaim navigation after another client
sends a goal.

## SAM3 bbox input

`sam3_table_bbox_node` generates `/target_bbox_3d` from SAM3 open-vocabulary
text segmentation plus aligned depth. It runs only when the service is called:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0

ros2 launch my_robot_table_viewpoint sam3_table_navigation.launch.py \
  sam3_model:=/path/to/sam3.pt \
  sam3_prompt:="office desk" \
  sam3_device:=cuda
```

Then trigger one detection from the latest RGB-D frame:

```bash
ros2 service call /detect_sam3_table std_srvs/srv/Trigger {}
```

The node publishes `/target_bbox_3d`, `/sam3_table_mask/debug`, and
`/sam3_table_bbox_marker`.

Load the historical YAML explicitly:

```bash
ros2 launch my_robot_table_viewpoint table_navigation.launch.py \
  input_mode:=yaml
```

Viewpoint endpoints are checked against `/global_costmap/costmap`, so a saved
table remains usable when it is outside the robot's 5 x 5 m rolling local
window. When a candidate enters `/local_costmap/costmap`, its footprint is
also checked there for current nearby obstacles. Nav2 continues to use its
local costmap while driving.

By default, camera framing and the blue marker use only a 5 cm tabletop slab
at the top of the saved table geometry. The full table height remains stored
for calibration and can be restored by setting `observe_tabletop_only` to
`false`. Candidate selection projects the four top corners, requires all four
to stay inside a 5 percent image margin, and then maximizes the projected
tabletop polygon area. Horizontal centering and costmap safety remain hard
constraints.

Save a verified calibration once:

```bash
ros2 service call /save_table_calibration std_srvs/srv/Trigger {}
```

The service remains available when an operator wants to resend the selected
goal manually:

```bash
ros2 service call /send_table_viewpoint std_srvs/srv/Trigger {}
```

Clearing the runtime result does not erase the saved database:

```bash
ros2 service call /clear_table_calibration std_srvs/srv/Trigger {}
```
