# Table Viewpoint Planner

This package is intentionally separate from `my_robot_perception` and the
existing real-navigation launch. It adds no nodes to the normal robot startup.

The planner uses an RViz `Publish Point` click as a search seed, fits a
horizontal tabletop from the next aligned Orbbec depth frame, constructs a
known-size 3D box, checks front-center camera viewpoints against the local
costmap, and sends the selected Nav2 goal once in automatic mode.

The configured desk size is `1.40 x 0.60 x 0.73 m`. Calibration is stored in
the separate `Test052601_table_viewpoint/tables.yaml` map sidecar. The original
`Test052601` map is not modified.

With the current level camera mount, a full-height table view requires roughly
2.3 m of standoff. The default minimum projected fill is therefore 28 percent;
the planner still prefers the closest fully framed, collision-free candidate.

## Run

The normal real-robot workflow is available as one combined command. It uses
the separate table map, starts Livox, Orbbec, Nav2, the viewpoint planner, and
the dedicated RViz configuration:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_table_viewpoint table_navigation.launch.py
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
viewpoint markers, `/rgbd_debug_image`, and a `Publish Point` tool bound to
`/clicked_point`. Green candidate markers fit the complete tabletop in the
camera and are footprint-safe; the blue arrow is the selected goal.

The default operating mode is automatic: once a valid viewpoint exists and
Nav2's `bt_navigator` lifecycle node is active, the node submits the goal and
verifies that Nav2 accepted it. Rejected startup requests are retried. The
final navigation result is also written to the node log. The saved table's
blue box and the selected blue goal arrow are displayed automatically; noisy
candidate dots stay hidden. Set `publish_candidate_markers:=true` in the
parameter file only when tuning the geometry or costmap filters.

The robot should remain stationary for the click and the following depth
frame. The planner prefers the depth capture timestamp, but falls back to the
latest TF when the camera timestamp is a few milliseconds ahead of the
`map -> base_footprint` stream. This fallback is logged once and avoids losing
an otherwise valid one-shot calibration.

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
