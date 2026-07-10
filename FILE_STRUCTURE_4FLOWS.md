# Current Robot Workflows

This workspace now keeps one navigation backbone active:
`my_robot_bringup/launch/real_navigation_mppi.launch.py` with
`my_robot_navigation/config/real_nav2_no_odom_mppi.yaml`.

YOLO RGB-D following and SAM3 table navigation both include that same backbone,
so they use the current MPPI/Smac/forward-only behavior-tree chain. Hybrid /
distance-split experiments were moved to
`/home/jensen/ros2_ws/archive/current_main_chain_backup_20260709`.

## 1. Normal Navigation

Purpose: standard Nav2 navigation, lidar obstacle avoidance, and manual RViz
goals.

Launch:

```bash
ros2 launch my_robot_bringup real_navigation_mppi.launch.py \
  lidar_source:=livox \
  use_rviz:=true
```

Main files:

- `my_robot_bringup/launch/real_navigation_mppi.launch.py`
- `my_robot_navigation/config/real_nav2_no_odom_mppi.yaml`
- `my_robot_navigation/behavior_trees/navigate_forward_only_replanning_if_path_invalid.xml`
- `my_robot_description/rviz/view_robot.rviz`

## 2. YOLO RGB-D Person Follow

Purpose: detect a person with YOLO + RGB-D, publish a navigation goal, and
optionally follow the tracked target.

Launch:

```bash
ros2 launch my_robot_yolo_follow yolo_rgbd_navigation.launch.py
```

Services:

```bash
ros2 service call /start_rgbd_follow std_srvs/srv/Trigger "{}"
ros2 service call /stop_rgbd_follow std_srvs/srv/Trigger "{}"
```

Main files:

- `my_robot_perception/my_robot_perception/rgbd_goal_finder.py`
- `my_robot_yolo_follow/launch/yolo_rgbd_navigation.launch.py`
- `my_robot_perception/config/rgbd_goal_finder.yaml`
- `my_robot_perception/config/bytetrack_person.yaml`
- `my_robot_bringup/launch/real_navigation_mppi.launch.py`

## 3. SAM3 Table Detection

Purpose: use SAM3 to detect a table, publish `/target_bbox_3d`, and plan a
safe viewpoint around the table.

Full navigation + SAM3 + viewpoint planner:

```bash
ros2 launch my_robot_table_viewpoint sam3_table_navigation.launch.py \
  sam3_model:=/home/jensen/ros2_ws/sam3.pt \
  sam3_prompt:="office desk" \
  sam3_device:=cuda
```

Trigger detection:

```bash
ros2 service call /detect_sam3_table std_srvs/srv/Trigger "{}"
```

Main files:

- `my_robot_table_viewpoint/launch/table_navigation.launch.py`
- `my_robot_table_viewpoint/launch/sam3_table_navigation.launch.py`
- `my_robot_table_viewpoint/launch/table_viewpoint.launch.py`
- `my_robot_table_viewpoint/launch/sam3_table_bbox.launch.py`
- `my_robot_table_viewpoint/config/table_viewpoint.yaml`
- `my_robot_table_viewpoint/my_robot_table_viewpoint/table_viewpoint_planner.py`
- `my_robot_table_viewpoint/my_robot_table_viewpoint/sam3_table_bbox_node.py`
- `my_robot_table_viewpoint/rviz/table_viewpoint.rviz`

## Archived: Distance-Split Omni Navigation

These files are no longer part of the active source tree. Backup location:
`/home/jensen/ros2_ws/archive/current_main_chain_backup_20260709`.

- `my_robot_bringup/launch/real_navigation_mppi_hybrid_distance.launch.py`
- `my_robot_navigation/config/real_nav2_no_odom_mppi_hybrid_distance.yaml`
- `my_robot_navigation/behavior_trees/navigate_long_forward_replanning_if_path_invalid.xml`
- `my_robot_navigation/behavior_trees/navigate_short_omni_replanning_if_path_invalid.xml`
- `my_robot_navigation/behavior_trees/navigate_fine_omni_replanning_if_path_invalid.xml`
- `my_robot_table_viewpoint/launch/hybrid_local_obstacle_test.launch.py`
- `my_robot_table_viewpoint/launch/hybrid_table_viewpoint.launch.py`
- `my_robot_table_viewpoint/launch/hybrid_table_server.launch.py`
- `my_robot_table_viewpoint/config/hybrid_table_viewpoint.yaml`
- `my_robot_table_viewpoint/my_robot_table_viewpoint/hybrid_viewpoint_orchestrator.py`
- `my_robot_bringup/rviz/hybrid_obstacle_test.rviz`
