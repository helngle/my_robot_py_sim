# YOLO RGB-D Follow

This package keeps the YOLO RGB-D follow workflow separate from the normal
robot bringup package. It includes the current real navigation backbone:
`my_robot_bringup/launch/real_navigation_mppi.launch.py`.

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0

ros2 launch my_robot_yolo_follow yolo_rgbd_navigation.launch.py \
  lidar_source:=livox \
  target_class:=person \
  use_rviz:=true
```

Start following:

```bash
ros2 service call /start_rgbd_follow std_srvs/srv/Trigger "{}"
```

Stop following:

```bash
ros2 service call /stop_rgbd_follow std_srvs/srv/Trigger "{}"
```
