# ROS 2 移动机械臂

[English](README.md)

这是一个基于 ROS 2 Humble 的移动机械臂工作区，包含实机导航、YOLO RGB-D
跟随、SAM3 桌子视点规划、仿真工具以及 VMR 底盘 SDK 桥接。

## 主要流程

三套实机功能共用同一条导航主链路：

```text
VMR SDK 位姿 + Livox/VMR 点云
        |
        v
/estimated_pose + /estimated_odom + map -> base_footprint
PointCloud2 -> /scan
        |
        v
Nav2 SmacPlanner2D + MPPI Omni 控制器
        |
        v
/cmd_vel -> vmr_base_bridge -> 实体底盘
```

共享入口是 `my_robot_bringup/launch/real_navigation_mppi.launch.py`。
YOLO 跟随和 SAM3 桌子导航都会 include 这个文件，因此修改
`my_robot_navigation/config/real_nav2_no_odom_mppi.yaml` 默认会同时影响三套
流程，除非启动时显式传入其他 `nav2_params_file`。

当前默认控制器是纯 MPPI，不使用已归档的 hybrid、distance-split 或
RotationShim。行为树的恢复动作是清理代价地图和等待；日志显示 Nav2 已加载
spin 插件，并不表示行为树实际执行了 Spin 恢复。

## 仓库结构

```text
src/
|-- my_robot_bringup/         # 实机和仿真的共享启动入口
|-- my_robot_navigation/      # Nav2、行为树、路线和安全配置
|-- my_robot_description/     # URDF、RViz2 和仿真资源
|-- my_robot_localization/    # SDK 位姿、里程计和 TF 辅助节点
|-- my_robot_perception/      # RGB-D 和点云处理节点
|-- my_robot_yolo_follow/     # YOLO RGB-D 跟随流程
|-- my_robot_table_viewpoint/ # SAM3 检测和桌子视点流程
|-- my_robot_maps/            # 版本化地图
|-- my_robot_tools/           # 路线、标记和操作工具
|-- vmr_base_bridge/          # VMR SDK ROS 2 桥接
|-- livox_ros_driver2/        # Livox MID360 ROS 2 驱动
`-- OrbbecSDK_ROS2/           # Orbbec Gemini 435Le ROS 2 驱动
```

旧 hybrid 和 distance-split 实现在 Git 仓库外的
`~/ros2_ws/archive/current_main_chain_backup_20260709` 中归档，不属于默认启动
链路。

## 编译

ROS 2 Humble 使用 Python 3.10。如果当前激活了 Anaconda，请强制使用系统
Python：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:$PATH colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

`livox_ros_driver2` 依赖安装到 `/usr/local` 的 Livox-SDK2。SDK 源码应放在
ROS 工作区之外：

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

## 环境配置

每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0
```

## 普通导航

Livox 是默认雷达。普通导航默认不启动 Orbbec 相机和 RGB-D 目标节点：

```bash
ros2 launch my_robot_bringup real_navigation_mppi.launch.py \
  lidar_source:=livox \
  use_orbbec_camera:=false \
  use_rgbd_goal:=false \
  use_rviz:=true
```

传入 `lidar_source:=vmr` 可以切换到 VMR SDK 点云。使用
`map:=/absolute/path/to/map.yaml` 可以覆盖默认地图。

Livox 转换链路已经轻量化：从 `0.10-0.80 m` 高度范围生成一度分辨率、最远
`10 m` 的二维扫描。普通导航 RViz 默认不显示原始 Livox 点云，以降低 CPU 和
GPU 负载。

## YOLO RGB-D 跟随

```bash
ros2 launch my_robot_yolo_follow yolo_rgbd_navigation.launch.py \
  lidar_source:=livox \
  target_class:=person \
  use_rviz:=true
```

启动后不会自动跟随，使用以下服务控制：

```bash
ros2 service call /send_rgbd_goal std_srvs/srv/Trigger "{}"
ros2 service call /start_rgbd_follow std_srvs/srv/Trigger "{}"
ros2 service call /stop_rgbd_follow std_srvs/srv/Trigger "{}"
ros2 service call /unlock_rgbd_target std_srvs/srv/Trigger "{}"
```

包内细节见 [my_robot_yolo_follow/README.md](my_robot_yolo_follow/README.md)。

## SAM3 桌子视点

完整入口会启动共享导航、Orbbec 相机、SAM3 检测、视点规划器和桌子专用
RViz：

```bash
ros2 launch my_robot_table_viewpoint sam3_table_navigation.launch.py \
  lidar_source:=livox \
  sam3_model:=/home/jensen/ros2_ws/sam3.pt \
  sam3_prompt:="office desk" \
  sam3_device:=cuda
```

SAM3 使用最新 RGB-D 帧按需检测：

```bash
ros2 service call /detect_sam3_table std_srvs/srv/Trigger "{}"
```

检测节点发布 `/target_bbox_3d`、`/sam3_table_mask/debug` 和
`/sam3_table_bbox_marker`。独立启动方式、输入模式、标定和手动发送目标等说明
见 [my_robot_table_viewpoint/README.md](my_robot_table_viewpoint/README.md)。

## 导航行为

实体底盘约为 `0.70 x 0.60 m`，Nav2 使用放大后的 `0.80 x 0.70 m` 矩形
footprint。MPPI 保留 Omni 运动能力，但在远离目标时优先让车头沿路径向前行驶。
进入距离目标 `1.2 m` 的区域后，向前和路径角度偏好不再占主导，使横移和旋转
能够完成终点收敛。

当前目标与进度判定参数为：

```yaml
progress_checker:
  required_movement_radius: 0.05
  movement_time_allowance: 30.0

goal_checker:
  stateful: true
  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.08
```

`stateful` 目标检查器会记住位置已经进入容差，之后可以专注完成朝向调整，不必
反复重新满足 XY。较小的进度阈值可以避免正常的终点微调被过早判定为卡死。
速度死区 `[0.025, 0.025, 0.05]` 会过滤碎小指令，减少四轮四转底盘在终点
附近反复摆轮。

## 位姿与时间戳

SDK 原始话题 `/vmr_base_bridge/pose` 和 `/vmr_base_bridge/odom` 可能因为厂商
时间戳未与 ROS 时间同步而显示出很大延迟。导航链路使用
`sdk_pose_to_map_tf.py`，并设置 `stamp_with_current_time: true`，由它发布
`/estimated_pose`、`/estimated_odom` 和 `map -> base_footprint`。Nav2 与速度
平滑器使用 `/estimated_odom`。

## 诊断

基础检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /scan
ros2 topic hz /estimated_odom
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_footprint livox_frame
```

需要完整记录实机导航时，运行已安装的诊断脚本：

```bash
ros2 run my_robot_navigation record_nav_diagnostics.sh
```

脚本会记录速度指令、原始与估算位姿/里程计、扫描、代价地图、TF、CPU 占用和
配置快照。

## 安全

实机参数调整应在低速、有操作员和急停可用的条件下进行。发送目标前检查地图、
footprint、TF 树、`/scan` 和两层代价地图。每次只修改一组控制参数，并保留诊断
日志用于前后对比。
