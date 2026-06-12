# vmr_base_bridge

`vmr_base_bridge` 是对厂商二进制 SDK `vmr_-amr_-sdk` 的 ROS 2 封装，当前主要服务：

- `/vmr_base_bridge/step_move`
- `/vmr_base_bridge/vector_move`
- `/vmr_base_bridge/nav_target`
- `/vmr_base_bridge/cancel_task`
- `/vmr_base_bridge/query_task_status`
- `/vmr_base_bridge/control_relay`

旧的任务式导航接口 `MoveTask.srv` 已删除。
取消和状态查询能力已按新的任务管理模型重新开发，不再沿用旧实现。

## 服务定义

### `srv/StepMove.srv`

```srv
uint8 DIR_FORWARD = 0
uint8 DIR_BACKWARD = 1
uint8 DIR_LEFT = 2
uint8 DIR_RIGHT = 3
uint8 DIR_ROTATE_CW = 4
uint8 DIR_ROTATE_CCW = 5

uint8 direction
float64 value
---
bool success
string task_id
string message
```

说明：

- 平移时 `value` 单位为米
- 旋转时 `value` 单位为弧度
- 服务收到请求并成功下发 SDK 后立即返回
- 返回的 `task_id` 可用于取消任务或查询状态
- 内部直接调用 SDK 的 `VMR_moveRelative` / `VMR_rotateInPlace`

示例：

```bash
ros2 service call /vmr_base_bridge/step_move vmr_base_bridge/srv/StepMove "{direction: 0, value: 1.0}"
ros2 service call /vmr_base_bridge/step_move vmr_base_bridge/srv/StepMove "{direction: 5, value: 1.5708}"
```

### `srv/VectorMove.srv`

```srv
# Move along an arbitrary relative direction.
# angle is in radians, relative to robot forward direction.
float64 distance
float64 angle
---
bool success
string task_id
string message
```

说明：

- `distance` 单位为米，必须大于 0
- `angle` 单位为弧度，相对机器人正前方
- `angle=0` 表示正前方，`angle=pi/2` 表示左方，`angle=-pi/2` 表示右方
- 服务收到请求并成功下发 SDK 后立即返回
- 内部直接调用 SDK 的 `VMR_moveRelative(distance, angle_deg)`

示例：

```bash
ros2 service call /vmr_base_bridge/vector_move vmr_base_bridge/srv/VectorMove "{distance: 1.3, angle: 0.785398}"
ros2 service call /vmr_base_bridge/vector_move vmr_base_bridge/srv/VectorMove "{distance: 1.838478, angle: -2.356194}"
```

### `srv/NavTarget.srv`

```srv
# Leave empty to navigate by x/y/theta.
string site_name
float64 x
float64 y
float64 theta
---
bool success
string task_id
string message
```

说明：

- `site_name` 为空时，使用 `x/y/theta` 坐标导航
- `site_name` 非空时，从 `site_mapping.yaml` 查目标点
- `theta` 单位为弧度
- 服务收到请求并成功下发 SDK 后立即返回
- 返回的 `task_id` 可用于取消任务或查询状态
- 内部直接构造 SDK 的 `VmrPose{x, y, theta}` 并调用 `VMR_moveTasks`

示例：

```bash
ros2 service call /vmr_base_bridge/nav_target vmr_base_bridge/srv/NavTarget "{site_name: charging_station}"
ros2 service call /vmr_base_bridge/nav_target vmr_base_bridge/srv/NavTarget "{x: 1.0, y: 2.0, theta: 0.0}"
```

### `srv/CancelTask.srv`

```srv
string task_id
---
bool success
string message
```

说明：

- `task_id` 非空时，只取消指定任务
- `task_id` 为空时，取消当前节点记录的全部活动任务
- 节点内部用向量保存当前活动任务 ID，便于批量取消

示例：

```bash
ros2 service call /vmr_base_bridge/cancel_task vmr_base_bridge/srv/CancelTask "{task_id: ''}"
ros2 service call /vmr_base_bridge/cancel_task vmr_base_bridge/srv/CancelTask "{task_id: 'task-123'}"
```

### `srv/QueryTaskStatus.srv`

```srv
string task_id
---
bool success
int32 task_flag
int32 task_result
string task_id_echo
string message
```

说明：

- 用于查询任意已知 `task_id` 的底层状态
- `task_result=0` 表示成功
- `task_result=-1` 表示执行中
- 其他值表示失败

示例：

```bash
ros2 service call /vmr_base_bridge/query_task_status vmr_base_bridge/srv/QueryTaskStatus "{task_id: 'task-123'}"
```

### `srv/ControlRelay.srv`

```srv
bool enable
---
bool success
string task_id
string message
```

说明：

- `enable=true` 打开继电器，`enable=false` 关闭继电器
- 服务收到请求并成功下发 SDK 后立即返回
- 返回的 `task_id` 可用于取消任务或查询状态

示例：

```bash
ros2 service call /vmr_base_bridge/control_relay vmr_base_bridge/srv/ControlRelay "{enable: true}"
```

## 配置

参数文件：`config/vmr_base_bridge.yaml`

```yaml
vmr_base_bridge_node:
  ros__parameters:
    service:
      step_move_name: "/vmr_base_bridge/step_move"
      vector_move_name: "/vmr_base_bridge/vector_move"
      nav_target_name: "/vmr_base_bridge/nav_target"
      cancel_task_name: "/vmr_base_bridge/cancel_task"
      query_task_status_name: "/vmr_base_bridge/query_task_status"
      control_relay_name: "/vmr_base_bridge/control_relay"
```

`sdk_config_file` 和 `navigation.site_mapping_file` 默认由包安装目录解析，也可在 launch 时覆盖：

```bash
ros2 launch vmr_base_bridge vmr_base_bridge.launch.py \
  sdk_config_file:=/path/to/vmr_sdk.ini \
  site_mapping_file:=/path/to/site_mapping.yaml
```

`static_map_publisher.py` 可单独发布栅格地图和 costmap。地图路径必须显式传入，
避免代码绑定到某台电脑的工作区：

```bash
ros2 run vmr_base_bridge static_map_publisher.py --ros-args \
  -p map_yaml:=/path/to/map.yaml
```

站点映射文件：`config/site_mapping.yaml`

```yaml
sites:
  charging_station:
    x: 0.0
    y: 0.0
    theta: 0.0
```

要求：

- `sites` 必须是 map
- 每个站点包含 `x/y/theta`
- 节点启动时加载一次，运行期只读内存映射

## 任务管理说明

- `StepMove` 和 `NavTarget` 成功下发后，任务 ID 会登记到内存向量
- `QueryTaskStatus` 查询到终态后，会自动从向量移除
- `CancelTask` 传空 `task_id` 时，会对当前向量快照中的所有任务逐个下发取消

## 状态话题

- `/vmr_base_bridge/laser/points`
- `/vmr_base_bridge/location`
- `/vmr_base_bridge/pose`
- `/vmr_base_bridge/odom`
- `/vmr_base_bridge/battery`
- `/vmr_base_bridge/imu`
- `/vmr_base_bridge/imu_pose`
- `/vmr_base_bridge/move_status`

`/vmr_base_bridge/move_status` 来自新 SDK 的 `VMR_registerMoveStatusCallback`，包含导航状态码、剩余时间和当前路径点。

## 编译与运行

编译：

```bash
colcon build --packages-select vmr_base_bridge
```

默认从包内 `lib/<arch>/<ubuntu_version>/libvmr_sdk.so` 链接官方 SDK，例如 x86 Ubuntu 22.04 会使用 `lib/x86/22.04/libvmr_sdk.so`。

运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vmr_base_bridge vmr_base_bridge.launch.py
```

执行等腰直角三角形轨迹：

```bash
ros2 run vmr_base_bridge right_triangle_trajectory.py
```

说明：

- 默认等腰边长为 `1.3m`
- 默认假设机器人当前朝向为第一条直角边方向
- 轨迹使用 `VectorMove` 斜向走斜边，避免为斜边额外旋转
- 脚本会等待每段移动任务完成后再执行下一段
