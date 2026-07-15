#include "vmr_base_bridge/vmr_base_bridge.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

namespace vmr_base_bridge
{

VmrBaseBridge * VmrBaseBridge::instance_ = nullptr;

namespace
{
constexpr size_t kPointFieldCount = 4;
constexpr double kRadiansToDegrees = 57.29577951308232;

builtin_interfaces::msg::Time toBuiltinTime(uint64_t timestamp_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
  return stamp;
}

float normalizePercentage(float percentage)
{
  if (percentage > 1.0f) {
    return percentage / 100.0f;
  }
  return percentage;
}

sensor_msgs::msg::BatteryState::_power_supply_status_type toBatteryStatus(int8_t status)
{
  switch (status) {
    case 1:
      return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    case 2:
      return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    default:
      return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }
}

std::string defaultPackagePath(const std::string & relative_path)
{
  try {
    return ament_index_cpp::get_package_share_directory("vmr_base_bridge") + "/" + relative_path;
  } catch (const std::exception &) {
    return {};
  }
}

double clampValue(double value, double limit)
{
  const double abs_limit = std::abs(limit);
  if (abs_limit <= 0.0) {
    return 0.0;
  }
  return std::clamp(value, -abs_limit, abs_limit);
}

}  // namespace

VmrBaseBridge::VmrBaseBridge(const rclcpp::NodeOptions & options)
: Node("vmr_base_bridge_node", options)
{
  if (instance_ != nullptr) {
    throw std::runtime_error("vmr_base_bridge only supports a single node instance per process");
  }
  instance_ = this;

  declareParameters();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth_));
  laser_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(laser_topic_, qos);
  location_publisher_ = create_publisher<vmr_base_bridge::msg::VmrLocation>(location_topic_, qos);
  pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, qos);
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, qos);
  battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(battery_topic_, qos);
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, qos);
  imu_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(imu_pose_topic_, qos);
  move_status_publisher_ = create_publisher<vmr_base_bridge::msg::VmrMoveStatus>(
    move_status_topic_, qos);
  timing_diagnostics_publisher_ =
    create_publisher<vmr_base_bridge::msg::VmrTimingDiagnostic>(
    timing_diagnostics_topic_, rclcpp::QoS(rclcpp::KeepLast(100)));
  step_move_service_ = create_service<vmr_base_bridge::srv::StepMove>(
    step_move_service_name_,
    std::bind(&VmrBaseBridge::handleStepMove, this, std::placeholders::_1, std::placeholders::_2));
  vector_move_service_ = create_service<vmr_base_bridge::srv::VectorMove>(
    vector_move_service_name_,
    std::bind(&VmrBaseBridge::handleVectorMove, this, std::placeholders::_1, std::placeholders::_2));
  nav_target_service_ = create_service<vmr_base_bridge::srv::NavTarget>(
    nav_target_service_name_,
    std::bind(&VmrBaseBridge::handleNavTarget, this, std::placeholders::_1, std::placeholders::_2));
  cancel_task_service_ = create_service<vmr_base_bridge::srv::CancelTask>(
    cancel_task_service_name_,
    std::bind(&VmrBaseBridge::handleCancelTask, this, std::placeholders::_1, std::placeholders::_2));
  query_task_status_service_ = create_service<vmr_base_bridge::srv::QueryTaskStatus>(
    query_task_status_service_name_,
    std::bind(
      &VmrBaseBridge::handleQueryTaskStatus, this, std::placeholders::_1,
      std::placeholders::_2));
  control_relay_service_ = create_service<vmr_base_bridge::srv::ControlRelay>(
    control_relay_service_name_,
    std::bind(&VmrBaseBridge::handleControlRelay, this, std::placeholders::_1, std::placeholders::_2));

  initializeSdk();
  loadSiteMappings();
  last_cmd_vel_diag_time_ = now();

  if (cmd_vel_enabled_) {
    last_cmd_vel_time_ = now();
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      qos,
      std::bind(&VmrBaseBridge::handleCmdVel, this, std::placeholders::_1));
    cmd_vel_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / cmd_vel_rate_hz_),
      std::bind(&VmrBaseBridge::publishCmdVelToSdk, this));
    RCLCPP_INFO(
      get_logger(),
      "VMR SDK speed control listens to %s at %.1f Hz",
      cmd_vel_topic_.c_str(),
      cmd_vel_rate_hz_);
  }
}

VmrBaseBridge::~VmrBaseBridge()
{
  if (cmd_vel_timer_) {
    cmd_vel_timer_->cancel();
    cmd_vel_timer_.reset();
  }
  cmd_vel_subscription_.reset();

  std::scoped_lock<std::mutex> lock(sdk_mutex_);
  if (sdk_initialized_) {
    if (sdk_ctrl_speed_enabled_) {
      VmrTwistInfo stop_twist{};
      VMR_setRobotTwist(sdk_handle_, stop_twist);
      VMR_enableSdkCtrlSpeed(sdk_handle_, false);
      sdk_ctrl_speed_enabled_ = false;
    }
    VMR_Handle_Destroy(sdk_handle_);
    sdk_initialized_ = false;
    sdk_handle_ = 0;
  }
  if (instance_ == this) {
    instance_ = nullptr;
  }
}

void VmrBaseBridge::declareParameters()
{
  sdk_config_file_ = declare_parameter<std::string>(
    "sdk_config_file", defaultPackagePath("config/vmr_sdk.ini"));
  step_move_service_name_ = declare_parameter<std::string>(
    "service.step_move_name", "/vmr_base_bridge/step_move");
  vector_move_service_name_ = declare_parameter<std::string>(
    "service.vector_move_name", "/vmr_base_bridge/vector_move");
  nav_target_service_name_ = declare_parameter<std::string>(
    "service.nav_target_name", "/vmr_base_bridge/nav_target");
  cancel_task_service_name_ = declare_parameter<std::string>(
    "service.cancel_task_name", "/vmr_base_bridge/cancel_task");
  query_task_status_service_name_ = declare_parameter<std::string>(
    "service.query_task_status_name", "/vmr_base_bridge/query_task_status");
  control_relay_service_name_ = declare_parameter<std::string>(
    "service.control_relay_name", "/vmr_base_bridge/control_relay");
  site_mapping_file_ = declare_parameter<std::string>(
    "navigation.site_mapping_file", defaultPackagePath("config/site_mapping.yaml"));
  laser_topic_ = declare_parameter<std::string>("topics.laser", "/vmr_base_bridge/laser/points");
  location_topic_ = declare_parameter<std::string>("topics.location", "/vmr_base_bridge/location");
  pose_topic_ = declare_parameter<std::string>("topics.pose", "/vmr_base_bridge/pose");
  odom_topic_ = declare_parameter<std::string>("topics.odom", "/vmr_base_bridge/odom");
  battery_topic_ = declare_parameter<std::string>("topics.battery", "/vmr_base_bridge/battery");
  imu_topic_ = declare_parameter<std::string>("topics.imu", "/vmr_base_bridge/imu");
  imu_pose_topic_ = declare_parameter<std::string>("topics.imu_pose", "/vmr_base_bridge/imu_pose");
  move_status_topic_ = declare_parameter<std::string>(
    "topics.move_status", "/vmr_base_bridge/move_status");
  timing_diagnostics_topic_ = declare_parameter<std::string>(
    "topics.timing_diagnostics", "/vmr_base_bridge/timing_diagnostics");
  cmd_vel_topic_ = declare_parameter<std::string>("topics.cmd_vel", "/cmd_vel");
  base_frame_ = declare_parameter<std::string>("frames.base_frame", "base_link");
  odom_frame_ = declare_parameter<std::string>("frames.odom_frame", "odom");
  map_frame_ = declare_parameter<std::string>("frames.map_frame", "map");
  qos_depth_ = declare_parameter<int>("publishers.qos_depth", 10);
  cmd_vel_enabled_ = declare_parameter<bool>("cmd_vel.enabled", false);
  cmd_vel_timeout_ = declare_parameter<double>("cmd_vel.timeout", 0.5);
  cmd_vel_rate_hz_ = declare_parameter<double>("cmd_vel.rate_hz", 30.0);
  cmd_vel_speed_factor_ = declare_parameter<double>("cmd_vel.speed_factor", 0.3);
  max_linear_x_ = declare_parameter<double>("cmd_vel.max_linear_x", 0.2);
  max_linear_y_ = declare_parameter<double>("cmd_vel.max_linear_y", 0.0);
  max_angular_z_ = declare_parameter<double>("cmd_vel.max_angular_z", 0.5);

  if (qos_depth_ <= 0) {
    throw std::runtime_error("publishers.qos_depth must be greater than 0");
  }
  if (cmd_vel_rate_hz_ <= 20.0) {
    throw std::runtime_error("cmd_vel.rate_hz must be greater than 20.0 for the VMR SDK");
  }
  if (cmd_vel_timeout_ <= 0.0) {
    throw std::runtime_error("cmd_vel.timeout must be greater than 0");
  }
  if (cmd_vel_speed_factor_ < 0.0 || cmd_vel_speed_factor_ > 1.0) {
    throw std::runtime_error("cmd_vel.speed_factor must be between 0.0 and 1.0");
  }
}

void VmrBaseBridge::initializeSdk()
{
  std::scoped_lock<std::mutex> lock(sdk_mutex_);

  const int init_result = VMR_init(sdk_config_file_.c_str());
  if (init_result != 0) {
    throw std::runtime_error("VMR_init failed with code " + std::to_string(init_result));
  }

  sdk_handle_ = VMR_Handle_Create();
  VMR_registerLaserCallback(sdk_handle_, &VmrBaseBridge::LaserCallbackThunk);
  VMR_registerLocationCallback(sdk_handle_, &VmrBaseBridge::LocationCallbackThunk);
  VMR_registerOdomCallback(sdk_handle_, &VmrBaseBridge::OdomCallbackThunk);
  VMR_registerBatteryCallback(sdk_handle_, &VmrBaseBridge::BatteryCallbackThunk);
  VMR_registerImuCallback(sdk_handle_, &VmrBaseBridge::ImuCallbackThunk);
  VMR_registerMoveStatusCallback(sdk_handle_, &VmrBaseBridge::MoveStatusCallbackThunk);
  sdk_initialized_ = true;

  RCLCPP_INFO(get_logger(), "VMR SDK initialized with config: %s", sdk_config_file_.c_str());
}

void VmrBaseBridge::loadSiteMappings()
{
  site_targets_.clear();

  const YAML::Node root = YAML::LoadFile(site_mapping_file_);
  const YAML::Node sites_node = root["sites"];
  if (!sites_node) {
    return;
  }
  if (!sites_node.IsMap()) {
    throw std::runtime_error("site_mapping.yaml 'sites' must be a map");
  }

  for (const auto & entry : sites_node) {
    const std::string site_name = entry.first.as<std::string>();
    const YAML::Node site_node = entry.second;

    SiteTarget site_target;
    site_target.x = site_node["x"].as<double>();
    site_target.y = site_node["y"].as<double>();
    site_target.theta = site_node["theta"].as<double>();

    site_targets_.emplace(site_name, std::move(site_target));
  }

  RCLCPP_INFO(
    get_logger(), "Loaded %zu site mappings from %s", site_targets_.size(),
    site_mapping_file_.c_str());
}

std_msgs::msg::Header VmrBaseBridge::makeHeader(const std::string & frame_id, uint64_t timestamp_ns) const
{
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = toBuiltinTime(timestamp_ns);
  return header;
}

sensor_msgs::msg::PointCloud2 VmrBaseBridge::buildPointCloud2(const VmrLaserScan & scan) const
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = makeHeader(scan.frame_id.empty() ? base_frame_ : scan.frame_id, scan.timestamp_ns);
  msg.height = 1;
  msg.width = static_cast<uint32_t>(scan.laser_scan.size());
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = sizeof(float) * kPointFieldCount;
  msg.row_step = msg.point_step * msg.width;

  msg.fields.resize(kPointFieldCount);
  const std::array<std::string, kPointFieldCount> field_names = {"x", "y", "z", "intensity"};
  for (size_t i = 0; i < field_names.size(); ++i) {
    msg.fields[i].name = field_names[i];
    msg.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }

  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < scan.laser_scan.size(); ++i) {
    const auto & point = scan.laser_scan[i];
    const float intensity = i < scan.intensities.size() ? scan.intensities[i] : 0.0f;
    uint8_t * row = msg.data.data() + i * msg.point_step;
    std::memcpy(row + 0 * sizeof(float), &point.x, sizeof(float));
    std::memcpy(row + 1 * sizeof(float), &point.y, sizeof(float));
    std::memcpy(row + 2 * sizeof(float), &point.z, sizeof(float));
    std::memcpy(row + 3 * sizeof(float), &intensity, sizeof(float));
  }

  return msg;
}

void VmrBaseBridge::LaserCallbackThunk(const VmrLaserScan & scan)
{
  if (instance_ != nullptr) {
    instance_->handleLaserScan(scan);
  }
}

void VmrBaseBridge::LocationCallbackThunk(const VmrLocationInfo & location)
{
  if (instance_ != nullptr) {
    instance_->handleLocation(location);
  }
}

void VmrBaseBridge::OdomCallbackThunk(const VmrOdomInfo & odom)
{
  if (instance_ != nullptr) {
    instance_->handleOdometry(odom);
  }
}

void VmrBaseBridge::BatteryCallbackThunk(const VmrBatteryInfo & battery)
{
  if (instance_ != nullptr) {
    instance_->handleBattery(battery);
  }
}

void VmrBaseBridge::ImuCallbackThunk(const VmrImuInfo & imu)
{
  if (instance_ != nullptr) {
    instance_->handleImu(imu);
  }
}

void VmrBaseBridge::MoveStatusCallbackThunk(const VmrMoveStatus & status)
{
  if (instance_ != nullptr) {
    instance_->handleMoveStatus(status);
  }
}

void VmrBaseBridge::handleLaserScan(const VmrLaserScan & scan)
{
  laser_publisher_->publish(buildPointCloud2(scan));
}

void VmrBaseBridge::handleLocation(const VmrLocationInfo & location)
{
  publishCallbackTiming("location_callback", location.timestamp_ns, location_timing_state_);

  vmr_base_bridge::msg::VmrLocation msg;
  msg.header = makeHeader(map_frame_, location.timestamp_ns);
  msg.x = location.x;
  msg.y = location.y;
  msg.theta = location.theta;
  msg.confidence = location.confidence;
  msg.status = location.status;
  location_publisher_->publish(msg);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = msg.header;
  pose_msg.pose.position.x = location.x;
  pose_msg.pose.position.y = location.y;
  pose_msg.pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, location.theta);
  pose_msg.pose.orientation.x = quaternion.x();
  pose_msg.pose.orientation.y = quaternion.y();
  pose_msg.pose.orientation.z = quaternion.z();
  pose_msg.pose.orientation.w = quaternion.w();
  pose_publisher_->publish(pose_msg);
}

void VmrBaseBridge::handleOdometry(const VmrOdomInfo & odom)
{
  publishCallbackTiming("odom_callback", odom.timestamp_ns, odom_timing_state_);

  nav_msgs::msg::Odometry msg;
  msg.header = makeHeader(odom_frame_, odom.timestamp_ns);
  msg.child_frame_id = base_frame_;
  msg.pose.pose.position.x = odom.x;
  msg.pose.pose.position.y = odom.y;
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, odom.theta);
  msg.pose.pose.orientation.x = quaternion.x();
  msg.pose.pose.orientation.y = quaternion.y();
  msg.pose.pose.orientation.z = quaternion.z();
  msg.pose.pose.orientation.w = quaternion.w();

  msg.twist.twist.linear.x = odom.vx;
  msg.twist.twist.linear.y = odom.vy;
  msg.twist.twist.angular.z = odom.w;
  odom_publisher_->publish(msg);
}

void VmrBaseBridge::handleBattery(const VmrBatteryInfo & battery)
{
  sensor_msgs::msg::BatteryState msg;
  msg.header = makeHeader(base_frame_, battery.timestamp_ns);
  msg.voltage = battery.voltage;
  msg.current = battery.current;
  msg.charge = battery.charge;
  msg.capacity = battery.capacity;
  msg.design_capacity = battery.design_capacity;
  msg.percentage = normalizePercentage(battery.percentage);
  msg.power_supply_status = toBatteryStatus(battery.power_supply_status);
  msg.serial_number = battery.serial_number;
  battery_publisher_->publish(msg);
}

void VmrBaseBridge::handleImu(const VmrImuInfo & imu)
{
  sensor_msgs::msg::Imu msg;
  msg.header = makeHeader(base_frame_, imu.timestamp_ns);
  msg.orientation.x = imu.orientation.x;
  msg.orientation.y = imu.orientation.y;
  msg.orientation.z = imu.orientation.z;
  msg.orientation.w = imu.orientation.w;
  msg.angular_velocity.x = imu.angular_velocity.x;
  msg.angular_velocity.y = imu.angular_velocity.y;
  msg.angular_velocity.z = imu.angular_velocity.z;
  msg.linear_acceleration.x = imu.linear_acceleration.x;
  msg.linear_acceleration.y = imu.linear_acceleration.y;
  msg.linear_acceleration.z = imu.linear_acceleration.z;
  imu_publisher_->publish(msg);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = msg.header;
  pose_msg.pose.orientation = msg.orientation;
  imu_pose_publisher_->publish(pose_msg);
}

void VmrBaseBridge::handleMoveStatus(const VmrMoveStatus & status)
{
  vmr_base_bridge::msg::VmrMoveStatus msg;
  msg.header = makeHeader(map_frame_, status.timestamp_ns);
  msg.move_state = static_cast<int32_t>(status.move_state);
  msg.remaining_time = status.remaining_time;
  msg.current_path.reserve(status.cur_path.size());

  for (const auto & pose : status.cur_path) {
    geometry_msgs::msg::Point point;
    point.x = pose.x;
    point.y = pose.y;
    point.z = 0.0;
    msg.current_path.push_back(point);
  }

  move_status_publisher_->publish(msg);
}

void VmrBaseBridge::handleStepMove(
  const std::shared_ptr<vmr_base_bridge::srv::StepMove::Request> request,
  std::shared_ptr<vmr_base_bridge::srv::StepMove::Response> response)
{
  std::scoped_lock<std::mutex> command_lock(command_mutex_);

  RCLCPP_INFO(
    get_logger(), "Received StepMove request: direction=%u, value=%.3f",
    request->direction, request->value);

  std::string error_message;
  const std::string task_id = submitStepMoveTask(*request, error_message);
  if (task_id.empty()) {
    response->success = false;
    response->task_id = "";
    response->message = error_message;
    RCLCPP_ERROR(get_logger(), "StepMove rejected: %s", error_message.c_str());
    return;
  }

  response->success = true;
  response->task_id = task_id;
  response->message = "StepMove task accepted, task_id=" + task_id;
  RCLCPP_INFO(get_logger(), "StepMove task accepted: task_id=%s", task_id.c_str());
}

void VmrBaseBridge::handleVectorMove(
  const std::shared_ptr<vmr_base_bridge::srv::VectorMove::Request> request,
  std::shared_ptr<vmr_base_bridge::srv::VectorMove::Response> response)
{
  std::scoped_lock<std::mutex> command_lock(command_mutex_);

  RCLCPP_INFO(
    get_logger(), "Received VectorMove request: distance=%.3f, angle=%.3f",
    request->distance, request->angle);

  std::string error_message;
  const std::string task_id = submitVectorMoveTask(request->distance, request->angle, error_message);
  if (task_id.empty()) {
    response->success = false;
    response->task_id = "";
    response->message = error_message;
    RCLCPP_ERROR(get_logger(), "VectorMove rejected: %s", error_message.c_str());
    return;
  }

  response->success = true;
  response->task_id = task_id;
  response->message = "VectorMove task accepted, task_id=" + task_id;
  RCLCPP_INFO(get_logger(), "VectorMove task accepted: task_id=%s", task_id.c_str());
}

void VmrBaseBridge::handleNavTarget(
  const std::shared_ptr<vmr_base_bridge::srv::NavTarget::Request> request,
  std::shared_ptr<vmr_base_bridge::srv::NavTarget::Response> response)
{
  std::scoped_lock<std::mutex> command_lock(command_mutex_);

  VmrPose target_pose;
  std::string error_message;
  if (!resolveNavTarget(*request, target_pose, error_message)) {
    response->success = false;
    response->task_id = "";
    response->message = error_message;
    RCLCPP_ERROR(get_logger(), "NavTarget rejected: %s", error_message.c_str());
    return;
  }

  if (request->site_name.empty()) {
    RCLCPP_INFO(
      get_logger(), "Received NavTarget request: x=%.3f, y=%.3f, theta=%.3f",
      target_pose.x, target_pose.y, target_pose.theta);
  } else {
    RCLCPP_INFO(
      get_logger(), "Received NavTarget request: site_name=%s, x=%.3f, y=%.3f, theta=%.3f",
      request->site_name.c_str(), target_pose.x, target_pose.y, target_pose.theta);
  }

  const std::string task_id = submitNavTargetTask(target_pose, error_message);
  if (task_id.empty()) {
    response->success = false;
    response->task_id = "";
    response->message = error_message;
    RCLCPP_ERROR(get_logger(), "NavTarget rejected: %s", error_message.c_str());
    return;
  }

  response->success = true;
  response->task_id = task_id;
  response->message = "NavTarget task accepted, task_id=" + task_id;
  RCLCPP_INFO(get_logger(), "NavTarget task accepted: task_id=%s", task_id.c_str());
}

void VmrBaseBridge::handleCancelTask(
  const std::shared_ptr<vmr_base_bridge::srv::CancelTask::Request> request,
  std::shared_ptr<vmr_base_bridge::srv::CancelTask::Response> response)
{
  std::scoped_lock<std::mutex> command_lock(command_mutex_);

  std::vector<std::string> task_ids;
  if (request->task_id.empty()) {
    task_ids = snapshotActiveTaskIds();
    RCLCPP_INFO(
      get_logger(), "Received CancelTask request for all active tasks: count=%zu",
      task_ids.size());
    if (task_ids.empty()) {
      response->success = true;
      response->message = "No active tasks to cancel";
      return;
    }
  } else {
    RCLCPP_INFO(
      get_logger(), "Received CancelTask request: task_id=%s",
      request->task_id.c_str());
    task_ids.push_back(request->task_id);
  }

  {
    std::scoped_lock<std::mutex> lock(sdk_mutex_);
    if (!sdk_initialized_) {
      response->success = false;
      response->message = "SDK is not initialized";
      return;
    }

    for (const auto & task_id : task_ids) {
      VMR_cancelTask(sdk_handle_, task_id.c_str());
      RCLCPP_INFO(get_logger(), "CancelTask request sent: task_id=%s", task_id.c_str());
    }
  }

  for (const auto & task_id : task_ids) {
    unregisterActiveTaskId(task_id);
  }

  response->success = true;
  if (request->task_id.empty()) {
    response->message = "Cancel requested for all active tasks, count=" +
      std::to_string(task_ids.size());
  } else {
    response->message = "Cancel requested for task_id=" + request->task_id;
  }
}

void VmrBaseBridge::handleQueryTaskStatus(
  const std::shared_ptr<vmr_base_bridge::srv::QueryTaskStatus::Request> request,
  std::shared_ptr<vmr_base_bridge::srv::QueryTaskStatus::Response> response)
{
  if (request->task_id.empty()) {
    response->success = false;
    response->message = "task_id is empty";
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received QueryTaskStatus request: task_id=%s",
    request->task_id.c_str());

  VmrTaskResult result;
  {
    std::scoped_lock<std::mutex> lock(sdk_mutex_);
    if (!sdk_initialized_) {
      response->success = false;
      response->message = "SDK is not initialized";
      return;
    }
    result = VMR_checkTaskStatus(sdk_handle_, request->task_id.c_str());
  }

  response->success = true;
  response->task_flag = result.task_flag;
  response->task_result = result.task_result;
  response->task_id_echo = result.task_id.empty() ? request->task_id : result.task_id;
  RCLCPP_INFO(
    get_logger(), "QueryTaskStatus result: task_id=%s, task_flag=%d, task_result=%d",
    response->task_id_echo.c_str(), response->task_flag, response->task_result);

  if (result.task_result == 0) {
    response->message = "Task completed, task_id=" + response->task_id_echo;
    unregisterActiveTaskId(response->task_id_echo);
    return;
  }

  if (result.task_result == -1) {
    response->message = "Task in progress, task_id=" + response->task_id_echo;
    return;
  }

  response->message = "Task failed, task_id=" + response->task_id_echo;
  unregisterActiveTaskId(response->task_id_echo);
}

void VmrBaseBridge::handleControlRelay(
  const std::shared_ptr<vmr_base_bridge::srv::ControlRelay::Request> request,
  std::shared_ptr<vmr_base_bridge::srv::ControlRelay::Response> response)
{
  std::scoped_lock<std::mutex> command_lock(command_mutex_);

  RCLCPP_INFO(
    get_logger(), "Received ControlRelay request: enable=%s",
    request->enable ? "true" : "false");

  std::string task_id;
  {
    std::scoped_lock<std::mutex> lock(sdk_mutex_);
    if (!sdk_initialized_) {
      response->success = false;
      response->task_id = "";
      response->message = "SDK is not initialized";
      return;
    }

    task_id = VMR_controlRelay(sdk_handle_, request->enable);
  }

  if (task_id.empty()) {
    response->success = false;
    response->task_id = "";
    response->message = "SDK rejected ControlRelay request";
    RCLCPP_ERROR(get_logger(), "ControlRelay rejected by SDK");
    return;
  }

  registerActiveTaskId(task_id);
  response->success = true;
  response->task_id = task_id;
  response->message = "ControlRelay task accepted, task_id=" + task_id;
  RCLCPP_INFO(get_logger(), "ControlRelay task accepted: task_id=%s", task_id.c_str());
}

void VmrBaseBridge::handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(cmd_vel_mutex_);
  last_cmd_vel_ = *msg;
  last_cmd_vel_time_ = now();
  has_cmd_vel_ = true;
  ++cmd_vel_received_count_;
}

void VmrBaseBridge::publishCmdVelToSdk()
{
  geometry_msgs::msg::Twist cmd;
  const auto current_time = now();
  bool has_fresh_cmd = false;
  {
    std::scoped_lock<std::mutex> lock(cmd_vel_mutex_);
    if (!has_cmd_vel_) {
      return;
    }
    const double age = (current_time - last_cmd_vel_time_).seconds();
    if (age <= cmd_vel_timeout_) {
      cmd = last_cmd_vel_;
      has_fresh_cmd = true;
    }
  }

  VmrTwistInfo twist{};
  twist.linear.x = clampValue(cmd.linear.x, max_linear_x_);
  twist.linear.y = clampValue(cmd.linear.y, max_linear_y_);
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = clampValue(cmd.angular.z, max_angular_z_);

  std::scoped_lock<std::mutex> lock(sdk_mutex_);
  if (!sdk_initialized_) {
    return;
  }

  const bool is_zero_cmd =
    std::fabs(twist.linear.x) < 1e-6 &&
    std::fabs(twist.linear.y) < 1e-6 &&
    std::fabs(twist.angular.z) < 1e-6;

  if (!sdk_ctrl_speed_enabled_ && (!has_fresh_cmd || is_zero_cmd)) {
    return;
  }

  if (!sdk_ctrl_speed_enabled_) {
    const auto factor_call_start = std::chrono::steady_clock::now();
    const int factor_result = VMR_setSpeedFactor(sdk_handle_, cmd_vel_speed_factor_);
    publishSdkCallTiming(
      "set_speed_factor",
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - factor_call_start).count());
    if (factor_result != 0) {
      RCLCPP_WARN(
        get_logger(),
        "VMR_setSpeedFactor(%.2f) returned %d",
        cmd_vel_speed_factor_,
        factor_result);
    }
    const auto enable_call_start = std::chrono::steady_clock::now();
    const std::string task_id = VMR_enableSdkCtrlSpeed(sdk_handle_, true);
    publishSdkCallTiming(
      "enable_sdk_ctrl_speed",
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - enable_call_start).count());
    if (task_id.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "VMR_enableSdkCtrlSpeed(true) was rejected");
      return;
    }
    sdk_ctrl_speed_enabled_ = true;
    RCLCPP_INFO(get_logger(), "VMR SDK speed control enabled, task_id=%s", task_id.c_str());
  }

  const auto call_start = std::chrono::steady_clock::now();
  VMR_setRobotTwist(sdk_handle_, twist);
  const double call_duration_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - call_start).count();
  publishSdkCallTiming("set_robot_twist", call_duration_sec);
  {
    std::scoped_lock<std::mutex> cmd_lock(cmd_vel_mutex_);
    last_sent_cmd_vel_.linear.x = twist.linear.x;
    last_sent_cmd_vel_.linear.y = twist.linear.y;
    last_sent_cmd_vel_.linear.z = 0.0;
    last_sent_cmd_vel_.angular.x = 0.0;
    last_sent_cmd_vel_.angular.y = 0.0;
    last_sent_cmd_vel_.angular.z = twist.angular.z;
    ++cmd_vel_sent_count_;
  }
}

void VmrBaseBridge::publishCallbackTiming(
  const std::string & event_name,
  uint64_t sdk_timestamp_ns,
  CallbackTimingState & state)
{
  const auto arrival_time = std::chrono::steady_clock::now();
  vmr_base_bridge::msg::VmrTimingDiagnostic diagnostic;
  diagnostic.header.stamp = now();
  diagnostic.header.frame_id = map_frame_;
  diagnostic.event_name = event_name;
  diagnostic.sdk_timestamp_ns = sdk_timestamp_ns;
  diagnostic.sdk_interval_sec = -1.0;
  diagnostic.arrival_interval_sec = -1.0;
  diagnostic.sdk_call_duration_sec = -1.0;

  {
    std::scoped_lock<std::mutex> lock(timing_diagnostics_mutex_);
    diagnostic.sequence = ++state.sequence;
    if (state.initialized) {
      diagnostic.arrival_interval_sec = std::chrono::duration<double>(
        arrival_time - state.last_arrival_time).count();
      diagnostic.duplicate_timestamp = sdk_timestamp_ns == state.last_sdk_timestamp_ns;
      diagnostic.timestamp_regressed = sdk_timestamp_ns < state.last_sdk_timestamp_ns;
      if (!diagnostic.timestamp_regressed) {
        diagnostic.sdk_interval_sec = static_cast<double>(
          sdk_timestamp_ns - state.last_sdk_timestamp_ns) / 1e9;
      }
    }
    state.initialized = true;
    state.last_sdk_timestamp_ns = sdk_timestamp_ns;
    state.last_arrival_time = arrival_time;
  }

  timing_diagnostics_publisher_->publish(diagnostic);
}

void VmrBaseBridge::publishSdkCallTiming(
  const std::string & event_name,
  double call_duration_sec)
{
  vmr_base_bridge::msg::VmrTimingDiagnostic diagnostic;
  diagnostic.header.stamp = now();
  diagnostic.header.frame_id = base_frame_;
  diagnostic.event_name = event_name;
  diagnostic.sdk_interval_sec = -1.0;
  diagnostic.arrival_interval_sec = -1.0;
  diagnostic.sdk_call_duration_sec = call_duration_sec;

  {
    std::scoped_lock<std::mutex> lock(timing_diagnostics_mutex_);
    diagnostic.sequence = ++sdk_call_sequence_;
  }

  timing_diagnostics_publisher_->publish(diagnostic);
}

void VmrBaseBridge::logCmdVelDiagnostics(const rclcpp::Time & current_time)
{
  if ((current_time - last_cmd_vel_diag_time_).seconds() < 1.0) {
    return;
  }

  geometry_msgs::msg::Twist last_received;
  geometry_msgs::msg::Twist last_sent;
  size_t received_count = 0;
  size_t sent_count = 0;
  double cmd_age = 0.0;
  bool has_cmd = false;
  {
    std::scoped_lock<std::mutex> lock(cmd_vel_mutex_);
    last_received = last_cmd_vel_;
    last_sent = last_sent_cmd_vel_;
    received_count = cmd_vel_received_count_;
    sent_count = cmd_vel_sent_count_;
    has_cmd = has_cmd_vel_;
    if (has_cmd) {
      cmd_age = (current_time - last_cmd_vel_time_).seconds();
    }
  }

  const double elapsed = (current_time - last_cmd_vel_diag_time_).seconds();
  const size_t received_delta = received_count - last_diag_received_count_;
  const size_t sent_delta = sent_count - last_diag_sent_count_;
  const double received_hz = received_delta / elapsed;
  const double sent_hz = sent_delta / elapsed;

  RCLCPP_INFO(
    get_logger(),
    "cmd_vel diag: recv=%.1f Hz sent=%.1f Hz age=%.2fs enabled=%s "
    "last_recv[x=%.3f,z=%.3f] last_sent[x=%.3f,z=%.3f]",
    received_hz,
    sent_hz,
    cmd_age,
    sdk_ctrl_speed_enabled_ ? "true" : "false",
    last_received.linear.x,
    last_received.angular.z,
    last_sent.linear.x,
    last_sent.angular.z);

  last_cmd_vel_diag_time_ = current_time;
  last_diag_received_count_ = received_count;
  last_diag_sent_count_ = sent_count;
}

bool VmrBaseBridge::resolveNavTarget(
  const vmr_base_bridge::srv::NavTarget::Request & request,
  VmrPose & target_pose,
  std::string & error_message) const
{
  if (!request.site_name.empty()) {
    const auto it = site_targets_.find(request.site_name);
    if (it == site_targets_.end()) {
      error_message = "Unknown site name: " + request.site_name;
      return false;
    }

    target_pose.x = it->second.x;
    target_pose.y = it->second.y;
    target_pose.theta = it->second.theta;
    return true;
  }

  target_pose.x = request.x;
  target_pose.y = request.y;
  target_pose.theta = request.theta;
  return true;
}

std::string VmrBaseBridge::submitStepMoveTask(
  const vmr_base_bridge::srv::StepMove::Request & request,
  std::string & error_message)
{
  if (request.value <= 0.0) {
    error_message = "value must be greater than 0";
    return {};
  }

  switch (request.direction) {
    case vmr_base_bridge::srv::StepMove::Request::DIR_FORWARD:
      return submitVectorMoveTask(request.value, 0.0, error_message);
    case vmr_base_bridge::srv::StepMove::Request::DIR_BACKWARD:
      return submitVectorMoveTask(request.value, M_PI, error_message);
    case vmr_base_bridge::srv::StepMove::Request::DIR_LEFT:
      return submitVectorMoveTask(request.value, M_PI / 2.0, error_message);
    case vmr_base_bridge::srv::StepMove::Request::DIR_RIGHT:
      return submitVectorMoveTask(request.value, -M_PI / 2.0, error_message);
    case vmr_base_bridge::srv::StepMove::Request::DIR_ROTATE_CW:
      break;
    case vmr_base_bridge::srv::StepMove::Request::DIR_ROTATE_CCW:
      break;
    default:
      error_message = "unsupported direction: " + std::to_string(request.direction);
      return {};
  }

  std::scoped_lock<std::mutex> lock(sdk_mutex_);
  if (!sdk_initialized_) {
    error_message = "SDK is not initialized";
    return {};
  }

  std::string task_id;
  switch (request.direction) {
    case vmr_base_bridge::srv::StepMove::Request::DIR_ROTATE_CW:
      task_id = VMR_rotateInPlace(
        sdk_handle_, static_cast<float>(-request.value * kRadiansToDegrees));
      break;
    case vmr_base_bridge::srv::StepMove::Request::DIR_ROTATE_CCW:
      task_id = VMR_rotateInPlace(
        sdk_handle_, static_cast<float>(request.value * kRadiansToDegrees));
      break;
    default:
      error_message = "unsupported direction: " + std::to_string(request.direction);
      return {};
  }

  if (task_id.empty()) {
    error_message = "SDK rejected StepMove request";
  } else {
    registerActiveTaskId(task_id);
  }
  return task_id;
}

std::string VmrBaseBridge::submitVectorMoveTask(
  double distance,
  double angle,
  std::string & error_message)
{
  if (distance <= 0.0) {
    error_message = "distance must be greater than 0";
    return {};
  }

  std::scoped_lock<std::mutex> lock(sdk_mutex_);
  if (!sdk_initialized_) {
    error_message = "SDK is not initialized";
    return {};
  }

  std::string task_id = VMR_moveRelative(
    sdk_handle_, static_cast<float>(distance), static_cast<float>(angle * kRadiansToDegrees));
  if (task_id.empty()) {
    error_message = "SDK rejected VectorMove request";
  } else {
    registerActiveTaskId(task_id);
  }
  return task_id;
}

std::string VmrBaseBridge::submitNavTargetTask(
  const VmrPose & target_pose,
  std::string & error_message)
{
  std::scoped_lock<std::mutex> lock(sdk_mutex_);
  if (!sdk_initialized_) {
    error_message = "SDK is not initialized";
    return {};
  }

  VmrNavTask task;
  task.target_pose = target_pose;

  std::string task_id = VMR_moveTasks(sdk_handle_, task);
  if (task_id.empty()) {
    error_message = "SDK rejected NavTarget request";
  } else {
    registerActiveTaskId(task_id);
  }
  return task_id;
}

void VmrBaseBridge::registerActiveTaskId(const std::string & task_id)
{
  if (task_id.empty()) {
    return;
  }

  std::scoped_lock<std::mutex> lock(active_task_ids_mutex_);
  if (std::find(active_task_ids_.begin(), active_task_ids_.end(), task_id) == active_task_ids_.end()) {
    active_task_ids_.push_back(task_id);
  }
}

void VmrBaseBridge::unregisterActiveTaskId(const std::string & task_id)
{
  if (task_id.empty()) {
    return;
  }

  std::scoped_lock<std::mutex> lock(active_task_ids_mutex_);
  active_task_ids_.erase(
    std::remove(active_task_ids_.begin(), active_task_ids_.end(), task_id),
    active_task_ids_.end());
}

std::vector<std::string> VmrBaseBridge::snapshotActiveTaskIds()
{
  std::scoped_lock<std::mutex> lock(active_task_ids_mutex_);
  return active_task_ids_;
}

}  // namespace vmr_base_bridge
