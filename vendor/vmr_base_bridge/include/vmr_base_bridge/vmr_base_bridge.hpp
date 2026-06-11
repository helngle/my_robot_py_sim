#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include "vmr_base_bridge/msg/vmr_location.hpp"
#include "vmr_base_bridge/msg/vmr_move_status.hpp"
#include "vmr_base_bridge/srv/cancel_task.hpp"
#include "vmr_base_bridge/srv/control_relay.hpp"
#include "vmr_base_bridge/srv/nav_target.hpp"
#include "vmr_base_bridge/srv/query_task_status.hpp"
#include "vmr_base_bridge/srv/step_move.hpp"
#include "vmr_base_bridge/srv/vector_move.hpp"

#include "VmrSDKApi.h"

namespace vmr_base_bridge
{

struct SiteTarget
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

class VmrBaseBridge : public rclcpp::Node
{
public:
  explicit VmrBaseBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VmrBaseBridge() override;

private:
  static void LaserCallbackThunk(const VmrLaserScan & scan);
  static void LocationCallbackThunk(const VmrLocationInfo & location);
  static void OdomCallbackThunk(const VmrOdomInfo & odom);
  static void BatteryCallbackThunk(const VmrBatteryInfo & battery);
  static void ImuCallbackThunk(const VmrImuInfo & imu);
  static void MoveStatusCallbackThunk(const VmrMoveStatus & status);

  void handleLaserScan(const VmrLaserScan & scan);
  void handleLocation(const VmrLocationInfo & location);
  void handleOdometry(const VmrOdomInfo & odom);
  void handleBattery(const VmrBatteryInfo & battery);
  void handleImu(const VmrImuInfo & imu);
  void handleMoveStatus(const VmrMoveStatus & status);

  void handleStepMove(
    const std::shared_ptr<vmr_base_bridge::srv::StepMove::Request> request,
    std::shared_ptr<vmr_base_bridge::srv::StepMove::Response> response);
  void handleVectorMove(
    const std::shared_ptr<vmr_base_bridge::srv::VectorMove::Request> request,
    std::shared_ptr<vmr_base_bridge::srv::VectorMove::Response> response);
  void handleNavTarget(
    const std::shared_ptr<vmr_base_bridge::srv::NavTarget::Request> request,
    std::shared_ptr<vmr_base_bridge::srv::NavTarget::Response> response);
  void handleCancelTask(
    const std::shared_ptr<vmr_base_bridge::srv::CancelTask::Request> request,
    std::shared_ptr<vmr_base_bridge::srv::CancelTask::Response> response);
  void handleQueryTaskStatus(
    const std::shared_ptr<vmr_base_bridge::srv::QueryTaskStatus::Request> request,
    std::shared_ptr<vmr_base_bridge::srv::QueryTaskStatus::Response> response);
  void handleControlRelay(
    const std::shared_ptr<vmr_base_bridge::srv::ControlRelay::Request> request,
    std::shared_ptr<vmr_base_bridge::srv::ControlRelay::Response> response);
  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishCmdVelToSdk();
  void logCmdVelDiagnostics(const rclcpp::Time & current_time);

  void declareParameters();
  void initializeSdk();
  void loadSiteMappings();
  std_msgs::msg::Header makeHeader(const std::string & frame_id, uint64_t timestamp_ns) const;
  sensor_msgs::msg::PointCloud2 buildPointCloud2(const VmrLaserScan & scan) const;
  bool resolveNavTarget(
    const vmr_base_bridge::srv::NavTarget::Request & request,
    VmrPose & target_pose,
    std::string & error_message) const;
  std::string submitStepMoveTask(
    const vmr_base_bridge::srv::StepMove::Request & request,
    std::string & error_message);
  std::string submitVectorMoveTask(double distance, double angle, std::string & error_message);
  std::string submitNavTargetTask(
    const VmrPose & target_pose,
    std::string & error_message);
  void registerActiveTaskId(const std::string & task_id);
  void unregisterActiveTaskId(const std::string & task_id);
  std::vector<std::string> snapshotActiveTaskIds();

  static VmrBaseBridge * instance_;

  std::mutex sdk_mutex_;
  std::mutex command_mutex_;
  std::mutex active_task_ids_mutex_;
  bool sdk_initialized_{false};
  VMR_Handle sdk_handle_{0};

  std::string sdk_config_file_;
  std::string step_move_service_name_;
  std::string vector_move_service_name_;
  std::string nav_target_service_name_;
  std::string cancel_task_service_name_;
  std::string query_task_status_service_name_;
  std::string control_relay_service_name_;
  std::string site_mapping_file_;
  std::string laser_topic_;
  std::string location_topic_;
  std::string pose_topic_;
  std::string odom_topic_;
  std::string battery_topic_;
  std::string imu_topic_;
  std::string imu_pose_topic_;
  std::string move_status_topic_;
  std::string cmd_vel_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;
  int qos_depth_{10};
  bool cmd_vel_enabled_{false};
  bool sdk_ctrl_speed_enabled_{false};
  bool has_cmd_vel_{false};
  double cmd_vel_timeout_{0.5};
  double cmd_vel_rate_hz_{30.0};
  double cmd_vel_speed_factor_{0.3};
  double max_linear_x_{0.2};
  double max_linear_y_{0.0};
  double max_angular_z_{0.5};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laser_publisher_;
  rclcpp::Publisher<vmr_base_bridge::msg::VmrLocation>::SharedPtr location_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_publisher_;
  rclcpp::Publisher<vmr_base_bridge::msg::VmrMoveStatus>::SharedPtr move_status_publisher_;
  rclcpp::Service<vmr_base_bridge::srv::StepMove>::SharedPtr step_move_service_;
  rclcpp::Service<vmr_base_bridge::srv::VectorMove>::SharedPtr vector_move_service_;
  rclcpp::Service<vmr_base_bridge::srv::NavTarget>::SharedPtr nav_target_service_;
  rclcpp::Service<vmr_base_bridge::srv::CancelTask>::SharedPtr cancel_task_service_;
  rclcpp::Service<vmr_base_bridge::srv::QueryTaskStatus>::SharedPtr query_task_status_service_;
  rclcpp::Service<vmr_base_bridge::srv::ControlRelay>::SharedPtr control_relay_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
  std::mutex cmd_vel_mutex_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  geometry_msgs::msg::Twist last_sent_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_cmd_vel_diag_time_;
  size_t cmd_vel_received_count_{0};
  size_t cmd_vel_sent_count_{0};
  size_t last_diag_received_count_{0};
  size_t last_diag_sent_count_{0};
  std::unordered_map<std::string, SiteTarget> site_targets_;
  std::vector<std::string> active_task_ids_;
};

}  // namespace vmr_base_bridge
