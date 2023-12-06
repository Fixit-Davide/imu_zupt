/*
Copyright 2023 Thales Alenia Space
*/

#ifndef IMU_ZUPT__IMU_ZUPT_HPP_
#define IMU_ZUPT__IMU_ZUPT_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>

#include <chrono>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace filter
{

class ImuZupt : public rclcpp::Node
{
public:
  explicit ImuZupt(const rclcpp::NodeOptions & options);

  ~ImuZupt() {}

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subs_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr zupt_publ_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_publ_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publ_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rover_status_subs_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void status_callback(const std_msgs::msg::Bool::SharedPtr status_msg);

  tf2::Quaternion q1 = {0, 0, 0, 1};
  tf2::Quaternion q2 = {0, 0, 0, 1};
  double last_roll, last_pitch, last_yaw;
  double yaw_error = 0;
  double prev_yaw = 0;
  bool active = false;

  std::string source_topic_imu;
  std::string dest_topic;
  std::string err_topic;
  std::string status_topic;
  std::string rover_status_topic;
  bool publish_status;
  bool publish_err;
  bool use_degree;
  bool override_covariance;
  double wait_time;
  double covariance;
};
}  // namespace filter

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(filter::ImuZupt)

#endif  // IMU_ZUPT__IMU_ZUPT_HPP_
