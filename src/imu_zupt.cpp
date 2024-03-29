/*
Copyright 2023 Thales Alenia Space
*/

#include "imu_zupt/imu_zupt.hpp"

using std::placeholders::_1;

namespace filter
{

ImuZupt::ImuZupt(const rclcpp::NodeOptions & options)
: Node("imu_zupt", options)
{
  source_topic_imu = declare_parameter<std::string>("topics.imu", "/imu/data");
  dest_topic = declare_parameter<std::string>("topics.output", "/imu/data_zupt");
  err_topic = declare_parameter<std::string>("topics.error", "/imu/yaw_err");
  filter_status_topic =
    declare_parameter<std::string>("topics.filter_status", "/imu_zupt/active");
  publish_err = declare_parameter<bool>("error.publish", true);
  use_degree = declare_parameter<bool>("error.in_degrees", true);
  override_covariance = declare_parameter<bool>("covariance.override", true);
  covariance = declare_parameter<double>("covariance.value", 0.001);
  publish_status = declare_parameter<bool>("zero_velocity_detection.publish", true);
  input_status_topic = declare_parameter<std::string>("topics.input_status", "/input_status");

  rover_status_subs_ =
    create_subscription<std_msgs::msg::Bool>(
    input_status_topic, 1,
    std::bind(&ImuZupt::status_callback, this, _1));
  imu_subs_ =
    create_subscription<sensor_msgs::msg::Imu>(
    source_topic_imu, 1,
    std::bind(&ImuZupt::imu_callback, this, _1));
  zupt_publ_ = create_publisher<sensor_msgs::msg::Imu>(dest_topic, 1);
  err_publ_ = create_publisher<std_msgs::msg::Float64>(err_topic, 1);
  status_publ_ = create_publisher<std_msgs::msg::Bool>(filter_status_topic, 1);
}

void ImuZupt::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  if (!active) {return;}
  std::unique_ptr<sensor_msgs::msg::Imu> pkt_imu_zupt(new sensor_msgs::msg::Imu);
  *pkt_imu_zupt = *imu_msg;

  tf2::fromMsg(imu_msg->orientation, q1);
  tf2::Matrix3x3 m(q1);
  m.getRPY(last_roll, last_pitch, last_yaw);

  yaw_error += last_yaw - prev_yaw;

  q2.setRPY(last_roll, last_pitch, last_yaw - yaw_error);
  pkt_imu_zupt->orientation = tf2::toMsg(q2);
  prev_yaw = last_yaw;

  if (override_covariance) {
    pkt_imu_zupt->orientation_covariance[0] = covariance;
    pkt_imu_zupt->orientation_covariance[4] = covariance;
    pkt_imu_zupt->orientation_covariance[8] = covariance;
  }

  zupt_publ_->publish(std::move(pkt_imu_zupt));

  if (publish_err) {
    std::unique_ptr<std_msgs::msg::Float64> errore_yaw(new std_msgs::msg::Float64);
    errore_yaw->data = yaw_error;
    if (use_degree) {
      errore_yaw->data *= 180 / M_PI;
    }
    err_publ_->publish(std::move(errore_yaw));
  }

  if (publish_status) {
    std::unique_ptr<std_msgs::msg::Bool> active_msg(new std_msgs::msg::Bool);
    active_msg->data = active;
    status_publ_->publish(std::move(active_msg));
  }
}

void ImuZupt::status_callback(const std_msgs::msg::Bool::SharedPtr status_msg)
{
  active = status_msg->data;
}

}  // namespace filter
