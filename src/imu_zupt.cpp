#include <chrono>
#include <cmath>
#include "imu_zupt/imu_zupt.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

namespace filter{    

    ImuZupt::ImuZupt(const rclcpp::NodeOptions & options) : Node("imu_zupt", options) {

        source_topic_odom = declare_parameter<std::string>("topics.locomotion", "/locomotion/odom");
        source_topic_imu = declare_parameter<std::string>("topics.imu", "/imu/data");
        dest_topic = declare_parameter<std::string>("topics.output", "/imu/data_zupt");
        err_topic = declare_parameter<std::string>("topics.error", "/imu/yaw_err");
        status_topic = declare_parameter<std::string>("topics.zero_velocity_detected", "/imu_zupt/active");
        publish_err = declare_parameter<bool>("error.publish", true);
        use_degree = declare_parameter<bool>("error.in_degrees", true);
        override_covariance = declare_parameter<bool>("covariance.override", true);
        covariance = declare_parameter<double>("covariance.value", 0.001);
        publish_status = declare_parameter<bool>("zero_velocity_detection.publish", true);
        wait_time = declare_parameter<double>("zero_velocity_detection.seconds", 2.0);

        loco_subs_ = create_subscription<nav_msgs::msg::Odometry>(source_topic_odom, 1, std::bind(&ImuZupt::loco_callback, this, _1));
        imu_subs_ =  create_subscription<sensor_msgs::msg::Imu>(source_topic_imu, 1, std::bind(&ImuZupt::imu_callback, this, _1));
        zupt_publ_ = create_publisher<sensor_msgs::msg::Imu>(dest_topic, 1);
        err_publ_ = create_publisher<std_msgs::msg::Float64>(err_topic, 1);
        status_publ_ = create_publisher<std_msgs::msg::Bool>(status_topic, 1);
    }

    void ImuZupt::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        if (!active) return;
    	std::unique_ptr<sensor_msgs::msg::Imu> pkt_imu_zupt(new sensor_msgs::msg::Imu);
        pkt_imu_zupt->header = imu_msg->header;

        tf2::fromMsg(imu_msg->orientation, q1);
        tf2::Matrix3x3 m(q1);
        m.getRPY(last_roll, last_pitch, last_yaw);

        zero_velocity_detected = (rclcpp::Time(imu_msg->header.stamp).seconds() - last_motion_time.seconds()) > wait_time;
        if(zero_velocity_detected) {
            yaw_error += last_yaw - prev_yaw;
        }

        q2.setRPY(last_roll, last_pitch, last_yaw - yaw_error);
        pkt_imu_zupt->orientation = tf2::toMsg(q2);
        prev_yaw = last_yaw;

        if(override_covariance){
            pkt_imu_zupt->orientation_covariance[0] = covariance;
            pkt_imu_zupt->orientation_covariance[4] = covariance;
            pkt_imu_zupt->orientation_covariance[8] = covariance;
        }

        zupt_publ_->publish(std::move(pkt_imu_zupt));

        if(publish_err){
            std::unique_ptr<std_msgs::msg::Float64> errore_yaw(new std_msgs::msg::Float64);
            errore_yaw->data = yaw_error;
            if(use_degree){
                errore_yaw->data *=  180 / M_PI;
            }
            err_publ_->publish(std::move(errore_yaw));
        }
        
        if(publish_status){
            std::unique_ptr<std_msgs::msg::Bool> active_msg(new std_msgs::msg::Bool);
            active_msg->data = zero_velocity_detected;
            status_publ_->publish(std::move(active_msg));
        }
    }

    void ImuZupt::loco_callback(const nav_msgs::msg::Odometry::SharedPtr loco_msg)
    {
        if(std::abs(loco_msg->twist.twist.linear.x) != 0.0 || std::abs(loco_msg->twist.twist.angular.z) != 0.0){
            last_motion_time = loco_msg->header.stamp;
        }
        active = true;
    }
  
} //namespace filter

