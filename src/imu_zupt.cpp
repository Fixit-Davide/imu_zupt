#include <chrono>
#include<cmath>
#include "imu_zupt/imu_zupt.hpp"

using std::placeholders::_1;
#define COV 0.0010000000474974513

namespace filter{    

    ImuZupt::ImuZupt(const rclcpp::NodeOptions & options) : Node("imu_zupt", options) {

        source_topic_odom = declare_parameter<std::string>("source_topic.locomotion_topic", "/locomotion/odom");
        source_topic_imu = declare_parameter<std::string>("source_topic.imu_topic", "/imu/data");
        dest_topic = declare_parameter<std::string>("dest_topic", "/imu/data_zupt");
        err_topic = declare_parameter<std::string>("err_topic", "/imu/yaw_err");
        publish_err = declare_parameter<bool>("publish_err", true);
        use_degree = declare_parameter<bool>("use_degree", true);
        covariance_pub = declare_parameter<bool>("pub_covariance", true);
        
        loco_subs_ = create_subscription<nav_msgs::msg::Odometry>(source_topic_odom, 1, std::bind(&ImuZupt::loco_callback, this, _1));
        imu_subs_ =  create_subscription<sensor_msgs::msg::Imu>(source_topic_imu, 1, std::bind(&ImuZupt::imu_callback, this, _1));
        publisher_ = create_publisher<sensor_msgs::msg::Imu>(dest_topic, 1);
        err_ = create_publisher<std_msgs::msg::Float64>(err_topic, 1);
    }

    void ImuZupt::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        if (!active) return;
        sensor_msgs::msg::Imu msg = *imu_msg;
        std_msgs::msg::Float64 errore_yaw;
        tf2::fromMsg(msg.orientation,q);
        prev_time = msg.header.stamp;
        auto header = msg.header;
        tf2::Matrix3x3 m(q);
        m.getRPY(last_roll, last_pitch, last_yaw);
        if((prev_time.seconds() - time_stamp.seconds()) > 1.0) {
            yaw_error += last_yaw - prev_yaw;
        }
        last_valid.setRPY(last_roll, last_pitch, last_yaw - yaw_error);
        msg.orientation = tf2::toMsg(last_valid);
        prev_yaw = last_yaw;
        if(use_degree){
            errore_yaw.data = yaw_error * 180 / M_PI;
        } else {
            errore_yaw.data = yaw_error;
        }
        if(covariance_pub){
            msg.orientation_covariance[0] = COV;
            msg.orientation_covariance[4] = COV;
            msg.orientation_covariance[8] = COV;
        }
        msg.header = header;
        publisher_->publish(msg);
        if(publish_err){
            err_->publish(errore_yaw);
        }
    }

    void ImuZupt::loco_callback(const nav_msgs::msg::Odometry::SharedPtr loco_msg)
    {
        if(std::abs(loco_msg->twist.twist.linear.x) > 1e-3 || std::abs(loco_msg->twist.twist.angular.z) > 1e-3){
            time_stamp = loco_msg->header.stamp;
        }
        active = true;
    }



ImuZupt::~ImuZupt(){}
  
} //namespace filter

