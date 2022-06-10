#ifndef ROS2__IMU_ZUPT
#define ROS2__IMU_ZUPT

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

namespace filter {

    class ImuZupt : public rclcpp::Node {
        public:
        
        ImuZupt(const rclcpp::NodeOptions & options);
        
        ~ImuZupt(){};

        private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr loco_subs_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subs_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr zupt_publ_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_publ_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publ_;

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
        void loco_callback(const nav_msgs::msg::Odometry::SharedPtr loco_msg);

        rclcpp::Time last_motion_time;
        tf2::Quaternion q1 = {0,0,0,1};
        tf2::Quaternion q2 = {0,0,0,1};
        double last_roll, last_pitch, last_yaw;
        double yaw_error = 0;
        double prev_yaw = 0;
        bool active = false;
        bool zero_velocity_detected = false;

        std::string source_topic_imu;
        std::string source_topic_odom;
        std::string dest_topic;
        std::string err_topic;
        std::string status_topic;
        bool publish_status;
        bool publish_err;
        bool use_degree;
        bool override_covariance;
        double wait_time;
        double covariance;
    };
} //namespace filter

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(filter::ImuZupt)

#endif //ROS2__IMU_ZUPT