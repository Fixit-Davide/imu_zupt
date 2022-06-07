#ifndef ODOMETRY_SUBSCRIBER
#define ODOMETRY_SUBSCRIBER

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"


namespace filter {

    class ImuZupt : public rclcpp::Node {
        public:
        
        ImuZupt(const rclcpp::NodeOptions & options); //Constructor
        
        ~ImuZupt(); //Desctructor
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr loco_subs_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subs_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_;
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
        void loco_callback(const nav_msgs::msg::Odometry::SharedPtr loco_msg);

        rclcpp::Time time_stamp;
        rclcpp::Time prev_time;
        tf2::Quaternion q = {0,0,0,1};
        tf2::Quaternion last_valid = {0,0,0,1};
        double last_yaw;
        double last_roll;
        double last_pitch;
        double yaw_error = 0;
        double prev_yaw = 0;
        double prev_2_yaw = 0;
        double yaw; 
        bool active = false;
    
        private:

        std::string source_topic_imu;
        std::string source_topic_odom;
        std::string dest_topic;
        std::string err_topic;
        bool publish_err;
        bool use_degree;
        bool covariance_pub;
    };
} //namespace filter

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(filter::ImuZupt)

#endif //ODOMETRY_SUBSCRIBER