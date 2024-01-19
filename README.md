# Zero-Velocity Update Algorithm: (ZUPT)

[![Ubuntu 22.04 Iron Build](https://github.com/Fixit-Davide/imu_zupt/actions/workflows/iron.yaml/badge.svg?branch=iron)](https://github.com/Fixit-Davide/imu_zupt/actions/workflows/iron.yaml)
[![Ubuntu 22.04 Rolling Build](https://github.com/Fixit-Davide/imu_zupt/actions/workflows/rolling.yaml/badge.svg?branch=iron)](https://github.com/Fixit-Davide/imu_zupt/actions/workflows/rolling.yaml)

A simple Zero-velocity update algorithm which provides accurate state information in order to maintain the reliability of the measurements incoming from the inertial navigation system once the stationary conditions are satisfied.
The cumulative error while stationary due to the movement of the Earth on the Yaw angle is estimated mathematically and subsequently removed. The filtered measurements are then published through a ROS topic.

## Parametrization:
- topics:
    - imu: Input topic for the IMU data. (sensor_msgs::msg::Imu)
    - input_status: Input topic for the system's status. (std_msgs::msg::Bool)
    - filter_status: Output topic that publish the filter's state. (std_msgs::msg::Bool)
    - output: output topic for the filtered measurements. (sensor_msgs::msg::Imu)
- zero_velocity_detection: 
    - seconds: time frame (in sec) which the algorithm will wait to establish if the stationary conditions are met. (double) 
    - publish: to publish whenever the ZUPT is active or not. (bool)
- covariance: 
    - override: if the covariance has to be overwritten (default: true). (bool)
    - value: set a fixed covariance value to add to the filtered measurements. (double)
- error:  
    - publish: choose if the calculated error has to be published. (bool)
    - in_degrees: the Iaw error can be published in degrees and not in radians. (bool) 
