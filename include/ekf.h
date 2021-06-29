#ifndef EKF_
#define EKF_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <array>
#include <Eigen/dense>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class EKF
{
public:
    EKF(ros::NodeHandle* nodehandle);
    
private:
    ros::NodeHandle nh_;

    nav_msgs::Odometry ekf_state_msg_;

    Eigen::Matrix<float, 5, 1> current_state_;
      
    //imu variables
    std::array<float, 4> imu_orientation_;
    std::array<float, 9> imu_orientation_covariance_;
    std::array<float, 3> imu_angular_velocity_;
    std::array<float, 9> imu_angular_velocity_covariance_;
    std::array<float, 3> imu_linear_acceleration_;
    std::array<float, 9> imu_linear_acceleration_covariance_;

    //gps variables
    std::array<float, 3> gps_position_;
    std::array<float, 3> gps_position_covariance_;
    std::array<float, 3> gps_velocity_;
    std::array<float, 3> gps_velocity_covariance_;

    //wheel encoder variables
    std::array<float, 3> wheel_encoder_position_;
    std::array<float, 3> wheel_encoder_position_covariance_;
    std::array<float, 3> wheel_encoder_velocity_;
    std::array<float, 3> wheel_encoder_velocity_covariance_;

    //ekf state variables
    std::array<float, 3> ekf_position_;
    std::array<float, 3> ekf_position_covariance_;
    std::array<float, 3> ekf_velocity_;
    std::array<float, 3> ekf_velocity_covariance_;

    //subscribers and publishers
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber wheel_encoder_sub;
    ros::Publisher  ekf_state_pub_;

    //callbacks
    void imuCallback(const std_msgs::Float32& message_holder);
    void gpsCallback(const std_msgs::Float32& message_holder);
    void wheelEncoderCallback(const std_msgs::Float32& message_holder);

    //methods
    void getImuData();
    void getGPSData();
    void getWheelencoderData();
    void filterData();
    void publishData();

#endif