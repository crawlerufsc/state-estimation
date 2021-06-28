#ifndef EKF_
#define EKF_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class EKF
{
public:
    EKF(ros::NodeHandle* nodehandle);
    
private:
    ros::NodeHandle nh_;
    
    //subscribers and publishers
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber wheel_encoder_sub;
    ros::Publisher  ekf_state_pub_;
    
    //imu variables
    std::vector<float> quaternion_orientation_;
    std::vector<float> quaternion_orientation_covariance_;
    std::vector<float> angular_velocity_;
    std::vector<float> angular_velocity_covariance_;
    std::vector<float> linear_velocity_;
    std::vector<float> linear_velocity_covariance_;

    //gps variables
    std::vector<float> gps_position_;
    std::vector<float> gps_position_covariance_;
    std::vector<float> gps_velocity_;
    std::vector<float> gps_velocity_covariance_;

    //wheel encoder variable
    std::vector<float> wheel_velocity_;
    std::vector<float> wheel_velocity_covariance_;

    //callbacks
    void imuCallback(const std_msgs::Float32& message_holder);
    void gpsCallback(const std_msgs::Float32& message_holder);
    void wheelEncoderCallback(const std_msgs::Float32& message_holder);

    //methods
    void getImuData()
    void getGPSData()
    void getWheelEnconderData()
    void filterData()
    void publishData()

#endif