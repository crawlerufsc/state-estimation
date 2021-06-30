#ifndef ARTIFICIAL_PUBLISHER_H_
#define ARTIFICIAL_PUBLISHER_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class ArtificialPublisher
{
public:
    ArtificialPublisher(ros::NodeHandle* nodehandle);

    //methods
    void publishData();
    
private:
    ros::NodeHandle nh_;

    sensor_msgs::Imu imu_msg_;
    nav_msgs::Odometry gps_msg_;
    nav_msgs::Odometry wheel_encoder_msg_;

    //subscribers and publishers
    ros::Publisher imu_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher wheel_encoder_pub_;
};

#endif