#include "artificial_publisher.h"

ArtificialPublisher::ArtificialPublisher(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("Artificial Publisher constructor");
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_topic", 1);
    gps_pub_ = nh_.advertise<nav_msgs::Odometry>("gps_topic", 1);
    wheel_encoder_pub_ = nh_.advertise<nav_msgs::Odometry>("wheel_encoder_topic", 1);
};

void ArtificialPublisher::publishData()
{
    imu_msg_.orientation.x = 0.0;
    imu_msg_.orientation.y = 0.0;
    imu_msg_.orientation.z = 0.0;
    imu_msg_.orientation.w = 1.0;
    imu_msg_.angular_velocity.x = 0.0;
    imu_msg_.angular_velocity.y = 0.0;
    imu_msg_.angular_velocity.z = 0.0;
    imu_msg_.linear_acceleration.x = 0.0;
    imu_msg_.linear_acceleration.y = 0.0;
    imu_msg_.linear_acceleration.z = 0.0;

    gps_msg_.pose.pose.position.x = 0.0;
    gps_msg_.pose.pose.position.y = 0.0;
    gps_msg_.pose.pose.position.z = 0.0;
    gps_msg_.twist.twist.linear.x = 0.0;
    gps_msg_.twist.twist.linear.y = 0.0;
    gps_msg_.twist.twist.linear.z = 0.0;

    wheel_encoder_msg_.pose.pose.position.x = 0.0;
    wheel_encoder_msg_.pose.pose.position.y = 0.0;
    wheel_encoder_msg_.pose.pose.position.z = 0.0;
    wheel_encoder_msg_.twist.twist.linear.x = 0.0;
    wheel_encoder_msg_.twist.twist.linear.y = 0.0;
    wheel_encoder_msg_.twist.twist.linear.z = 0.0;

    imu_pub_.publish(imu_msg_);
    gps_pub_.publish(gps_msg_);
    wheel_encoder_pub_.publish(wheel_encoder_msg_);
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "artificial_publisher_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ArtificialPublisher artificial_publisher(&nh);

    while(ros::ok())
    {
        artificial_publisher.publishData();
        rate.sleep();
    };

    return 0;
};