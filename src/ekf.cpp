#include <ekf.h>

EKF::EKF(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("EKF constructor");
    getImuData();
    getGpsData();
    getWheelEncoderData();
    filterData();
    publishData();
}

void EKF::getImuData()
{
    ROS_INFO("Getting Imu data");
    imu_sub_ = nh_.subscribe("/imu_data", 1, &EKF::imuCallback,this);   
}

void EKF::getGPSData()
{
    ROS_INFO("Getting gps data");
    imu_sub_ = nh_.subscribe("/imu_data", 1, &EKF::gpsCallback,this);  
}

void EKF::getWheelencoderData()
{
    ROS_INFO("Getting wheel encoder data");
    imu_sub_ = nh_.subscribe("/imu_data", 1, &EKF::wheelEncoderCallback,this);   
}

void EKF::getImuData()
{
    ROS_INFO("Getting Imu data");
    imu_sub_ = nh_.subscribe("/imu_data", 1, &EKF::imuCallback,this);   
}

void imuCallback(const sensor_msgs::Imu& imu_msg)
{
    imu_orientation_[0] = imu_msg.orientation.x;
    imu_orientation_[1] = imu_msg.orientation.y;
    imu_orientation_[2] = imu_msg.orientation.z;
    imu_orientation_[3] = imu_msg.orientation.w;

    imu_angular_velocity_[0] = imu_msg.angular_velocity.x;
    imu_angular_velocity_[1] = imu_msg.angular_velocity.y;
    imu_angular_velocity_[2] = imu_msg.angular_velocity.z;

    imu_linear_acceleration_[0] = imu_msg.linear_acceleration.x;
    imu_linear_acceleration_[1] = imu_msg.linear_acceleration.y;
    imu_linear_acceleration_[2] = imu_msg.linear_acceleration.z;
}

void gpsCallback(const nav_msgs::Odometry& gps_msg)
{
    gps_position_[0] = gps_msg.pose.pose.point.x;
    gps_position_[1] = gps_msg.pose.pose.point.y;
    gps_position_[2] = gps_msg.pose.pose.point.z;

    gps_velocity_[0] = gps_msg.twist.twist.linear.x;
    gps_velocity_[1] = gps_msg.twist.twist.linear.y;
    gps_velocity_[2] = gps_msg.twist.twist.linear.z;
}

void odomCallback(const nav_msgs::Odometry& wheel_encoder_msg)
{
    wheel_encoder_position_[0] = wheel_encoder_msg.pose.pose.point.x;
    wheel_encoder_position_[1] = wheel_encoder_msg.pose.pose.point.y;
    wheel_encoder_position_[2] = wheel_encoder_msg.pose.pose.point.z;

    wheel_encoder_velocity_[0] = wheel_encoder_msg.twist.twist.linear.x;
    wheel_encoder_velocity_[1] = wheel_encoder_msg.twist.twist.linear.y;
    wheel_encoder_velocity_[2] = wheel_encoder_msg.twist.twist.linear.z;
}

void EKF::filterData()
{
    
}

void EKF::publishData()
{
    ekf_state_pub_.publish(ekf_state_msg_);
}


int main(int agrc, char **argv)
{
    ros::init(arg, argv, "ekf_node");
    ros::NodeHandle nh;
    EKF ekf_estimator(&nh);
    ros::spin();

    return 0;
}