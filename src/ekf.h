#ifndef EKF_
#define EKF_

#include <string>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class EKF
{
public:
    EKF(ros::NodeHandle* nodehandle);

    //public methods
    void initMatrices();
    void predict();
    void correct_gps();
    void correct_wheel_encoder();
    void publishData();
    
private:
    ros::NodeHandle nh_;

    //time variables
    double last_time_imu_;
    double current_time_imu_;

    //identity matrix
    Eigen::Matrix<double, 9, 9> identity_matrix_;

    //message to be published
    nav_msgs::Odometry ekf_state_msg_;

    //state vectors (x_k|k+1, x_k+1|k, x_k+1|k+1 )
    Eigen::Matrix<double, 9, 1> current_state_;
    Eigen::Matrix<double, 9, 1> predicted_state_;
    Eigen::Matrix<double, 9, 1> corrected_state_;

    //covariance matrices (P_k|k+1, P_k+1|k, P_k+1|k+1 )
    Eigen::Matrix<double, 9, 9> current_covariance_;
    Eigen::Matrix<double, 9, 9> predicted_covariance_;
    Eigen::Matrix<double, 9, 9> corrected_covariance_;
    Eigen::Matrix<double, 6, 6> Q_imu_;
    Eigen::Matrix<double, 6, 6> R_gps_;
    Eigen::Matrix<double, 1, 1> R_wheel_encoder_;

    //kalman gain matrices
    Eigen::Matrix<double, 9, 6> kalman_gain_gps_;
    Eigen::Matrix<double, 9, 1> kalman_gain_wheel_encoder_;

    //Jacobian matrices
    Eigen::Matrix<double, 9, 9> F_imu_;
    Eigen::Matrix<double, 9, 9> G_imu_;
    Eigen::Matrix<double, 9, 6> L_imu_;
    Eigen::Matrix<double, 6, 9> H_gps_;
    Eigen::Matrix<double, 6, 6> M_gps_;
    Eigen::Matrix<double, 1, 9> H_wheel_encoder_;
    Eigen::Matrix<double, 1, 1> M_wheel_encoder_;

    //sensor inputs
    Eigen::Matrix<double, 9, 1> imu_input_;
    Eigen::Matrix<double, 6, 1> gps_input_;
    Eigen::Matrix<double, 1, 1> wheel_encoder_input_;

    //subscribers and publishers
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber wheel_encoder_sub_;
    ros::Publisher  ekf_state_pub_;

    //callbacks
    void imuCallback(const sensor_msgs::Imu& imu_msg);
    void gpsCallback(const nav_msgs::Odometry& gps_msg);
    void wheelEncoderCallback(const nav_msgs::Odometry& gps_msg);

    //private methods
    void initImuSub();
    void initGpsSub();
    void initWheelEncoderSub();
    void initPublisher();

    //others
    bool corrected_by_gps;
};

#endif