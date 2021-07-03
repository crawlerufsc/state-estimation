#include "ekf.h"
#include <math.h>

//constructor
EKF::EKF(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("EKF constructor");

    //initiale subs and pub
    initImuSub();
    initGpsSub();
    initWheelEncoderSub();
    initPublisher();
};

//init some matrices values
void EKF::initMatrices()
{
    //aux vector for diagonalizing matrices
    Eigen::Matrix<double, 9, 1> aux_vector_9;
    aux_vector_9 << 1, 1, 1, 1, 1, 1, 1, 1, 1;
    Eigen::Matrix<double, 4, 1> aux_vector_4;
    aux_vector_4 << 1, 1, 1, 1;
    Eigen::Matrix<double, 2, 1> aux_vector_2;
    aux_vector_2 << 1, 1;

    //define identity matrix
    identity_matrix_ = aux_vector_9.asDiagonal();

    //define initial state and covariance matrix
    current_state_.setZero();
    current_covariance_ = aux_vector_9.asDiagonal()*10;

    //define initial imu msg time
    double last_time_imu_ = ros::Time::now().toNSec() +
                            (ros::Time::now().toNSec()/1000000000);

    //init values for jacobians
    //imu
    F_imu_ = aux_vector_9.asDiagonal();
    G_imu_.setZero();
    L_imu_ = aux_vector_9.asDiagonal();
    //gps
    H_gps_.setZero();
    H_gps_(0,0) = 1;
    H_gps_(1,1) = 1;
    H_gps_(2,2) = 1;
    M_gps_ = aux_vector_4.asDiagonal();;
    //wheel encoder
    H_wheel_encoder_.setZero();
    M_wheel_encoder_ = aux_vector_2.asDiagonal();

    //init values for covariances
    Q_imu_.setZero();
    R_gps_.setZero();
    R_wheel_encoder_.setZero();

}

//creating subscribers
void EKF::initImuSub()
{
    ROS_INFO("Initializing Imu sub");
    imu_sub_ = nh_.subscribe("/imu_topic", 1, &EKF::imuCallback,this);   
};
void EKF::initGpsSub()
{
    ROS_INFO("Initializing gps sub");
    gps_sub_ = nh_.subscribe("/gps_topic", 1, &EKF::gpsCallback,this);  
};
void EKF::initWheelEncoderSub()
{
    ROS_INFO("Initializing wheel encoder sub");
    wheel_encoder_sub_ = nh_.subscribe("/wheel_encoder_topic", 1, &EKF::wheelEncoderCallback,this);   
};

//creating publisher
void EKF::initPublisher()
{
    ROS_INFO("Initializing state publisher");
    ekf_state_pub_ = nh_.advertise<nav_msgs::Odometry>("ekf_state", 1);
}

//imu callback
void EKF::imuCallback(const sensor_msgs::Imu& imu_msg)
{
    //Converting angle from quartenion to euler
    Eigen::Quaternionf q(imu_msg.orientation.w, imu_msg.orientation.x,
                         imu_msg.orientation.y, imu_msg.orientation.z);
    auto euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);

    //putting values from imu topic into imu_input_ vector
    imu_input_(0) = imu_msg.linear_acceleration.x;
    imu_input_(1) = imu_msg.linear_acceleration.y;
    imu_input_(2) = imu_msg.linear_acceleration.z;
    imu_input_(3) = imu_msg.angular_velocity.x;
    imu_input_(4) = imu_msg.angular_velocity.y;
    imu_input_(5) = imu_msg.angular_velocity.z;
};

//gps callback
void EKF::gpsCallback(const nav_msgs::Odometry& gps_msg)
{
    //putting values from gps topic into gps_input_ vector
    gps_input_(0) = gps_msg.pose.pose.position.x;
    gps_input_(1) = gps_msg.pose.pose.position.y;
    gps_input_(2) = gps_msg.pose.pose.position.z;
    gps_input_(3) = gps_msg.twist.twist.linear.x;
};

//gps callback
void EKF::wheelEncoderCallback(const nav_msgs::Odometry& wheel_encoder_msg)
{
    //putting values from wheel encoder topic into wheel_encoder_input_ vector
    wheel_encoder_input_(0) = wheel_encoder_msg.twist.twist.linear.x;
    wheel_encoder_input_(1) = wheel_encoder_msg.twist.twist.angular.z;
};

//predict state
void EKF::predict()
{   
    //if you are in predict step, the state was not yet corrected by gps
    //this boolean is used by the wheel encoder update
    corrected_by_gps = false;

    //getting current time
    double current_time_imu_ = ros::Time::now().toNSec() +
                                (ros::Time::now().toNSec()/1000000000);

    //update time differences in F matrix
    F_imu_(0,3) = (current_time_imu_ - last_time_imu_);
    F_imu_(1,4) = (current_time_imu_ - last_time_imu_);
    F_imu_(2,5) = (current_time_imu_ - last_time_imu_);

    //update time differences in G matrix
    G_imu_(3,0) = (current_time_imu_ - last_time_imu_);
    G_imu_(4,1) = (current_time_imu_ - last_time_imu_);
    G_imu_(5,2) = (current_time_imu_ - last_time_imu_);
    G_imu_(6,3) = (current_time_imu_ - last_time_imu_);
    G_imu_(7,4) = (current_time_imu_ - last_time_imu_);
    G_imu_(8,5) = (current_time_imu_ - last_time_imu_);

    //predict
    // x_k+1|k = F_k * x_k|k+1 + G_k * u_k
    predicted_state_ = F_imu_*current_state_ + G_imu_*imu_input_;
    // P_k+1|k = F_k * P_k|k+1 * F_k^T + L_k * Q_k * L_k^T
    predicted_covariance_ = F_imu_*current_covariance_*F_imu_.transpose() +
                            L_imu_*Q_imu_*L_imu_.transpose();

    //update current_state
    //if the state is not corrected by the others sensors, it can be used as input for
    //the next prediction step
    current_state_ = predicted_state_;
    current_covariance_ = predicted_covariance_;
    double last_time_imu_ = current_time_imu_;
};

//gps correction
void EKF::correct_gps()
{   

    if(sqrt(pow(predicted_state_(3), 2) + pow(predicted_state_(4),2)) < 0.000001)
    {
        H_gps_(3,3) = 0.0;
        H_gps_(3,4) = 0.0;
        //K_k = 0
        kalman_gain_gps_.setZero();
    } else {
        H_gps_(3,3) = predicted_state_(3)/sqrt(pow(predicted_state_(3), 2) + pow(predicted_state_(4),2));
        H_gps_(3,4) = predicted_state_(4)/sqrt(pow(predicted_state_(3), 2) + pow(predicted_state_(4),2));
        //K_k = P_k+1|k * H_k^T * (H_k * P_k+1|k * H_k^T + M_k * R_k * M_k^T)^-1
        kalman_gain_gps_ = predicted_covariance_*H_gps_.transpose()*
                       (H_gps_*predicted_covariance_*H_gps_.transpose()+
                        M_gps_*R_gps_*M_gps_.transpose()).inverse();
    };

    //correcting
    //x_k+1|k+1 = x_k+1|k + K_k * (y_k - H_k * x_k+1|k)
    corrected_state_ = predicted_state_ + kalman_gain_gps_*(gps_input_ - H_gps_*predicted_state_);
    //P_k+1|k+1 = (I - K_k * H_k) * P_k+1|k
    corrected_covariance_ = (identity_matrix_ - kalman_gain_gps_*H_gps_)*predicted_covariance_;

    //update current_state
    //if the state is not corrected by the others sensors, it can be used as input for
    //the next prediction step
    current_state_ = corrected_state_;
    current_covariance_ = corrected_covariance_;

    //the state was corrected by gps
    corrected_by_gps = true;
};

void EKF::correct_wheel_encoder()
{   
    //if the state was corrected, use the corrected state as input
    //otherwise use the predicted state
    if(corrected_by_gps){
        predicted_state_ = corrected_state_;
        predicted_covariance_ = corrected_covariance_;
    };
    
    double current_time_wheel_encoder_ = ros::Time::now().toNSec() +
                                        (ros::Time::now().toNSec()/1000000000);

    H_wheel_encoder_(1,8) = (current_time_wheel_encoder_ - last_time_imu_);
    
    //avoiding NaN results
    if(sqrt(pow(predicted_state_(3), 2) + pow(predicted_state_(4),2)) < 0.000001)
    {
        H_wheel_encoder_(0,3) = 0.0;
        H_wheel_encoder_(0,4) = 0.0;
        //K_k = 0
        kalman_gain_wheel_encoder_.setZero();
    } else {
        H_wheel_encoder_(0,3) = predicted_state_(3)/sqrt(pow(predicted_state_(3), 2) + pow(predicted_state_(4),2));
        H_wheel_encoder_(0,4) = predicted_state_(4)/sqrt(pow(predicted_state_(3), 2) + pow(predicted_state_(4),2));
        //K_k = P_k+1|k * H_k^T * (H_k * P_k+1|k * H_k^T + M_k * R_k * M_k^T)^-1
        kalman_gain_wheel_encoder_ = predicted_covariance_*H_wheel_encoder_.transpose()*
                       (H_wheel_encoder_*predicted_covariance_*H_wheel_encoder_.transpose()+
                        M_wheel_encoder_*R_wheel_encoder_*M_wheel_encoder_.transpose()).inverse();
    };

    //correcting
    //x_k+1|k+1 = x_k+1|k + K_k * (y_k - H_k * x_k+1|k)
    corrected_state_ = predicted_state_ + kalman_gain_wheel_encoder_*
                       (wheel_encoder_input_ - H_wheel_encoder_*predicted_state_);

    //P_k+1|k+1 = (I - K_k * H_k) * P_k+1|k
    corrected_covariance_ = (identity_matrix_ - kalman_gain_wheel_encoder_*H_wheel_encoder_)*
                            predicted_covariance_;

    //update current_state
    current_state_ = corrected_state_;
    current_covariance_ = corrected_covariance_;
};

//publish
void EKF::publishData()
{
    //populating ekf_state_msg_ with the ekf results

    double secs = ros::Time::now().toSec();
    unsigned long nsecs = ros::Time::now().toNSec();

    ekf_state_msg_.pose.pose.position.x = corrected_state_(0);
    ekf_state_msg_.pose.pose.position.y = corrected_state_(1);
    ekf_state_msg_.pose.pose.position.z = corrected_state_(2);

    ekf_state_msg_.twist.twist.linear.x = corrected_state_(3);
    ekf_state_msg_.twist.twist.linear.y = corrected_state_(4);
    ekf_state_msg_.twist.twist.linear.z = corrected_state_(5);

    ekf_state_msg_.header.stamp.sec = secs;
    ekf_state_msg_.header.stamp.nsec = nsecs;

    //publishing
    ekf_state_pub_.publish(ekf_state_msg_);
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    EKF ekf_estimator(&nh);

    ekf_estimator.initMatrices();

    while(ros::ok())
    {
        ros::spinOnce();
        ekf_estimator.predict();
        ekf_estimator.correct_gps();
        ekf_estimator.correct_wheel_encoder();
        ekf_estimator.publishData();
        rate.sleep();
    };

    return 0;
};