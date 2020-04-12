#include "srcp2_main/odometry.h"

/**
 * @brief      Constructs a new instance.
 *
 * @param[in]  robot_name  The robot name
 */
Odometry::Odometry(std::string robot_name) : robot_name_(robot_name)
{
}

/**
 * @brief      Delayed initialization
 *
 *             Subscribes to the imu and joint states associated with robot_name_. Starts the tf listener, tf_listener_.
 *
 * @return     True if success false otherwise
 */
bool Odometry::setup()
{
    // Connect to ros topics
    ros::NodeHandle nh;
    imu_sub_.reset(new ros::Subscriber);
    *imu_sub_ = nh.subscribe("/" + robot_name_ + "/imu", 10, &Odometry::imuCallback, this);
    joint_states_sub_.reset(new ros::Subscriber);
    *joint_states_sub_ = nh.subscribe("/" + robot_name_ + "/joint_states", 10, &Odometry::jointStatesCallback, this);
    // Start tf listener
    tf_buf_.reset(new tf2_ros::Buffer);
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buf_));
}

/**
 * @brief      Imu callback
 *
 * @param[in]  msg   The message
 */
void Odometry::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
}

/**
 * @brief      Joint state callback
 *
 * @param[in]  msg   The message
 */
void Odometry::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
}
