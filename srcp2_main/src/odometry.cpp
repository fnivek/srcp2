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

void Odometry::update()
{
    while (!imu_queue_.empty())
    {
        // Get oldest imu msg
        // TODO(Kevin): Avoid a copy by using a reference
        sensor_msgs::Imu msg;
        {
            std::lock_guard<std::mutex> lock(imu_lock_);
            msg = imu_queue_.back();
        }

        // Do math
        imuUpdate(msg);

        // Remove last element
        {
            std::lock_guard<std::mutex> lock(imu_lock_);
            imu_queue_.pop();
        }
    }
}

/**
 * @brief      Imu callback
 *
 * @param[in]  msg   The message
 */
void Odometry::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_queue_.emplace(*msg);
}

void Odometry::imuUpdate(const sensor_msgs::Imu& msg)
{
    // Simple constant acceleration motion model
    float dt = (float)(msg.header.stamp - cur_header_.stamp).toSec();
    cur_pose_.position.x += 0.5 * cur_wrench_.force.x * cur_wrench_.force.x * dt + cur_twist_.linear.x * dt;
    cur_pose_.position.y += 0.5 * cur_wrench_.force.y * cur_wrench_.force.y * dt + cur_twist_.linear.y * dt;
    cur_pose_.position.z += 0.5 * cur_wrench_.force.z * cur_wrench_.force.z * dt + cur_twist_.linear.z * dt;
    cur_twist_.linear.x += cur_wrench_.force.x * dt;
    cur_twist_.linear.y += cur_wrench_.force.y * dt;
    cur_twist_.linear.z += cur_wrench_.force.z * dt;

    cur_header_ = msg.header;

    cur_twist_.angular.x = msg.angular_velocity.x;
    cur_twist_.angular.y = msg.angular_velocity.y;
    cur_twist_.angular.z = msg.angular_velocity.z;

    cur_pose_.orientation.x = msg.orientation.x;
    cur_pose_.orientation.x = msg.orientation.y;
    cur_pose_.orientation.x = msg.orientation.z;
    cur_pose_.orientation.w = msg.orientation.w;

    cur_wrench_.force.x = msg.linear_acceleration.x;
    cur_wrench_.force.y = msg.linear_acceleration.y;
    cur_wrench_.force.z = msg.linear_acceleration.z;

    std::cout << cur_pose_ << std::endl;
    std::cout << cur_twist_ << std::endl;
    std::cout << cur_wrench_ << std::endl;
}

/**
 * @brief      Joint state callback
 *
 * @param[in]  msg   The message
 */
void Odometry::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
}
