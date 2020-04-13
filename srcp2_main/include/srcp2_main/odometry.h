#ifndef SRP2_MAIN_ODOMETRY_H_
#define SRP2_MAIN_ODOMETRY_H_

// Ros
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
// Ros msgs
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Header.h>

// C++
#include <string>
#include <memory>
#include <queue>
#include <mutex>
#include <iostream>

class Odometry
{
  public:
    // Constructors and destructors
    Odometry(std::string robot_name);

    // Delayed initialization
    bool setup();

    // update
    void update();

  private:
    // Subscriber callbacks
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

    // Imu
    void imuUpdate(const sensor_msgs::Imu& msg);

  private:
    // Name of the robot
    std::string robot_name_;
    // Subscribers
    std::unique_ptr<ros::Subscriber> imu_sub_;
    std::unique_ptr<ros::Subscriber> joint_states_sub_;
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    // Incoming msg queues
    std::mutex imu_lock_;
    std::queue<sensor_msgs::Imu> imu_queue_;
    // Current state
    std_msgs::Header cur_header_;
    geometry_msgs::Pose cur_pose_;
    geometry_msgs::Twist cur_twist_;
    geometry_msgs::Wrench cur_wrench_;
};

#endif
