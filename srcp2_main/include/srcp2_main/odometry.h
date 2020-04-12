#ifndef SRP2_MAIN_ODOMETRY_H_
#define SRP2_MAIN_ODOMETRY_H_

// Ros
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
// Ros msgs
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

// C++
#include <string>
#include <memory>

class Odometry
{
  public:
    // Constructors and destructors
    Odometry(std::string robot_name);

    // Delayed initialization
    bool setup();

  private:
    // Subscriber callbacks
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

  private:
    // Name of the robot
    std::string robot_name_;
    // Subscribers
    std::unique_ptr<ros::Subscriber> imu_sub_;
    std::unique_ptr<ros::Subscriber> joint_states_sub_;
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif
