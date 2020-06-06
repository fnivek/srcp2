#ifndef SRCP2_MAIN_ODOMETRY_H_
#define SRCP2_MAIN_ODOMETRY_H_

// Ros
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
// Ros msgs
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>

// PCL
#include <pcl/registration/icp.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// C++
#include <string>
#include <memory>
#include <queue>
#include <mutex>
#include <iostream>
#include <math.h>
#include <iostream>

#define SAMPLEFILTER_TAP_NUM 15

typedef struct
{
    double history[SAMPLEFILTER_TAP_NUM];
    unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter* f);
void SampleFilter_put(SampleFilter* f, double input);
double SampleFilter_get(SampleFilter* f);

static double filter_taps[SAMPLEFILTER_TAP_NUM] = {
    -0.008268912593598236, -0.012977645844522306, 0.027099288599285735, 0.024009121542281034,  -0.07849418531704068,
    -0.035125416972925406, 0.30807679910566016,   0.5394467059932627,   0.30807679910566016,   -0.035125416972925406,
    -0.07849418531704068,  0.024009121542281034,  0.027099288599285735, -0.012977645844522306, -0.008268912593598236
};

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
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // Imu
    void imuUpdate(const sensor_msgs::Imu& msg);

    // Laser
    void laserUpdate(const sensor_msgs::LaserScan& msg);

    // Tf
    void publishTf(std::string frame_id, std::string child_frame_id, ros::Time stamp, const Eigen::Affine3d& tf);

  private:
    // Name of the robot
    std::string robot_name_;
    // Subscribers
    std::unique_ptr<ros::Subscriber> imu_sub_;
    std::unique_ptr<ros::Subscriber> laser_sub_;
    std::unique_ptr<ros::Subscriber> joint_states_sub_;
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // Incoming msg queues
    std::mutex imu_lock_;
    std::queue<sensor_msgs::Imu> imu_queue_;
    std::mutex laser_lock_;
    std::queue<sensor_msgs::LaserScan> laser_queue_;
    // Current state
    float last_time_;
    Eigen::Vector3f pos_;
    Eigen::Vector3f vel_;
    Eigen::Quaternionf head_;
    Eigen::Vector3f accel_bias_;
    Eigen::Vector3f ang_vel_bias_;
    Eigen::Vector3f grav_;
    Eigen::Matrix<float, 18, 18> err_cov_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_pc_;
    Eigen::Affine3d last_laser_tf_;

    // Debug
    ros::Publisher debug_pub_;
    SampleFilter debug_filt_x_;
    SampleFilter debug_filt_y_;
    SampleFilter debug_filt_z_;
};

#endif  // SRCP2_MAIN_ODOMETRY_H_
