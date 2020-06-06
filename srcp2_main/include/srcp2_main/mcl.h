#ifndef SRCP2_MAIN_MCL_H_
#define SRCP2_MAIN_MCL_H_

// STL
#include <vector>
#include <random>
#include <string>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Ros
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
// Ros msgs
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>

// srcp2_main
#include "srcp2_main/particle.h"
#include "srcp2_main/sensor_model.h"

/**
 * @brief      This class describes a Monte Carlo Locilization algorithm
 */
class Mcl
{
  public:
    typedef std::vector<Particle> Particles;

  public:
    Mcl(std::string robot_name);
    bool setup(Eigen::Affine3d init_pose);
    void update(const sensor_msgs::LaserScan& msg);

  private:
    void computePosterior(const sensor_msgs::LaserScan& msg);

  private:
    Particles particles_;
    uint32_t num_particles_;
    std::string robot_name_;
    SensorModel sensor_model_;
};

#endif  // SRCP2_MAIN_MCL_H_
