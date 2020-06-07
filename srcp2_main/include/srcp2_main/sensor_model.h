#ifndef SRCP2_MAIN_SENSOR_MODEL_H_
#define SRCP2_MAIN_SENSOR_MODEL_H_

// STL
#include <iostream>

// ROS
#include <ros/ros.h>
// Ros msgs
#include <sensor_msgs/LaserScan.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// igl
#include <igl/readSTL.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/AABB.h>
#include <igl/ray_mesh_intersect.h>
#include <igl/Hit.h>

// SRCP2_main
#include "srcp2_main/particle.h"

class SensorModel
{
  public:
    // Constructors and destructors
    SensorModel();
    // Delayed initialization
    bool setup();
    // Get likelihood
    double likelihood(const Particle::Pose& pose, const sensor_msgs::LaserScan& scan);

  private:
  private:
    // Mesh
    Eigen::MatrixXd lunar_terrain_v_;
    Eigen::MatrixXd lunar_terrain_f_;
    Eigen::MatrixXd lunar_terrain_n_;
    igl::AABB<Eigen::MatrixXd, 3> lunar_terrain_tree_;
    ros::Publisher debug_laser_pub_;
};

#endif  // SRCP2_MAIN_SENSOR_MODEL_H_
