#ifndef SRCP2_MAIN_SENSOR_MODEL_H_
#define SRCP2_MAIN_SENSOR_MODEL_H_

// ROS
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// igl
#include <igl/readSTL.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/AABB.h>

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
    double likelihood(const Particle::Pose& pose, const Eigen::MatrixXd& scan);

  private:
  private:
    // Mesh
    Eigen::MatrixXd lunar_terrain_v_;
    Eigen::MatrixXd lunar_terrain_f_;
    Eigen::MatrixXd lunar_terrain_n_;
    igl::AABB<Eigen::MatrixXd, 3> lunar_terrain_tree_;
};

#endif  // SRCP2_MAIN_SENSOR_MODEL_H_
