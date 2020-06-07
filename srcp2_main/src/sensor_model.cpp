#include "srcp2_main/sensor_model.h"
SensorModel::SensorModel()
{
}

bool SensorModel::setup()
{
    // Load mesh resources
    // TOOD(Kevin): Get stl file from rosparam or rospack
    ROS_INFO("Start load lunar_terrain");
    igl::readSTL("/home/user/catkin_ws/src/srcp2/data/lunar_terrain.stl", lunar_terrain_v_, lunar_terrain_f_,
                 lunar_terrain_n_);
    ROS_INFO("Finish load lunar_terrain");
    // Note this takes a few seconds
    ROS_INFO("Build aabb of lunar_terrain");
    // lunar_terrain_tree_.init(lunar_terrain_v_, lunar_terrain_f_);
    ROS_INFO("Built aabb of lunar_terrain");

    ros::NodeHandle nh;
    debug_laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("debug_sensor_model_scan", 1);
}

/**
 * @brief      Compute the likelihood of a scan at a given pose
 *
 * @param[in]  pose  The pose of the base footprint frame in world frame
 * @param[in]  scan  The laser scan as a 4xn matrix where each column is the homogenous coordinate [x,y,z,1] of the
 *                   endpoint of a laser scan in the base footprint frame of the robot
 *
 * @return     The likelihood of this particle
 */
double SensorModel::likelihood(const Particle::Pose& pose, const sensor_msgs::LaserScan& scan)
{
    // TODO(Kevin): Handle the fact that if scans are infinit they do not add penalty, i.e. if Laser sees nothing
    //              sum_squared distances is 0 Convert to pcl point cloud xyz

    // Eigen::MatrixXd world_scan = pose.matrix() * scan;
    // Eigen::VectorXd sqr_dists;
    // Eigen::VectorXi index;
    // Eigen::MatrixXd clossest_pts;
    // lunar_terrain_tree_.squared_distance(lunar_terrain_v_, lunar_terrain_f_,
    //                                      world_scan.block(0, 0, 3, world_scan.cols()).transpose(), sqr_dists, index,
    //                                      clossest_pts);
    // ROS_INFO("sum_sqr_dists: %f", sqr_dists.sum());
    sensor_msgs::LaserScan debug_scan = scan;
    debug_scan.ranges.clear();

    float angle = scan.angle_min;

    for (float range : scan.ranges)
    {
        // Calculate direction of laser scan
        Eigen::Vector3d dir =
            pose.linear() * Eigen::AngleAxis<double>(angle, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitX();
        igl::Hit hit;
        igl::ray_mesh_intersect(pose.translation(), dir, lunar_terrain_v_, lunar_terrain_f_, hit);
        std::cout << "(Expected range, measured range, diff): (" << hit.t << ", " << range << ", " << range - hit.t
                  << ")" << std::endl;
        angle += scan.angle_increment;
        debug_scan.ranges.emplace_back(hit.t);
    }
    debug_laser_pub_.publish(debug_scan);
    return 1;
}
