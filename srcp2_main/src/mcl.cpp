#include "srcp2_main/mcl.h"

Mcl::Mcl(std::string robot_name) : num_particles_(100), robot_name_(robot_name)
{
}

/**
 * @brief      Setup monte carlo locilization.
 *
 * @param[in]  init_pose  The initialize pose
 *
 * @return     True on success false on faliure
 */
bool Mcl::setup(Eigen::Affine3d init_pose)
{
    // Make all particles near the initial pose
    Particle::Weight uniform_weight = 1.0 / num_particles_;
    for (int i = 0; i < num_particles_; ++i)
    {
        // TODO(Kevin): Lookup how to uniform sample in SE(3)
        // std::random_device rd;
        // std::mt19937 generator(rd());
        // std::normal_distribution<> dist(0.0, 0.001);
        particles_.emplace_back(Particle{ init_pose, uniform_weight });
    }

    // Setup sensor model
    sensor_model_.setup();
    return true;
}

/**
 * @brief      Updates the mcl algorithm.
 *
 * @param[in]  msg   Laser scan message
 */
void Mcl::update(const sensor_msgs::LaserScan& msg)
{
    computePosterior(msg);
}

void Mcl::computePosterior(const sensor_msgs::LaserScan& msg)
{
    // TODO(Kevin): Avoid this insane amount of conversions
    // TODO(Kevin): Transform these points to robot footprint
    // Convert to pcl point cloud xyz
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 ros_pc;
    projector.projectLaser(msg, ros_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(ros_pc, *pcl_pc);
    // Convert to Eigen
    // TODO(Kevin): Make sure pts is a homgenous coordinate with [x,y,z,1]
    Eigen::MatrixXd pts = ((Eigen::MatrixXf)pcl_pc->getMatrixXfMap()).cast<double>();

    for (auto&& particle : particles_)
    {
        particle.weight_ = sensor_model_.likelihood(particle.pose_, pts);
    }
}
