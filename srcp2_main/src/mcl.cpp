#include "srcp2_main/mcl.h"

Mcl::Mcl(std::string robot_name) : num_particles_(1), robot_name_(robot_name), tf_listener_(tf_buf_)
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
    std::cout << "init_pose\n" << init_pose.matrix() << std::endl;
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

    // Debug
    ros::NodeHandle nh;
    debug_laser_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("debug_laser_pose", 1);
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
    // Get the transform between the laser and the base footprint
    geometry_msgs::TransformStamped tf_msg;
    try
    {
        // TODO(Kevin): Don't sit around waitng for 0.1 seconds, if its not ready yet just use the most recent tf
        tf_msg = tf_buf_.lookupTransform(robot_name_ + "_tf/base_footprint", robot_name_ + "_tf/hokuyo_link",
                                         msg.header.stamp, ros::Duration(0.1));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        // TODO(Kevin): Handle faliure gracefully
    }
    // Transform to Eigen
    Eigen::Affine3d laser_pose = Eigen::Translation3d(tf_msg.transform.translation.x, tf_msg.transform.translation.y,
                                                      tf_msg.transform.translation.z) *
                                 Eigen::Quaterniond(tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
                                                    tf_msg.transform.rotation.y, tf_msg.transform.rotation.z);

    Particle::Weight total_weight = 0;
    for (auto&& particle : particles_)
    {
        Eigen::Affine3d world_laser_pose = particle.pose_ * laser_pose;
        // Debug publish laser pose
        geometry_msgs::PoseStamped debug_msg;
        debug_msg.header.frame_id = "gt_world";
        debug_msg.header.stamp = debug_msg.header.stamp;
        debug_msg.pose.position.x = world_laser_pose.translation()[0];
        debug_msg.pose.position.y = world_laser_pose.translation()[1];
        debug_msg.pose.position.z = world_laser_pose.translation()[2];
        Eigen::Quaterniond q(world_laser_pose.rotation().matrix());
        debug_msg.pose.orientation.x = q.x();
        debug_msg.pose.orientation.y = q.y();
        debug_msg.pose.orientation.z = q.z();
        debug_msg.pose.orientation.w = q.w();
        debug_laser_pose_pub_.publish(debug_msg);

        std::cout << "laser_pose:\n"
                  << laser_pose.matrix() << std::endl
                  << "particle:\n"
                  << particle.pose_.matrix() << std::endl
                  << "world_laser_pose:\n"
                  << world_laser_pose.matrix() << std::endl;

        particle.weight_ = sensor_model_.likelihood(world_laser_pose, msg);
        total_weight += particle.weight_;
    }

    // Normalize
    for (auto&& particle : particles_)
    {
        particle.weight_ /= total_weight;
    }
}
