// Stl
#include <string>
#include <iostream>

// Ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/GetModelState.h>

// srcp2
#include "srcp2_main/odometry.h"
#include "srcp2_main/mcl.h"
#include <srcp2_msgs/LocalizationSrv.h>

class Quals1
{
  public:
    std::string robot_name_;
    Mcl mcl_;
    ros::Subscriber laser_sub_;
    // Odometry odom_;

  public:
    Quals1() : robot_name_("scout_1"), mcl_(robot_name_)  //  , odom_("scout_1")
    {
        // Setup robot
        // odom.setup();
        geometry_msgs::Pose pose;
        bool use_gazebo = true;
        if (use_gazebo)
        {
            gazebo_msgs::GetModelState true_pose_srv;
            true_pose_srv.request.model_name = robot_name_;
            true_pose_srv.request.relative_entity_name = "lunar_terrain";
            ros::service::call("/gazebo/get_model_state", true_pose_srv);
            pose = true_pose_srv.response.pose;
        }
        else
        {
            srcp2_msgs::LocalizationSrv true_pose_srv;
            ros::service::call("/" + robot_name_ + "/get_true_pose", true_pose_srv);
            pose = true_pose_srv.response.pose;
        }

        std::cout << "pose:\n" << pose << std::endl;
        Eigen::Affine3d true_pose =
            Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) *
            Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        mcl_.setup(true_pose);

        // Start subsribers
        ros::NodeHandle nh;
        laser_sub_ = nh.subscribe("/" + robot_name_ + "/laser/scan", 10, &Quals1::laserCallback, this);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        mcl_.update(*msg);
    }

    void run()
    {
        ros::Rate loop_rate(1000);
        while (ros::ok())
        {
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "srcp2_quals_1");
    ros::NodeHandle nh;

    Quals1 q1;
    q1.run();
}
