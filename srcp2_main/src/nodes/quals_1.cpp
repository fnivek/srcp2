// Stl
#include <string>

// Ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// srcp2
#include "srcp2_main/odometry.h"
#include "srcp2_main/mcl.h"

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
        Eigen::Affine3d zero(Eigen::Translation3d(0, 0, 0));
        mcl_.setup(zero);

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
