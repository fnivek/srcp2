#include <ros/ros.h>
#include "srcp2_main/odometry.h"
#include "srcp2_main/mcl.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "srcp2_quals_1");
    ros::NodeHandle nh;
    // Odometry odom("scout_1");
    // odom.setup();
    Mcl mcl("scout_1");
    mcl.setup(Eigen::Affine3d());

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // odom.update();
        // mcl.update();
        loop_rate.sleep();
        ros::spinOnce();
    }
}
