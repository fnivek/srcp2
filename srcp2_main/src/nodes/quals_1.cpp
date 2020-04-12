#include <ros/ros.h>
#include "srcp2_main/odometry.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "srcp2_quals_1");
    ros::NodeHandle nh;
    Odometry odom("scout_1");
    odom.setup();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
}
