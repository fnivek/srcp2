#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "srcp2_quals_1");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
}
