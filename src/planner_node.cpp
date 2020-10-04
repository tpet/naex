//
// Created by petrito1 on 10/1/20.
//

#include <naex/planner.h>
#include <ros/ros.h>

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh, pnh("~");
    naex::Planner planner(nh, pnh);
    ros::spin();
}
