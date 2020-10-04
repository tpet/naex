//
// Created by petrito1 on 10/1/20.
//

#include <naex/follower.h>
#include <ros/ros.h>

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "follower");
    ros::NodeHandle nh, pnh("~");
    naex::Follower planner(nh, pnh);
    ros::spin();
}
