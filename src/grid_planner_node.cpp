#include <naex/grid/planner.h>
#include <ros/ros.h>

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "grid_planner");
    ros::NodeHandle nh, pnh("~");
    naex::grid::Planner planner(nh, pnh);
    ros::spin();
}
