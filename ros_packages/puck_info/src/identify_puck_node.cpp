#include "ros/ros.h"
#include "puck_info/PuckInfo.hpp"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "fetch_puck_identify");

    IdentifyPuck identifyPuck;
    identifyPuck.spin();

    return 0;
}