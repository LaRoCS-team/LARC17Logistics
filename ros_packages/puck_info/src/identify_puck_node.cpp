#include "ros/ros.h"
#include "../include/fetch_puck/IdentifyPuck.hpp"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "fetch_puck_identify");

    IdentifyPuck identifyPuck;
    identifyPuck.spin();

    return 0;
}