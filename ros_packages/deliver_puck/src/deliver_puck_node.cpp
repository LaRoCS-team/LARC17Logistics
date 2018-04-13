    //
    // Created by rafael on 07/11/17.
    //

    #include "ros/ros.h"
    #include "../include/DeliverPuck.hpp"

    int main(int argc, char **argv)
    {

        //Initiate ROS
        ros::init(argc, argv, "deliver_puck_node");

        //Create an object of class DeliverPuck that will take care of everything
        DeliverPuck deliverPuck;
        deliverPuck.spin();

        return 0;
    }