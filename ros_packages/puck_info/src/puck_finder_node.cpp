#include "puck_info/CheckPuck.hpp"
#include <iostream>

int main(int argc, char** argv){

    ros::init(argc,argv,"puck_finder");

    CheckPuck obj;
	obj.spin();

    return 0;
}
