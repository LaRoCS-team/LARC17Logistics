#include <iostream>
#include "nav_manager/Manager.hpp"

using namespace std;

int main(int argc, char** argv) {

	ros::init(argc, argv, "nav_manager");

	Manager nav_manager(ros::this_node::getName());
	nav_manager.spin();

	return 0;

}
