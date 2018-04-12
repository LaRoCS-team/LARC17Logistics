#include "go_dest/GoDest.hpp"

int main(int argc, char** argv){
	ros::init(argc, argv, "go_dest");
	GoDest go_dest(ros::this_node::getName());
	ros::spin();
}
