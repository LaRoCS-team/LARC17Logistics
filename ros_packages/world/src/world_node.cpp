#include <world/State.hpp>

int main(int argc, char** argv){
    ros::init(argc,argv,"world_node");
    State initial;
    while(ros::ok()){
  		initial.PublishLoop();
  		ros::spinOnce();
	  }
}
