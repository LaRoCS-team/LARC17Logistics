#include <world/WorldState.hpp>

int main(int argc, char** argv){
    ros::init(argc,argv,"world_node");
    WorldState obj;
    while(ros::ok()){
  		obj.PublishLoop();
  		ros::spinOnce();
	  }
}
