#include <world/MapCreator.hpp>

int main(int argc, char** argv){
    ros::init(argc,argv,"map_creator_node");
    MapCreator obj;
    while(ros::ok()){
  		obj.PublishLoop();
  		ros::spinOnce();
	  }
}
