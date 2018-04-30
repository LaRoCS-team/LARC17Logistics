#include <iostream>
#include "world_grid/WorldGrid.hpp"
#include <string>
using namespace grid_map;

int main(int argc, char** argv)
{
  ros::init(argc,argv,"world_grid");
  WorldGrid mapa;
  while(ros::ok()){
    mapa.publishState();
    ros::spinOnce();
  }
}
