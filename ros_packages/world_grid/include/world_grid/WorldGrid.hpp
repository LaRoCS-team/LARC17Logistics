#ifndef WorldGrid_H
#define WorldGrid_H
#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>

using namespace grid_map;

const int vSize = 25;
enum colorInfo {unknown, blue, yellow, red};// Equivalente a 0,1,2,...
enum occupation {nothing=0, obstacle=100}; 

class WorldGrid{
public:
	WorldGrid(); //construtor
	bool getState(); //retorna o vetor de estados
	void publishState(); //publica o estado no ROS
private:
	GridMap map; //Grid map
	bool currState[vSize]; //Vetor de estados booleanos (RL)
	ros::NodeHandle nh;
	ros::Subscriber odomSub, obstSub, puckSub;
	ros::Publisher statePub, gridPub;
	ros::Rate loopRate;
	tf::TransformBroadcaster mapBroadcaster;
};

#endif 