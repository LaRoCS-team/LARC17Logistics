#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "world/Machine.hpp"
#include "world/DistrCenter.hpp"
#include "nav_msgs/Odometry.h"
#include <std_msgs/Int32.h>
#include "world/Pose2d.hpp"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <world/State.h>
#include <world/DistrCenter.h>
#include <world/Machine.h>
#include <yaml-cpp/yaml.h>

class State{
	public:
		State();
		~State();
    	void PublishLoop();
	private:
		//Ros and TF stuff
    	ros::NodeHandle nh;
    	ros::Publisher statePub;
    	//ros::Rate loopRate;
    	tf::TransformBroadcaster mapBroadcaster;
		//Task information
    	int task;
    	//Machines and DCs
		std::vector<Machine> machines;
    	std::vector<DistrCenter> dcs;
		int nMachines;
		int nDCs;
};
