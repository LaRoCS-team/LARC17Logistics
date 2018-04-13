#ifndef Manager_H
#define Manager_H
#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_manager/NavManagerAction.h>

using namespace std;

class Manager {

public:
	Manager(string name);
	~Manager() {}
	void spin();


private:
		
	void distSensorsCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
	void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
	void goalCB();	
	void preemptCB();

	ros::NodeHandle nh;
	actionlib::SimpleActionServer<nav_manager::NavManagerAction> as_;
	ros::Subscriber dist_sensors_sub, checkGoalStatus;;
	ros::Publisher goToGoal, cancelGoal, startFuzzy;

	geometry_msgs::PoseStamped goal;
	actionlib_msgs::GoalStatusArray status;
	string action_name;
	int node_loop_rate;
	int nav_status;

	nav_manager::NavManagerResult result;
	nav_manager::NavManagerFeedback feedback;	
};

	

#endif



