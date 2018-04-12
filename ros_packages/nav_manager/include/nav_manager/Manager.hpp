#ifndef Manager_H
#define Manager_H
#pragma once

#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

using namespace std;


class Manager {

public:
	Manager();
	~Manager();


private:
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<nav_manager::NavManagerAction> as_;
	ros::Subscriber dist_sensors_sub;
	ros::Publisher goToGoal, cancelGoal;
		
	void distSensorsCallback(const sensor_msgs::PointCloud::COnstPtr& msg);

};

#endif



