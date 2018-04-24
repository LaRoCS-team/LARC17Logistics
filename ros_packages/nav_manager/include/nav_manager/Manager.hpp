#ifndef Manager_H
#define Manager_H
#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_manager/NavManagerAction.h>
#include <navigation_fuzzy/FuzzyAction.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace std;

class Manager {

public:
	Manager(string name);
	~Manager() {}
	void spin();


private:
	float squareDistance(std::pair<float,float> point);
	bool checkDangerSituation();

	void distSensorsCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
	void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);


	void goalCB();
	void preemptCB();

	void doneNav(const actionlib::SimpleClientGoalState& state, const MoveBaseResultConstPtr& result);
	void activeNav();
  void feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

	ros::NodeHandle nh;
	actionlib::SimpleActionServer<nav_manager::NavManagerAction> as_;
	actionlib::SimpleActionClient<navigation_fuzzy::FuzzyAction> ac_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;
	ros::Subscriber dist_sensors_sub, checkGoalStatus;;
	ros::Publisher goToGoal, cancelGoal, startFuzzy;

	geometry_msgs::PoseStamped goal_go_dest;
	move_base_msgs::MoveBaseGoal goal;
	actionlib_msgs::GoalStatusArray status;
	string action_name;
	int node_loop_rate;
	int nav_status;
	int fuzzyOn;

	nav_manager::NavManagerResult result;
	nav_manager::NavManagerFeedback feedback;
};



#endif
