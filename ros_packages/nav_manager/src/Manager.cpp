#include "nav_manager/Manager.hpp"
#include <array>
#include <stdlib.h>

#define N_SENSORS 9 

std::vector <std::pair<float, float>>pointsDetected(N_SENSORS);

Manager::Manager(std::string name) :
	as_(nh, name, false),
	action_name(name),
	node_loop_rate(20),
	nav_status(0) {

	as_.registerGoalCallback(boost::bind(&Manager::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&Manager::preemptCB, this));	

	cancelGoal = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
	goToGoal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
	startFuzzy = nh.advertise<std_msgs::Bool>("response", 1);

	dist_sensors_sub = nh.subscribe("distance_sensors", 1, &Manager::distSensorsCallback, this);
	checkGoalStatus = nh.subscribe("move_base/status", 1, &Manager::goalStatusCallback, this);

	as_.start();
}

float Manager::squareDistance(std::pair<float, float> point) {
	return (std::pow(point.first, 1) + std::pow(point.second, 2));
}

void Manager::goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
	//actionlib_msgs::GoalStatusArray status = msg->status_list;
	if((msg->status_list).size() > 0) {
		nav_status = msg->status_list[0].status;
	}
}


void Manager::distSensorsCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
	for(int i =0; i < N_SENSORS; i++) {
		pointsDetected[i] = std::make_pair(msg->points[i].x, msg->points[i].y);
	}
}

void Manager::goalCB() {
	goal = as_.acceptNewGoal()->destination;
	std::cout << "new goal set" << std::endl;
	//navigation should be able to overwrite old route
	goToGoal.publish(goal);
	//v_status = 1; //navigation has started

}

void Manager::preemptCB() {
	std::cout << action_name << " preempted"<< std::endl;
	as_.setPreempted();
}

void Manager::spin() {
	ros::Rate lr(node_loop_rate);
	while(nh.ok()) {
		if(!as_.isActive()) {
			std::cout <<"No navigation goal active" << std::endl;	
		}
		else {
			if(nav_status == 1)	//ACTIVE
				std::cout << "Navigation active!" << std::endl;
			else if(nav_status == 3) { //SUCCEEDED
				result.result = true;
				as_.setSucceeded(result);
			}
			else if(nav_status == 4) { //ABORTED
				result.result = false;
				as_.setSucceeded(result);
			}
		}

		lr.sleep();
		ros::spinOnce();
	}
	result.result = false;
	as_.setSucceeded(result);
}
