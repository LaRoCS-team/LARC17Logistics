#include "nav_manager/Manager.hpp"
#include <array>
#include <stdlib.h>

#define N_SENSORS 9 
#define MAX_SQUARE_RADIUS_INT 40
#define MAX_SQUARE_RADIUS_SIDES 30



std::vector <std::pair<float, float>>pointsDetected(N_SENSORS);

Manager::Manager(std::string name) :
	as_(nh, name, false),
	ac_("navigation_fuzzy", true),
	action_name(name),
	node_loop_rate(20),
	nav_status(0),
	fuzzyOn(0) {

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
	return std::sqrt(std::pow(point.first, 2) + std::pow(point.second, 2));
}

bool Manager::checkDangerSituation() {
	float intEsq = 100*squareDistance(pointsDetected[1]); 
	float esq = 100*squareDistance(pointsDetected[2]); 
	float dir = 100*squareDistance(pointsDetected[7]);
	float intDir = 100*squareDistance(pointsDetected[8]);

	return (esq > 0 && esq < MAX_SQUARE_RADIUS_SIDES) || (dir > 0 && dir < MAX_SQUARE_RADIUS_SIDES) || (intEsq > 0 && intEsq < MAX_SQUARE_RADIUS_SIDES) || (intDir > 0 && intDir <  MAX_SQUARE_RADIUS_SIDES); 
	
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
			if(nav_status == 1){	//ACTIVE
				std::cout << "Navigation active!" << std::endl;
				if(checkDangerSituation()){  //CLOSE OBSTACLE(S)
					cout << "cancelou rota" << endl;
					actionlib_msgs::GoalID cancel;
					cancelGoal.publish(cancel); //PREEMPTS MOVE_BASE
					fuzzyOn = 0;
				}
			}
			else if(nav_status == 2) {  //PREEMPTED
				cout << "Navigation is preempted" << endl;
				if(fuzzyOn == 0) {
					ac_.waitForServer();
					navigation_fuzzy::FuzzyGoal goal;
					goal.order = true;
					ac_.sendGoal(goal);
					fuzzyOn = 1;
				}
				else {
					bool success = ac_.waitForResult(ros::Duration(60.0));
					if(success) { //REACTIVATES NAVIGATION
						const auto res = ac_.getResult();
						if(res)
							goToGoal.publish(goal);
						else {
							result.result = false;
							as_.setSucceeded(result);
						}
					}
					else { //ROBOT STUCK, FUZZY COULD NOT HANDLE
						result.result = false;
						as_.setSucceeded(result);
					}
				}
			}
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
