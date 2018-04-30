#include "nav_manager/Manager.hpp"
#include <array>
#include <iostream>
#include <cmath>

static int N_SENSORS = 9;
static int MAX_SQUARE_RADIUS_INT = 40;
static int MAX_SQUARE_RADIUS_SIDES = 30;
std::vector <std::pair<float, float>>pointsDetected(N_SENSORS);

Manager::Manager(std::string name) :
	navManagerServer(nh, name, false),
	fuzzyClient("navigation_fuzzy", true),
	moveBaseClient("move_base", true),
	action_name(name),
	node_loop_rate(20),
	nav_status(1),
	fuzzyOn(0) {

	navManagerServer.registerGoalCallback(boost::bind(&Manager::goalCB, this));
	navManagerServer.registerPreemptCallback(boost::bind(&Manager::preemptCB, this));

	dist_sensors_sub = nh.subscribe("distance_sensors", 1, &Manager::distSensorsCallback, this);

	navManagerServer.start();
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
	if((msg->status_list).size() > 0) {
		nav_status = msg->status_list[0].status;
		cout << "native nav status " << nav_status << endl;
	}
}

void Manager::distSensorsCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
	for(int i =0; i < N_SENSORS; i++) {
		pointsDetected[i] = std::make_pair(msg->points[i].x, msg->points[i].y);
	}
}

void Manager::feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {}

void Manager::doneNav(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
	nav_done = state.isDone();
	if (nav_done) {
		nav_status = state.state_;
	}
}

void Manager::activeNav() {}

void Manager::preemptCB() {
	std::cout << action_name << " preempted"<< std::endl;
  result.result = false;
  navManagerServer.setPreempted(result);
}

void Manager::goalCB() {
	goal_go_dest = navManagerServer.acceptNewGoal()->destination;
	std::cout << "New goal set" << std::endl;
	//navigation should be able to overwrite old route
	goal.target_pose.header= goal_go_dest.header;
	goal.target_pose.pose = goal_go_dest.pose;
	goal.target_pose.header.stamp = ros::Time::now();
	moveBaseClient.waitForServer();
	moveBaseClient.sendGoal(goal, boost::bind(&Manager::doneNav, this, _1, _2), boost::bind(&Manager::activeNav, this), boost::bind(&Manager::feedbackCB, this, _1));
	nav_status = 1;
}

void Manager::spin() {
  ros::Rate lr(node_loop_rate);
  while(nh.ok()) {
    if(navManagerServer.isActive()) {
			if (nav_status == 1) {
				if(checkDangerSituation()) {
					moveBaseClient.cancelAllGoals();
					moveBaseClient.stopTrackingGoal();
					cout << "Preempted route" << endl;
					fuzzyClient.waitForServer();
					navigation_fuzzy::FuzzyGoal fuzzyOn;
					fuzzyOn.order = true;
					actionlib::SimpleClientGoalState state = fuzzyClient.sendGoalAndWait(fuzzyOn, ros::Duration(10.0), ros::Duration(1.0));
					if(state.state_ == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) { //REACTIVATES NAVIGATION
							cout << "About to resend goal" << endl;
							goal.target_pose.header.stamp = ros::Time::now();
							moveBaseClient.waitForServer();
							moveBaseClient.sendGoal(goal, boost::bind(&Manager::doneNav, this, _1, _2),
																			boost::bind(&Manager::activeNav, this),
																			boost::bind(&Manager::feedbackCB, this, _1));
					}
					else {
							result.result = false;
							navManagerServer.setSucceeded(result);
					}
				}
			}
			else if(nav_status == 4) {  //PREEMPTED
				cout << "Navigation is preempted" << endl;
				nav_status = -1;
			}
			else if(nav_status == 5) { //ABORTED
				cout << "Navigation Aborted" << endl;
				result.result = false;
				navManagerServer.setAborted(result);
				nav_status = -1;
			}
			else if(nav_status == 6) { //SUCCEEDED
				cout << "Navigation succeeded" << endl;
				result.result = true;
				navManagerServer.setSucceeded(result);
				nav_status = -1;
			}
			else {

			}
  	}
    lr.sleep();
    ros::spinOnce();
	}
	result.result = false;
	navManagerServer.setSucceeded(result);
}
