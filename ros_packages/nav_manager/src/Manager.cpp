#include "nav_manager/Manager.hpp"
#include <array>
#include <iostream>
#include <cmath>

#define N_SENSORS 9
#define MAX_SQUARE_RADIUS_INT 40
#define MAX_SQUARE_RADIUS_SIDES 30



std::vector <std::pair<float, float>>pointsDetected(N_SENSORS);

Manager::Manager(std::string name) :
	as_(nh, name, false),
	ac_("navigation_fuzzy", true),
	moveBaseClient("move_base", true),
	action_name(name),
	node_loop_rate(20),
	nav_status(1),
	fuzzyOn(0) {

	as_.registerGoalCallback(boost::bind(&Manager::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&Manager::preemptCB, this));
	//as_.registerFeedbackCallback(boost::bind(&Manager::feedbackCB, this));

	//cancelGoal = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
	//goToGoal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
	//startFuzzy = nh.advertise<std_msgs::Bool>("response", 1);

	dist_sensors_sub = nh.subscribe("distance_sensors", 1, &Manager::distSensorsCallback, this);
	//checkGoalStatus = nh.subscribe("move_base/status", 1, &Manager::goalStatusCallback, this);


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
		cout << "native nav status " << nav_status << endl;
	}
}


void Manager::distSensorsCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
	for(int i =0; i < N_SENSORS; i++) {
		pointsDetected[i] = std::make_pair(msg->points[i].x, msg->points[i].y);
	}
}

void Manager::feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {

}

void Manager::doneNav(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
	nav_done = state.isDone();
	if (nav_done) {
		nav_status = state.state_;
	}
}

void Manager::activeNav() {

}

void Manager::preemptCB() {
	std::cout << action_name << " preempted"<< std::endl;
    result.result = false;
    as_.setPreempted(result);
}

void Manager::goalCB() {
	goal_go_dest = as_.acceptNewGoal()->destination;
	std::cout << "new goal set" << std::endl;
	//navigation should be able to overwrite old route
	//goToGoal.publish(goal);

	goal.target_pose.header= goal_go_dest.header;
	goal.target_pose.pose = goal_go_dest.pose;
	goal.target_pose.header.stamp = ros::Time::now();
	moveBaseClient.waitForServer();
	//moveBaseClient.sendGoal(goal, feedback_cb=);
	moveBaseClient.sendGoal(goal, boost::bind(&Manager::doneNav, this, _1, _2), boost::bind(&Manager::activeNav, this), boost::bind(&Manager::feedbackCB, this, _1));
	nav_status = 1;
}

void Manager::spin() {
    ros::Rate lr(node_loop_rate);
    while(nh.ok()) {
        if(!as_.isActive()) {
            //std::cout <<"No navigation goal active" << std::endl;
        }
        else {
            //goal was sent
        /*if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                result.result = true;
                as_.setSucceeded(result);
        }
        if(checkDangerSituation()) {
                moveBaseClient.cancelAllGoals();
                moveBaseClient.stopTrackingGoal();
                cout << "cancelou rota" << endl;
                ac_.waitForServer();
                navigation_fuzzy::FuzzyGoal fuzzyOn;
                fuzzyOn.order = true;
                ac_.sendGoal(fuzzyOn);
                bool success = ac_.waitForResult(ros::Duration(60.0));
                if(success == true) { //REACTIVATES NAVIGATION
                        bool res = ac_.getResult()->res;
                        if(res == true) {
                            //goToGoal.publish(goal);
                                //goal.target_pose.header= goal_go_dest.header;
                                //goal.target_pose.pose = goal_go_dest.pose;
                                goal.target_pose.header.stamp = ros::Time::now();
                                moveBaseClient.waitForServer();
                                moveBaseClient.sendGoal(goal);
                        }
                }
                else {
                        result.result = false;
                        as_.setSucceeded(result);
                }
        }
}*/

            if (nav_status == 1) {  //(nav_status == 1)	//ACTIVE
                std::cout << "Navigation active!" << std::endl;
                /*if(checkDangerSituation()){  //CLOSE OBSTACLE(S)
                    moveBaseClient.cancelAllGoals();
                    moveBaseClient.stopTrackingGoal();
                    cout << "cancelou rota" << endl;
                    //actionlib_msgs::GoalID cancel;
                    //cancelGoal.publish(cancel); //PREEMPTS MOVE_BASE
                    fuzzyOn = 0;
                }*/
                if(checkDangerSituation()) {
                    moveBaseClient.cancelAllGoals();
                    moveBaseClient.stopTrackingGoal();
                    cout << "cancelou rota" << endl;
                    ac_.waitForServer();
                    navigation_fuzzy::FuzzyGoal fuzzyOn;
                    fuzzyOn.order = true;
                    actionlib::SimpleClientGoalState state = ac_.sendGoalAndWait(fuzzyOn, ros::Duration(10.0), ros::Duration(1.0));
                    //bool success = ac_.waitForResult(ros::Duration(10.0));
                    cout << "fuzzy succss is " << state.state_ << endl;
                    if(state.state_ == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) { //REACTIVATES NAVIGATION
                        cout << "enviou nova rota" << endl;
                        //goToGoal.publish(goal);
                        //goal.target_pose.header= goal_go_dest.header;
                        //goal.target_pose.pose = goal_go_dest.pose;
                        goal.target_pose.header.stamp = ros::Time::now();
                        moveBaseClient.waitForServer();
                        moveBaseClient.sendGoal(goal, boost::bind(&Manager::doneNav, this, _1, _2),
                                                boost::bind(&Manager::activeNav, this),
                                                boost::bind(&Manager::feedbackCB, this, _1));
                    }
                    else {
                        result.result = false;
                        as_.setSucceeded(result);
                    }
                }
            }
            else if(nav_status == 2) {  //PREEMPTED
                cout << "Navigation is preempted" << endl;
                //result.result = false;
                //as_.setPreempted(result);
                /*if(fuzzyOn == 0) {
                    //cout << "Fuzzy to be turned on" << endl;
                    ac_.waitForServer();
                    navigation_fuzzy::FuzzyGoal goal;
                    goal.order = true;
                    ac_.sendGoal(goal);
                    fuzzyOn = 1;
                }
                else if(fuzzyOn == 1){
                    cout << "Fuzzy on" << endl;
                    bool success = ac_.waitForResult(ros::Duration(60.0));
                    if(success == true) { //REACTIVATES NAVIGATION
                        bool res = ac_.getResult()->res;
                        if(res == true) {
                            //goToGoal.publish(goal);
                            moveBaseClient.waitForServer();
                            moveBaseClient.sendGoal(goal);
                            fuzzyOn = -1;
                        }
                        else {
                            result.result = false;
                            as_.setSucceeded(result);
                        }
                    }
                    else { //ROBOT STUCK, FUZZY COULD NOT HANDLE
                        //cout << "fuzzy failed for some reason";
                        result.result = false;
                        as_.setSucceeded(result);
                    }
                }*/
                nav_status = -1;
            }
            else if(nav_status == 3) { //SUCCEEDED
                result.result = true;
                as_.setSucceeded(result);
                nav_status = -1;
            }
            else if(nav_status == 4) { //ABORTED
                result.result = false;
                as_.setAborted(result);
                nav_status = -1;
            }
            else {

            }

        }
        lr.sleep();
        ros::spinOnce();
	}
	result.result = false;
	as_.setSucceeded(result);
}
