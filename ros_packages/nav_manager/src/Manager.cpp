#include "nav_manager/Manager.hpp"
#include <array>
#include <stdlib.h>

Manager::Manager(std::string name) :
	as_(nh, name, false),
	action_name(name),
	node_loop_rate(20),
	nav_status(0) {

	as_.registerGoalCallback(boost::bind(&Manager::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&Manager::preemptCB, this));	

	this->cancelGoal = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
	this->goToGoal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
	this->startFuzzy = nh.advertise<std_msgs::Bool>("response", 1);
	
	as_.start();
	


}

void Manager::goalCB() {
	goal = as_.acceptNewGoal()->destination;
	std::cout << "new goal set" << std::endl;
	//navigation should be able to overwrite old route
	goToGoal.publish(goal);
	nav_status = 1; //navigation has started

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
			std::cout << "Navigation active!" << std::endl;
			result.result = true;
			as_.setSucceeded(result);
		}

		lr.sleep();
		ros::spinOnce();
	}

	
}
