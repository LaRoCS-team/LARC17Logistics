#include "go_dest/GoDest.hpp"

GoDest::GoDest(std::string name):
	as_(nh_, name, false),
	action_name_(name){

	//register goal and feedback callbacks
	as_.registerGoalCallback(boost::bind(&GoDest::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&GoDest::preemptCB, this));
		
	//subscribe to the data topic of interest
	sub_ = nh_.subscribe("/state", 1, &GoDest::stateCB, this);
	as_.start();
}

GoDest::~GoDest(){
}

//Callback to the state (poses of machines and DCs)
void GoDest::stateCB(const world::State::ConstPtr& msg){
	// read the machine's poses and Dc's poses
}

void GoDest::goalCB(){
	// accept the new goal
	goal_ = as_.acceptNewGoal()->id;
}

void GoDest::preemptCB(){
	as_.setPreempted();
}	
