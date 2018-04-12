#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "go_dest/GoDestAction.h"
#include "go_dest/GoDestResult.h"
#include "go_dest/GoDestFeedback.h"
#include "world/State.h"

class GoDest{
	public:
		GoDest(std::string name);
		~GoDest();

		//Action functions/callbacks
		void goalCB();
		void preemptCB();
		void stateCB(const world::State::ConstPtr& msg);

	private:
	    ros::NodeHandle nh_;
		actionlib::SimpleActionServer<go_dest::GoDestAction> as_;
		std::string action_name_;
		ros::Subscriber sub_;
		int goal_;
		go_dest::GoDestFeedback feedback_;
		go_dest::GoDestResult result_;
		//put pose variables/vectors here
};	
