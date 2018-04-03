#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/GrabPuckAction.h>

// THIS ENTIRE FILE IS A TEST

class GrabPuckAction {
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<actionlib::GrabPuckAction> as_;

    std::string action_name_;

    actionlib::GrabPuckResult result_;

public:
    GrabPuckAction(std::string name) :
        as_(nh_, name, boost::bind(&GrabPuckAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    void executeCB(const actionlib::GrabPuckGoalConstPtr &goal) {
        result_.color_grabbed_puck = goal.color_id;
        result_.grabbed_puck = true;
        as_.setSucceeded(result_);
    }
};

int main(int argc, char** argv) {
   ros::init(argc, argv, "grab_puck");

   GrabPuckAction grab_puck("grab_puck");
   ros::spin();

    return 0;
}
