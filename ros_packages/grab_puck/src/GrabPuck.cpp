#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <grab_puck/GrabPuckAction.h>

// THIS ENTIRE FILE IS A TEST

class GrabPuckAction {
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<grab_puck::GrabPuckAction> as_;

    std::string action_name_;

    grab_puck::GrabPuckResult result_;

public:
    GrabPuckAction(std::string name) :
        as_(nh_, name, boost::bind(&GrabPuckAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    void executeCB(const grab_puck::GrabPuckGoalConstPtr &goal) {
        result_.color_grabbed_puck.data = goal->color_id.data;
        result_.grabbed_puck.data = true;
        as_.setSucceeded(result_);
    }
};

int main(int argc, char** argv) {
   ros::init(argc, argv, "grab_puck");

   GrabPuckAction grab_puck("grab_puck");
   ros::spin();

    return 0;
}
