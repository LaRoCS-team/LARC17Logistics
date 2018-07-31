#include "go_dest/go_action.hpp"

Go::Go(std::string name):as_(n, name, boost::bind(&Go::calcPose, this, _1), false),
    actionName(name){
    //subscribe to WorldState.
    subWorldState = n.subscribe("state", 1, &Go::world_state_msg_Callback, this);

    //initialize last_msg vectors.
    for(int i = 0; i < 13; i++)
        for(int j = 0; j < 3; j++)
            last_msg[i][j] = 0;

    //boot up server.
    as_.start();
}

void Go::calcPose(const go_dest::GoDestGoalConstPtr &goal){
    int id = goal->order;

    //find the pose, since it has an ID.
    pose_full.header.frame_id = "map";
    pose_full.pose.position.x = last_msg[id][0];
    pose_full.pose.position.y = last_msg[id][1];;
    pose_full.pose.position.z = 0;
    pose_full.pose.orientation =
      tf::createQuaternionMsgFromYaw(last_msg[id][2]);

    result_.sequence = this->sendGoalToNav(pose_full);

    std::cout << "Go_dest Log:" << std::endl;
    std::cout << " Received action " << id << std::endl;
    std::cout << " going to (" << last_msg[id][0] << ", " << last_msg[id][1]
       << ")" << std::endl;


    as_.setSucceeded(result_);

}

void Go::world_state_msg_Callback(const world::State::ConstPtr& msg){
    // Dock position
    last_msg[0][0] = msg->dock[0].goalPose.x;
    last_msg[0][1] = msg->dock[0].goalPose.y;
    last_msg[0][2] = msg->dock[0].goalPose.y;

    // Machines positions
    for(int i = 0; i < msg->machines.size(); i++){
      last_msg[i+1][0] = msg->machines[i].goalPose.x;
      last_msg[i+1][1] = msg->machines[i].goalPose.y;
      last_msg[i+1][2] = msg->machines[i].goalPose.theta;
    }

    // Docks positions
    // Notice that we always start in 7 (action >=7: go to a dc)
    for(int i = 0; i < msg->dcs.size(); i++){
      last_msg[i+7][0] = msg->dcs[i].goalPose.x;
      last_msg[i+7][1] = msg->dcs[i].goalPose.y;
      last_msg[i+7][2] = msg->dcs[i].goalPose.theta;
    }
}

bool Go::sendGoalToNav(geometry_msgs::PoseStamped pose){
    actionlib::SimpleActionClient<nav_manager::NavManagerAction> ac("nav_manager", true);

    ac.waitForServer(); //will wait for infinite time

    nav_manager::NavManagerGoal goal;
    goal.destination = pose;
    ac.sendGoal(goal); //will send a pose to NavManager

    //wait for the action to return
    bool success = ac.waitForResult(ros::Duration(60.0));

    if(success){
        success = ac.getResult()->result;
    }

    return success;
}
