#include "go_dest/go_action.hpp"

Go::Go(std::string name):as_(n, name, boost::bind(&Go::calcPose, this, _1), false),
    actionName(name){
    //subscribe to WorldState.
    subWorldState = n.subscribe("world_state", 1, &Go::world_state_msg_Callback, this);
    
    //initialize last_msg vectors.
    for(int i = 0; i < 12; i++)
        for(int j = 0; j < 3; j++)
            last_msg[i][j] = 0;
    
    //boot up server.
    as_.start();
}

void Go::calcPose(const go_dest::GoDestGoalConstPtr &goal){
    int Id = goal->order;
    
    //find the pose, since it has an ID.
    if(Id < 12){
        pose_full.header.frame_id = "map";
        quat = tf::createQuaternionMsgFromYaw(last_msg[Id][2]);
        pose_full.pose.position.x = last_msg[Id][0];
        pose_full.pose.position.y = last_msg[Id][1];;
        pose_full.pose.position.z = 0;
        pose_full.pose.orientation = quat;
    }
    if(Id == 15){
        //bot gets back to the initial pose.
        pose_full.header.frame_id = "odom";
        pose_full.pose.position.x = 0;
        pose_full.pose.position.y = 0;
        pose_full.pose.position.z = 0;
    }

    result_.sequence = this->sendGoalToNav(pose_full);
    
    as_.setSucceeded(result_);

}

void Go::world_state_msg_Callback(const robotino_msgs::WorldState::ConstPtr& msg){
    int i;

    //get the current poses from the WorldState.
    for(i = 0; i < 12; i++){
        if(i < 6){
            last_msg[i][0] = msg->machines[i].goalPose.x;
            last_msg[i][1] = msg->machines[i].goalPose.y;
            last_msg[i][2] = msg->machines[i].goalPose.theta;
        }
        else{
            last_msg[i][0] = msg->dcs[i - 6].goalPose.x;
            last_msg[i][1] = msg->dcs[i - 6].goalPose.y;
            last_msg[i][2] = msg->dcs[i - 6].goalPose.theta;
        }
    }
}

bool Go::sendGoalToNav(geometry_msgs::PoseStamped pose){
    actionlib::SimpleActionClient<nav_manager::NavManagerAction> ac("manager-node", true);

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
