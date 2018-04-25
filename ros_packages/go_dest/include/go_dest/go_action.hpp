#ifndef _GO_DEST_SERVER_H
#define _GO_DEST_SERVER_H

#include <ros/ros.h>
#include <iostream>

//transformations
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

//msgs
#include <std_msgs/Int64.h>
#include "world/State.h"

//server
#include <actionlib/server/simple_action_server.h>
#include <go_dest/GoDestAction.h>

//client
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_manager/NavManagerAction.h>

class Go{
    public:
        Go(std::string name);
        void calcPose(const go_dest::GoDestGoalConstPtr &goal);
        void world_state_msg_Callback(const world::State::ConstPtr& msg);
        bool sendGoalToNav(geometry_msgs::PoseStamped pose);
    private:
        float last_msg[12][3];
        geometry_msgs::Quaternion quat;
        geometry_msgs::PoseStamped pose_full;
        ros::NodeHandle n;
        ros::Subscriber subWorldState;
        actionlib::SimpleActionServer<go_dest::GoDestAction> as_;
        go_dest::GoDestFeedback feedback_;
        go_dest::GoDestResult result_;
        std::string actionName;
};

#endif
