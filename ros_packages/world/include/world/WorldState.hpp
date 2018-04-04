#ifndef WorldState_H
#define WorldState_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "world/Machine.hpp"
#include "world/DistrCenter.hpp"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include <std_msgs/UInt64.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <robotino_msgs/WorldState.h>
#include <robotino_msgs/StateId.h>
#include <robotino_msgs/PuckInfo.h>
#include "world/Pose2d.hpp"
#include <tf/transform_broadcaster.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <robotino_msgs/Machine.h>
#include <robotino_msgs/DistrCenter.h>
#include <string>
#include <list>
#include "robotino_msgs/ResetOdometry.h"


class WorldState{
public:
    WorldState();
    bool getHasPuck();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void hasPuckCallback(const std_msgs::Bool::ConstPtr& msg);
    void gotPuckCallback(const std_msgs::Bool::ConstPtr& msg);
    void leftPuckCallback(const std_msgs::Bool::ConstPtr& msg);
    void navGoalCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
    void puckInfoCallback(const robotino_msgs::PuckInfo::ConstPtr& msg);
    void actionCallback(const std_msgs::UInt64::ConstPtr& msg);
    void gotColorCallback(const std_msgs::Bool::ConstPtr& msg);
    void PublishLoop();
    void drawMap();
    void update();
private:
    int task;
    ros::ServiceClient client;
    std::vector<Machine> machines;
    std::vector<DistrCenter> dcs;
    int puckColor, nav_status, sorteio;
    std_msgs::UInt64 actMsg;
    bool hasPuck, gotPuck, leftPuck, navigating, inAction;
    bool startedDelivery,  gotColor;
    Pose2d robPosition;
    long stateId;
    robotino_msgs::ResetOdometry srv;
    std::vector<int> vpuck;
    long int action;
    long int nextAction();
    long int nextAction2(int action);


    // Variavies para o modulo 3
    int  lastColor          = -1;
    int  currentMachine     = 0;
    int  puckCount          = 0;
    bool  jumpNextActionCounter  = false;

//--------------------------------------
    bool discoveringMachine = true;
    bool returningToMachine = false;
    bool fetchingPuck       = false;
    bool identifyColor      = false;
    bool verifyColor        = false;

    int resetMachine    =0;

    int  destinyDC      = 0;
    bool goingToDC      = false;
    bool deliveringPuck = false;
//--------------------------------------
    bool pickPuck       = false;
    bool deliverPuck    = false;
    int  goToDc         = 0;
    int  totalDelivered = 0;

    grid_map::GridMap map;
    ros::NodeHandle nh;
    ros::Publisher statePub, rlPub, mapPub, actPub;
    ros::Subscriber odomSub, gotSub, leftSub, hasSub, goalSub, actionSub, gotColorSub;
    ros::Rate loopRate;
    tf::TransformBroadcaster mapBroadcaster;
};

std::vector<int> meter2grid(float x, float y, float resolution);

#endif
