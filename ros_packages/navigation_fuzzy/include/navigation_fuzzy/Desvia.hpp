#ifndef Desvia_H
#define Desvia_H
#pragma once

#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <utility>
#include <vector>
#include "fl/Headers.h"
#include <actionlib/server/simple_action_server.h>
#include "navigation_fuzzy/FuzzyAction.h"

using namespace std;

class Desvia {

public:
	      Desvia(std::string name);
        void distanceSensorsCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
				void spin();
				//void executeCB(const fuzzy::FuzzyGoalConstPtr &goal);

				void goalCB();
        void preemptCB();
private:
        time_t name;
        ros::NodeHandle nh;
	      ros::Subscriber getDistanceSensors;
        ros::Publisher pubTwistMsg;
				actionlib::SimpleActionServer<navigation_fuzzy::FuzzyAction> as_;
				std::string action_name_;
				navigation_fuzzy::FuzzyFeedback feedback_;
        navigation_fuzzy::FuzzyResult result_;

				void print(const std::string str);

        int node_loop_rate;
				bool goal_;

				//float squareDistance(std::pair <float,float> point);

        /*fuzzy stuff*/
        fl::Engine *engine = new fl::Engine;
        fl::InputVariable *intDir = new fl::InputVariable;
        fl::InputVariable *dir = new fl::InputVariable;
        fl::InputVariable *intEsq = new fl::InputVariable;
        fl::InputVariable *esq = new fl::InputVariable;
        fl::OutputVariable *linX = new fl::OutputVariable;
        fl::OutputVariable *linY = new fl::OutputVariable;
        fl::OutputVariable *angular = new fl::OutputVariable;
        fl::RuleBlock *mamdani = new fl::RuleBlock;
};

#endif
