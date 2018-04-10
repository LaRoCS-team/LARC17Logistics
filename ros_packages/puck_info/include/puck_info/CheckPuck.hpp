#ifndef CheckPuck_H
#define CheckPuck_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "robotino_msgs/PuckInfo.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <iostream>

using namespace cv;
using namespace std;

class CheckPuck{
public:
	CheckPuck();
	void sensorCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void pubBool();
    void spin();

private:
	bool hasPuck, puckCamera, puckSensor;
    bool ha;
	float distance;
		float distancey;
		float distancez;
	int color;
    double puck_center_Y_;
    int image_height_;
    int altura {240};

	int iLowH;
	int iHighH;
	int iLowS = 150; 
    int iHighS = 255;
    int iLowV = 60;
    int iHighV = 255;

	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sensorSub;
    ros::Subscriber puck_info_sub_;
    ros::Subscriber image_sub_;
    image_transport::Subscriber imgSub;
	//ros::Subscriber stateSub;
	ros::Rate loopRate;
	image_transport::Subscriber imageSub;

    void puckInfoCallback(const robotino_msgs::PuckInfo::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif
