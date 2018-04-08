#ifndef identify_puck_node_h
#define identify_puck_node_h

// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <robotino_msgs/PuckInfo.h>
#include "std_msgs/UInt64.h"

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// C++ libraries
#include <iostream>
#include <vector>
#include <utility>
#include <string>

using namespace cv;
using namespace std;
using namespace ros;

class IdentifyPuck
{
  public:
    IdentifyPuck();
    // Set true to enable debugging mode
    bool DEBUG;

    pair<int, Point> findPuck(Mat frame, vector<Scalar> colors, int desiredColor);
    void spin();

  private:
    // TODO: DELETA ISSO PF
    image_transport::Publisher teste_pub_;
    sensor_msgs::ImagePtr msg;

    unsigned long action_id_;
    double objMinSize;
    double objMaxSize;
    pair<int, Point> puck;

    pair<int, pair<Point, int> > detectColor(Mat frame, Mat hsv, Scalar minColor, Scalar maxColor, int colorIndex);
    Mat getTreatedInRangeHSV(Mat hsv, Scalar minColor, Scalar maxColor);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void actionIdCallback(const std_msgs::UInt64::ConstPtr& msg);
    pair<Mat, pair<Point, int> > filterCentroid(Mat frame, Mat mask, Mat labels, Mat status, Mat centroids, int i);
    vector<Scalar> initColorVector(vector<Scalar> colors);

    NodeHandle node;
    Publisher pub;
    Subscriber camSub;
    Subscriber actionIdSub;

    Rate loopRate;
    image_transport::Subscriber imgSub;
};

#endif
