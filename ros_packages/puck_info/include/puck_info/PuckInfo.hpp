//
// Created by previato on 08/04/18.
//

#ifndef PUCK_INFO_PUCKINFO_HPP
#define PUCK_INFO_PUCKINFO_HPP

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
#include <cmath>

using std::vector;
using std::pair;
using std::pow;
using std::sqrt;

using cv::Mat;
using cv::Scalar;
using cv::Point;

using namespace ros;

class PuckInfo {
public:
    PuckInfo();
    // Set true to enable debugging mode
    bool debug_mode_;
    bool vrep_mode_;

    pair<int, Point> findPuck(Mat frame, vector<Scalar> colors, int desiredColor);
    void spin();

private:
    // TODO: DELETA ISSO PF
    image_transport::Publisher teste_pub_;
    sensor_msgs::ImagePtr msg;

    double objMinSize;
    double objMaxSize;
    pair<int, Point> puck;

    int CENTROID_Y_LOWER_BOUND {200};
    int CENTROID_X_LOWER_BOUND {135};
    int CENTROID_X_UPPER_BOUND {165};

    pair<int, pair<Point, int> > detectColor(Mat frame, Mat hsv, Scalar minColor, Scalar maxColor, int colorIndex);
    Mat getTreatedInRangeHSV(Mat hsv, Scalar minColor, Scalar maxColor);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    pair<Mat, pair<Point, int> > filterCentroid(Mat frame, Mat mask, Mat labels, Mat status, Mat centroids, int i);
    vector<Scalar> initColorVector(vector<Scalar> colors);

    NodeHandle node;
    Publisher pub;

    Rate loopRate;
    image_transport::Subscriber imgSub;
    double distance_x_, distance_y_, distance_z_;
    bool grabbed_puck_ {false};
    bool has_puck_sensor_ {false};

    bool has_puck_ {false};

    cv::Mat frame_;

    void print(const std::string str);

    void checkPuck();
};


#endif //PUCK_INFO_PUCKINFO_HPP
