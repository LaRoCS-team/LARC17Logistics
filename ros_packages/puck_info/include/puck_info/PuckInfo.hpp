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

#include "Color.hpp"
#include "Centroid.hpp"

using std::vector;
using std::pair;
using std::pow;
using std::sqrt;

using cv::Mat;
using cv::Scalar;
using cv::Point;

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
    ros::NodeHandle nh_;
    ros::Publisher puck_info_pub_;
    ros::Subscriber dist_sensors_sub_;
    ros::Rate loop_rate_;

    image_transport::Subscriber img_sub_;

    image_transport::Publisher teste_pub_;

    cv::Mat frame_;

    vector<Color> colors_;

    double obj_min_size_;
    double obj_max_size_;

    sensor_msgs::ImagePtr msg;


    pair<int, Point> puck;
    Centroid puck_;

    // int CENTROID_Y_UPPER_BOUND {165};
    int CENTROID_Y_LOWER_BOUND {180};
    int CENTROID_X_LOWER_BOUND {135};
    int CENTROID_X_UPPER_BOUND {165};


    //pair<Mat, pair<Point, int> > filterCentroid(Mat frame, Mat mask, Mat labels, Mat status, Mat centroids, int i);

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void initColorVector();
    Centroid findPuck();
    void getMaskInRangeHSV(const Mat &hsv, const Scalar &min_color, const Scalar &max_color, Mat &mask);
    void getMaskRectangles(Mat &mask);
    void filterCentroid(Mat &mask, Mat labels, Mat status, Mat centroids, Color_e color_index,
                        Point &mask_centroid_point, int &mask_centroid_index);
    Centroid detectColor(const Mat &hsv, const Scalar &min_color,
                                                       const Scalar &max_color, Color_e color_index);

    image_transport::Subscriber imgSub;
    double distance_x_, distance_y_, distance_z_;
    bool grabbed_puck_ {false};
    bool has_puck_sensor_ {false};

    bool has_puck_ {false};




    void sensorCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void print(const std::string str);
};

#endif //PUCK_INFO_PUCKINFO_HPP
