//
// Created by previato on 08/04/18.
//

#include "puck_info/NewPuckInfo.hpp"
#include "puck_info/PuckInfoMsg.h"

#include <std_msgs/UInt64.h>
#include <puck_info/Centroid.hpp>
#include <puck_info/PuckInfo.hpp>

using cv::Mat;
using cv::flip;
using cv::Scalar;
using cv::imshow;
using cv::waitKey;

int main(int argc, char** argv) {
    ros::init(argc, argv, "puck_info");

    PuckInfo puck_info;
    puck_info.spin();

    return 0;
}

void PuckInfo::print(const std::string str) {
    if (debug_mode_) {
        std::cout << str;
    }
}

PuckInfo::PuckInfo(): loop_rate_{2}, obj_min_size_{0}, obj_max_size_{1.0}
{
    // We obtain the image from ROS using the image_transport package, a package that
    // convert images from ROS to OpenCV
    image_transport::ImageTransport raw_img_transport(nh_);
    img_sub_ = raw_img_transport.subscribe("image_raw", 2, &PuckInfo::imageCallback, this);

    dist_sensors_sub_ = nh_.subscribe("distance_sensors", 10, &PuckInfo::sensorCallback, this);

    puck_info_pub_ = nh_.advertise<puck_info::PuckInfoMsg>("puck_info", 10);

    // We send the image with the puck's centroid obtained with OpenCV to a ROS topic
    // using the image_transport package
    image_transport::ImageTransport image_with_centroids(nh_);
    teste_pub_ = image_with_centroids.advertise("tamo_sofrendo", 1);

    // Debug parameter
    bool debug_str;
    if (nh_.hasParam("puck_info/debug")) {
        nh_.getParam("puck_info/debug", debug_str);
        debug_mode_ = debug_str;
        std::cout << "Debug entry: " << debug_str << std::endl;
    }
    else {
        debug_mode_ = false;
    }

    // Vrep parameter
    bool vrep_str;
    if (nh_.hasParam("puck_info/vrep")) {
        nh_.getParam("puck_info/vrep", vrep_str);
        vrep_mode_ = vrep_str;
        std::cout << "V-REP entry: " << vrep_str << std::endl;
    }
    else {
        vrep_mode_ = false;
    }

    initColorVector();
}

void PuckInfo::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Treat image from ROS to OpenCV Mat format
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("PuckInfo: cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(frame_, cv_ptr->image);

    // V-REP image ins horizontally flipped
    if (vrep_mode_) {
        flip(frame_, frame_, 0);
    }
}

// Initialize the range of all three the detectable colors
void PuckInfo::initColorVector()
{
    std::vector<int> color_min_from_param;
    std::vector<int> color_max_from_param;

    ros::param::get("/MinYellow", color_min_from_param);
    // MIN Yellow 20

    ros::param::get("/MaxYellow", color_max_from_param);
    // MAX Yellow 40

    colors_.emplace_back(Color(color_min_from_param[0], color_min_from_param[1], color_min_from_param[2],
                         color_max_from_param[0], color_max_from_param[1], color_max_from_param[2],
                         Color_e::yellow));

    ros::param::get("/MinGreen", color_min_from_param);
    // MIN Green 50

    ros::param::get("/MaxGreen", color_max_from_param);
    // MAX Green 70

    colors_.emplace_back(Color(color_min_from_param[0], color_min_from_param[1], color_min_from_param[2],
                         color_max_from_param[0], color_max_from_param[1], color_max_from_param[2],
                         Color_e::green));

    ros::param::get("/MinRed", color_min_from_param);
    // MIN Red 0

    ros::param::get("/MaxRed", color_max_from_param);
    // MAX Red 10

    colors_.emplace_back(Color(color_min_from_param[0], color_min_from_param[1], color_min_from_param[2],
                         color_max_from_param[0], color_max_from_param[1], color_max_from_param[2],
                         Color_e::red));
}

void PuckInfo::filterCentroid(Mat &mask, Mat labels, Mat status, Mat centroids, Color_e color_index,
                              Point &mask_centroid_point, int &mask_centroid_index)
{
    int region_area = 0;
    int region_label_index = -1;

    for(int j = 0; j < status.rows; j++) {
        if(status.at<int>(j, 4) > region_area && j > 0) {
            region_area = status.at<int>(j, 4);
            region_label_index = j;
        }
    }

    Point chosen_centroid {(int)(centroids.at<double>(region_label_index, 0)), (int)(centroids.at<double>(region_label_index, 1))};

    switch (color_index){
        case Color_e::yellow:
            circle(frame_, Point((int)(centroids.at<double>(region_label_index, 0)), (int)(centroids.at<double>(region_label_index, 1))), 5, Scalar(0, 255, 255), 5);
            break;
        case Color_e::green:
            circle(frame_, Point((int)(centroids.at<double>(region_label_index, 0)), (int)(centroids.at<double>(region_label_index, 1))), 5, Scalar(0, 255, 0), 5);
            break;
        case Color_e::red:
            circle(frame_, Point((int)(centroids.at<double>(region_label_index, 0)), (int)(centroids.at<double>(region_label_index, 1))), 5, Scalar(255, 0, 0), 5);
            break;
        default:
            std::cout << "Preguiça de descrever o erro agora, contatar o Previato" << std::endl;
            exit(1);
    }

    if(region_label_index == -1)
    {
        mask_centroid_point = Point(-1, -1);
        mask_centroid_index = 0;
    }

    for(int j = 0; j < mask.rows; j++) {
        for (int k = 0; k < mask.cols; k++) {
            if (labels.at<int>(j, k) == region_label_index) {
                mask.at<uchar>(j, k) = 255;
            } else {
                mask.at<uchar>(j, k) = 0;
            }
        }
    }

    mask_centroid_point = chosen_centroid;
    mask_centroid_index = status.at<int>(region_label_index, 4);
}

// Filters the colors between min and max, also applies dilate and erode twice
// on the mask
void PuckInfo::getMaskInRangeHSV(const Mat &hsv, const Scalar &min_color, const Scalar &max_color, Mat mask)
{
    inRange(hsv, min_color, max_color, mask);

    erode(mask, mask, Mat(), Point(-1, -1), 2);
    dilate(mask, mask, Mat(), Point(-1, -1), 2);
}

void PuckInfo::getMaskRectangles(Mat mask)
{
    int width = mask.cols;
    int height = mask.rows;

    rectangle(mask, Point(0, (int)(height*0.57)), Point((int)(width*0.3), height), Scalar(0, 0, 0), -1);
    rectangle(mask, Point((int)(width*0.7), (int)(height*0.57)), Point(width, height), Scalar(0, 0, 0), -1);
}

Centroid PuckInfo::detectColor(const Mat &hsv, const Scalar &min_color,
                                                   const Scalar &max_color, Color_e color_index)
{
    Mat mask;

    Mat labels, status, centroids;
    pair<Mat, pair<Point, int> > maskCentroid;

    getMaskInRangeHSV(hsv, min_color, max_color, mask);

    getMaskRectangles(mask);

    connectedComponentsWithStats(mask, labels, status, centroids, 4);

    Point mask_centroid_point;
    int mask_centroid_index;

    filterCentroid(mask, labels, status, centroids, color_index, mask_centroid_point,
                   mask_centroid_index);

    Mat res;
    msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame_).toImageMsg();

    if(debug_mode_)
    {
        std::string win_name = "Color: ";
        win_name = win_name + std::to_string(color_index);

        Mat res_l;
        bitwise_or(frame_, frame_, res_l, maskCentroid.first);
        // imshow(win_name + '1', maskCentroid.first);
        imshow(win_name, res_l);
        // imshow("Frame", frame);
        waitKey(30);
    }

    return {color_index, mask_centroid_point, mask_centroid_index};
}

// Search for a puck in the frame, frame has colorspace RGB
Centroid PuckInfo::findPuck()
{
    Mat hsv;

    int larger_area = 0;
    Centroid larger_centroid;


    cvtColor(frame_, hsv, cv::COLOR_BGR2HSV);

    for(auto& color : colors_)
    {
        Centroid centroid = detectColor(hsv, color.min, color.max, color.color);

        if(centroid.point.x != -1 && centroid.point.y != -1 && centroid.index > larger_area)
        {
            larger_area = centroid.index;
            larger_centroid = centroid;
        }

       print("CentroidArea: " + std::to_string(centroid.index) + "\nLarger: " + std::to_string(larger_area) + "\n");
    }

    return larger_centroid;
}

void PuckInfo::sensorCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    //Empirical approach
    //Simulator measures indicate that has_puck should be true when centroid_y >= 218 and 135 <= centroid_x <= 165
    //In theory, centroid_x could be bounded between 145 and 155, but the oscillations indicate it is best to use a wider range.
    has_puck_ = (puck.second.y >= CENTROID_Y_LOWER_BOUND && CENTROID_X_LOWER_BOUND <= puck.second.x <= CENTROID_X_UPPER_BOUND);

    std::string has_puck_str;
    if (has_puck_) has_puck_str = "true";
    else has_puck_str = "false";

    print("HasPuck: \t" + has_puck_str + "\n");
}


void PuckInfo::spin()
{
    while (nh_.ok()) {
        Centroid puck = findPuck();

        system("clear");
        std::cout << std::boolalpha;
        std::cout << "Debug mode: " << debug_mode_ << std::endl;
        std::cout << "V-REP mode: " << vrep_mode_ << std::endl;
        std::cout << "\n\n\n";

        if(puck.index == -1) {
            print("Nothing found\n");
        }
        else {
            print("Color: \t" + std::to_string(puck.color) + "\n");
            print("Centroid: \t" + std::to_string(puck.point.x) + " " + std::to_string(puck.point.y) + "\n");
        }

        // Setting ROS mesage
        puck_info::PuckInfoMsg puck_info_msg;
        puck_info_msg.color = static_cast<int>(puck.color);
        puck_info_msg.center.x = puck.point.x;
        puck_info_msg.center.y = puck.point.y;
        puck_info_msg.center.z = 0;
        puck_info_msg.has_puck = static_cast<unsigned char>(has_puck_);

        puck_info_pub_.publish(puck_info_msg);
        teste_pub_.publish(msg);

        ros::spinOnce();
        loop_rate_.sleep();
    }
}
