//
// Created by previato on 08/04/18.
//

#include "puck_info/PuckInfo.hpp"
#include "puck_info/PuckInfoMsg.h"

#include <std_msgs/UInt64.h>

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

PuckInfo::PuckInfo(): loopRate(2)
{
    image_transport::ImageTransport imgTrans(node);
    imgSub = imgTrans.subscribe("image_raw", 2, &PuckInfo::imageCallback, this);
    sensor_sub_ = node.subscribe("distance_sensors", 10, &PuckInfo::sensorCallback, this);

    pub = node.advertise<puck_info::PuckInfoMsg>("puck_info", 10);

    image_transport::ImageTransport it(node);
    teste_pub_ = it.advertise("tamo_sofrendo", 1);

    bool debug_str;
    if (node.hasParam("puck_info/debug")) {
        node.getParam("puck_info/debug", debug_str);
        debug_mode_ = debug_str;
        std::cout << "Debug entry: " << debug_str << std::endl;
    }
    else {
        debug_mode_ = false;
    }

    bool vrep_str;
    if (node.hasParam("puck_info/vrep")) {
        node.getParam("puck_info/vrep", vrep_str);
        vrep_mode_ = vrep_str;
        std::cout << "V-REP entry: " << vrep_str << std::endl;
    }
    else {
        vrep_mode_ = false;
    }
}

void PuckInfo::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Treat image from ROS to OpenCV Mat format
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("PuckInfo: cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(frame_, cv_ptr->image);

    // V-REP image ins horizontally flipped
    if (vrep_mode_) {
        flip(frame_, frame_, 0);
    }

    // Color to be detected
    // [0] Min Blue
    // [1] Max Blue
    // [2] Min Red
    // [3] Max Red
    // [4] Min Yellow
    // [5] Max Yellow
    vector<Scalar> colors;
    colors = initColorVector(colors);

    //if(DEBUG)
    //resize(frame, frame, Size(), 0.5, 0.5);

    system("clear");
    std::cout << std::boolalpha;
    std::cout << "Debug mode: " << debug_mode_ << std::endl;
    std::cout << "V-REP mode: " << vrep_mode_ << std::endl;
    std::cout << "\n\n\n";


    puck = findPuck(frame_, colors, -1);
    // puck = pair<int, Point>(0, Point(-1, -1));

    if(puck.first == -1)
        print("Nothing found\n");
    else{
        print("Color: \t" + std::to_string(puck.first) + "\n");
        print("Centroid: \t" + std::to_string(puck.second.x) + " " + std::to_string(puck.second.y) + "\n");
    }

    //if(waitKey(30) >= 0 && DEBUG)
    //   break;
}

// Initialize the range of all three the detectable colors
vector<Scalar> PuckInfo::initColorVector(vector<Scalar> colors)
{
    // NO Color
    colors.push_back(Scalar(0, 0, 0));

    // NO Color
    colors.push_back(Scalar(0, 0, 0));

    // MIN Yellow 20
    colors.push_back(Scalar(20, 25, 0));

    // MAX Yellow 40
    colors.push_back(Scalar(40, 255, 255));

    // MIN Green 50
    colors.push_back(Scalar(50, 25, 0));

    // MAX Green 70
    colors.push_back(Scalar(70, 255, 255));

    // MIN Red 0
    colors.push_back(Scalar(0, 25, 0));

    // MAX Red 10
    colors.push_back(Scalar(10, 255, 255));

    // MIN object size
    objMinSize = 0;

    // Max Object size
    objMaxSize = 1.0;

    return colors;
}

pair<Mat, pair<Point, int> >
PuckInfo::filterCentroid(Mat frame, Mat mask, Mat labels, Mat status, Mat centroids, int i)
{
    int regionArea = 0;
    int regionLabelIndex = -1;
    int frameSize = frame.rows * frame.cols;
    Point chosenCentroid;

    for(int j = 0 ; j < status.rows ; j++)
    {
        //circle(frame, Point((int)(centroids.at<double>(i, 0)), (int)(centroids.at<double>(i, 1))), 5, Scalar(0, 255, 0), 5);
        if(status.at<int>(j, 4) > regionArea && j > 0)
            // if(status.at<int>(i, 4) > regionArea &&
            //    i > 0 &&
            //    status.at<int>(i, 4) >= objMinSize * frameSize &&
            //    status.at<int>(i, 4) <= objMaxSize * frameSize)
        {
            regionArea = status.at<int>(j, 4);
            regionLabelIndex = j;
        }
    }

    chosenCentroid = Point((int)(centroids.at<double>(regionLabelIndex, 0)), (int)(centroids.at<double>(regionLabelIndex, 1)));


    switch (i){
        case 2:
            circle(frame, Point((int)(centroids.at<double>(regionLabelIndex, 0)), (int)(centroids.at<double>(regionLabelIndex, 1))), 5, Scalar(0, 255, 255), 5);
            break;
        case 4:
            circle(frame, Point((int)(centroids.at<double>(regionLabelIndex, 0)), (int)(centroids.at<double>(regionLabelIndex, 1))), 5, Scalar(0, 255, 0), 5);
            break;
        case 6:
            circle(frame, Point((int)(centroids.at<double>(regionLabelIndex, 0)), (int)(centroids.at<double>(regionLabelIndex, 1))), 5, Scalar(255, 0, 0), 5);
            break;
        default:
            circle(frame, Point((int)(centroids.at<double>(regionLabelIndex, 0)), (int)(centroids.at<double>(regionLabelIndex, 1))), 5, Scalar(255, 0, 255), 5);
            print("DEU RUIM DE VEZ: " + std::to_string(i) + "\n");
    }

    if(regionLabelIndex == -1)
    {
        pair<Mat, pair<Point, int> > output(mask, pair<Point, int>(Point(-1, -1), 0));
        return output;
    }

    //if(DEBUG)
    //circle(frame, Point((int)(centroids.at<double>(regionLabelIndex, 0)), (int)(centroids.at<double>(regionLabelIndex, 1))), 5, Scalar(0, 255, 0), 5);

    for(int j = 0 ; j < mask.rows ; j++) {
        for (int k = 0; k < mask.cols; k++) {
            if (labels.at<int>(j, k) == regionLabelIndex) {
                mask.at<uchar>(j, k) = 255;
            } else {
                mask.at<uchar>(j, k) = 0;
            }
        }
    }

    pair<Mat, pair<Point, int> > output (mask, pair<Point, int>(chosenCentroid, status.at<int>(regionLabelIndex, 4)));
    return output;
}

// Filters the colors between min and max, also applies dilate and erode twice
// on the mask
Mat PuckInfo::getTreatedInRangeHSV(Mat hsv, Scalar minColor, Scalar maxColor)
{
    Mat treatedMask;

    int width = hsv.cols;
    int height = hsv.rows;

    inRange(hsv, minColor, maxColor, treatedMask);

    erode(treatedMask, treatedMask, Mat(), Point(-1, -1), 2);
    dilate(treatedMask, treatedMask, Mat(), Point(-1, -1), 2);

    rectangle(treatedMask, Point(0, (int)(height*0.57)), Point((int)(width*0.3), height), Scalar(0, 0, 0), -1);
    rectangle(treatedMask, Point((int)(width*0.7), (int)(height*0.57)), Point(width, height), Scalar(0, 0, 0), -1);

    return treatedMask;
}

pair<int, pair<Point, int> > PuckInfo::detectColor(Mat frame, Mat hsv, Scalar minColor, Scalar maxColor, int colorIndex)
{
    Mat mask, labels, status, centroids;
    pair<Mat, pair<Point, int> > maskCentroid;

    mask = getTreatedInRangeHSV(hsv, minColor, maxColor);

    connectedComponentsWithStats(mask, labels, status, centroids, 4);

    maskCentroid = filterCentroid(frame, mask, labels, status, centroids, colorIndex);

    Mat res;
    //bitwise_or(frame, frame, res, maskCentroid.first);
    msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();

    if(debug_mode_)
    {
        std::string win_name = "Color: ";
        win_name = win_name + std::to_string(colorIndex);

        Mat res_l;
        bitwise_or(frame, frame, res_l, maskCentroid.first);
        // imshow(win_name + '1', maskCentroid.first);
        imshow(win_name, res_l);
        // imshow("Frame", frame);
        waitKey(30);
    }

    return pair<int, pair<Point, int> >(colorIndex, pair<Point, int>(maskCentroid.second.first, maskCentroid.second.second));
}

// Search for a puck in the frame, frame has colorspace RGB
pair<int, Point> PuckInfo::findPuck(Mat frame, vector<Scalar> colors, int desiredColor)
{
    Mat hsv;
    pair<int, pair<Point, int> > centroid;
    int larger = 0, largerId = 0;
    pair<int, pair<Point, int> > largerCentroid;


    cvtColor(frame, hsv, cv::COLOR_RGB2HSV);

    if(desiredColor != -1)
        desiredColor *= 2;

    if(desiredColor == -1) {
        for(int i = 2 ; i < colors.size() ; i += 2)
        {
            centroid = detectColor(frame, hsv, colors[i], colors[i + 1], i);
            //cout << "Y: " << colors[2].val[0] << " " << colors[2+1].val[0] << endl;
            //centroid = detectColor(frame, hsv, colors[2], colors[2 + 1], i);


            if(centroid.second.first.x != -1 && centroid.second.first.y != -1 && centroid.second.second > larger)
            {
                larger = centroid.second.second;
                largerId = i;
                largerCentroid = centroid;
            }

           print("CentroidArea: " + std::to_string(centroid.second.second) + "\nLarger: " + std::to_string(larger) + "\n");
        }
    }
    else {
        centroid = detectColor(frame, hsv, colors[desiredColor], colors[desiredColor + 1], desiredColor);
        return pair<int, Point>(desiredColor / 2, largerCentroid.second.first);
    }

    return pair<int, Point>(largerId / 2, largerCentroid.second.first);
}

double normDistance(const double a, const double b, const double c) {
    return sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
}

double sigmoidFunction(double a) {
    return 1 / (1 + std::exp(-a));
}

double parSigmoidFunction(double a, double par1, double par2) {
    return sigmoidFunction((a + par1) * par2);
}

void PuckInfo::sensorCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    //cout << "Estou no callback" << endl;
    distance_x_ = msg->points[0].x;
    distance_y_ = msg->points[0].y;
    distance_z_ = msg->points[0].z;

    double distance = normDistance(distance_x_, distance_y_, distance_z_);

    double weight_sensors = parSigmoidFunction(-distance, 0.22, 40);
    double weight_camera = parSigmoidFunction(puck.second.y, -frame_.rows/2, 8.0/frame_.rows);

    double puck_prob = weight_camera + weight_sensors;

    // Prob approach
    has_puck_ = puck_prob >= 1.0;

    std::string has_puck_str;
    if (has_puck_) has_puck_str = "true";
    else has_puck_str = "false";

    print("HasPuck: \t" + has_puck_str + "\n");

    // Linear approach
    /*if(distance < 0.25) {
        grabbed_puck_ = true;
        has_puck_sensor_ = true;
        //std::cout<< "PuckSensor:" << puckSensor<<"\n";
    }
    else {
        if (grabbed_puck_ && distance < 0.3) {
            has_puck_sensor_ = true;
        }
        else {
            has_puck_sensor_ = false;
            grabbed_puck_ = false;
        }
    }

    has_puck_ = has_puck_sensor_;*/
    //ROS_INFO("ha = %s, distancex = %f, distancey = %f, distancez = %f", ha?"true":"false", distance, distancey, distancez);
}


void PuckInfo::spin()
{
    ros::Rate lr(loopRate);
    while (node.ok()) {
        // Setting ROS mesage
        puck_info::PuckInfoMsg puck_info_msg;
        puck_info_msg.color = puck.first;
        puck_info_msg.center.x = puck.second.x;
        puck_info_msg.center.y = puck.second.y;
        puck_info_msg.center.z = 0;
        puck_info_msg.has_puck = has_puck_;
//        puck_info_msg.has_puck = true;

        pub.publish(puck_info_msg);
        teste_pub_.publish(msg);

        ros::spinOnce();
        lr.sleep();
    }
}
