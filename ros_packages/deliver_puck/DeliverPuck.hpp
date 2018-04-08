//
// Created by rafael on 07/11/17.
//

#ifndef VISAO_ENTREGAR_PUCK_H
#define VISAO_ENTREGAR_PUCK_H

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/UInt64.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Twist.h"
#include <opencv2/highgui/highgui.hpp>
#include <robotino_msgs/DigitalReadings.h>
#include "robotino_msgs/WorldState.h"
#include "robotino_msgs/PuckInfo.h"


using namespace ros;

class DeliverPuck
{

public:
    DeliverPuck();
    ~DeliverPuck();

    void spin();

private:
    // ROS Handles, Publishers, Subscriber and Messages
    NodeHandle n_;

    Publisher cmd_vel_pub;
    Publisher delivered_puck_pub;
    Publisher set_digital_readings_pub;

    Subscriber image_raw_sub;
    Subscriber distance_sensors_sub;
    Subscriber has_puck_sub;
    Subscriber world_state_sub;
    Subscriber action_id_sub;
    Subscriber digital_readings_sub;
    Subscriber puck_info_sub;
    Subscriber odometry_sub;

    ServiceClient client;

    geometry_msgs::Twist cmd_vel_msg;
    std_msgs::Bool deliver_puck_msg;
    robotino_msgs::DigitalReadings set_digital_readings_msg;

    //Parameters
    bool aligned_horizontal_flag;
    bool aligned_vertical_flag;
    bool stop_flag;
    bool finished_flag;
    bool has_puck_flag;
    bool right_sensor_flag;
    bool no_black_line_flag;
    bool left_sensor_flag;
    bool first_finish_call_flag;
    bool move_left_flag, move_right_flag;
    int node_loop_rate;
    double y_centroid;
    double dist_ir_2, dist_ir_9;
    double avg_slope;
    double avg_x_pos;
    long int state_id;
    int count_horizontal;
    int count_vertical;

    double world_state_x, world_state_y, world_state_theta;

    cv::Mat image_;

    // Constants
    float LINEAR_VEL{0.2f}, ANGULAR_VEL {-0.1f};
    float LINE_SLOPE_THRESHOLD {0.02};
    float OBSTACLE_STOP_DIST {0.60};
//    float CENTROID_STOP_DIST {87};
    float CENTROID_STOP_DIST {60};
//    float NO_IR_READING_DIST{0.55};
    int DELIVER_PUCK_ID{13};

    // Private Members Functions

    // Callbacks
    void visionCallback (const sensor_msgs::Image::ConstPtr& msg);
    void distanceCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void hasPuckCallback(const std_msgs::Bool::ConstPtr& msg);
    void worldStateCallback(const robotino_msgs::WorldState::ConstPtr& msg);
    void actionIdCallback (const std_msgs::UInt64::ConstPtr& msg);
    void digitalReadingsCallback (const robotino_msgs::DigitalReadings::ConstPtr& msg);
    void puckInfoCallback (const robotino_msgs::PuckInfo::ConstPtr& msg);

    //Helper functions
    void calculate_slope(std::vector<cv::Vec4i> lines);
    void align_horizontal();
    void align_vertical();
    void move_to_distribution_center(std::vector<cv::Vec4i> lines);
    void finish_delivery();
    void reset_odometry();

};


#endif //VISAO_ALIGNPUCK_HPP
