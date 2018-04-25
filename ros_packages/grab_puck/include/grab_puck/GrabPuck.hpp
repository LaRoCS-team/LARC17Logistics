#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "grab_puck/GrabPuckAction.h"
#include <puck_info/PuckInfoMsg.h>

#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Twist.h"
#include "robotino_msgs/PuckInfo.h"
#include "std_msgs/UInt64.h"
//#include "robotino_msgs/DigitalReadings.h"

// THIS ENTIRE FILE IS A TEST

class GrabPuckAction {
private:
    // ROS Handle, ActionLib Server, Publishers, Subscribers and Messages and Results
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<grab_puck::GrabPuckAction> as_;
    std::string action_name_;

    ros::Publisher cmd_vel_pub_;

    ros::Subscriber distance_sensors_sub_;
    ros::Subscriber puck_info_sub_;

    geometry_msgs::Twist cmd_vel_msg_;
    //robotino_msgs::DigitalReadings led_msg_;

    grab_puck::GrabPuckResult result_;

    // ROS Parameters
    bool debug_mode_;

    // ROS spin loop rate
    int node_loop_rate_;

    // ActionLib goal variables
    int goal_;

    // Grab Puck Flags
    bool first_time_turn_, finished_grabbed_puck_;
    int side_turn_flag_;

    // Puck info informations
    double puck_center_X_, puck_center_Y_;
    bool has_puck_;
    int puck_color_;

    // IR sensors readings
    std::array<std::array<float, 3>, 9> dist_ir_;

    // Frontal euclidean norm distances
    std::array<double, 9> dist_norm_ir_;

    // Constants
    double CAMERA_WIDTH {320}, CAMERA_HEIGHT {240};
    double SPEED_VEL {0}, TURN_VEL {0};


    // Pub/Sub Callbacks
    void IRCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void puckInfoCallback(const puck_info::PuckInfoMsg::ConstPtr& msg);

    // ActionLib Callbacks
    void goalCB();
    void preemptCB();

    // Reseting functions
    void resetFlags();

    // Grab puck functions, go to the puck, turn to deliver and stop deliver
    void goToPuck();
    void turnToDeliver();
    void stopDeliver();

    // Aux grab puck functions
    void turnToDeliverSetSide();
    double calculateNormDistance(std::array<float, 3> &dist);
    void calculateFrontalDistances();
    void print(std::string str);

public:
    explicit GrabPuckAction(std::string name);
    ~GrabPuckAction();

    void spin();
};
