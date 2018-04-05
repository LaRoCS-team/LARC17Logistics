#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <grab_puck/GrabPuckAction.h>

#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Twist.h"
#include "robotino_msgs/PuckInfo.h"
#include "std_msgs/UInt64.h"
//#include "robotino_msgs/DigitalReadings.h"

#include <opencv2/highgui/highgui.hpp>

// THIS ENTIRE FILE IS A TEST

class GrabPuckAction {
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<grab_puck::GrabPuckAction> as_;

    std::string action_name_;

    grab_puck::GrabPuckResult result_;

    // ROS Handles, Publishers, Subscriber and Messages

    ros::Publisher cmd_vel_pub_;

    ros::Subscriber distance_sensors_sub_;
    ros::Subscriber puck_info_sub_;

    geometry_msgs::Twist cmd_vel_msg_;
    std_msgs::Bool got_puck_msg_;
    //robotino_msgs::DigitalReadings led_msg_;

    // Parameters
    double puck_center_X_, puck_center_Y_;
    int turn_flag_, forward_flag_;
    bool has_puck_flag_, first_time_turn_;
    int node_loop_rate_;
    unsigned int puck_color_;
    unsigned long action_id_;
    std::array<std::array<float, 3>, 9> dist_ir_;
    cv::Mat image_;
    int side_turn_flag_;
    // Frontal euclidean norm distances
    double dist_norm_ir_2_, dist_norm_ir_3_, dist_norm_ir_8_, dist_norm_ir_9_;
    double dist_norm_ir_5_, dist_norm_ir_6_;
    // Constants
    int CAMERA_WIDTH {320}, CAMERA_HEIGHT {240};
    float SPEED_VEL {0}, TURN_VEL {0};
    float PUCK_DISTANCE_REDUCE_VEL {0.3};
    int debug_mode_;

    bool finished_grabbed_puck_;

    // Private Members Functions
    // Callbacks
    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
    void IRCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void hasPuckCallback(const std_msgs::Bool::ConstPtr& msg);
    void puckInfoCallback(const robotino_msgs::PuckInfo_<std::allocator<void>>::ConstPtr &msg);
    void actionIdCallback(const std_msgs::UInt64::ConstPtr& msg);
    // Movement flags setters
    void turnLeftFlag();
    void turnRightFlag();
    void turnStopFlag();
    void forwardFlag();
    void forwardStopFlag();
    void controlSpeed(int sub_action);

    // Go and take puck functions
    void goToPuck();
    void turnToDeliver();

    // Auxiliary calc functions
    double calculateNormDistance(std::array<float, 3> &dist);
    void calculateFrontalDistances();

    void ledPubPega(int color);

    void turnToDeliverSetSide();
    void print(const std::string str);

public:
    GrabPuckAction(std::string name);
    ~GrabPuckAction();

    void executeCB(const grab_puck::GrabPuckGoalConstPtr &goal);
};
