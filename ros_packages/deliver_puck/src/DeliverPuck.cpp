//
// Created by rafael on 05/11/17.
//

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include "robotino_msgs/ResetOdometry.h"
#include "../include/DeliverPuck.hpp"


using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "deliver_puck");

    DeliverPuck deliver_puck("deliver_puck");
    ros::spin();

    return 0;
}

void DeliverPuck::print(const std::string str) {
    if (debug_mode_) {
        std::cout << str << std::endl;
    }
}

DeliverPuck::DeliverPuck(std::string name) :
        as_(n_, name, boost::bind(&DeliverPuck::executeCB, this, _1), false),
        action_name_(name),
        node_loop_rate(20)
{
    //Topic you want to publish
    cmd_vel_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    //delivered_puck_pub = n_.advertise<std_msgs::Bool>("delivered_puck", 1000);
    //client = n_.serviceClient<robotino_msgs::ResetOdometry>("reset_odometry");
    //set_digital_readings_pub = n_.advertise<robotino_msgs::DigitalReadings>("set_digital_values", 100);

    //Topic you want to subscribe
    image_raw_sub = n_.subscribe("image_raw", 1000, &DeliverPuck::visionCallback, this);
    distance_sensors_sub = n_.subscribe("distance_sensors", 1000, &DeliverPuck::distanceCallback, this);
    //has_puck_sub = n_.subscribe("hasPuck", 1000, &DeliverPuck::hasPuckCallback, this);
    //world_state_sub = n_.subscribe("world_state", 1000, &DeliverPuck::worldStateCallback, this);
    //action_id_sub = n_.subscribe("action_id", 1000, &DeliverPuck::actionIdCallback, this);
    digital_readings_sub = n_.subscribe("digital_readings", 1000, &DeliverPuck::digitalReadingsCallback, this);
    puck_info_sub = n_.subscribe("puck_info", 1000, &DeliverPuck::puckInfoCallback, this);

    aligned_horizontal_flag = false;
    stop_flag = false;
    finished_flag = false;
    no_black_line_flag = false;
    first_finish_call_flag = false;
    aligned_vertical_flag = false;
    move_left_flag = false;
    move_right_flag = false;

    for (int i = 0; i < 12; i++)
    {
        (set_digital_readings_msg.values).push_back(0);
    }

    as_.start();
}

DeliverPuck::~DeliverPuck()
{
    cmd_vel_pub.shutdown();
    delivered_puck_pub.shutdown();

    image_raw_sub.shutdown();
    distance_sensors_sub.shutdown();
    has_puck_sub.shutdown();
    world_state_sub.shutdown();
    action_id_sub.shutdown();
    digital_readings_sub.shutdown();
    puck_info_sub.shutdown();
}

void DeliverPuck::puckInfoCallback (const puck_info::PuckInfoMsg::ConstPtr& msg) {
    y_centroid = msg->center.y;
    has_puck_flag = msg->has_puck;
//    ROS_INFO("y_centroid = %g", y_centroid);

}

void DeliverPuck::digitalReadingsCallback(const robotino_msgs::DigitalReadings::ConstPtr &msg) {
    //0 ta na direita
    //1 ta na esquerda
//    ROS_INFO("Inside digitalReadingsCallback");
    right_sensor_flag = msg->values[0];
    left_sensor_flag = msg->values[1];
}

void DeliverPuck::actionIdCallback (const std_msgs::UInt64::ConstPtr& msg) {
    state_id = msg->data;
}

void DeliverPuck::worldStateCallback(const robotino_msgs::WorldState::ConstPtr &msg) {
    world_state_x = msg->machines[0].goalPose.x;
    world_state_y = msg->machines[0].goalPose.y;
    world_state_theta = msg->machines[0].goalPose.theta;
}

void DeliverPuck::hasPuckCallback(const std_msgs::Bool::ConstPtr& msg) {
    has_puck_flag = msg->data;
}

void DeliverPuck::distanceCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    dist_ir_2 = sqrt(pow(msg->points[1].x, 2) + pow(msg->points[1].y, 2));
    dist_ir_9 = sqrt(pow(msg->points[8].x, 2) + pow(msg->points[8].y, 2));
}

void DeliverPuck::visionCallback (const sensor_msgs::Image::ConstPtr& msg) {
    Mat src;
    cv_bridge::CvImageConstPtr cv_cptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    src = cv_cptr->image;

//    cv::inRange(src, cv::Scalar(0, 0, 0), cv::Scalar(30, 30, 30),src);

    cv::flip(src, src, 0);
    Canny(src, image_, 100, 300, 3);

}

void DeliverPuck::calculate_slope(vector<cv::Vec4i> lines) {
    double  mSum = 0;
    count_horizontal = 0;
    count_vertical = 0;
    double x0, y0, x1, y1, m;

    for(int i = 0; i < lines.size(); i++ )
    {
        x0 = lines[i][0];
        y0 = lines[i][1];
        x1 = lines[i][2];
        y1 = lines[i][3];

        m = (y1 - y0)/(x1 - x0);

        if (m > -.4 && m < .4) {
            ROS_INFO("slope of countable line = %g", m);
            mSum += m;
            count_horizontal++;
        }

        if (m < -.4 || m > .4) {
            avg_x_pos = (x0+x1)/2;
            count_vertical++;
            ROS_INFO ("avgX = %g, slope = %g", avg_x_pos, m);
        }
    }

    if (count_horizontal != 0) {
        avg_slope = mSum/count_horizontal;
        ROS_INFO("mSum = %f, count_horizontal = %d, Avg slope = %f", mSum, count_horizontal, avg_slope);
    }
    else {
        ROS_INFO("No readable lines detected (|slope| > 1)!");
        avg_slope = 1000;
    }

}

void DeliverPuck::align_horizontal() {
    if (avg_slope < -LINE_SLOPE_THRESHOLD || avg_slope > LINE_SLOPE_THRESHOLD) {
        if (avg_slope < 0) {
            ROS_INFO("Estamos na condicao avg slope < 0");
            cmd_vel_msg.angular.z = -ANGULAR_VEL;
        }
        else {
            ROS_INFO("Estamos na condicao avg slope > 0");
            cmd_vel_msg.angular.z = ANGULAR_VEL;
        }
    }
    else {
        aligned_horizontal_flag = true;
    }
    ROS_INFO("Avg slope = %g", avg_slope);
}

//Estamos assumindo que no máximo uma linha vertical será escaneada.
void DeliverPuck::align_vertical() {
    ROS_INFO("Entramos no ALINE_VERTICAL");
    cmd_vel_msg.linear.x = 0;
    if (!aligned_vertical_flag) {
        if (count_vertical != 0) {
            if (!move_right_flag && !move_left_flag) {
                if (avg_x_pos < 120) {
                    move_right_flag = true;
                } else {
                    move_left_flag = true;
                }
            } else {
                if (move_left_flag) {
                    if (avg_x_pos < 285) {
                        cmd_vel_msg.linear.y = LINEAR_VEL;
                    } else {
                        aligned_vertical_flag = true;
                        cmd_vel_msg.linear.y = 0;
                    }
                } else {
                    if (avg_x_pos > 35) {
                        cmd_vel_msg.linear.y = -LINEAR_VEL;
                    } else {
                        aligned_vertical_flag = true;
                        cmd_vel_msg.linear.y = 0;
                    }
                }
            }

        } else {
            aligned_vertical_flag = true;
        }
    }
}



void DeliverPuck::move_to_distribution_center(vector<cv::Vec4i> lines) {

    cmd_vel_msg.linear.y = 0;
//    if (!no_black_line_flag) {
//        if (count_horizontal == 0) {
//            no_black_line_flag = true;
//        }
//        else {
//            cmd_vel_msg.linear.x = .1;
//        }
//
//    }
//    else {
    if (right_sensor_flag && left_sensor_flag) {
        cmd_vel_msg.linear.x = 0;
        stop_flag = true;
    }
    else {
        cmd_vel_msg.linear.x = .1;
    }
 //   }
}

void DeliverPuck::finish_delivery() {
    ROS_INFO("y_centroid = %g", y_centroid);
    if (y_centroid > CENTROID_STOP_DIST) {
        cmd_vel_msg.linear.x = -LINEAR_VEL;
    }
    else {
        finished_flag = true;
    }
}

void DeliverPuck::reset_odometry() {
    robotino_msgs::ResetOdometry srv;
    srv.request.x = world_state_x;
    srv.request.y = world_state_y;
    srv.request.phi = world_state_theta;
    ROS_INFO("world state x = %f, y = %f, theta = %f", world_state_x, world_state_y, world_state_theta);

    if (client.call(srv))
    {
        ROS_INFO("True!!! Resetou!");
    }
    else {
        ROS_ERROR("Failed to call service");
    }

}

void DeliverPuck::executeCB(const deliver_puck::DeliverPuckGoalConstPtr &goal) {
    ROS_INFO("We received the goal %d", goal->action_id);
    ros::Rate lr(node_loop_rate);
    ROS_INFO("We're inside executeCB");
    while (!deliver_puck_msg.data) {
        //if (state_id == DELIVER_PUCK_ID) {
        std::vector<cv::Vec4i> lines;
        HoughLinesP(image_, lines, 1, CV_PI / 180, 75, 50, 10);
//        ROS_INFO("lines = %ld", lines.size());

        cmd_vel_msg.angular.z = 0;
        cmd_vel_msg.linear.x = 0;
        calculate_slope(lines);
        if (!has_puck_flag) {
            ROS_INFO("Has puck is false");
            //TODO:Condicao para indicar que o comportamento falhou e devemos mudá-lo
            deliver_puck_msg.data = static_cast<unsigned char>(true);
        } else {
            ROS_INFO("Has puck is true");
            deliver_puck_msg.data = static_cast<unsigned char>(false);
            if (!aligned_horizontal_flag) {
                ROS_INFO("Align horizontal is false");
                align_horizontal();
            } else {
                if (!aligned_vertical_flag) {
                    ROS_INFO("Align vertical is false");
                    align_vertical();
                } else {
                    if (!stop_flag) {
                        ROS_INFO("Stop flag is false");
                        move_to_distribution_center(lines);
                    } else {
                        if (!first_finish_call_flag) {
                            ROS_INFO("First finish flas is false");
//                            reset_odometry();
//                            set_digital_readings_pub.publish(set_digital_readings_msg);
                            first_finish_call_flag = true;

                        } else {
                            if (!finished_flag) {
                                ROS_INFO("Finished flag is false");
                                finish_delivery();
                            } else {
                                deliver_puck_msg.data = static_cast<unsigned char>(true);
                            }
                        }

                    }
                }

            }
        }
//            cmd_vel_msg.linear.y = .1;
//            if (avgX > 280) {
//                cmd_vel_msg.linear.y = 0;
//            }
        ROS_INFO("linear.x = %f, linear.y = %f", cmd_vel_msg.linear.x, cmd_vel_msg.linear.y);
        ROS_INFO("angular.z = %f", cmd_vel_msg.angular.z);
        cmd_vel_pub.publish(cmd_vel_msg);
        //delivered_puck_pub.publish(deliver_puck_msg);
    }

    result_.delivered = true;
    as_.setSucceeded(result_);
}
