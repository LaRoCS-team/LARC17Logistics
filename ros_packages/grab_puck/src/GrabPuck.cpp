#include "grab_puck/GrabPuck.hpp"

#include "std_msgs/String.h"
#include "grab_puck/util.hpp"

using std::vector;


int main(int argc, char** argv) {
   ros::init(argc, argv, "grab_puck");

   GrabPuckAction grab_puck(ros::this_node::getName());
   grab_puck.spin();

    return 0;
}

void GrabPuckAction::print(const std::string str) {
    if (debug_mode_) {
        std::cout << str << std::endl;
    }
}

GrabPuckAction::GrabPuckAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name),
    node_loop_rate_(2)
{
    first_time_turn_ = true;
    finished_grabbed_puck_ = false;

    dist_ir_.fill({0, 0, 0});

    if (nh_.hasParam("grab_puck/debug")) {
        nh_.getParam("grab_puck/debug", debug_mode_);
    }
    else {
        debug_mode_ = false;
    }

    // Initialize Publishers
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    // Initialize Subscribers
    distance_sensors_sub_ = nh_.subscribe("distance_sensors", 100, &GrabPuckAction::IRCallback, this);
    puck_info_sub_ = nh_.subscribe("puck_info", 100, &GrabPuckAction::puckInfoCallback, this);

    // ActionLib Callbacks
    as_.registerGoalCallback(boost::bind(&GrabPuckAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&GrabPuckAction::preemptCB, this));

    // Starting ActionLib Server
    as_.start();
}

void GrabPuckAction::goalCB() {
    print("New goal set.");

    goal_ = as_.acceptNewGoal()->color_id;

    // Reset Grab Puck flags
    resetFlags();
}

void GrabPuckAction::preemptCB() {
    print(action_name_ + ": Preempted");

    // Set the action state to preempted
    as_.setPreempted();
}

void GrabPuckAction::spin() {
    ros::Rate lr(node_loop_rate_);

    while (nh_.ok()) {
        if (!as_.isActive()) {
            print("No Goal active");
        } else {
            print("Goal active, grab puck of color: " + std::to_string(goal_));

            if (puck_color_ != goal_) {
                print("Puck color does not match goal color");
                result_.grabbed_puck = static_cast<unsigned char>(false);
                result_.color_grabbed_puck = puck_color_;
                as_.setPreempted();
            }
            else if (!finished_grabbed_puck_) {
                if (!has_puck_) {
                    print("Going to Puck");

                    goToPuck();
                }
                else {
                    SPEED_VEL = 0;
                    TURN_VEL = 0;

                    calculateFrontalDistances();

                    // TODO: PRECISA AVALIAR ESSE VALOR
                    if (dist_norm_ir_[4] >= 0.5 || dist_norm_ir_[5] >= 0.5) {
                        if (first_time_turn_) {
                            turnToDeliverSetSide();
                            first_time_turn_ = false;
                        }

                        turnToDeliver();
                        print("Turning to Deliver");

                    } else {
                        stopDeliver();

                        finished_grabbed_puck_ = true;
                        print("Delivered");

                    }
                }

                cmd_vel_msg_.linear.x = SPEED_VEL;
                cmd_vel_msg_.angular.z = TURN_VEL;

                cmd_vel_pub_.publish(cmd_vel_msg_);
            }
            else {
                result_.grabbed_puck = static_cast<unsigned char>(true);
                as_.setSucceeded(result_);
            }
        }

        lr.sleep();
        ros::spinOnce();
    }
}

GrabPuckAction::~GrabPuckAction()
{
    cmd_vel_pub_.shutdown();
    distance_sensors_sub_.shutdown();
    puck_info_sub_.shutdown();
    as_.shutdown();
}

void GrabPuckAction::IRCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    /**
     * Obtém os dados publicados como PointCloud e coloca-os em um vetor. O primeiro ponto obtido
     * é o IR frontal, e os subsequentes em sentido anti-horário.
     */
    for (int i = 0; i < 9; i++)
    {
        dist_ir_[i][0] = msg->points[i].x;
        dist_ir_[i][1] = msg->points[i].y;
        dist_ir_[i][2] = msg->points[i].z;
    }
}

void GrabPuckAction::puckInfoCallback(const puck_info::PuckInfoMsg::ConstPtr& msg)
{
    puck_center_X_ = msg->center.x;
    puck_center_Y_ = msg->center.y;
    puck_color_ = msg->color;
    has_puck_ = msg->has_puck;
    print("Has puck: " + std::to_string(has_puck_));
}

void GrabPuckAction::goToPuck()
{
    SPEED_VEL = arcTgWithPar(0.9*CAMERA_HEIGHT - puck_center_Y_, 0.05, 0.025, -2.5, M_PI_2);
    print("Linear vel: "+ std::to_string(SPEED_VEL));

    TURN_VEL = arcTgWithPar(CAMERA_WIDTH/2 - puck_center_X_, 0.15, 0.015, 0, 0);
    print("Angular vel: "+ std::to_string(TURN_VEL));
}

double GrabPuckAction::calculateNormDistance(std::array<float, 3>& dist)
{
    return sqrt(pow(dist[0], 2) + pow(dist[1], 2) + pow(dist[2], 2));
}

void GrabPuckAction::calculateFrontalDistances()
{
    dist_norm_ir_[0] = calculateNormDistance(dist_ir_[0]);
    dist_norm_ir_[1] = calculateNormDistance(dist_ir_[1]);
    dist_norm_ir_[2] = calculateNormDistance(dist_ir_[2]);
    dist_norm_ir_[3] = calculateNormDistance(dist_ir_[3]);
    dist_norm_ir_[5] = calculateNormDistance(dist_ir_[4]);
    dist_norm_ir_[6] = calculateNormDistance(dist_ir_[5]);
    dist_norm_ir_[7] = calculateNormDistance(dist_ir_[6]);
    dist_norm_ir_[8] = calculateNormDistance(dist_ir_[7]);
    dist_norm_ir_[9] = calculateNormDistance(dist_ir_[8]);

}

void GrabPuckAction::turnToDeliverSetSide()
{
    if (dist_norm_ir_[1] >= dist_norm_ir_[8])
    {
        /**
         * Nesse caso temos uma parede a esquerda
         */
        side_turn_flag_ = -1;
    }
    else
    {
        side_turn_flag_ = 1;
    }
}

void GrabPuckAction::turnToDeliver()
{
    if (side_turn_flag_ == -1)
    {
        TURN_VEL = -0.2;
    }
    else if (side_turn_flag_ == 1)
    {
        TURN_VEL = 0.2;
    }

    SPEED_VEL = 0;
}

double arcTgWithPar(double x, double a, double b, double c, double d) {
    return a * (std::atan(x * b + c) + d);
}

void GrabPuckAction::stopDeliver() {
    SPEED_VEL = 0;
    TURN_VEL = 0;
}

void GrabPuckAction::resetFlags() {
    first_time_turn_ = true;
    finished_grabbed_puck_ = false;
}

// void GrabPuckAction::ledPubPega(int color)
// {
//     //std::cout << "Antes de tudo" << std::endl;
//     //std::cout << "Size: " << led_msg_.values.size() << std::endl;
//
//     for (int i = 0; i < 12; i++)
//     {
//         led_msg_.values[i] = 0;
//     }
//
//     switch(color) {
//         case 1:
//             //std::cout << "C1" << std::endl;
//             led_msg_.values[3] = 1;
//             break;
//         case 2:
//             //std::cout << "C2" << std::endl;
//             led_msg_.values[11] = 1;
//             break;
//         case 3:
//             //std::cout << "C3" << std::endl;
//             led_msg_.values[7] = 1;
//             break;
//         default:
//             //std::cout << "DEFAULT" << std::endl;
//             break;
//     }
//     //std::cout << "Antes do Tempo" << std::endl;
//     led_msg_.stamp = ros::Time::now();
// }
