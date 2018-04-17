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
    turn_flag_(0),
    forward_flag_(0),
    node_loop_rate_(2)
{
    std::cout << "t1" << std::endl;
    first_time_turn_ = true;
    finished_grabbed_puck_ = false;

    puck_color_ = 0;

    dist_ir_.fill({0, 0, 0});

    if (nh_.hasParam("grab_puck/debug")) {
        nh_.getParam("grab_puck/debug", debug_mode_);
    }
    else {
        debug_mode_ = false;
    }

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    // got_puck_pub_ = nh_.advertise<std_msgs::Bool>("got_puck", 100);
    // led_pub_ = nh_.advertise<robotino_msgs::DigitalReadings>("set_digital_values", 100);

    // image_raw_sub_ = nh_.subscribe("image_raw", 100, &GrabPuckAction::cameraCallback, this);
    distance_sensors_sub_ = nh_.subscribe("distance_sensors", 100, &GrabPuckAction::IRCallback, this);
    // has_puck_sub_ = nh_.subscribe("hasPuck", 100, &GrabPuckAction::hasPuckCallback, this);
    puck_info_sub_ = nh_.subscribe("puck_info", 100, &GrabPuckAction::puckInfoCallback, this);
    // action_id_sub_ = nh_.subscribe("action_id", 100, &GrabPuckAction::actionIdCallback, this);

    as_.registerGoalCallback(boost::bind(&GrabPuckAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&GrabPuckAction::preemptCB, this));

    as_.start();
    std::cout << "t2" << std::endl;
}

void GrabPuckAction::goalCB() {
    goal_ = as_.acceptNewGoal()->color_id;
    print("New goal set.");
    std::cout << "t3" << std::endl;
}

void GrabPuckAction::preemptCB() {
    print(action_name_ + ": Preempted");
    // set the action state to preempted
    as_.setPreempted();
}

void GrabPuckAction::spin() {
    // result_.color_grabbed_puck.data = goal->color_id.data;
    // result_.grabbed_puck.data = true;

    // as_.setSucceeded(result_);

    ros::Rate lr(node_loop_rate_);
    std::cout << "t4" << std::endl;

    while (nh_.ok()) {
        //std::cout << "Antes do LED" << std::endl;
        //ledPubPega(puck_color_);
        //std::cout << "actionID:" << action_id_ << std::endl;
        if (!as_.isActive()) {
            print("No Goal active");
        } else {
            print("Goal active, puck of color: " + goal_);
            if (!finished_grabbed_puck_) {
                if (!has_puck_flag_) {
                    print("Going to Puck");
                    got_puck_msg_.data = static_cast<unsigned char>(false);

                    goToPuck();
                } else {
                    got_puck_msg_.data = static_cast<unsigned char>(true);

                    //controlSpeed(1);
                    SPEED_VEL = 0;
                    TURN_VEL = 0;

                    result_.grabbed_puck = true;
                    as_.setSucceeded(result_);

                    /*calculateFrontalDistances();

                    // TODO: PRECISA AVALIAR ESSE VALOR
                    std::cout << "Dis5: " << dist_norm_ir_5_ << "\nDis6: " << dist_norm_ir_6_ << std::endl;
                    if (dist_norm_ir_5_ >= 0.5 || dist_norm_ir_6_ >= 0.5) {
                        //std::cout << "Comecou o turn to deliver" << std::endl;
                        if (first_time_turn_) {
                            turnToDeliverSetSide();
                            first_time_turn_ = false;
                        }

                        turnToDeliver();
                        print("Turning to Deliver");

                    } else {
                        forwardStopFlag();
                        turnStopFlag();

                        finished_grabbed_puck_ = true;
                        print("Delivered");

                    }*/
                }

                cmd_vel_msg_.linear.x = SPEED_VEL;
                cmd_vel_msg_.angular.z = TURN_VEL;

                cmd_vel_pub_.publish(cmd_vel_msg_);
            } else {
                result_.grabbed_puck = true;
                as_.setSucceeded(result_);
            }
        }

        lr.sleep();
        ros::spinOnce();
    }
    
    result_.grabbed_puck = true;
    as_.setSucceeded(result_);
}

GrabPuckAction::~GrabPuckAction()
{
    cmd_vel_pub_.shutdown();
    distance_sensors_sub_.shutdown();
    puck_info_sub_.shutdown();
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
    print("CenterY: " + std::to_string(puck_center_Y_));
    puck_color_ = msg->color;
    has_puck_flag_ = msg->has_puck;

    // TODO: Maybe has_puck will have another name
    //has_puck_flag_ = msg->has_puck;
    //std::cout << "Lendo a cor" << std::endl;
}

void GrabPuckAction::goToPuck()
{
    SPEED_VEL = arcTgWithPar(0.9*CAMERA_HEIGHT - puck_center_Y_, 0.05, 0.025, -2.5, M_PI_2);
    print(std::to_string(SPEED_VEL));
    TURN_VEL = arcTgWithPar(CAMERA_WIDTH/2 - puck_center_X_, 0.15, 0.01, 0, 0);
}

double GrabPuckAction::calculateNormDistance(std::array<float, 3>& dist)
{
    return sqrt(pow(dist[0], 2) + pow(dist[1], 2) + pow(dist[2], 2));
}

void GrabPuckAction::calculateFrontalDistances()
{
    dist_norm_ir_2_ = calculateNormDistance(dist_ir_[1]);
    dist_norm_ir_3_ = calculateNormDistance(dist_ir_[2]);
    dist_norm_ir_8_ = calculateNormDistance(dist_ir_[7]);
    dist_norm_ir_9_ = calculateNormDistance(dist_ir_[8]);
    dist_norm_ir_5_ = calculateNormDistance(dist_ir_[4]);
    dist_norm_ir_6_ = calculateNormDistance(dist_ir_[5]);
}

void GrabPuckAction::turnToDeliverSetSide()
{
    if (dist_norm_ir_2_ >= dist_norm_ir_9_)
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
        turnRightFlag();
    }
    else if (side_turn_flag_ == 1)
    {
        turnLeftFlag();
    }

    forwardStopFlag();
}

double arcTgWithPar(double x, double a, double b, double c, double d) {
    return a * (std::atan(x * b + c) + d);
}

//TODO: VOLTAR PARA VELOCIDADES COM CHAO NO ATRICT
void GrabPuckAction::controlSpeed(int sub_action)
{
    if (sub_action == 0) {
        SPEED_VEL = arcTgWithPar(0.9*CAMERA_HEIGHT - puck_center_Y_, 0.001, 0.005, -2.5,M_PI_2);

        TURN_VEL = arcTgWithPar(CAMERA_WIDTH/2 - puck_center_X_, 0.02, 0.04, 0,0);
    }
    else
    {
        SPEED_VEL = arcTgWithPar(0.9*CAMERA_HEIGHT - puck_center_Y_, 0.1, 0.008, -2.5,M_PI_2);

        TURN_VEL = arcTgWithPar(CAMERA_WIDTH/2 - puck_center_X_, 0.1, 0.05, 0,0);
    }
}

void GrabPuckAction::turnLeftFlag()
{
    turn_flag_ = 1;
}

void GrabPuckAction::turnRightFlag()
{
    turn_flag_ = -1;
}

void GrabPuckAction::turnStopFlag()
{
    turn_flag_ = 0;
}

void GrabPuckAction::forwardStopFlag()
{
    forward_flag_ = 0;
}

void GrabPuckAction::forwardFlag()
{
    forward_flag_ = 1;
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
