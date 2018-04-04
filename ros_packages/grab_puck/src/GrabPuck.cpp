#include "GrabPuck.hpp"

#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"

using cv::Mat;
using std::vector;


int main(int argc, char** argv) {
   ros::init(argc, argv, "grab_puck");

   GrabPuckAction grab_puck("grab_puck");
   ros::spin();

    return 0;
}

GrabPuckAction::GrabPuckAction(std::string name) :
    as_(nh_, name, boost::bind(&GrabPuckAction::executeCB, this, _1), false),
    action_name_(name),
    turn_flag_(0),
    forward_flag_(0),
    node_loop_rate_(20)
{
    first_time_turn_ = true;
    finished_grabbed_puck_ = false;

    puck_color_ = 0;

    dist_ir_.fill({0, 0, 0});

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    // got_puck_pub_ = nh_.advertise<std_msgs::Bool>("got_puck", 100);
    // led_pub_ = nh_.advertise<robotino_msgs::DigitalReadings>("set_digital_values", 100);

    // image_raw_sub_ = nh_.subscribe("image_raw", 100, &GrabPuckAction::cameraCallback, this);
    distance_sensors_sub_ = nh_.subscribe("distance_sensors", 100, &GrabPuckAction::IRCallback, this);
    // has_puck_sub_ = nh_.subscribe("hasPuck", 100, &GrabPuckAction::hasPuckCallback, this);
    puck_info_sub_ = nh_.subscribe("puck_info", 100, &GrabPuckAction::puckInfoCallback, this);
    // action_id_sub_ = nh_.subscribe("action_id", 100, &GrabPuckAction::actionIdCallback, this);

    as_.start();
}

void GrabPuckAction::executeCB(const grab_puck::GrabPuckGoalConstPtr &goal) {
    // result_.color_grabbed_puck.data = goal->color_id.data;
    // result_.grabbed_puck.data = true;

    // as_.setSucceeded(result_);

    ros::Rate lr(node_loop_rate_);
    //std::cout << "Antes do LED" << std::endl;
    //ledPubPega(puck_color_);
    //std::cout << "actionID:" << action_id_ << std::endl;

    //std::cout << "Comecou a pegar Puck" << std::endl;
    while (!finished_grabbed_puck_) {
        if (!has_puck_flag_) {
            got_puck_msg_.data = static_cast<unsigned char>(false);

            controlSpeed(0);
            goToPuck();
            //ROS_INFO("goToPuck()");
        }
        else {
            got_puck_msg_.data = static_cast<unsigned char>(true);

            controlSpeed(1);
            calculateFrontalDistances();

            // TODO: PRECISA AVALIAR ESSE VALOR
            //std::cout << "Dis2: " << dist_norm_ir_5_ << "\nDis9: " << dist_norm_ir_6_ << std::endl;
            if (dist_norm_ir_5_ >= 0.5 || dist_norm_ir_6_ >= 0.5) {
                //std::cout << "Comecou o turn to deliver" << std::endl;
                if (first_time_turn_) {
                    turnToDeliverSetSide();
                    first_time_turn_ = false;
                }

                turnToDeliver();

            }
            else {
                forwardStopFlag();
                turnStopFlag();

                finished_grabbed_puck_ = true;

            }
        }

        cmd_vel_msg_.linear.x = forward_flag_ * SPEED_VEL;
        cmd_vel_msg_.angular.z = turn_flag_ * TURN_VEL;

        cmd_vel_pub_.publish(cmd_vel_msg_);
    }


    result_.grabbed_puck.data = true;
    as_.setSucceeded(result_);
}

GrabPuckAction::~GrabPuckAction()
{
    cmd_vel_pub_.shutdown();
    distance_sensors_sub_.shutdown();
    puck_info_sub_.shutdown();
}

void GrabPuckAction::cameraCallback(const sensor_msgs::Image_<std::allocator<void>>::ConstPtr &msg)
{
    /**
      * Utilizando o package cv_bridge do ROS, podemos obter um objeto do tipo Mat (OpenCV) a partir
      * de uma mensagem do tipo sensor_msgs/Image.
      *
      * Utilizando um ponteiro do tipo CvImageConstPtr, obtemos o objeto Mat na forma "shared", ou
      * seja, não faremos uma cópia dos dados, mas também não poderemos edita-los.
      */
    cv_bridge::CvImageConstPtr cv_cptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    image_ = cv_cptr->image;

    /**
      * O V-REP passa a imagem invertida no eixo X (horizontal), por isso, utilizando a função cv::flip()
      * com o parâmetro '0', fazemos um flip no eixo X.
      */
    cv::flip(image_, image_, 0);

    // cv::imwrite("/home/previato/catkin_ws/src/visao/image.ppm", image);
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

void GrabPuckAction::hasPuckCallback(const std_msgs::Bool::ConstPtr& msg)
{
    has_puck_flag_ = msg->data;
    //ROS_INFO("has_puck_flag_ %d", has_puck_flag_);
}

void GrabPuckAction::puckInfoCallback(const robotino_msgs::PuckInfo::ConstPtr& msg)
{
    puck_center_X_ = msg->centroid.x;
    puck_center_Y_ = msg->centroid.y;
    puck_color_ = msg->color;

    // TODO: Maybe has_puck will have another name
    //has_puck_flag_ = msg->has_puck;
    //std::cout << "Lendo a cor" << std::endl;
}

void GrabPuckAction::actionIdCallback(const std_msgs::UInt64::ConstPtr& msg)
{
    action_id_ = msg->data;
}

void GrabPuckAction::goToPuck()
{
        if (puck_center_X_ < CAMERA_WIDTH/2 - 5  || puck_center_X_ > CAMERA_WIDTH/2 + 5)
        {
            if (puck_center_X_ < CAMERA_WIDTH/2)
            {
                turnLeftFlag();
                forwardStopFlag();
            }
            else if (puck_center_X_ > CAMERA_WIDTH/2)
            {
                turnRightFlag();
                forwardStopFlag();
            }
        }
        else
        {
            turnStopFlag();
            forwardFlag();
        }
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

//TODO: VOLTAR PARA VELOCIDADES COM CHAO NO ATRICT
void GrabPuckAction::controlSpeed(int sub_action)
{
    if (sub_action == 0) {
        SPEED_VEL = 0.1;

        if(puck_center_Y_ < 120)
        {
            TURN_VEL = 0.16;
        } else
        {
            TURN_VEL = 0.075;
        }
    }
    else if (sub_action == 1)
    {
        SPEED_VEL = 0.1;
        TURN_VEL = 0.2;
    }
    else {
        SPEED_VEL = 0.075;
        TURN_VEL = 0.075;
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

// void GrabPuckAction::spin()
// {
//     ros::Rate lr(node_loop_rate_);
//     while(nh_.ok())
//     {
//         //std::cout << "Antes do LED" << std::endl;
//         ledPubPega(puck_color_);
//         //std::cout << "actionID:" << action_id_ << std::endl;
//         if (action_id_ == 14) {
//
//             //std::cout << "Comecou a pegar Puck" << std::endl;
//
//             if (!has_puck_flag_) {
//                 got_puck_msg_.data = static_cast<unsigned char>(false);
//
//                 controlSpeed(0);
//                 goToPuck();
//                 //ROS_INFO("goToPuck()");
//             } else {
//                 got_puck_msg_.data = static_cast<unsigned char>(true);
//
//                 controlSpeed(1);
//                 calculateFrontalDistances();
//
//                 // TODO: PRECISA AVALIAR ESSE VALOR
//                 //std::cout << "Dis2: " << dist_norm_ir_5_ << "\nDis9: " << dist_norm_ir_6_ << std::endl;
//                 if (dist_norm_ir_5_ >= 0.5 || dist_norm_ir_6_ >= 0.5) {
//                     //std::cout << "Comecou o turn to deliver" << std::endl;
//                     if (first_time_turn_)
//                     {
//                         turnToDeliverSetSide();
//                         first_time_turn_ = false;
//                     }
//
//                     turnToDeliver();
//
//                 } else
//                 {
//                     forwardStopFlag();
//                     turnStopFlag();
//                 }
//             }
//
//             cmd_vel_msg_.linear.x = forward_flag_ * SPEED_VEL;
//             cmd_vel_msg_.angular.z = turn_flag_ * TURN_VEL;
//
//             cmd_vel_pub_.publish(cmd_vel_msg_);
//             got_puck_pub_.publish(got_puck_msg_);
//
//         }
//
//         led_pub_.publish(led_msg_);
//         ros::spinOnce();
//         lr.sleep();
//     }
// }
