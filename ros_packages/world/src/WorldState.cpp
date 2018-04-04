#include "world/WorldState.hpp"
#include <string>
#include <yaml-cpp/yaml.h>

using namespace grid_map;
using namespace std;
//Alterar parâmetros da competição por aqui, no construtor:
const float res = 0.04;
const int nMachines = 6, nDcs = 4;

WorldState::WorldState(): map({"occup"}),loopRate(5){
    //Estados booleanos
    puckColor = 0;
    task = 3;

    //Definição das máquinas: pos x, pos y, orientagoal.header = msg->header;
    //ex: mi(float x,float y,float phi, int task, int color);
    Machine m1(0.65,2.43,-15, task, 0);
    Machine m2(2.15,2.50,-90, task, 0);
    Machine m3(3.89,2.50,180, task, 0);
    Machine m4(1.19,1.24,135, task, 0);
    Machine m5(3.24,1.48,-135, task, 0);
    Machine m6(2.2,0.10,90, task, 0);
    //Add ao vetor de máquinas
    machines.push_back(m1);
    machines.push_back(m2);
    machines.push_back(m3);
    machines.push_back(m4);
    machines.push_back(m5);
    machines.push_back(m6);
    
    //Mapa
    map.setGeometry(Length(4.0, 4.0), res);
    map.setFrameId("map");
    map.setPosition(Position(2.0,2.0));
    map["occup"].setConstant(0);
    drawMap(); //Ativar para gravar um mapa novo.

    mapPub = nh.advertise<grid_map_msgs::GridMap>("world_map", 1000, true);
}

void WorldState::PublishLoop(){
    //Publicar tf
    ros::Time time = ros::Time::now();
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = time;
    trans.header.frame_id = "map";
    trans.child_frame_id = "odom";
    trans.transform.translation.x = 0.30;
    trans.transform.translation.y = 0.30;
    trans.transform.translation.z = 0.0;
    trans.transform.rotation = quat;
    //Mapa
    grid_map_msgs::GridMap mapMsg;
    mapBroadcaster.sendTransform(trans);
    map.setTimestamp(time.toNSec());
    grid_map::GridMapRosConverter::toMessage(map, mapMsg);
    mapPub.publish(mapMsg);
}

void WorldState::drawMap(){
        grid_map::Position start;
        grid_map::Position end;
        Pose2d poseMeters;
        float xposWall1, yposWall1, xposWall2, yposWall2, r=0.5;
        std::vector<int> gridEnd, gridStart;
    for (std::vector<Machine>::iterator it = machines.begin(); it != machines.end(); ++it){
        poseMeters = it->getMachPose();
        xposWall1 = poseMeters.x + r*cos((poseMeters.phi + 0.25*3.14159));
        yposWall1 = poseMeters.y + r*sin((poseMeters.phi + 0.25*3.14159));
        xposWall2 = poseMeters.x + r*cos((poseMeters.phi - 0.25*3.14159));
        yposWall2 = poseMeters.y + r*sin((poseMeters.phi - 0.25*3.14159));

        start << poseMeters.x, poseMeters.y;
        end << xposWall1, yposWall1;
        for (grid_map::LineIterator iterator(map, start, end);
        !iterator.isPastEnd(); ++iterator) {
        map.at("occup", *iterator) = 1.0;
        }
        //gridEnd = meter2grid(xposWall2, yposWall2, 0.05);
        end << xposWall2, yposWall2;
        for (grid_map::LineIterator iterator(map, start, end);
        !iterator.isPastEnd(); ++iterator) {
        map.at("occup", *iterator) = 1.0;
        }
    }
}
