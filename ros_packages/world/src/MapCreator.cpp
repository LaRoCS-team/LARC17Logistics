#include "world/MapCreator.hpp"
#include <string>
#include <yaml-cpp/yaml.h>

using namespace grid_map;
using namespace std;
//Alterar parâmetros da competição por aqui, no construtor:
const float res = 0.04;
//const int nMachines = 6, nDcs = 4;

MapCreator::MapCreator(): map({"occup"}),loopRate(5){
    //Estados booleanos
    puckColor = 0;
    task = 3;

    //Necessary variables;
    int i = 0, nMachines = 0;
    float x = 0.0, y = 0.0, phi = 0.0;
    char buffer[20]; 

    //Get Number of Machines, from rosparam
    ros::param::get("/Numero_maquinas", nMachines);

    //Intaciate a vector of machines
    Machine m[nMachines];

    //Get the position of the machines
    for (i = 1; i <= nMachines; i++) {
	sprintf(buffer, "/Maquina%d/x", i);
	ros::param::get(buffer,x);
	sprintf(buffer, "/Maquina%d/y", i);
	ros::param::get(buffer,y);
	sprintf(buffer, "/Maquina%d/phi", i);
	ros::param::get(buffer,phi);

	m[i-1].setPose(x,y,phi);
	machines.push_back(m[i-1]);
    }

    //Mapa
    map.setGeometry(Length(4.0, 4.0), res);
    map.setFrameId("map");
    map.setPosition(Position(2.0,2.0));
    map["occup"].setConstant(0);
    drawMap(); //Ativar para gravar um mapa novo.

    mapPub = nh.advertise<grid_map_msgs::GridMap>("world_map", 1000, true);
}

void MapCreator::PublishLoop(){
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

void MapCreator::drawMap(){
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
