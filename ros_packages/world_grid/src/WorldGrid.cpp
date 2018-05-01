#include "world_grid/WorldGrid.hpp"

WorldGrid::WorldGrid():map({"occup"}),loopRate(16){
	map.setGeometry(Length(4.0, 4.0), 0.08);
	map.setFrameId("map");
	map.setPosition(Position(2.0,2.0));
	map["occup"].setConstant(nothing);
	for(int i=0; i<vSize; i++){
		currState[i] = false;
	}
	statePub = nh.advertise<std_msgs::Int32>("state_id",1);
	gridPub = nh.advertise<grid_map_msgs::GridMap>("world_map", 1, true);
}

bool WorldGrid::getState(){
	return currState;
}

void WorldGrid::publishState(){
	int stateIndex = 0;
	for (int i = 0; i< vSize;i++){
		stateIndex += std::pow(2,i) * currState[i];
	}
	
	std_msgs::Int32 stateMsg;
	grid_map_msgs::GridMap mapMsg;
	ros::Time time = ros::Time::now();
	 grid_map::Index start(25, 25);
	 grid_map::Index end(25, 45);
	 for (grid_map::LineIterator iterator(map, start, end);
	 !iterator.isPastEnd(); ++iterator) {
	 map.at("occup", *iterator) = 1.0;
	 }
	//Publicar tf
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(0);
	geometry_msgs::TransformStamped trans;
	trans.header.stamp = time;
    trans.header.frame_id = "map";
    trans.child_frame_id = "odom";
    trans.transform.translation.x = 0.50;
    trans.transform.translation.y = 0.50;
    trans.transform.translation.z = 0.0;
	trans.transform.rotation = quat;
	mapBroadcaster.sendTransform(trans);

	stateMsg.data = stateIndex;
	map.setTimestamp(time.toNSec());
	GridMapRosConverter::toMessage(map, mapMsg);
	statePub.publish(stateMsg);
	gridPub.publish(mapMsg);
	loopRate.sleep();
}