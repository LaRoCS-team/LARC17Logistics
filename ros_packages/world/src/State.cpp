#include <world/State.hpp>

State::State(){

    //Necessary variables;
    int i = 0;
    float x = 0.0, y = 0.0, phi = 0.0;
    char buffer[20];

    //Get Number of Machines and DCs, from rosparam
    ros::param::get("/Numero_maquinas", nMachines);
    ros::param::get("/Numero_DCs", nDCs);
    ros::param::get("/Numero_docks", nDocks);

    task=1;

    //Get the position of the machines
    for (i = 1; i <= nMachines; i++) {
    	sprintf(buffer, "/Maquina%d/x", i);
    	ros::param::get(buffer,x);
    	sprintf(buffer, "/Maquina%d/y", i);
    	ros::param::get(buffer,y);
    	sprintf(buffer, "/Maquina%d/phi", i);
    	ros::param::get(buffer,phi);
    	machines.push_back(Machine(x,y,phi,task,0));
    }

    for (i = 1; i <= nDCs; i++) {
    	sprintf(buffer, "/DC%d/x", i);
    	ros::param::get(buffer,x);
    	sprintf(buffer, "/DC%d/y", i);
    	ros::param::get(buffer,y);
    	sprintf(buffer, "/DC%d/phi", i);
    	ros::param::get(buffer,phi);
    	dcs.push_back(DistrCenter(x,y,phi,task,0));
    }

    for (i = 1; i <= nDocks; i++) {
    	sprintf(buffer, "/DC%d/x", i);
    	ros::param::get(buffer,x);
    	sprintf(buffer, "/DC%d/y", i);
    	ros::param::get(buffer,y);
    	sprintf(buffer, "/DC%d/phi", i);
    	ros::param::get(buffer,phi);
    	docks.push_back(Dock(x,y,phi,task));
    }

	//Publisher initialization
	statePub = nh.advertise<world::State>("state",10);
}

State::~State(){

}

void State::PublishLoop(){
	ros::Time time = ros::Time::now();

	//tf message setup:
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = time;
    trans.header.frame_id = "map";
    trans.child_frame_id = "odom";
    trans.transform.translation.x = 0.30;
    trans.transform.translation.y = 0.30;
    trans.transform.translation.z = 0.0;
    trans.transform.rotation = quat;

	//State message setup
	world::State stateMsg;
	stateMsg.task = task;
	std::vector<world::Machine> mchMsgVec;
  world::Machine mchMsg;

  std::vector<world::Dock> dockMsgVec;
  world::Dock dockMsg;

  std::vector<world::DistrCenter> dcMsgVec;
  world::DistrCenter dcMsg;

	for (int i = 0; i < nMachines; i++){
        mchMsg.goalPose.x = machines[i].getGoalPose().x;
        mchMsg.goalPose.y = machines[i].getGoalPose().y;
        mchMsg.goalPose.theta = machines[i].getGoalPose().phi;
        mchMsg.mapPose.x = machines[i].getMachPose().x;
        mchMsg.mapPose.y = machines[i].getMachPose().y;
        mchMsg.mapPose.theta = machines[i].getMachPose().phi;
        mchMsg.puck = machines[i].getPuckColor();
        mchMsg.isActive = machines[i].isActive();
        mchMsgVec.push_back(mchMsg);
    }
  for (int i = 0; i < nDCs; i++){
      dcMsg.goalPose.x = dcs[i].getGoalPose().x;
      dcMsg.goalPose.y = dcs[i].getGoalPose().y;
      dcMsg.goalPose.theta = dcs[i].getGoalPose().phi;
      dcMsg.mapPose.x = dcs[i].getDcPose().x;
      dcMsg.mapPose.y = dcs[i].getDcPose().y;
      dcMsg.mapPose.theta = dcs[i].getDcPose().phi;
      dcMsg.puck1 = dcs[i].getPucks()[0];
      //dcMsg.puck2 = dcs[i].getPucks()[1];
      dcMsg.isActive = dcs[i].isActive();
      dcMsgVec.push_back(dcMsg);
  }
  for (int i = 0; i < nDocks; i++){
      dockMsg.goalPose.x = docks[i].getGoalPose().x;
      dockMsg.goalPose.y = docks[i].getGoalPose().y;
      dockMsg.goalPose.theta = docks[i].getGoalPose().phi;
      dockMsg.mapPose.x = docks[i].getDockPose().x;
      dockMsg.mapPose.y = docks[i].getDockPose().y;
      dockMsg.mapPose.theta = docks[i].getDockPose().phi;
      dockMsgVec.push_back(dockMsg);
  }

	stateMsg.machines = mchMsgVec;
  stateMsg.dcs = dcMsgVec;
  stateMsg.dock = dockMsgVec;
	//publishing...
	mapBroadcaster.sendTransform(trans);
	statePub.publish(stateMsg);

}
