#ifndef Machine_H
#define Machine_H

#include <iostream>
#include <vector>
#include <cmath>
#include "world/Pose2d.hpp"

class Machine{
public:

	Machine();
	Machine(float x,float y,float phi, int task, int color);
	void setPose(float x,float y,float phi);
	void setPuckColor(int color);
	void deactivate();
	void activate();
	Pose2d getMachPose();
	Pose2d getGoalPose();
	int getPuckColor();
	int getID();
	void generateNextPuck();
	bool isActive();
	bool isVisited();//tarefa 2: chamar para trocar a cor do puck na sequencia

private:
	static int count;
	int id;
	int puckCounter;
	int task;
	Pose2d p;
	const float radius = 0.7;
	int puckColor;
	bool active;
	Pose2d robotPose;
	bool visited;
};

#endif
