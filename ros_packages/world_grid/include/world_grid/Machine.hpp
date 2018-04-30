#ifndef Machine_H
#define Machine_H

#include <iostream>
#include <vector>


typedef struct Pose2d{
	float x;
	float y;
	float phi;
}Pose2d;

class Machine{
public:
	Machine();
	Machine(float x,float y,float phi, int task, int color);
	void setPose(float x,float y,float phi);
	void setPuckColor(int color);
	void deactivate();
	Pose2d getPose();
	int getPuckColor();
	void generateNextPuck();//tarefa 2: chamar para trocar a cor do puck na sequencia

private:
	static int id;
	int task;
	Pose2d p;
	const float radius = 0.2;
	int puckColor;
	bool isActive;
};

#endif