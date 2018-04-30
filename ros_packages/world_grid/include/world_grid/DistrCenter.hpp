#ifndef DistrCenter_H
#define DistrCenter_H

#include <iostream>
#include <vector>

typedef struct Pose2d{
	float x;
	float y;
	float phi;
}Pose2d;

class DistrCenter{
public:
    int getPuckColor();
private:
    static int id;
    Pose2d p;
    int task;
    
};

#endif