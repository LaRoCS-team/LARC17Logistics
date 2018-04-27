#ifndef DistrCenter_H
#define DistrCenter_H

#include <iostream>
#include <vector>
#include "world/Pose2d.hpp"
#include <cmath>
#include <list>

class DistrCenter{
public:
    DistrCenter();
    DistrCenter(float x,float y,float phi, int task, int color1);
    DistrCenter(float x,float y,float phi, int task, int color1,int color2);
    DistrCenter(Pose2d pose, int task, int color1,int color2);
    //std::vector<int> getPucks(); //tarefas 2 e 3
    Pose2d getDcPose();
    Pose2d getGoalPose();

    std::vector<int> getPucks(){
        return pucks;
    }

    int getPuck(int c){
        int r;

        if (pucks.front() == c){
            r = c;
            pucks.erase(pucks.begin());
        }

        if (pucks.back() == c){
            r = c;
            pucks.erase(pucks.begin()+1);
        }
        return r;
    }

    bool isEmpty();
    bool isActive();
    void deactivate();
    int getID();
private:
    static int count;
    int id;
    Pose2d p;
    int task;
    const float radius = 0.5;
    std::vector<int> pucks; //tarefas 2 e 3
   // int puck;
    bool active;
    Pose2d robotPose;
};

#endif
