#include "world_grid/Machine.hpp"

int Machine::id = 0;

Machine::Machine(){
    task = 0;
    isActive = true;
    puckColor = 0;
    std::cout<<"Posição da máquina indefinida"<<std::endl;
    p.x = -1.0;
    p.y = -1.0;
    p.phi = -1.0;
    id++;
}

Machine::Machine(float x,float y,float phi,int task, int color){
    task = 1;
    isActive = true;
    puckColor = color;
    p.x = x;
    p.y = y;
    p.phi = phi;
    id++;
}

void Machine::setPose(float x,float y,float phi){
    p.x = x;
    p.y = y;
    p.phi = phi;
}

void Machine::setPuckColor(int color){
    puckColor = color;
}

void Machine::deactivate(){
    isActive = false;
}

void Machine::generateNextPuck(){
    if (task == 2){
        if(puckColor >= 3){
            puckColor = 1;   
        }
        else{
            puckColor++;
        }
    }
}

Pose2d Machine::getPose(){
    return p;
}

int Machine::getPuckColor(){
    return puckColor;
}