#include "world/Machine.hpp"

int Machine::count = 0;

Machine::Machine(){
    task = 0; //Indefinida
    active = true;
    puckColor = 0;
    puckCounter = 0;
    //std::cout<<"Posição da máquina indefinida"<<std::endl;
    id = count;
    count++;
}

Machine::Machine(float x,float y,float phi,int task, int color){
    this->task = task;
    active = true;
    puckColor = color;
    puckCounter = 0;
    p.x = x;
    p.y = y;
    p.phi = (phi/180)*3.14159;
    id = count;
    visited = false;
    count++;
}

void Machine::setPose(float x,float y,float phi){
    p.x = x;
    p.y = y;
    p.phi = (phi/180)*3.14159;
}

void Machine::setPuckColor(int color){
    puckColor = color;
}

void Machine::deactivate(){
    active = false;
}

void Machine::activate(){
    active = true;
}

void Machine::generateNextPuck(){
    if (task == 2){
        puckCounter--;
        if(puckColor == 0){
            puckColor = 3;
        }
        else{
            puckColor--;
        }
        if (puckCounter>=3)
            deactivate();
    }
}

Pose2d Machine::getMachPose(){
    return p;
}

Pose2d Machine::getGoalPose(){
    Pose2d rp;
    rp.x = p.x + radius*cos(p.phi);
    rp.y = p.y + radius*sin(p.phi);
    if (p.phi >=0){
        rp.phi = (p.phi - 3.14159);
    }
    else{
        rp.phi = (p.phi + 3.14159);
    }
    return rp;
}

int Machine::getPuckColor(){
    return puckColor;
}

int Machine::getID(){
    return id;
}

bool Machine::isActive(){
    return active;
}

bool Machine::isVisited(){
    return visited;
}
