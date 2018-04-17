#include "world/DistrCenter.hpp"

int DistrCenter::count = 0;

DistrCenter::DistrCenter(){

}

DistrCenter::DistrCenter(float x,float y,float phi,int task, int color){
    id = count;
    p.x = x;
    p.y = y;
    p.phi = (phi/180)*3.14159;
    this->task = task;
    pucks.push_back(color);
    count++;
}

DistrCenter::DistrCenter(Pose2d pose, int task, int color1,int color2){
    id = count;
    p.x = pose.x;
    p.y = pose.y;
    p.phi = (pose.phi/180)*3.14159;
    this->task = task;
    pucks.push_back(color1);
    pucks.push_back(color2);
    count++;
}

// int DistrCenter::getPuck(){
//     return puck;
// }

/*
std::vector<int> DistrCenter::getPucks(){
    return pucks;
}
*/

bool DistrCenter::isEmpty(){
    return pucks.empty();
}

bool DistrCenter::isActive(){
    return active;
}

void DistrCenter::deactivate(){
    active = false;
}

Pose2d DistrCenter::getDcPose(){
    return p;
}

Pose2d DistrCenter::getGoalPose(){
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
int DistrCenter::getID(){
    return id;
}
