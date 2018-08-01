#include "world/Dock.hpp"

Dock::Dock(float x,float y,float phi,int task){
    p.x = x;
    p.y = y;
    p.phi = (phi/180)*3.14159;
}

Pose2d Dock::getGoalPose(){
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

Pose2d Dock::getDockPose(){
    return p;
}
