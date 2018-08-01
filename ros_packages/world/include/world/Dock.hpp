#ifndef Dock_H
#define Dock_H

#include <iostream>
#include <vector>
#include <cmath>
#include "world/Pose2d.hpp"

class Dock{
  public:
  	Dock(float x,float y,float phi, int task);
    Pose2d getGoalPose();
    Pose2d getDockPose();
  private:
    Pose2d p;
    const float radius = 0.7;

};

#endif
