#ifndef MISSION_COMMANDS_HPP
#define MISSION_COMMANDS_HPP

#include <ros/ros.h>
#include <vision/Vision.h>
#include "control/atmega.hpp"

float align(vision::Vision &, ros::ServiceClient &, int);
void move(const State &); 

#endif
