#ifndef MISSION_COMMANDS_HPP
#define MISSION_COMMANDS_HPP

#include <ros/ros.h>
#include <vision/Perception.h>
#include "control/atmega.hpp"

float align(vision::Perception &, ros::ServiceClient &, int);
void move(const State &); 

#endif
