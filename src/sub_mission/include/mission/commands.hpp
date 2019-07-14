#ifndef MISSION_COMMANDS_HPP
#define MISSION_COMMANDS_HPP

#include <ros/ros.h>
#include <vision/Vision.h>
#include "control/state.hpp"

float align(int);
void move(const State &); 

#endif
