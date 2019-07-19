#ifndef MISSION_COMMANDS_HPP
#define MISSION_COMMANDS_HPP

#include <ros/ros.h>
#include <vision/Vision.h>
#include "control/state.hpp"
#include "vision/observation.hpp"

float align(int, Task, int);
void move(const State &); 

#endif
