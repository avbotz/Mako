/** @file commands.hpp
 *  @brief Generic actions done during a competition run, not specific to a
 *         certain task.
 *
 *  @author David Zhang
 */
#ifndef MISSION_COMMANDS_HPP
#define MISSION_COMMANDS_HPP

#include <ros/ros.h>
#include <vision/Vision.h>
#include "control/state.hpp"
#include "vision/observation.hpp"

float align(int, Task, int);
void move(const State &); 

#endif
