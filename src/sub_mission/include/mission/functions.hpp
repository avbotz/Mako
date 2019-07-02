#ifndef MISSION_FUNCTIONS_HPP
#define MISSION_FUNCTIONS_HPP 

#include <ros/ros.h>
#include <vision/Perception.h>

void gate(vision::Perception &, ros::ServiceClient &);
void octagon();
void printResponse(vision::Perception &);

#endif
