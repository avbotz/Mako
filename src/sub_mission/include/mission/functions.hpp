#ifndef MISSION_FUNCTIONS_HPP
#define MISSION_FUNCTIONS_HPP 

#include <ros/ros.h>
#include <vision/Vision.h>

void gate(vision::Vision &, ros::ServiceClient &);
void octagon();
void printResponse(vision::Vision &);

#endif
