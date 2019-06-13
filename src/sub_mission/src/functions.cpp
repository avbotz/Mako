#include "mission/functions.hpp"
#include "mission/commands.hpp"
#include "vision/tasks.hpp"
#include "vision/config.hpp"


void gate(vision::Perception &per, ros::ServiceClient &client)
{
    ROS_INFO("Beginning GATE function.");
    std::cout << atmega::state().text() << std::endl;
    per.request.task = Task::GATE;
    per.request.camera = FRONT;
    
    float angle = align(per, client, 5);
    ROS_INFO("Aligning to angle @ %f", angle);
    State move1 = atmega::state();
    move1.axis[YAW] = angle;
    move(move1);
}

void octagon(vision::Perception &per, ros::ServiceClient &client)
{

}

void printResponse(vision::Perception &per)
{
    ROS_INFO("%i observation @ %f H-deg, %f V-deg, and %f meters. Image location @ (%f, %f).", 
            per.request.task, per.response.hangle, per.response.vangle, per.response.dist,
            per.response.r, per.response.c);
}
