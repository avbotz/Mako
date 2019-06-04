#include "mission/functions.hpp"
#include "vision/tasks.hpp"
#include "vision/config.hpp"


void gate(vision::Perception &per, ros::ServiceClient &client)
{
    per.request.task = Task::GATE;
    per.request.camera = FRONT;
    
    while (true)
    {
        client.call(per);
        
    }
}

void octagon(vision::Perception &per, ros::ServiceClient &client)
{

}

void printResponse(vision::Perception &per)
{
    ROS_INFO("%s observation @ %f H-deg, %f V-deg, and %f meters. Image location @ (%f, %f).", 
            per.request.task, per.response.hangle, per.response.vangle, per.response.dist,
            per.response.r, per.response.c);
}
