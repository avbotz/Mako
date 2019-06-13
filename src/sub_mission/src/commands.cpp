#include "mission/commands.hpp"
#include "mission/functions.hpp"
#include "control/atmega.hpp"


float align(vision::Perception &per, ros::ServiceClient &client, int attempts)
{
    float average = 0.0f;
    for (int i = 0; i < attempts; i++)
    {
        client.call(per);
        printResponse(per);

        average += atmega::state().axis[YAW] + per.response.hangle; 
    }
    
    average /= attempts;
    return average;
}

void move(const State &dest)
{
    bool quit = false;
    while (!quit)
    {
        State state = atmega::state();
        if (std::fabs(dest.axis[X]-state.axis[X]) > 1.0f)
            ros::Duration(3.0).sleep();
        else if (std::fabs(dest.axis[Y]-state.axis[Y]) > 1.0f)
            ros::Duration(3.0).sleep();
        else if (std::fabs(dest.axis[YAW]-state.axis[YAW]) > 10.0f)
            ros::Duration(3.0).sleep();
        else
        {
            ros::Duration(3.0).sleep();
            quit = true; 
        }
    }
}
