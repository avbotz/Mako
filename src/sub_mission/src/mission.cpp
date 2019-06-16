#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision/Perception.h>
#include "control/atmega.hpp"
#include "mission/functions.hpp"
#include "vision/tasks.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle node;   

    // Setup observation client.
    ros::ServiceClient client = node.serviceClient<vision::Perception>("perception");    
    vision::Perception perception;

    // Wait until kill switch is flipped.
    bool start = false;
    while (!start && !SIM)
    {
        if (!atmega::alive())
            ros::Duration(0.5).sleep();
        else 
            start = true;
    }
    atmega::write("p 0.2\n");

    // Run mission functions.
    gate(perception, client);
    octagon(perception, client); 

}
