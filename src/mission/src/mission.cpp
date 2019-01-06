#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision/Observation.h>
#include "control/atmega.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_mock_detect");
    ros::NodeHandle node;   

    // Setup observation client.
    ros::ServiceClient client = node.serviceClient<vision::Observation>("observation");    
    vision::Observation obs;

    // Setup original sub state.
    bool alive = atmega::read_alive();
    State state = atmega::read_state();
}
