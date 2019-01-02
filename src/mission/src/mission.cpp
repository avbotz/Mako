#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission");
    ros::NodeHandle node;
    // ros::ServiceClient client = node.serviceClient<vision::Observation>("vision/request");
    ros::spin();
}
