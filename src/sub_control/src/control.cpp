#include <ros/ros.h>
#include "control/service.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_node");
	ros::NodeHandle node;

	ros::ServiceServer control_alive = 
		node.advertiseService("control_alive", alive); 
	ros::ServiceServer control_state = 
		node.advertiseService("control_state", state); 
	ros::ServiceServer control_write = 
		node.advertiseService("control_write", write); 
	ros::ServiceServer control_write_state = 
		node.advertiseService("control_write_state", writeState); 

	while (ros::ok())
	{
		ros::spinOnce();
	}
}
