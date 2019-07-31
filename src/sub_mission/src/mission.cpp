/** @file mission.cpp 
 *  @brief Main node runner for mission.
 *
 *  @author David Zhang
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "mission/functions.hpp"
#include "mission/client.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "mission_node");
	ros::NodeHandle node;   
	
	ROS_INFO("Setting up clients.");

	// Setup observation client.
	vision_client::client = node.serviceClient<vision::Vision>("vision"); 

	// Setup control clients.
	control_client::alive_client = 
		node.serviceClient<control::ControlAlive>("control_alive");
	control_client::state_client = 
		node.serviceClient<control::ControlState>("control_state");
	control_client::depth_client = 
		node.serviceClient<control::ControlDepth>("control_depth");
	control_client::write_client = 
		node.serviceClient<control::ControlWrite>("control_write");
	control_client::write_state_client = 
		node.serviceClient<control::ControlWriteState>("control_write_state");
	control_client::write_depth_client = 
		node.serviceClient<control::ControlWriteDepth>("control_write_depth");

	// Wait until kill switch is flipped.
	bool start = false;
	while (!start && !SIM && ros::ok())
	{
		if (!control_client::alive())
			ros::Duration(0.5).sleep();
		else 
			start = true;
	}
	control_client::write("p 0.2\n");

	// Run mission functions.
	// gate();
	// gate_extra();
	bins();
	// target();
	// octagon(); 
}
