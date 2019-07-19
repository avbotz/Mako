/** @file mission.cpp 
 *  @brief Main node runner for mission.
 *
 *  @author David Zhang
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision/Vision.h>
#include "mission/functions.hpp"
#include "mission/client.hpp"
#include "vision/observation.hpp"


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
	control_client::write_client = 
		node.serviceClient<control::ControlWrite>("control_write");
	control_client::write_state_client = 
		node.serviceClient<control::ControlWriteState>("control_write_state");

	// Wait until kill switch is flipped.
	bool start = false;
	while (!start && !SIM && ros::ok())
	{
		if (!control_client::alive())
			ros::Duration(0.5).sleep();
		else 
			start = true;
	}
	control_client::write("p 0.25\n");

	// Run mission functions.
	gate();
	octagon(); 
}
