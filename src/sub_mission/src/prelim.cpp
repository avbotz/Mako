#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision/Vision.h>
#include "mission/commands.hpp"
#include "mission/client.hpp"
#include "vision/config.hpp"
#include "vision/observation.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "mission_prelim_node");
	ros::NodeHandle node;   

	// Wait until kill switch is flipped.
	bool start = false;
	while (!start && !SIM && ros::ok())
	{
		ROS_INFO("Waiting to start PRELIM.");
		if (!control_client::alive())
			ros::Duration(0.5).sleep();
		else 
			start = true;
	}
	control_client::write("p 0.2\n");

	// Run mission functions.
	move(State(3, -1, 0, 0, 0, 0));
	move(State(3, 1, 0, 0, 0, 0));
	move(State(1, 0, 0, 0, 0, 0));
	move(State(-1, 0, 0, 0, 0, 0));
}
