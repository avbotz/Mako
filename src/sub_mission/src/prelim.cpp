#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision/Perception.h>
#include "control/atmega.hpp"
#include "mission/commands.hpp"
#include "vision/config.hpp"
#include "vision/tasks.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "mission_prelim_node");
	ros::NodeHandle node;   

	// Wait until kill switch is flipped.
	bool start = false;
	while (!start && !SIM)
	{
		ROS_INFO("Waiting to start PRELIM.");
		if (!atmega::alive())
			ros::Duration(0.5).sleep();
		else 
			start = true;
	}
	atmega::write("p 0.2\n");

	// Run mission functions.
	move(State(3, -1, 0, 0, 0, 0));
	move(State(3, 1, 0, 0, 0, 0));
	move(State(1, 0, 0, 0, 0, 0));
	move(State(-1, 0, 0, 0, 0, 0));
}
