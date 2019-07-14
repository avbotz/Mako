#include "mission/commands.hpp"
#include "mission/functions.hpp"
#include "mission/client.hpp"
#include "vision/config.hpp"


float align(int attempts)
{
	float average = 0.0f;
	for (int i = 0; i < attempts; i++)
	{
		Observation obs = vision_client::vision();
		if (obs.prob > 0.5)
		{
			average += control_client::state().axis[YAW];
			average += obs.hangle; 
		}
		ros::Duration(2.0).sleep();
	}

	average /= attempts;
	return average;
}

void move(const State &dest)
{
	control_client::writeState(dest);
	bool quit = false;
	while (!quit && ros::ok())
	{
		State now = control_client::state();
		if (std::fabs(dest.axis[X]-now.axis[X]) > 1.0f)
			ros::Duration(3.0).sleep();
		else if (std::fabs(dest.axis[Y]-now.axis[Y]) > 1.0f)
			ros::Duration(3.0).sleep();
		else if (std::fabs(dest.axis[YAW]-now.axis[YAW]) > 5.0f)
			ros::Duration(3.0).sleep();
		else
		{
			ros::Duration(3.0).sleep();
			quit = true; 
		}
	}
}

