/** @file commands.cpp
 *  @brief Functions for generic competition actions.
 *
 *  @author David Zhang
 */
#include "mission/commands.hpp"
#include "mission/functions.hpp"
#include "mission/client.hpp"
#include "mission/euler.hpp"


float align(int attempts, Task task, int camera)
{
	float average = 0.0f;
	for (int i = 0; i < attempts; i++)
	{
		State current_state = control_client::state();
		Observation obs = vision_client::vision(task, camera);
		ROS_INFO("Observation @ %s", obs.text().c_str());
		
		if (obs.prob > 0.5)
		{
			average += current_state.axis[YAW];
			average += obs.hangle; 
		}
		else 
		{
			attempts -= 1;
		}
		ros::Duration(2.0).sleep();
	}

	if (attempts == 0)
		return 0.;
	average /= attempts;
	return average;
}

std::pair<float, float> down_align(int attempts, float dist, Task task, 
		int camera)
{
	float x_avg = 0.0f;
	float y_avg = 0.0f;
	for (int i = 0; i < attempts; i++)
	{
		State state = control_client::state();
		Observation obs = vision_client::vision(task, camera);
		ROS_INFO("Observation @ %s", obs.text().c_str());
		
		if (obs.prob > 0.5)
		{
			// Get body offsets.
			float x = std::tan(obs.vangle*M_PI/180.)*dist;
			float y = std::tan(obs.hangle*M_PI/180.)*dist;

			// Convert body offsets to inertial frame.
			float input[3] = {x, y, 0.};
			float angles[3] = {state.axis[YAW], state.axis[PITCH], state.axis[ROLL]};
			float output[3];
			bodyToInertial(input, angles, output);

			// Add to average.
			x_avg += state.axis[X] + output[0];
			y_avg += state.axis[Y] + output[1];
			ROS_INFO("Valid observation w/ offset @ %f %f", output[0], output[1]);
		}
		else 
		{
			attempts -= 1;
		}
		ros::Duration(1.0).sleep();
	}

	if (attempts == 0)
		return std::make_pair(0., 0.);
	x_avg /= attempts;
	y_avg /= attempts;
	return std::make_pair(x_avg, y_avg);
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

