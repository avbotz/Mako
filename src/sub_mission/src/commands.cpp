/** @file commands.cpp
 *  @brief Functions for generic competition actions.
 *
 *  @author David Zhang
 */
#include <cmath>
#include "mission/commands.hpp"
#include "mission/functions.hpp"
#include "mission/client.hpp"
#include "mission/euler.hpp"


float angleDifference(float a1, float a2)
{
	// For [0, 360].
	/*
	   float c1 = a1 - a2;
	   float c2 = a1 - a2 + 360.;
	   float c3 = a1 - a2 - 360.;
	   if (abs(c1) < abs(c2) && abs(c1) < abs(c3))
	   return c1;
	   if (abs(c2) < abs(c3))
	   return c2;
	   else 
	   return c3;
	*/

	// For [-180, 180].
	float b1 = a1-a2;
	if (std::fabs(b1) > 180.)
	{
		if (a1 < a2)
			a1 += 360.;
		else 
			a2 += 360.;
		b1 = a1-a2;
	}
	return b1;
}

float angleAdd(float a1, float add)
{
	float temp = a1 + add;
	if (temp > 180.0)
		return temp - 360.0;
	else if (temp < -180.)
		return temp + 360.0;

	return temp;
}
float align(int attempts, Task task, int camera)
{
	int threshold = attempts/2;
	float average = 0.;
	for (int i = 0; i < attempts; i++)
	{
		State current_state = control_client::state();
		Observation obs = vision_client::vision(task, camera);
		ROS_INFO("Observation @ %s", obs.text().c_str());
		
		if (obs.prob > 0.70)
		{
			average += current_state.axis[YAW];
			average += obs.hangle; 
		}
		else 
		{
			attempts -= 1;
		}
		ros::Duration(2.).sleep();
	}

	if (attempts <= threshold)
	{
		ROS_INFO("Failed to pass threshold for down alignment.");
		return -999.;
	}
	average /= attempts;
	return average;
}

float distance(int attempts, Task task, int camera)
{
	int threshold = attempts/2;
	float average = 0.;
	for (int i = 0; i < attempts; i++)
	{
		State current_state = control_client::state();
		Observation obs = vision_client::vision(task, camera);
		ROS_INFO("Observation @ %s", obs.text().c_str());
		
		if (obs.prob > 0.70)
		{
			average += obs.dist;
		}
		else 
		{
			attempts -= 1;
		}
		ros::Duration(1.).sleep();
	}

	if (attempts <= threshold)
	{
		ROS_INFO("Failed to pass threshold for down alignment.");
		return -999.;
	}
	average /= attempts;
	return average;
}

Coordinate downAlign(int attempts, Task task, int camera)
{
	int threshold = attempts/2;
	int original = attempts;
	float x_avg = 0.;
	float y_avg = 0.;
	for (int i = 0; i < original; i++)
	{
		State state = control_client::state();
		float dist = control_client::depth();
		Observation obs = vision_client::vision(task, camera);
		ROS_INFO("Observation @ %s", obs.text().c_str());
		
		if (obs.prob > 0.5)
		{
			// Get body offsets.
			float x = std::tan(obs.vangle*M_PI/180.)*(dist-1.);
			float y = std::tan(obs.hangle*M_PI/180.)*(dist-1.);

			// Convert body offsets to inertial frame.
			float input[3] = {x+0.15, y-0.05, 0.};
			float angles[3] = {state.axis[YAW], state.axis[PITCH], 
				state.axis[ROLL]};
			float output[3];
			bodyToInertial(input, angles, output);

			// Add to average.
			x_avg += state.axis[X] + output[0]/1.5;
			y_avg += state.axis[Y] + output[1]/1.5;
			ROS_INFO("Debug offset @ %f %f", input[0], input[1]);
			ROS_INFO("Debug state @ %s", state.text().c_str());
			ROS_INFO("Debug angles @ %f %f %f", angles[0], angles[1], 
					angles[2]);
			ROS_INFO("Valid observation offset @ %f %f depth @ %f\n", output[0], 
					output[1], dist-1.);
		}
		else 
		{
			attempts -= 1;
		}
		ros::Duration(1.).sleep();
	}

	if (attempts <= 1)
	{
		ROS_INFO("Failed to pass threshold for down alignment.");
		return std::make_pair(-999., -999.);
	}
	x_avg /= attempts;
	y_avg /= attempts;
	return std::make_pair(x_avg, y_avg);
}

void setForward(float dist)
{
	State now = control_client::state();
	float input[3] = {dist, 0., 0.};
	float angles[3] = {now.axis[YAW], 0., 0.};
	float output[3];
	bodyToInertial(input, angles, output);
	now.axis[X] += output[0];
	now.axis[Y] += output[1];
	ROS_INFO("Setting forward state @ %s", now.text().c_str());
	move(now);
}

void setAngle(float angle)
{
	State now = control_client::state();
	now.axis[YAW] = angle;
	ROS_INFO("Setting angle state @ %s.", now.text().c_str());
	move(now);
}

void addAngle(float angle)
{
	State now = control_client::state();
	now.axis[YAW] = angleAdd(now.axis[YAW], angle);
	if (now.axis[YAW] > 180.) now.axis[YAW] -= 360.;
	if (now.axis[YAW] < -180.) now.axis[YAW] -= 360.;
	ROS_INFO("Setting add angle state @ %s.", now.text().c_str());
	move(now);
}

void setCoordinate(Coordinate coordinate)
{
	State now = control_client::state();
	now.axis[X] = coordinate.first;
	now.axis[Y] = coordinate.second;
	ROS_INFO("Setting coordinate state @ %s.", now.text().c_str());
	move(now);
}

void addCoordinate(Coordinate coordinate)
{
	State now = control_client::state();
	float input[3] = {coordinate.first, coordinate.second, 0.};
	float angles[3] = {now.axis[YAW], 0., 0.};
	float output[3];
	bodyToInertial(input, angles, output);
	now.axis[X] += output[0];
	now.axis[Y] += output[1];
	ROS_INFO("Setting add coordinate state @ %s.", now.text().c_str());
	move(now);
}

bool isValidCoordinate(Coordinate coordinate)
{
	return coordinate.first > -50. && coordinate.second < 50.;
}

bool isValidOffsetCoordinate(Coordinate coordinate)
{
	return coordinate.first > -5. && coordinate.second < 5.;
}

void move(const State &dest)
{
	control_client::writeState(dest);
	bool quit = false;
	while (!quit && ros::ok())
	{
		State now = control_client::state();
		if (std::fabs(dest.axis[X]-now.axis[X]) > 1.)
			ros::Duration(3.).sleep();
		else if (std::fabs(dest.axis[Y]-now.axis[Y]) > 1.)
			ros::Duration(3.).sleep();
		else if (std::fabs(angleDifference(dest.axis[YAW], now.axis[YAW])) > 5.)
			ros::Duration(3.).sleep();
		else
		{
			quit = true; 
			ros::Duration(3.).sleep();
		}
	}
}
