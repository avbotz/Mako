/** @file functions.cpp
 *  @brief Functions for each task process during competition.
 *
 *  @author David Zhang
 */
#include "mission/functions.hpp"
#include "mission/commands.hpp"
#include "mission/client.hpp"
#include "vision/observation.hpp"


void gate()
{
	ROS_INFO("Beginning GATE function.");
	
	ROS_INFO("Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(3.0).sleep();

	ROS_INFO("Turn towards gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float angle = align(5, Task::GATE, FRONT);
	ROS_INFO("Angle @ %f.", angle);
	State move1 = control_client::state();
	move1.axis[YAW] = angle;
	ROS_INFO("New State @ %s.", move1.text().c_str());
	move(move1);
	ros::Duration(1.5).sleep();

	ROS_INFO("Go through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float dist = 8.;
	State move2 = control_client::state();
	move2.axis[X] += std::cos(angle*M_PI/180.)*dist;
	move2.axis[Y] += std::sin(angle*M_PI/180.)*dist;
	ROS_INFO("New State @ %s", move2.text().c_str());
	move(move2);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
}

void bins()
{
	ROS_INFO("Beginning BINS function.");
	float dist = 1.;
	
	ROS_INFO("Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(3.0).sleep();

	ROS_INFO("Set initial depth.");
	control_client::writeDepth(1.);

	ROS_INFO("Find offsets for bins.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	std::pair<float, float> coordinate = down_align(3, dist, Task::BINS, DOWN);
	ROS_INFO("Offset @ %f, %f.", coordinate.first, coordinate.second);
	State move1 = control_client::state();
	move1.axis[X] = coordinate.first;
	move1.axis[Y] = coordinate.second;
	ROS_INFO("New State @ %s.", move1.text().c_str());
	move(move1);
	ros::Duration(3.0).sleep();

	/*
	 * TODO Add code to drop into bins.
	 */

	ROS_INFO("Reset depth.");
	control_client::writeDepth(-1.);
}

void octagon()
{
	
}

