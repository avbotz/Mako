/** @file functions.cpp
 *  @brief Functions for each task process during competition.
 *
 *  @author David Zhang
 */
#include "mission/functions.hpp"
#include "mission/commands.hpp"
#include "mission/client.hpp"
#include "vision/observation.hpp"
#include "mission/config.hpp"


void gate()
{
	ROS_INFO("Beginning GATE function.");

	ROS_INFO("Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	initial.axis[Z] = 1.25;
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(3.).sleep();

	ROS_INFO("Turn towards gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float angle = align(5, Task::GATE, FRONT);
	ROS_INFO("Angle @ %f.", angle);
	if (angle > -180. && angle < 180.) setAngle(angle);
	ros::Duration(1.).sleep();

	ROS_INFO("Go through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setForward(12.5);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
}

void gate_extra()
{
	ROS_INFO("Beginning GATE_EXTRA function.");

	ROS_INFO("Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	initial.axis[Z] = 1.5;
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(3.).sleep();

	/*
	ROS_INFO("Set initial depth.");
	// float dist = control_client::depth();
	float dist = 0.8;
	control_client::writeDepth(dist);
	*/

	/*
	ROS_INFO("Turn towards gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float angle = align(5, Task::GATE, FRONT);
	ROS_INFO("Angle @ %f.", angle);
	if (angle > -180. && angle < 180.) setAngle(angle);
	ros::Duration(1.).sleep();
	*/

	ROS_INFO("Go through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setForward(12.5);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());

	ROS_INFO("Spin in gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	for (int i = 0; i < 4; i++)
	{
		State temp = control_client::state();
		temp.axis[YAW] = angleAdd(temp.axis[YAW], 90.);
		temp.axis[Z] = 1.5;
		move(temp);
		ROS_INFO("Spin %d @ %s", i+1, control_client::state().text().c_str());
		ros::Duration(3.).sleep();
	}
	ROS_INFO("New state @ %s.", control_client::state().text().c_str());

	ROS_INFO("Continue forward through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	setForward(3.);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
}

void target()
{
	ROS_INFO("Beginning TARGET function.");

	ROS_INFO("Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(1.).sleep();

	ROS_INFO("Set initial depth.");
	control_client::writeDepth(1.8);

	ROS_INFO("Turn towards target.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float angle = align(5, Task::TARGET, FRONT);
	ROS_INFO("Angle @ %f.", angle);
	if (angle > -180. && angle < 180.) setAngle(angle);
	ros::Duration(1.).sleep();

	ROS_INFO("Ram target.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float ram_dist = distance(5, Task::TARGET, FRONT);
	if (ram_dist > 0. && ram_dist < 8.) setForward(ram_dist);
	ros::Duration(1.).sleep();

	ROS_INFO("Backup from target.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float backup_dist = -2.;
	setForward(backup_dist);
	ros::Duration(1.).sleep();
}

void bins()
{
	ROS_INFO("Beginning BINS function.");

	ROS_INFO("Set initial state.");
	// State initial(0., 0., 0., 30., 5., 0.);
	State initial = control_client::state();
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(1.).sleep();

	ROS_INFO("Go towards bins, avoid target.");
	/*
	addCoordinate(std::make_pair(3., 0.));
	ros::Duration(1.).sleep();
	addAngle(30.);
	addCoordinate(std::make_pair(6., 0.));
	ros::Duration(1.).sleep();
	addCoordinate(std::make_pair(2., 3.));
	*/

	ROS_INFO("Set initial depth.");
	control_client::writeDepth(3.5);
	ros::Duration(6.).sleep();

	ROS_INFO("Find offsets for bins.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	Coordinate coordinate1 = downAlign(5, Task::BINS_ML, DOWN);
	ROS_INFO("Body Offset @ %f, %f.", coordinate1.first, coordinate1.second);
	if (isValidCoordinate(coordinate1))
	{
		setCoordinate(coordinate1);
		ros::Duration(6.).sleep();
	}

	ROS_INFO("Set second depth.");
	control_client::writeDepth(3.);
	ros::Duration(6.).sleep();

	ROS_INFO("Find second offsets for bins.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	Coordinate coordinate2 = downAlign(5, Task::BINS_ML, DOWN);
	ROS_INFO("Body Offset @ %f, %f.", coordinate2.first, coordinate2.second);
	if (isValidCoordinate(coordinate2))
	{
		setCoordinate(coordinate2);
		ros::Duration(4.).sleep();
	}

	ROS_INFO("Set third depth.");
	control_client::writeDepth(2.);
	ros::Duration(6.).sleep();

	for (int i = 0; i < 3; i++)
	{
		ROS_INFO("Find second offsets for bins.");
		ROS_INFO("State @ %s.", control_client::state().text().c_str());
		Coordinate coordinate3 = downAlign(5, Task::BINS_ML, DOWN);
		ROS_INFO("Body Offset @ %f, %f.", coordinate3.first, coordinate3.second);
		if (isValidCoordinate(coordinate3))
		{
			setCoordinate(coordinate3);
			ros::Duration(4.).sleep();
		}
	}

	ROS_INFO("Set dropping depth.");
	float dropping_depth = 1.;
	control_client::writeDepth(dropping_depth);
	// control_client::write("z 0.6\n");
	ros::Duration(6.).sleep();

	ROS_INFO("Drop the balls.");
	for (int i = 0; i < 6; i++)
	{
		control_client::write("g 0 1");
		control_client::write("g 1 1");
		control_client::write("g 0 0");
		control_client::write("g 1 0");
		ros::Duration(1.).sleep();
	}
	ros::Duration(1.).sleep();

	/*
	ROS_INFO("Reset depth.");
	// control_client::write("z -1\n");
	control_client::writeDepth(-1.);
	ros::Duration(6.).sleep();
	*/
}

void octagon()
{
	ROS_INFO("Beginning GATE_EXTRA function.");

	ROS_INFO("Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	initial.axis[Z] = 1.;
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(1.).sleep();

	ROS_INFO("Going towards octagon.");
	State move1 = control_client::state();
	move1.axis[X] = 28.;
	move1.axis[Y] = 3.;
	ROS_INFO("New State @ %s.", move1.text().c_str());
	move(move1);
	ros::Duration(8.).sleep();

	ROS_INFO("Surfacing.");
	control_client::write("p 0");
}

