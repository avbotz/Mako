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
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(1.).sleep();

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
	float dist = 2.8;
	State move2 = control_client::state();
	move2.axis[X] += std::cos(angle*D2R)*dist;
	move2.axis[Y] += std::sin(angle*D2R)*dist;
	ROS_INFO("New State @ %s", move2.text().c_str());
	move(move2);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());
}

void gate_extra()
{
	ROS_INFO("Beginning GATE_EXTRA function.");
	
	ROS_INFO("Set initial state.");
	// State initial(3.28, 2.95, 1.37, 28.12, 5.12, 3.12);
	State initial = control_client::state();
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(1.).sleep();

	ROS_INFO("Set initial depth.");
	// float dist = control_client::depth();
	float dist = 0.8;
	control_client::writeDepth(dist);

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
	dist = 1.;
	State move2 = control_client::state();
	move2.axis[X] += std::cos(angle*D2R)*dist;
	move2.axis[Y] += std::sin(angle*D2R)*dist;
	ROS_INFO("New State @ %s", move2.text().c_str());
	move(move2);
	ROS_INFO("Final State @ %s", control_client::state().text().c_str());

	ROS_INFO("Spin in gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	for (int i = 0; i < 4; i++)
	{
		State move3 = control_client::state();
		move3.axis[YAW] += 90.;
		if (move3.axis[YAW] > 180.) move3.axis[YAW] -= 360.;
		move(move3);
		ROS_INFO("First spin @ %s", control_client::state().text().c_str());
	}
	ROS_INFO("New state @ %s.", control_client::state().text().c_str());

	ROS_INFO("Go through gate.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	dist = 0.5;
	State move4 = control_client::state();
	move4.axis[X] += std::cos(angle*D2R)*dist;
	move4.axis[Y] += std::sin(angle*D2R)*dist;
	ROS_INFO("New State @ %s", move4.text().c_str());
	move(move4);
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
	// float dist = control_client::depth();
	float dist = 0.68;
	control_client::writeDepth(dist);

	ROS_INFO("Turn towards target.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float angle = align(5, Task::TARGET, FRONT);
	ROS_INFO("Angle @ %f.", angle);
	State move1 = control_client::state();
	move1.axis[YAW] = angle;
	ROS_INFO("New State @ %s.", move1.text().c_str());
	move(move1);
	ros::Duration(1.5).sleep();

	ROS_INFO("Ram target.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	dist = distance(5, Task::TARGET, FRONT);
	State move2 = control_client::state();
	move2.axis[X] += std::cos(angle*D2R)*dist;
	move2.axis[Y] += std::sin(angle*D2R)*dist;
	ROS_INFO("New State @ %s", move2.text().c_str());
	move(move2);
	ros::Duration(1.5).sleep();

	ROS_INFO("Backup from target.");
	ROS_INFO("State @ %s.", control_client::state().text().c_str());
	float back_dist = -2.;
	State move3 = control_client::state();
	move3.axis[X] += std::cos(angle*D2R)*back_dist;
	move3.axis[Y] += std::sin(angle*D2R)*back_dist;
	ROS_INFO("New State @ %s", move3.text().c_str());
	move(move3);
	ros::Duration(1.5).sleep();
}

void bins()
{
	ROS_INFO("Beginning BINS function.");
	ros::Duration(6.).sleep();
	
	ROS_INFO("Set initial state.");
	// State initial(0., 0., 0., 30., 5., 0.);
	State initial = control_client::state();
	ROS_INFO("State @ %s.", initial.text().c_str());
	move(initial);
	ros::Duration(1.).sleep();

	ROS_INFO("Set initial depth.");
	// float dist = control_client::depth();
	float dist = 1.6;
	control_client::writeDepth(dist);
	// control_client::write("z " + std::to_string(dist) + "\n");
	ros::Duration(6.).sleep();

	for (int i = 0; i < 2; i++)
	{
		ROS_INFO("Find offsets for bins.");
		ROS_INFO("State @ %s.", control_client::state().text().c_str());
		std::pair<float, float> coordinate1 = down_align(5, Task::BINS, DOWN);
		ROS_INFO("Body Offset @ %f, %f.", coordinate1.first, coordinate1.second);
		State move1 = control_client::state();
		move1.axis[X] = coordinate1.first;
		move1.axis[Y] = coordinate1.second;
		ROS_INFO("New State @ %s.", move1.text().c_str());
		move(move1);
		ros::Duration(12.).sleep();
	}
	
	ROS_INFO("Set dropping depth.");
	control_client::writeDepth(0.6);
	// control_client::write("z 0.6\n");
	ros::Duration(6.).sleep();

	ROS_INFO("Drop the balls.");
	control_client::write("g 0 1");
	control_client::write("g 1 1");
	ros::Duration(2.).sleep();

	ROS_INFO("Reset ball dropper.");
	control_client::write("g 0 0");
	control_client::write("g 1 0");
	ros::Duration(2.).sleep();

	ROS_INFO("Reset depth.");
	// control_client::write("z -1\n");
	control_client::writeDepth(-1.);
	ros::Duration(6.).sleep();
}

void octagon()
{
	
}

