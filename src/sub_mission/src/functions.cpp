#include "mission/functions.hpp"
#include "mission/commands.hpp"
#include "vision/tasks.hpp"
#include "vision/config.hpp"
#include "control/atmega.hpp"


void gate(vision::Perception &per, ros::ServiceClient &client)
{
	ROS_INFO("Beginning GATE function.");
	std::cout << atmega::state().text() << std::endl;
	per.request.task = Task::GATE;
	per.request.camera = FRONT;

	float angle = align(per, client, 5);
	ROS_INFO("Angle @ %f.", angle);
	State move1 = atmega::state();
	move1.axis[YAW] = angle;
	move(move1);

	float dist = 10.;
	State move2 = atmega::state();
	move2.axis[X] += std::sin(angle)*dist;
	move2.axis[Y] += std::cos(angle)*dist;
	ROS_INFO("State @ (%f, %f, %f, %f, %f, %f).", move2.axis[0], move2.axis[1], 
			move2.axis[2], move2.axis[3], move2.axis[4], move2.axis[5]);
	move(move2);
}

void octagon()
{
	
}

void printResponse(vision::Perception &per)
{
	ROS_INFO("%i observation @ %f H-deg, %f V-deg, and %f meters. Image location @ (%f, %f).", 
			per.request.task, per.response.hangle, per.response.vangle, per.response.dist,
			per.response.r, per.response.c);
}
