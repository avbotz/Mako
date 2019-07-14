#include <iostream>
#include <ros/ros.h>
#include "control/service.hpp"
#include "control/atmega.hpp"


bool alive(control::ControlAlive::Request &request,
		control::ControlAlive::Response &response)
{
	ROS_INFO("Received alive request.");
	response.data = atmega::alive();	
	return true;
}

bool state(control::ControlState::Request &request, 
		control::ControlState::Response &response)
{
	ROS_INFO("Received state request.");
	State state = atmega::state();	
	response.F = state.axis[X];
	response.H = state.axis[Y];
	response.V = state.axis[Z];
	response.Y = state.axis[YAW];
	response.P = state.axis[PITCH];
	response.R = state.axis[ROLL];
	return true;
}

bool write(control::ControlWrite::Request &request, 
		control::ControlWrite::Response &response)
{
	ROS_INFO("Received write request.");
	std::string data = request.data;
	atmega::write(data);
	return true;
}

bool writeState(control::ControlWriteState::Request &request,
		control::ControlWriteState::Response &response)
{
	ROS_INFO("Received write state request.");
	State state(request.F, request.H, request.V, request.Y, request.P, 
			request.R);
	atmega::write(state);
	return true;
}

