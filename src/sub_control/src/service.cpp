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
	State state = atmega::state();	
	ROS_INFO("Received state request. State @ %s", state.text().c_str());
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
	std::string data = request.data;
	ROS_INFO("Received write request. Data @ %s", data.c_str());
	atmega::write(data);
	return true;
}

bool writeState(control::ControlWriteState::Request &request,
		control::ControlWriteState::Response &response)
{
	State state(request.F, request.H, request.V, request.Y, request.P, 
			request.R);
	ROS_INFO("Received write state request. State @ %s", state.text().c_str());
	atmega::write(state);
	return true;
}

