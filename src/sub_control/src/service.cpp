#include <iostream>
#include "control/service.hpp"
#include "control/atmega.hpp"


bool alive(control::ControlAlive::Request &request,
		control::ControlAlive::Response &response)
{
	response.data = atmega::alive();	
	return true;
}

bool state(control::ControlState::Request &request, 
		control::ControlState::Response &response)
{
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
	std::string data = request.data;
	atmega::write(data);
	return true;
}

bool writeState(control::ControlWriteState::Request &request,
		control::ControlWriteState::Response &response)
{
	State state(request.F, request.H, request.V, request.Y, request.P, 
			request.R);
	atmega::write(state);
	return true;
}

