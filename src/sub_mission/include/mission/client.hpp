#ifndef MISSION_SERVICE_HPP
#define MISSION_SERVICE_HPP

#include <ros/ros.h>
#include <control/ControlAlive.h>
#include <control/ControlState.h>
#include <control/ControlWrite.h>
#include <control/ControlWriteState.h>
#include <vision/Vision.h>
#include "control/state.hpp"
#include "vision/observation.hpp"
#include "vision/config.hpp"

namespace vision_client 
{
	extern ros::ServiceClient client;

	Observation vision(Task, int); 
};

namespace control_client
{
	extern ros::ServiceClient alive_client;
	extern ros::ServiceClient state_client;
	extern ros::ServiceClient write_client;
	extern ros::ServiceClient write_state_client;

	bool alive();
	State state();
	void write(std::string);
	void writeState(const State &);
};

#endif
