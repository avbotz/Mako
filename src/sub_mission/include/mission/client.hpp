#ifndef MISSION_SERVICE_HPP
#define MISSION_SERVICE_HPP

#include <ros/ros.h>
#include <control/ControlAlive.h>
#include <control/ControlState.h>
#include <control/ControlWrite.h>
#include <control/ControlWriteState.h>
#include <vision/Vision.h>
#include "control/state.hpp"
#include "vision/tasks.hpp"
#include "vision/config.hpp"

namespace vision_client 
{
	static vision::Vision srv;
	static ros::ServiceClient client;

	void setTask(Task);
	void setCamera(int);
	Observation vision(); 
};

namespace control_client
{
	static ros::ServiceClient alive_client;
	static ros::ServiceClient state_client;
	static ros::ServiceClient write_client;
	static ros::ServiceClient write_state_client;

	bool alive();
	State state();
	void write(std::string);
	void writeState(const State &);
};


#endif
