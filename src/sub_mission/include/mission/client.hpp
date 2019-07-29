/** @file client.hpp
 *  @brief Wrapper namespaces for using ROS clients.
 *
 *  @author David Zhang
 */
#ifndef MISSION_CLIENT_HPP
#define MISSION_CLIENT_HPP

#include <ros/ros.h>
#include <control/ControlAlive.h>
#include <control/ControlState.h>
#include <control/ControlDepth.h>
#include <control/ControlWrite.h>
#include <control/ControlWriteState.h>
#include <control/ControlWriteDepth.h>
#include <vision/Vision.h>
#include "control/state.hpp"
#include "vision/observation.hpp"

namespace vision_client 
{
	extern ros::ServiceClient client;

	Observation vision(Task, int); 
};

namespace control_client
{
	extern ros::ServiceClient alive_client;
	extern ros::ServiceClient state_client;
	extern ros::ServiceClient depth_client;
	extern ros::ServiceClient write_client;
	extern ros::ServiceClient write_state_client;
	extern ros::ServiceClient write_depth_client;

	bool alive();
	State state();
	float depth();
	void write(std::string);
	void writeState(const State &);
	void writeDepth(float);
};

#endif
