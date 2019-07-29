/** @file client.cpp
 *  @brief Wrapper definitions for using ROS clients.
 *
 *  @author David Zhang
 */
#include "mission/client.hpp"


namespace vision_client 
{
	ros::ServiceClient client;

	Observation vision(Task task, int camera)
	{
		vision::Vision::Request req;
		vision::Vision::Response rep;
		req.task = task;
		req.camera = camera;
		client.call(req, rep);
		Observation obs(rep.prob, rep.r, rep.c, rep.dist, rep.hangle, 
				rep.vangle); 
		// ROS_INFO("%s", obs.text().c_str());
		return obs;
	}
};

namespace control_client
{
	ros::ServiceClient alive_client;
	ros::ServiceClient state_client;
	ros::ServiceClient depth_client;
	ros::ServiceClient write_client;
	ros::ServiceClient write_state_client;
	ros::ServiceClient write_depth_client;

	bool alive()
	{
		control::ControlAlive::Request req;
		control::ControlAlive::Response rep;
		alive_client.call(req, rep);
		return rep.data;
	}

	State state()
	{
		control::ControlState::Request req;
		control::ControlState::Response rep;
		state_client.call(req, rep);
		return State(rep.F, rep.H, rep.V, rep.Y, rep.P, rep.R);
	}
	
	float depth()
	{
		control::ControlDepth::Request req;
		control::ControlDepth::Response rep;
		depth_client.call(req, rep);
		return rep.depth;
	}

	void write(std::string input)
	{
		control::ControlWrite::Request req;
		control::ControlWrite::Response rep;
		req.data = input;
		write_client.call(req, rep);
	}

	void writeState(const State &state)
	{
		control::ControlWriteState::Request req;
		control::ControlWriteState::Response rep;
		req.F = state.axis[X];
		req.H = state.axis[Y];
		req.V = state.axis[Z];
		req.Y = state.axis[YAW];
		req.P = state.axis[PITCH];
		req.R = state.axis[ROLL];
		write_state_client.call(req, rep);
	}
	
	void writeDepth(float dist)
	{
		control::ControlWriteDepth::Request req;
		control::ControlWriteDepth::Response rep;
		req.dist = dist;
		write_depth_client.call(req, rep);
	}
};
