#include "mission/client.hpp"


namespace vision_client 
{
	void setTask(Task task)
	{
		srv.request.task = task;
	}

	void setCamera(int camera)
	{
		srv.request.camera = camera;
	}

	Observation vision()
	{
		client.call(srv);
		return Observation(srv.response.prob, srv.response.r, srv.response.c, 
				srv.response.hangle, srv.response.vangle);
	}
};

namespace control_client
{
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
};
