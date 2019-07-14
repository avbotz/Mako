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
		Observation 
	}
};

namespace control_client
{
	bool alive()
	{

	}

	State state()
	{

	}

	void write(std::string) 
	{

	}

	void writeState(const State &) 
	{

	}
};
