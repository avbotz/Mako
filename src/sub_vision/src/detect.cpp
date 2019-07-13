#include "vision/service.hpp"
#include "vision/tasks.hpp"
#include "vision/config.hpp"


bool VisionService::detectCallback(vision::Vision::Request &req, 
		vision::Vision::Response &res)
{
	ROS_INFO("Received detection request for %i.", req.task);
	if (req.task == Task::GATE)
	{
		Observation obs = findGate(this->front);
		obs.calcAngles(FRONT);
		setResponse(obs, res);
		return true;
	}
	if (req.task == Task::OCTAGON) 
	{

	}
	ROS_INFO("Finished detection request.");
	return false;
}

void setResponse(const Observation &obs, vision::Vision::Response &res)
{
	res.prob = obs.prob;
	res.r = obs.r;
	res.c = obs.c;
	res.dist = obs.dist;
	res.hangle = obs.hangle;
	res.vangle = obs.vangle;
}
