#include <ros/ros.h>
#include "vision/config.hpp"
#include "vision/service.hpp"
#include "vision/log.hpp"


void VisionService::captureCallback(const sensor_msgs::ImageConstPtr &msg)
{
	// Read front camera data from ROS Spinnaker publisher.
	try 
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
		image.copyTo(this->front);
		if (LOG) 
		{
			log(this->front, 'f');
		}
		else 
		{
			// ROS_INFO("Received image from acquisition.");
		}
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not read image from Spinnaker publisher.");
	}
}

bool VisionService::detectCallback(vision::Vision::Request &req, 
		vision::Vision::Response &res)
{
	ROS_INFO("Received detection request for %i.", req.task);
	if (req.task == Task::GATE)
	{
		Observation obs = findGate(this->front);
		obs.calcAngles(FRONT);
		ROS_INFO("Sending observation @ %s", obs.text().c_str());
		setResponse(obs, res);
		return true;
	}
	if (req.task == Task::GATE_ML)
	{
		Observation obs = findGateML(this->front);
		obs.calcAngles(FRONT);
		ROS_INFO("Sending observation @ %s", obs.text().c_str());
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
