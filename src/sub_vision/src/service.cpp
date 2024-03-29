/** @file vision/src/service.cpp
 *  @brief Wrapper definitions to handle the different vision callbacks.
 *
 *  @author David Zhang
 *  @author Emil Tu
 */
#include <ros/ros.h>
#include "vision/service.hpp"


void VisionService::frontCaptureCallback(const sensor_msgs::ImageConstPtr &msg)
{
	// Read front camera data from ROS Spinnaker publisher.
	try 
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
		image.copyTo(this->front);
		if (LOG_FRONT && !FAST_LOG) 
			log(this->front, 'f');
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not read image from Spinnaker publisher.");
	}
}

void VisionService::downCaptureCallback(const sensor_msgs::ImageConstPtr &msg)
{
	// Read front camera data from ROS Spinnaker publisher.
	try 
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
		image.copyTo(this->down);
		if (LOG_DOWN && !FAST_LOG) 
			log(this->down, 'd');
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not read image from Spinnaker publisher.");
	}
}

bool VisionService::detectCallback(vision::Vision::Request &req, 
		vision::Vision::Response &res)
{
	// Load new vision model if new task is different and requires a machine
	// learning model.
	if (req.task != this->task)
	{
		if (req.task == Task::GATE_ML)
		{
			ROS_INFO("Starting to setup gate model.");
			this->model.setup("models/cpu_gate.pb");
			this->task = Task::GATE_ML;
			ROS_INFO("Done setting up gate model.");
		}
		else if (req.task == Task::BINS_ML)
		{
			ROS_INFO("Starting to setup bins model.");
			this->model.setup("models/gpu_bins.pb");
			this->task = Task::BINS_ML;
			ROS_INFO("Done settings up bins model.");
		}
	}

	// Get new observation from vision functions. 
	ROS_INFO("Received detection request for %i.", req.task);
	if (req.task == Task::GATE)
	{
		Observation obs = this->findGate(this->front);
		obs.calcAngles(FRONT);
		ROS_INFO("Sending observation @ %s", obs.text().c_str());
		setResponse(obs, res);
		return true;
	}
	else if (req.task == Task::GATE_ML)
	{
		Observation obs = this->findGateML(this->front);
		obs.calcAngles(FRONT);
		ROS_INFO("Sending observation @ %s", obs.text().c_str());
		setResponse(obs, res);
		return true;
	}
	else if (req.task == Task::TARGET)
	{
		Observation obs = this->findTarget(this->front);
		obs.calcAngles(FRONT);
		ROS_INFO("Sending observation @ %s", obs.text().c_str());
		setResponse(obs, res);
		return true;
	}
	else if (req.task == Task::TARGET_ML)
	{
		Observation obs = this->findTarget(this->front);
		obs.calcAngles(FRONT);
		ROS_INFO("Sending observation @ %s", obs.text().c_str());
		setResponse(obs, res);
		return true;
	}
	else if (req.task == Task::BINS)
	{
		Observation obs = this->findBins(this->down);
		obs.calcAngles(DOWN);
		ROS_INFO("Sending observation @ %s", obs.text().c_str());
		setResponse(obs, res);
		return true;
	}
	else if (req.task == Task::BINS_ML)
	{
		Observation obs = this->findBinsML(this->down);
		obs.calcAngles(DOWN);
		ROS_INFO("Sending observation @ %s", obs.text().c_str());
		setResponse(obs, res);
		return true;
	}
	else if (req.task == Task::OCTAGON) 
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
