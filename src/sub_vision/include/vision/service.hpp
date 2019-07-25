/** @file vision/service.hpp
 *  @brief Wrapper class to handle the different vision callbacks.
 *
 *  The main purpose of this class is to ensure that the images read from the
 *  ROS image publisher can be used for object detection. It keeps the images in
 *  one location and allows ros::spin() to update them as needed.
 *
 *  @author David Zhang
 *  @author Emil Tu
 */
#ifndef VISION_SERVICE_HPP
#define VISION_SERVICE_HPP 

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vision/Vision.h>
#include "vision/observation.hpp"
#include "vision/model.hpp"
#include "vision/tensor.hpp"
#include "vision/log.hpp"

class VisionService 
{
	public:
		cv::Mat front, down;
		Model model;
		Task task;

		bool detectCallback(vision::Vision::Request &, 
				vision::Vision::Response &);
		void frontCaptureCallback(const sensor_msgs::ImageConstPtr &);
		void downCaptureCallback(const sensor_msgs::ImageConstPtr &);
		Observation findGate(const cv::Mat &);
		Observation findGateML(cv::Mat);
		Observation findBins(const cv::Mat &);
};

void setResponse(const Observation &, vision::Vision::Response &);

#endif 
