#ifndef VISION_SERVICE_HPP
#define VISION_SERVICE_HPP 

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vision/Vision.h>
#include "vision/observation.hpp"
#include "vision/model.hpp"
#include "vision/tensor.hpp"

class VisionService 
{
	public:
		cv::Mat front, down;
		Model model;
		Task task;

		bool detectCallback(vision::Vision::Request &, 
				vision::Vision::Response &);
		void captureCallback(const sensor_msgs::ImageConstPtr &);
		Observation findGate(const cv::Mat &);
		Observation findGateML(cv::Mat);
};

void setResponse(const Observation &, vision::Vision::Response &);

#endif 
