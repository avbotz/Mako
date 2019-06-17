#ifndef VISION_SERVICE_HPP
#define VISION_SERVICE_HPP 

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vision/Perception.h>
#include "vision/tasks.hpp"

class VisionService 
{
	public:
		cv::Mat front, down;
		bool detectCallback(vision::Perception::Request &, vision::Perception::Response &);
		void captureCallback(const sensor_msgs::ImageConstPtr &);
};

void setResponse(const Observation &, vision::Perception::Response &);

#endif 
