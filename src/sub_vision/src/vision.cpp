/** @file vision.cpp
 *  @brief Main node runner for vision.
 *
 *  @author David Zhang
 */
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "vision/filters.hpp"
#include "vision/config.hpp"
#include "vision/service.hpp"
#include "vision/camera.hpp"
#include "vision/log.hpp"


int main(int argc, char** argv)
{
	srand((unsigned) time(0)); 
	ros::init(argc, argv, "vision_node");
	ros::NodeHandle node;

	// Set no task in the beginning so the first model is loaded.
	VisionService service;
	service.task = Task::NONE;

	// Setup observation request.
	ros::ServiceServer server = node.advertiseService("vision", 
			&VisionService::detectCallback, &service);

	// Setup front camera to receive images. 
	image_transport::ImageTransport it(node);
	image_transport::Subscriber sub = it.subscribe("front_camera", 1, 
			&VisionService::captureCallback, &service);

	// Setup down camera to receive images.
	Camera down;
	bool isdown = down.init(1);

	// Create directory to log images. 
	init(); 

	while (ros::ok())
	{
		switch (CAMERA_MODE)
		{
			case CameraMode::MOCK:
				service.front = cv::imread("mock.png", 1);
				service.down = cv::imread("mock.png", 1);
				break;
			case CameraMode::LIVE:

				// Updates front camera.
				ros::spinOnce();

				// Read down camera without using ROS.
				if (isdown) service.down = down.capture(false);

				break;
		}
	}
}
