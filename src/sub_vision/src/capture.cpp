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
