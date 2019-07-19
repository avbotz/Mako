/** @file mock_camera.cpp
 *  @brief Main node runner to simulate acquisition_node with a test image.
 *  
 *  @author David Zhang
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_mock_camera_node");
	ros::NodeHandle node;

	// Rather than using Spinnaker to publish an image, read test_image.png and
	// publish that instead.
	image_transport::ImageTransport it(node);
	image_transport::Publisher pub = it.advertise("front_camera", 1);
	cv::Mat image = cv::imread("test_image.png", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = 
		cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

	ros::Rate rate(5);
	while (node.ok()) 
	{
		ROS_INFO("Published image.");
		ros::Duration(0.5).sleep();
		pub.publish(msg);
		ros::spinOnce();
	}
}
