#include <ros/ros.h>
#include "vision/tasks.hpp"
#include "vision/filters.hpp"
#include "vision/log.hpp"


Observation findGate(const cv::Mat &img)
{
	// Check that image isn't null.
	if (!img.data)
	{
		ROS_INFO("No image data for gate.");
		return Observation(0, 0, 0, 0);
	}

	// Illuminate image.
	cv::Mat illum = illumination(img);

	// Light blur.
	cv::Mat blur;
	cv::blur(illum, blur, cv::Size(5, 5));

	// Canny edge detection.
	cv::Mat can, cdst;
	cv::Canny(blur, can, 150, 200, 3);
	cv::cvtColor(can, cdst, cv::COLOR_GRAY2BGR);

	// Get lines with HoughLinesP
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(can, lines, 1, CV_PI/180, 35, 20, 20);
	for (int i = 0; i < lines.size(); i++) 
	{
		cv::Vec4i line = lines[i];
		int x1=line[0],y1=line[1],x2=line[2],y2=line[3];
		float dist = std::sqrt(std::pow(std::abs(x1 - x2), 2) + std::pow(std::abs(y1 - y2), 2));
		float rotation = std::acos(std::abs(y1 - y2) / dist) * 180/CV_PI;
		if (std::abs(rotation) <= 15)
		{
			cv::line(cdst, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 3, CV_AA);		
			int xmid = (x1+x2)/2;
			int ymid = (y1+y2)/2;	
			log(cdst, 'e');

			return Observation(1.0, xmid, ymid, 0.0);
		}
	}

	return Observation(0, 0, 0, 0);
}
