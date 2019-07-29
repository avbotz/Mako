/** @file target.cpp
 *  @brief Vision functions to detect the target.
 *
 *  @author David Zhang
 */
#include <ros/ros.h>
#include "vision/service.hpp"


Observation VisionService::findTarget(const cv::Mat &input)
{
	/*
	 * This target code is meant to be run at Suhas' pool, with a black outline.
	 * Do not use for competition.
	 */
	// Illuminate image using filter.
	// cv::Mat illum = illumination(input);
	cv::Mat illum = input;

	// Strong blur to remove noise from image.
	cv::Mat blur;
	cv::blur(illum, blur, cv::Size(9, 9));
	log(illum, 'e');

	// Threshold for black.
	cv::Mat thresh;
	cv::Mat cdst;
	cv::inRange(blur, cv::Scalar(0, 0, 0), cv::Scalar(60, 50, 50), thresh);
	cv::cvtColor(thresh, cdst, cv::COLOR_GRAY2BGR);
	// cv::cvtColor(thresh, thresh, cv::COLOR_BGR2GRAY);

	// Contour detection.
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(thresh, contours, hierarchy, CV_RETR_TREE, 
			CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::drawContours(cdst, contours, i, cv::Scalar(0, 255, 0), 2, 8, 
				hierarchy, 0, cv::Point());
	}

	// Approximate and convert to rectangles.
	std::vector<std::vector<cv::Point>> contour_polygons (contours.size());
	std::vector<cv::Rect> rectangles (contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP(cv::Mat(contours[i]), contour_polygons[i], 
				0.01*cv::arcLength(cv::Mat(contours[i]), true), true);
		rectangles[i] = cv::boundingRect(cv::Mat(contour_polygons[i]));
	}
	for (int i = 0; i < contours.size(); i++)
	{
		cv::drawContours(cdst, contour_polygons, i, cv::Scalar(255, 0, 0), 1, 8, 
				std::vector<cv::Vec4i>(), 0, cv::Point());
		cv::rectangle(cdst, rectangles[i].tl(), rectangles[i].br(), 
				cv::Scalar(0, 0, 255), 2, 8, 0);
	}

	// Choose largest rectangle that is in an appropriate ratio.
	std::sort(rectangles.begin(), rectangles.end(), [](const cv::Rect &a, 
				const cv::Rect &b) -> bool { return a.area() > b.area(); });
	for (int i = 0; i < rectangles.size(); i++)
	{
		float height = rectangles[i].height;
		float width = rectangles[i].width;
		float rect_ratio = height/width;
		if (rect_ratio < 4. && rect_ratio > 0.25 && 
				rectangles[i].tl().x+width/2. > 456 && 
				rectangles[i].br().x+height/2. < 5016)
		{
			cv::rectangle(cdst, rectangles[i].tl(), rectangles[i].br(), 
					cv::Scalar(255, 0, 255), 3, 8, 0);		
			int x = (rectangles[i].tl().x+rectangles[i].br().x)/2;
			int y = (rectangles[i].tl().y+rectangles[i].br().y)/2;
			cv::circle(cdst, cv::Point(x, y), 4, cv::Scalar(255, 0, 255), 3);
			log(cdst, 'e');
			float dist = 2250./rectangles[i].width;
			if (dist > 10.) dist = 10.;
			return Observation(0.8, y, x, dist);
		}
	}

	// No contours found. I doubt this will run often because the sub is more
	// prone to picking up noise instead of nothing at all.
	return Observation(0, 0, 0, 0);
}
