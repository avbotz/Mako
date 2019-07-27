/** @file bins.cpp
 *  @brief Vision functions to detect the bins
 *
 *  @author David Zhang
 */
#include "vision/service.hpp"
#include "vision/filters.hpp"


Observation VisionService::findBins(const cv::Mat &input)
{
	/*
	 * This bins code is meant to be run at Suhas' pool, with a black outline
	 * and white inside. Do not use for competition.
	 */
	// Illuminate image using filter.
	// cv::Mat illum = illumination(input);
	cv::Mat illum = input;
	log(illum, 'e');

	// Strong blur to remove noise from image.
	cv::Mat blur;
	cv::blur(illum, blur, cv::Size(9, 9));

	// Threshold for black.
	cv::Mat thresh;
	cv::Mat cdst;
	cv::inRange(blur, cv::Scalar(0, 0, 0), cv::Scalar(30, 30, 30), thresh);
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
		float rect_ratio = (float)(rectangles[i].height)/(float)(rectangles[i].width);
		if (rect_ratio < 2. && rect_ratio > 0.5)
		{
			cv::rectangle(cdst, rectangles[i].tl(), rectangles[i].br(), 
					cv::Scalar(255, 0, 255), 3, 8, 0);		
			int x = (rectangles[i].tl().x+rectangles[i].br().x)/2;
			int y = (rectangles[i].tl().y+rectangles[i].br().y)/2;
			cv::circle(cdst, cv::Point(x, y), 4, cv::Scalar(255, 0, 255), 3);
			log(cdst, 'e');
			return Observation(0.8, x, y, 0);
		}
	}

	// No contours found. I doubt this will run often because the sub is more
	// prone to picking up noise instead of nothing at all.
	return Observation(0, 0, 0, 0);
}
