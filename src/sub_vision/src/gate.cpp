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

    // Illuminate image using filter.
    // cv::Mat illum = illumination(img);
    
	// Strong blur to remove noise from image.
	cv::Mat blur;
	cv::blur(img, blur, cv::Size(17, 17));

	// Canny edge detection with low threshold.
	cv::Mat can, cdst;
    cv::Canny(blur, can, 20, 60, 3);
    cv::cvtColor(can, cdst, cv::COLOR_GRAY2BGR);
	
    // Get lines using OpenCV Hough Lines algorithm, and store probable lines to
	// use later. The last three parameters for HoughLinesP are threshold,
	// minLength, and maxGap.
    std::vector<cv::Vec4i> lines;
	std::vector<cv::Vec4i> probable_lines;
	int ac=0, bc=0, ar=0, br=0;
	cv::Vec4i a_line, b_line;
    cv::HoughLinesP(can, lines, 2, CV_PI/180, 50, 80, 30);
    for (int i = 0; i < lines.size(); i++) 
    {
        cv::Vec4i line = lines[i];
		int x1=line[0],y1=line[1],x2=line[2],y2=line[3];
    	float dist = std::sqrt(std::pow(std::abs(x1-x2), 2) + 
				std::pow(std::abs(y1-y2), 2));
    	float rotation = std::abs(std::acos(std::abs(y1-y2)/dist)*180/CV_PI);
        cv::line(cdst, cv::Point(x1, y1), cv::Point(x2, y2), 
				cv::Scalar(0, 0, 255), 3, CV_AA);		
        if (rotation <= 30 && dist > 100. && y1 < 2500 && y2 < 2500)
		{
			if (ac == 0)
			{
				ac = (x1+x2)/2;
				ar = (y1+y2)/2;
				a_line = cv::Vec4i(x1, y1, x2, y2);
			}
			else if (std::abs(x1-ac) > 150 && br == 0)
			{
				bc = (x1+x2)/2;
				br = (y1+y2)/2;
				b_line = cv::Vec4i(x1, y1, x2, y2);
			}
			else 
			{
				cv::line(cdst, cv::Point(x1, y1), cv::Point(x2, y2), 
						cv::Scalar(0, 255, 255), 3, CV_AA);		
			}
        }
    }
	cv::line(cdst, cv::Point(a_line[0], a_line[1]), cv::Point(a_line[2], a_line[3]), 
			cv::Scalar(255, 255, 255), 3, CV_AA);
	cv::line(cdst, cv::Point(b_line[0], b_line[1]), cv::Point(b_line[2], b_line[3]), 
			cv::Scalar(255, 255, 255), 3, CV_AA);
	cv::circle(cdst, cv::Point((ac+bc)/2, (ar+br)/2), 50, cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);
	log(cdst, 'e');

	// Calculate midpoint and return observation if valid.
	if (ac == 0 && bc == 0)
		return Observation(0, 0, 0, 0);
	return Observation(0.8, (ar+br)/2, (ac+bc)/2, 0);
}
