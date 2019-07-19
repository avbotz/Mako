/** @file log.cpp
 *  @brief Logging functions for images or text.
 *  
 *  @author David Zhang
 */
#include <iostream>
#include <string>
#include <ctime>
#include <unistd.h>
#include <sys/stat.h>
#include <ros/ros.h>
#include "vision/log.hpp"


void log(const cv::Mat &img, char ending)
{
	// Add random number.
	int i = (rand()%1000) + 1; 

	// Get date and time to form directory and file name.
	time_t t = std::time(0);
	struct tm now = *std::localtime(&t);
	char date[80], name[80];
	std::strftime(date, sizeof(date), "%Y_%m_%d", &now);
	std::strftime(name, sizeof(name), "%H_%M_%S", &now);
	std::string loc = "logs/" + std::string(date) + "/" + std::string(name) + 
		"_" + std::string(1, ending) + "_" + std::to_string(i) + ".png";
	ROS_INFO("%s", loc.c_str());

	// Create directory if it doesn't exist.
	mkdir(std::string("logs/" + std::string(date)).c_str(), ACCESSPERMS);

	// Write the images to the correct path, but make sure they exist first. 
	if (!img.data)
		ROS_ERROR("Could not find logging image data.");
	if (!cv::imwrite(loc, img))
		ROS_ERROR("Could not log image.");
}

void init()
{
	// Get date and time to form directory and file name.
	time_t t = std::time(0);
	struct tm now = *std::localtime(&t);
	char date[80], curr[80];
	std::strftime(date, sizeof(date), "%Y_%m_%d", &now);
	std::strftime(curr, sizeof(curr), "%H_%M_%S", &now);

	// Create directory if it doesn't exist.
	mkdir(std::string("logs/text/" + std::string(date)).c_str(), ACCESSPERMS);
}
