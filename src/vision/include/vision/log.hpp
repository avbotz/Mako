#ifndef VISION_LOG_HPP
#define VISION_LOG_HPP 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void log(const cv::Mat &, char ending);
void init();

#endif 