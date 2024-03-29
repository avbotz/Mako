/** @file bins.cpp
 *  @brief Vision functions to detect the bins
 *
 *  @author David Zhang
 */
#include <ros/ros.h>
#include "vision/service.hpp"
#include "vision/filters.hpp"
#include "vision/log.hpp"


Observation VisionService::findBins(const cv::Mat &input)
{
	/*
	 * This bins code is meant to be run at Suhas' pool, with a black outline
	 * and white inside. Do not use for competition.
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
	cv::inRange(blur, cv::Scalar(0, 0, 0), cv::Scalar(46, 36, 36), thresh);
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
		if (rect_ratio < 2. && rect_ratio > 0.5 && 
				rectangles[i].tl().x+width/2. > 150 && 
				rectangles[i].br().x+height/2. < 1138)
		{
			cv::rectangle(cdst, rectangles[i].tl(), rectangles[i].br(), 
					cv::Scalar(255, 0, 255), 3, 8, 0);		
			int x = (rectangles[i].tl().x+rectangles[i].br().x)/2;
			int y = (rectangles[i].tl().y+rectangles[i].br().y)/2;
			cv::circle(cdst, cv::Point(x, y), 4, cv::Scalar(255, 0, 255), 3);
			log(cdst, 'e');
			return Observation(0.8, y, x, 0);
		}
	}

	// No contours found. I doubt this will run often because the sub is more
	// prone to picking up noise instead of nothing at all.
	return Observation(0, 0, 0, 0);
}


Observation VisionService::findBinsML(cv::Mat img)
{
	// log(img, 'e');

	auto outNames1 = new Tensor(model, "num_detections");
	auto outNames2 = new Tensor(model, "detection_scores");
	auto outNames3 = new Tensor(model, "detection_boxes");
	auto outNames4 = new Tensor(model, "detection_classes");
	auto inpName = new Tensor(model, "image_tensor");

	int rows = img.rows;
	int cols = img.cols;

	cv::Mat inp;
	cv::Mat temp;
	img.copyTo(temp);
	cv::cvtColor(img, inp, CV_BGR2RGB);

	// Put image in tensor.
	std::vector<uint8_t > img_data;
	img_data.assign(inp.data, inp.data + inp.total()*inp.channels());
	inpName->set_data(img_data, { 1, DIMG_DIM[0], DIMG_DIM[1], 3 });

	model.run(inpName, { outNames1, outNames2, outNames3, outNames4 });

	// Visualize detected bounding boxes.
	int num_detections = (int)outNames1->get_data<float>()[0];
	for (int i = 0; i < num_detections; i++) 
	{
		int class_id = (int)outNames4->get_data<float>()[i];
		float score = outNames2->get_data<float>()[i];
		auto bbox_data = outNames3->get_data<float>();
		std::vector<float> bbox = { bbox_data[i*4], bbox_data[i*4+1], 
			bbox_data[i*4+2], bbox_data[i*4+3] };
		if (score > 0.3) 
		{
			float x = bbox[1]*cols;
			float y = bbox[0]*rows;
			float right = bbox[3]*cols;
			float bottom = bbox[2]*rows;

			if (class_id == 2 || class_id == 3)
			{
				if (class_id == 2)
					ROS_INFO("Found wolf bin.");
				if (class_id == 3)
					ROS_INFO("Found bat bin.");
				cv::rectangle(temp, {(int)x, (int)y}, {(int)right, (int)bottom}, 
						{255, 0, 255}, 5);
				log(temp, 'e');
				return Observation(score, (y+bottom)/2, (x+right)/2, 0);
			}
			else if (class_id == 1)
			{
				cv::rectangle(temp, {(int)x, (int)y}, {(int)right, (int)bottom}, 
						{0, 255, 0}, 5);
				log(temp, 'e');
				ROS_INFO("Found closed bin and ignoring for now.");
			}
		}
	}

	return Observation(0, 0, 0, 0);
}
