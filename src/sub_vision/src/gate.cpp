/** @file gate.cpp
 *  @brief Vision functions to detect the gate.
 *
 *  @author David Zhang
 *  @author Emil Tu
 */
#include <ros/ros.h>
#include "vision/service.hpp"


Observation VisionService::findGate(const cv::Mat &img)
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
	cv::line(cdst, cv::Point(a_line[0], a_line[1]), cv::Point(a_line[2], 
				a_line[3]), cv::Scalar(255, 255, 255), 3, CV_AA);
	cv::line(cdst, cv::Point(b_line[0], b_line[1]), cv::Point(b_line[2], 
				b_line[3]), cv::Scalar(255, 255, 255), 3, CV_AA);
	cv::circle(cdst, cv::Point((ac+bc)/2, (ar+br)/2), 50, 
			cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);
	log(cdst, 'e');

	// Calculate midpoint and return observation if valid.
	if (ac == 0 && bc == 0)
		return Observation(0, 0, 0, 0);
	return Observation(0.8, (ar+br)/2, (ac+bc)/2, 0);
}

Observation VisionService::findGateML(cv::Mat img)
{
	auto outNames1 = new Tensor(model, "num_detections");
	auto outNames2 = new Tensor(model, "detection_scores");
	auto outNames3 = new Tensor(model, "detection_boxes");
	auto outNames4 = new Tensor(model, "detection_classes");
	auto inpName = new Tensor(model, "image_tensor");

	int rows = img.rows;
	int cols = img.cols;

	cv::Mat inp;
	cv::cvtColor(img, inp, CV_BGR2RGB);

	// Put image in tensor.
	std::vector<uint8_t > img_data;
	img_data.assign(inp.data, inp.data + inp.total()*inp.channels());
	inpName->set_data(img_data, { 1, FIMG_DIM[1], FIMG_DIM[0], 3 });

	model.run(inpName, { outNames1, outNames2, outNames3, outNames4 });

	// Visualize detected bounding boxes.
	int num_detections = (int)outNames1->get_data<float>()[0];
	for (int i = 0; i < num_detections; i++) 
	{
		int classId = (int)outNames4->get_data<float>()[i];
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

			cv::rectangle(img, {(int)x, (int)y}, {(int)right, (int)bottom}, 
					{125, 255, 51}, 2);
			log('e', img);
			return Observation(score, (x+right)/2, (y+bottom)/2, 0);
		}
	}

	return Observation(0, 0, 0, 0);
}
