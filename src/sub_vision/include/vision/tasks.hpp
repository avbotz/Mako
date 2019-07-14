#ifndef TASKS_HPP
#define TASKS_HPP 

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "vision/config.hpp"

struct Observation
{
	float prob;
	float r, c;
	float dist;
	float hangle, vangle;
	bool valid;

	Observation(float _prob, float _r, float _c, float _dist) 
	{
		this->prob = _prob;
		this->r = _r;
		this->c = _c;
		this->dist = _dist;
	}

	Observation(float _prob, float _r, float _c, float _dist, bool _valid) 
	{
		this->prob = _prob;
		this->r = _r;
		this->c = _c;
		this->dist = _dist;
		this->valid = _valid;
	}

	Observation(float _prob, float _r, float _c, float _dist, float _hangle, 
			float _vangle) 
	{
		Observation(_prob, _r, _c, _dist);
		this->hangle = _hangle;
		this->vangle = _vangle;
	}
	
	void calcAngles(int camera) 
	{
		if (camera == FRONT)
		{   
			this->vangle = (r-FIMG_DIM[0]/2.0)/FIMG_DIM[0]*VFOV;
			this->hangle = (c-FIMG_DIM[1]/2.0)/FIMG_DIM[1]*HFOV;
		}
	}

	std::string text()
	{
		std::ostringstream os;
		os.precision(2);
		os << std::fixed;
		os << "(" << hangle << " " << vangle << ", " << r << " " << c << ", " <<
			dist << ", " << prob << ")";
		return os.str();
	}
};

Observation findGate(const cv::Mat &);

#endif
