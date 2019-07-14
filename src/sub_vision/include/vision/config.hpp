#ifndef VISION_CONFIG_HPP
#define VISION_CONFIG_HPP 

/*
 * TODO Rewrite config using parameters.
 */
enum class CameraMode { MOCK, LIVE };
enum Task { 
	GATE,
	OCTAGON
};
const CameraMode CAMERA_MODE = CameraMode::LIVE;
const bool LOG = false;
const bool SIM = true;
const float HFOV = 83;
const float VFOV = 90;
const int FRONT = 0;
const int DOWN = 1;
const float FIMG_DIM[2] = { 3648, 5472 };
const float DIMG_DIM[2] = { 480, 640 };

#endif 
