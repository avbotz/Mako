#ifndef VISION_CONFIG_HPP
#define VISION_CONFIG_HPP 

enum class CameraMode { MOCK, LIVE };
enum Task { 
    GATE,
    OCTAGON
};
const CameraMode CAMERA_MODE = CameraMode::MOCK;
const bool LOG = false;
const float HFOV = 135;
const float VFOV = 90;
const int FRONT = 0;
const int DOWN = 1;
const float FIMG_DIM[2] = { 5472, 3648 };
const float DIMG_DIM[2] = { 640, 480 };

#endif 
