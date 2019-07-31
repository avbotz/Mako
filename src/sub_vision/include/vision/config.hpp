/** @file config.hpp
 *  @brief Vision configuration that is used in other packages as well.
 *
 *  @author David Zhang
 */
#ifndef VISION_CONFIG_HPP
#define VISION_CONFIG_HPP 

/*
 * TODO Rewrite config using parameters.
 */

/*
 * Deprecated. Use mock_camera_node instead.
 */
enum class CameraMode { MOCK, LIVE };

enum Task { 
	NONE,
	GATE,
	GATE_ML,
	TARGET,
	TARGET_ML,
	BINS,
	BINS_ML,
	OCTAGON
};
const CameraMode CAMERA_MODE = CameraMode::LIVE;
const bool LOG_FRONT = false;
const bool LOG_DOWN = false;
const bool FAST_LOG = false;
const float HFOV = 83;
const float VFOV = 90;
const float DOWN_HFOV = 135;
const float DOWN_VFOV = 119;
const int FRONT = 0;
const int DOWN = 1;
const float FIMG_DIM[2] = { 3648, 5472 };
const float DIMG_DIM[2] = { 964, 1288 };

#endif 
