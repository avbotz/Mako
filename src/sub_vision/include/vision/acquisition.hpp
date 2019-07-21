/** @file acquisition.hpp
 *  @brief Function definitions for image acquisition from BlackFly cameras.
 *
 *  @author David Zhang
 */
#ifndef VISION_ACQUISITION_HPP
#define VISION_ACQUISITION_HPP

#include <string>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

/** @brief Sets up camera to continuous acquisition.
 *
 *  @param camera Camera to setup.
 */
void setupContinuousAcquisition(Spinnaker::CameraPtr camera);

/** @brief Sets up camera to continuous acquisition with custom exposure time.
 *
 *  @param camera Camera to setup.
 */
void setupContinuousAcquisition(Spinnaker::CameraPtr camera, int exposure_time);

/** @brief Runs camera with current settings and publishes to ROS channel.
 */
void runCamera(Spinnaker::CameraPtr camera, std::string channel);

#endif
