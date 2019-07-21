/** @file acquisition.cpp
 *  @brief Main node runner for acquiring front camera images with Spinnaker.
 *
 *  @author David Zhang
 */
#include <thread>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "vision/acquisition.hpp"
#include "vision/log.hpp"
#include "vision/config.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;


void setupContinuousAcquisition(CameraPtr camera)
{
	try 
	{
		INodeMap &nm_device = camera->GetTLDeviceNodeMap();
		camera->Init();
		INodeMap &nm = camera->GetNodeMap();

		// Check if front camera is writable.
		CEnumerationPtr acq_mode = nm.GetNode("AcquisitionMode");
		if (!IsAvailable(acq_mode) || !IsWritable(acq_mode))
		{
			ROS_ERROR("Front camera is either unavailable and/or unwritable");
			return;
		}

		// Check if front camera can be set to continuous acquisition.
		CEnumEntryPtr acq_mode_cont = acq_mode->GetEntryByName("Continuous");
		if (!IsAvailable(acq_mode_cont) || !IsReadable(acq_mode_cont))
		{
			ROS_ERROR("Front camera cannot be set to continuous.");
			return;
		}

		// Set acquisition mode to continuous.
		int64_t acq_cont_val = acq_mode_cont->GetValue();
		acq_mode->SetIntValue(acq_cont_val);
	}
	catch (Spinnaker::Exception &e)
	{
		ROS_ERROR("Error: %s", e.what());
	}
}

void setupContinuousAcquisition(CameraPtr camera, int exposure_time)
{
	try 
	{
		INodeMap &nm_device = camera->GetTLDeviceNodeMap();
		camera->Init();
		INodeMap &nm = camera->GetNodeMap();

		// Check if front camera is writable.
		CEnumerationPtr acq_mode = nm.GetNode("AcquisitionMode");
		if (!IsAvailable(acq_mode) || !IsWritable(acq_mode))
		{
			ROS_ERROR("Front camera is either unavailable and/or unwritable");
			return;
		}

		// Check if front camera can be set to continuous acquisition.
		CEnumEntryPtr acq_mode_cont = acq_mode->GetEntryByName("Continuous");
		if (!IsAvailable(acq_mode_cont) || !IsReadable(acq_mode_cont))
		{
			ROS_ERROR("Front camera cannot be set to continuous.");
			return;
		}

		// Set exposure settings.
		Spinnaker::GenApi::CEnumerationPtr auto_exp_ = 
			camera->GetNodeMap().GetNode("ExposureAuto");
		auto_exp_->SetIntValue(auto_exp_->GetEntryByName("Off")->GetValue());
		Spinnaker::GenApi::CEnumerationPtr exp_mode_ = 
			camera->GetNodeMap().GetNode("ExposureMode");
		exp_mode_->SetIntValue(exp_mode_->GetEntryByName("Timed")->GetValue());
		Spinnaker::GenApi::CFloatPtr exp_time_ = 
			camera->GetNodeMap().GetNode("ExposureTime");
		exp_time_->SetValue(exposure_time);

		// Set acquisition mode to continuous.
		int64_t acq_cont_val = acq_mode_cont->GetValue();
		acq_mode->SetIntValue(acq_cont_val);
	}
	catch (Spinnaker::Exception &e)
	{
		ROS_ERROR("Error: %s", e.what());
	}
}

void runCamera(CameraPtr camera, std::string channel)
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise(channel, 1);

	// Begin acquisition for camera.
	camera->BeginAcquisition();
	ROS_INFO("Beginning acquisition for %s channel.", channel.c_str());

	// Setup FPS logger.
	ros::Time tracker = ros::Time::now();

	while (ros::ok())
	{
		try
		{
			// Down camera is too fast, slow it down a bit.
			if (channel == "down_camera")
				ros::Duration(0.20).sleep();

			// Camera will hang here if buffer has nothing.
			// ROS_INFO("Attempt for %s channel.", channel.c_str());
			ImagePtr img_ptr = camera->GetNextImage();
			// ROS_INFO("Completed attempt for %s channel.", channel.c_str());

			// Ensure image completion.
			if (img_ptr->IsIncomplete())
			{
				ROS_ERROR("Incomplete image for %s channel.", channel.c_str());
			}
			else
			{
				// Get image information.
				size_t width = img_ptr->GetWidth();
				size_t height = img_ptr->GetHeight();

				// Convert to OpenCV.
				ImagePtr ip = img_ptr->Convert(Spinnaker::PixelFormat_BGR8, 
						Spinnaker::HQ_LINEAR);
				cv::Mat img(ip->GetHeight(), ip->GetWidth(), 
						CV_8UC3, ip->GetData(), ip->GetStride()); 
				cv::Mat out;
				img.copyTo(out);

				// Log images as needed.
				if (FAST_LOG && channel == "down_camera")
					log(out, 'd');
				if (FAST_LOG && channel == "front_camera")
					log(out, 'f');

				// Publish image.
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
						"bgr8", out).toImageMsg();
				pub.publish(msg);
			}

			// Release image to prevent buffer overflow.
			img_ptr->Release();

			// Calculate FPS.
			ros::Time temp = ros::Time::now();
			double dt = (temp-tracker).toSec();
			double fps = 1./dt;
			tracker = ros::Time::now();
			ROS_INFO("FPS for %s: %f", channel.c_str(), fps);
		}
		catch (Spinnaker::Exception &e)
		{
			ROS_ERROR("Error: %s", e.what());
		}
	}

	// End acquisition ensures that cameras do not have to be powered cycled
	// to run again.
	camera->EndAcquisition();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_acquisition_node");

	// Setup log folder if needed.
	init();

	// Find connected cameras from system.
	SystemPtr system = System::GetInstance();
	CameraList cameras = system->GetCameras();
	int num_cameras = cameras.GetSize();
	ROS_INFO("Number of cameras: %i", num_cameras);

	// Exit if no cameras are detected.
	if (num_cameras == 0)
	{
		cameras.Clear();
		system->ReleaseInstance();
		ROS_INFO("Not enough cameras. Exiting now.");
		getchar();
		return 0;
	}

	/*  
	 * The camera serial numbers make this code a bit of a headache. Since the
	 * down camera SN is less than the front camera, it shows up first.
	 * However, if the down camera isn't connected, which happens often, then
	 * the front camera has the lowest SN.
	 */
	CameraPtr down_cam = NULL;
	CameraPtr front_cam = NULL;
	if (num_cameras == 1)
	{
		front_cam = cameras.GetByIndex(0);
		setupContinuousAcquisition(front_cam, 1305);
		runCamera(front_cam, "front_camera");
	}
	if (num_cameras == 2)
	{
		down_cam = cameras.GetByIndex(0);
		front_cam = cameras.GetByIndex(1);
		setupContinuousAcquisition(down_cam);
		setupContinuousAcquisition(front_cam, 1305);
		std::thread t1(runCamera, down_cam, "down_camera");
		std::thread t2(runCamera, front_cam, "front_camera");
		t1.join();
		t2.join();
	}

	// Deinitialize cameras.
	if (down_cam != NULL)
		down_cam->DeInit();
	front_cam->DeInit();
	down_cam = NULL;
	front_cam = NULL;
	cameras.Clear();
	system->ReleaseInstance();
	ROS_INFO("Press enter to exit.");
	getchar();
}

