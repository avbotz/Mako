/** @file acquisition.cpp
 *  @brief Main node runner for acquiring front camera images with Spinnaker.
 *
 *  The naming conventions behind this code might seem off at first glance, but
 *  they are (somewhat) justified. Under conventional logic, it makes sense that
 *  the down camera variables are labeled with a 2, since we don't use the down
 *  camera each run. However, its serial number is smaller than front cam, so it 
 *  makes more sense to suffix it with a 1.
 *
 *  Someone should also multithread this code so the down camera is slowed down
 *  because of the front camera's low FPS. However, don't forget to release
 *  pointers to all of the camera buffers, otherwise they will need to be power
 *  cycled.
 *
 *  @author David Zhang
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;


void acquireImages(CameraPtr down_cam, CameraPtr front_cam, INodeMap &nm1, 
		INodeMap &nm2, INodeMap &nm_device1, INodeMap &nm_device2)
{
	// Setup ROS publishers.
	ROS_INFO("Beginning acquisition node.");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher front_pub = it.advertise("front_camera", 1);
	image_transport::Publisher down_pub = it.advertise("down_camera", 1);

	try
	{
		// Check if front camera is writable.
		CEnumerationPtr acq_mode2 = nm2.GetNode("AcquisitionMode");
		if (!IsAvailable(acq_mode2) || !IsWritable(acq_mode2))
		{
			ROS_ERROR("Front camera is either unavailable and/or unwritable");
			return;
		}

		// Check if front camera can be set to continuous acquisition.
		CEnumEntryPtr acq_mode_cont2 = acq_mode2->GetEntryByName("Continuous");
		if (!IsAvailable(acq_mode_cont2) || !IsReadable(acq_mode_cont2))
		{
			ROS_ERROR("Front camera cannot be set to continuous.");
			return;
		}

		// Set exposure settings.
		Spinnaker::GenApi::CEnumerationPtr auto_exp_2 = 
			front_cam->GetNodeMap().GetNode("ExposureAuto");
		auto_exp_2->SetIntValue(auto_exp_2->GetEntryByName("Off")->GetValue());
		Spinnaker::GenApi::CEnumerationPtr exp_mode_2 = 
			front_cam->GetNodeMap().GetNode("ExposureMode");
		exp_mode_2->SetIntValue(exp_mode_2->GetEntryByName("Timed")->GetValue());
		Spinnaker::GenApi::CFloatPtr exp_time_2 = 
			front_cam->GetNodeMap().GetNode("ExposureTime");
		exp_time_2->SetValue(2305);

		// Set acquisition mode to continuous.
		int64_t acq_cont_val2 = acq_mode_cont2->GetValue();
		acq_mode2->SetIntValue(acq_cont_val2);

		// Begin acquisition for camera.
		front_cam->BeginAcquisition();
		ROS_INFO("Beginning acquisition for front camera.");

		// Get serial number.
		gcstring serial_no_2("");
		CStringPtr string_serial_2 = nm_device2.GetNode("DeviceSerialNumber");
		if (IsAvailable(string_serial_2) && IsReadable(string_serial_2))
		{
			serial_no_2 = string_serial_2->GetValue();
			ROS_INFO("Front Serial Number: %s", serial_no_2);
		}

		// Check if down camera is plugged in.
		if (down_cam != NULL)
		{
			// Check if down camera is writable.
			CEnumerationPtr acq_mode1 = nm1.GetNode("AcquisitionMode");
			if (!IsAvailable(acq_mode1) || !IsWritable(acq_mode1))
			{
				ROS_ERROR("Down camera is either unavailable and/or unwritable");
				return;
			}

			// Check if down camera can be set to continuous acquisition.
			CEnumEntryPtr acq_mode_cont1 = acq_mode1->GetEntryByName("Continuous");
			if (!IsAvailable(acq_mode_cont1) || !IsReadable(acq_mode_cont1))
			{
				ROS_ERROR("Down camera cannot be set to continuous.");
				return;
			}

			// Set exposure settings.
			/*
			Spinnaker::GenApi::CEnumerationPtr auto_exp_1 = 
				down_cam->GetNodeMap().GetNode("ExposureAuto");
			auto_exp_1->SetIntValue(auto_exp_1->GetEntryByName("Off")->GetValue());
			Spinnaker::GenApi::CEnumerationPtr exp_mode_1 = 
				down_cam->GetNodeMap().GetNode("ExposureMode");
			exp_mode_1->SetIntValue(exp_mode_1->GetEntryByName("Timed")->GetValue());
			Spinnaker::GenApi::CFloatPtr exp_time_1 = 
				down_cam->GetNodeMap().GetNode("ExposureTime");
			exp_time_1->SetValue(1305);

			// Set acquisition mode to continuous.
			int64_t acq_cont_val1 = acq_mode_cont1->GetValue();
			acq_mode1->SetIntValue(acq_cont_val1);
			*/

			// Begin acquisition for camera.
			down_cam->BeginAcquisition();
			ROS_INFO("Beginning acquisition for down camera.");

			// Get serial number.
			gcstring serial_no_1("");
			CStringPtr string_serial_1 = nm_device1.GetNode("DeviceSerialNumber");
			if (IsAvailable(string_serial_1) && IsReadable(string_serial_1))
			{
				serial_no_1 = string_serial_1->GetValue();
				ROS_INFO("Down Serial Number: %s", serial_no_1);
			}
		}

		while (ros::ok())
		{
			try
			{
				ROS_INFO("Attempting to get next front image.");
				ImagePtr front_img = front_cam->GetNextImage();
				ROS_INFO("Finished getting next front image.");

				// Ensure image completion.
				if (front_img->IsIncomplete())
				{
					ROS_ERROR("Incomplete front image.");
				}
				else
				{
					// Get image information.
					size_t width = front_img->GetWidth();
					size_t height = front_img->GetHeight();

					// Convert to OpenCV.
					ImagePtr ip = front_img->Convert(Spinnaker::PixelFormat_BGR8, 
							Spinnaker::HQ_LINEAR);
					cv::Mat img(ip->GetHeight(), ip->GetWidth(), 
							CV_8UC3, ip->GetData(), ip->GetStride()); 
					cv::Mat out;
					img.copyTo(out);

					// Publish image.
					sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
							"bgr8", out).toImageMsg();
					front_pub.publish(msg);
				}

				// Release image to prevent buffer overflow.
				front_img->Release();

				if (down_cam != NULL)
				{
					ROS_INFO("Attempting to get next down image.");
					ImagePtr down_img = down_cam->GetNextImage();
					ROS_INFO("Finished getting next down image.");

					if (down_img->IsIncomplete())
					{
						ROS_ERROR("Incomplete down image.");
					}
					else
					{
						// Get image information.
						size_t width = down_img->GetWidth();
						size_t height = down_img->GetHeight();

						// Convert to OpenCV.
						ImagePtr ip = down_img->Convert(Spinnaker::PixelFormat_BGR8, 
								Spinnaker::HQ_LINEAR);
						cv::Mat img(ip->GetHeight(), ip->GetWidth(), 
								CV_8UC3, ip->GetData(), ip->GetStride()); 
						cv::Mat out;
						img.copyTo(out);

						// Publish image.
						sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
								"bgr8", out).toImageMsg();
						down_pub.publish(msg);
					}

					// Release image to prevent buffer overflow.
					down_img->Release();
				}
			}
			catch (Spinnaker::Exception &e)
			{
				ROS_ERROR("Error: %s", e.what());
			}
		}

		// End acquisition ensures that cameras do not have to be powered cycled
		// to run again.
		front_cam->EndAcquisition();
		if (down_cam != NULL)
			down_cam->EndAcquisition();
	}
	catch (Spinnaker::Exception &e)
	{
		ROS_ERROR("Error: %s", e.what());
	}
}

void runCameras(CameraPtr down_cam, CameraPtr front_cam)
{
	try
	{
		INodeMap &nm2 = front_cam->GetTLDeviceNodeMap();
		front_cam->Init();
		INodeMap &nm_device2 = front_cam->GetNodeMap();

		if (down_cam != NULL)
		{
			INodeMap &nm1 = down_cam->GetTLDeviceNodeMap();
			down_cam->Init();
			INodeMap &nm_device1 = down_cam->GetNodeMap();
			acquireImages(down_cam, front_cam, nm1, nm2, nm_device1, nm_device2);
		}
		else 
		{
			/** References need to be tied to a variable, so this bad solution
			 *  exists. The gist is that I tied the down cam references to the
			 *  front cam, but since acquireImages() should notice that the down
			 *  cam is set to NULL, it won't interfere with them.
			 */
			INodeMap &nm1 = front_cam->GetTLDeviceNodeMap();
			front_cam->Init();
			INodeMap &nm_device1 = front_cam->GetNodeMap();
			acquireImages(down_cam, front_cam, nm1, nm2, nm_device1, nm_device2);
		}

		// Deinitialize cameras.
		if (down_cam != NULL)
			down_cam->DeInit();
		front_cam->DeInit();
	}
	catch (Spinnaker::Exception &e)
	{
		ROS_ERROR("Error: %s", e.what());
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_acquisition_node");

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

	/** The camera serial numbers make this code a bit of a headache. Since the
	 *  down camera SN is less than the front camera, it shows up first.
	 *  However, if the down camera isn't connected, which happens often, then
	 *  the front camera has the lowest SN.
	 */
	CameraPtr down_cam = NULL;
	CameraPtr front_cam = NULL;
	if (num_cameras == 1)
	{
		front_cam = cameras.GetByIndex(0);
	}
	if (num_cameras == 2)
	{
		down_cam = cameras.GetByIndex(0);
		front_cam = cameras.GetByIndex(1);
	}
	runCameras(down_cam, front_cam);

	// Deinitialize cameras. 
	down_cam = NULL;
	front_cam = NULL;
	cameras.Clear();
	system->ReleaseInstance();
	ROS_INFO("Press enter to exit.");
	getchar();
}

