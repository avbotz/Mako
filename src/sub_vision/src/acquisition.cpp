/** @file acquisition.cpp
 *  @brief Main node runner for acquiring front camera images with Spinnaker.
 *
 *  @author Point Grey Research, Inc.
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


void acquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
{
	// Setup ROS publishers.
	ROS_INFO("Beginning acquisition node.");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher front_pub = it.advertise("front_camera", 1);
	image_transport::Publisher down_pub = it.advertise("down_camera", 1);

	try
	{
		CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
		if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
			return -1;

		CEnumEntryPtr ptrAcquisitionModeContinuous = 
			ptrAcquisitionMode->GetEntryByName("Continuous");
		if (!IsAvailable(ptrAcquisitionModeContinuous) ||
				!IsReadable(ptrAcquisitionModeContinuous))
		{
			std::cout << "Unable to set acquisition to continuous." << std::endl;
			return -1;
		}

		// Set exposure settings.
		Spinnaker::GenApi::CEnumerationPtr exposureAuto = pCam->GetNodeMap().GetNode("ExposureAuto");
		exposureAuto->SetIntValue(exposureAuto->GetEntryByName("Off")->GetValue());
		Spinnaker::GenApi::CEnumerationPtr exposureMode = pCam->GetNodeMap().GetNode("ExposureMode");
		exposureMode->SetIntValue(exposureMode->GetEntryByName("Timed")->GetValue());
		Spinnaker::GenApi::CFloatPtr exposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
		exposureTime->SetValue(1305);

		// Set acquisition mode to continuous.
		int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
		ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

		// Begin acquisition for camera.
		pCam->BeginAcquisition();
		ROS_INFO("Beginning acquisition.");

		// Get serial number.
		gcstring deviceSerialNumber("");
		CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
		if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
		{
			deviceSerialNumber = ptrStringSerial->GetValue();
			std::cout << "Serial number: " << deviceSerialNumber << std::endl;
		}

		while (ros::ok())
		{
			try
			{
				std::cout << std::endl << "Attempting to get next image." << std::endl;
				ImagePtr pResultImage = pCam->GetNextImage();
				std::cout << "Next image received." << std::endl;

				// Ensure image completion.
				if (pResultImage->IsIncomplete())
				{

					std::cout << "Incomplete image with status " << 
						pResultImage->GetImageStatus() << std::endl;
				}
				else
				{
					// Get image information.
					size_t width = pResultImage->GetWidth();
					size_t height = pResultImage->GetHeight();

					// Convert to OpenCV.
					ImagePtr convertedImage = pResultImage->Convert(Spinnaker::PixelFormat_BGR8,
							Spinnaker::HQ_LINEAR); 
					cv::Mat img(convertedImage->GetHeight(), convertedImage->GetWidth(), 
							CV_8UC3, convertedImage->GetData(), convertedImage->GetStride()); 
					cv::Mat out;
					img.copyTo(out);

					// Publish image.
					sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
							"bgr8", out).toImageMsg();
					front_pub.publish(msg);
				}

				// Release image to prevent buffer overflow.
				pResultImage->Release();
			}
			catch (Spinnaker::Exception &e)
			{
				std::cout << "Error: " << e.what() << std::endl;
			}
		}

		// End acquisition ensures that cameras do not have to be powered cycled
		// to run again.
		pCam->EndAcquisition();
	}
	catch (Spinnaker::Exception &e)
	{
		std::cout << "Error: " << e.what() << std::endl;
	}

}

void runCameras(CameraPtr down_cam, CameraPtr front_cam)
{
	try
	{
		// Acquire images from both cameras.
		INodeMap &inm2 = down_cam->GetTLDeviceNodeMap();
		INodeMap &inm1 = front_cam->GetTLDeviceNodeMap();
		down_cam->Init();
		front_cam->Init();
		INodeMap &nm1 = down_cam->GetNodeMap();
		INodeMap &nm2 = front_cam->GetNodeMap();
		acquireImages(pCam, nodeMap, nodeMapTLDevice);

		// Deinitialize cameras.
		down_cam->DeInit();
		front_cam->DeInit();
	}
	catch (Spinnaker::Exception &e)
	{
		std::cout << "Error: " << e.what() << std::endl;
	}
}

int main(int argc, char** argv)
{
	int result = 0;
	ros::init(argc, argv, "vision_acquisition_node");

	SystemPtr system = System::GetInstance();
	CameraList cameras = system->GetCameras();
	int num_cameras = cameras.GetSize();
	ROS_INFO("Number of cameras: %i", num_cameras);

	if (num_cameras == 0)
	{
		cameras.Clear();
		system->ReleaseInstance();
		std::cout << "Not enough cameras!" << std::endl;
		std::cout << "Done! Press Enter to exit..." << std::endl;
		getchar();

		return -1;
	}

	CameraPtr down_cam = NULL;
	CameraPtr front_cam = cameras.GetByIndex(0);
	if (num_cameras == 2)
		down_cam = cameras.GetByIndex(1);
	runCameras(down_cam, front_cam);

	down_cam = NULL;
	front_cam = NULL;
	cameras.Clear();
	system->ReleaseInstance();
	ROS_INFO("Press enter to exit.");
	getchar();
	return result;
}

